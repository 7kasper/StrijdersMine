// =========================== //
// =         IMPORTS         = //
// =========================== //

#include <Arduino.h>
#include <Wire.h>
#include <ESP8266WiFi.h> 
#include <ESP8266WebServer.h> 
#include <ESP8266mDNS.h> 
#include <LittleFS.h>
#include <DNSServer.h>

// =========================== //
// =     PIN DEFINITIONS     = //
// =========================== //

// 74HC595s
const uint8_t PIN_74HC595_DATA = 14;        // DS  -> 74HC595 pin 14
const uint8_t PIN_74HC595_CLOCK = 13;       // SH_CP/SRCLK -> pin 11
const uint8_t PIN_74HC595_LATCH = 12;       // ST_CP/RCLK  -> pin 12
const uint8_t PIN_74HC595_OUTPUTENABLE = 5; // OE (active LOW) -> pin 13
// I2C pins (match wiring above)
const uint8_t PIN_I2C_SDA = 4;              // IO2
const uint8_t PIN_I2C_SCL = 2;              // IO4 (Weird its swapped but whatever)

// =========================== //
// =     Mine Definitions    = //
// =========================== //

static const uint16_t SOLENOID_FIRE_TIME = 500;

static const uint8_t GRID_COLS = 4;
static const uint8_t GRID_ROWS = 12;
static const uint8_t SECTION_ROWS = 4;
static const uint8_t NUM_SECTIONS = GRID_ROWS / SECTION_ROWS;
static const float DEFAULT_MINE_RATIO = 0.60f;

static uint8_t currentSection = 0;
static bool mines[GRID_ROWS][GRID_COLS];
// Non-blocking 500 timers for the mines
static uint32_t relayOffAt[16] = {0};

// =========================== //
// =     I2C Definitions     = //
// =========================== //

// MCP23017 base address (A2..A0 = 000)
const uint8_t I2C_MCP_ADDR = 0x27;

// MCP23017 registers (BANK=0/default)
enum {
    MCP_IODIRA = 0x00, MCP_IODIRB = 0x01,
    MCP_GPPUA  = 0x0C, MCP_GPPUB  = 0x0D,
    MCP_GPIOA  = 0x12, MCP_GPIOB  = 0x13
};

// =========================== //
// =    WiFi DEFINITIONS     = //
// =========================== //

ESP8266WebServer server(80);

const char* MDNS_HOST = "mijnenveld"; // will resolve xxx.local
const char* AP_PASS = "strijders"; // Access point password

// STA credentials (phone hotspot). Currently unused but easy to setup.
const char* STA_SSID = "Eduraam";
const char* STA_PASS = "zegiklekkerniet";

DNSServer dns;
IPAddress apIP(192,168,4,1);
IPAddress netMsk(255,255,255,0);

// =========================== //
// =        Variables        = //
// =========================== //

bool buttonsDriveRelays = true;    // when true, physical buttons control relays
volatile uint16_t lastButtonsMask = 0; // latest read from MCP (1=pressed)

// =========================== //
// =     HELPER FUNCTIONS     = //
// =========================== //
static void printBin(uint32_t v, uint8_t bits) {
    for (int i = bits - 1; i >= 0; --i) {
        Serial.write((v >> i) & 1 ? '1' : '0');
        if ((i % 8) == 0 && i != 0) Serial.write('_'); // group bytes
    }
    Serial.println();
}

// =========================== //
// =     RELAY FUNCTIONS     = //
// =========================== //

// Relay board settings
bool RELAY_ACTIVE_LOW = false; 
bool MSB_FIRST_ORDER = true;
bool HIGH_BYTE_FIRST = true;

// Relay ENUM
enum RelayId : uint8_t {
    RELAY_0 = 0,
    RELAY_1,
    RELAY_2,
    RELAY_3,
    RELAY_4,
    RELAY_5,
    RELAY_6,
    RELAY_7,
    RELAY_8,
    RELAY_9,
    RELAY_A,
    RELAY_B,
    RELAY_C,
    RELAY_D,
    RELAY_E,
    RELAY_F
};

static uint16_t relayState = 0;  // 1 bit per relay: 1=ON (logical), 0=OFF

// Push current relayState to the shift registers
static void writeOutputs(uint16_t value) {
    // Map logical ON/OFF to actual output levels
    uint16_t v = RELAY_ACTIVE_LOW ? ~value : value;

    // Optionally blank during update to avoid glitches
    digitalWrite(PIN_74HC595_OUTPUTENABLE, HIGH);  // disable outputs (OE is active-low)
    digitalWrite(PIN_74HC595_LATCH, LOW);

    uint8_t hi = (v >> 8) & 0xFF;
    uint8_t lo = v & 0xFF;

    if (HIGH_BYTE_FIRST) {
        shiftOut(PIN_74HC595_DATA, PIN_74HC595_CLOCK, MSB_FIRST_ORDER ? MSBFIRST : LSBFIRST, hi);
        shiftOut(PIN_74HC595_DATA, PIN_74HC595_CLOCK, MSB_FIRST_ORDER ? MSBFIRST : LSBFIRST, lo);
    } else {
        shiftOut(PIN_74HC595_DATA, PIN_74HC595_CLOCK, MSB_FIRST_ORDER ? MSBFIRST : LSBFIRST, lo);
        shiftOut(PIN_74HC595_DATA, PIN_74HC595_CLOCK, MSB_FIRST_ORDER ? MSBFIRST : LSBFIRST, hi);
    }

    digitalWrite(PIN_74HC595_LATCH, HIGH);
    digitalWrite(PIN_74HC595_OUTPUTENABLE, LOW);  // enable outputs
}

/**
 * Writes to a specified relay.
 * @param id - The ID of the Relay (Relay_0 to Relay_F)
 * @param level - HIGH or LOW
 */
void writeRelay(RelayId id, uint8_t level) {
    if (id > RELAY_F)
        return;
    bool on = (level == HIGH);  // HIGH means "turn ON"
    if (on)
        relayState |= (1U << id);
    else
        relayState &= ~(1U << id);
    writeOutputs(relayState);
}

/**
 * Reads the current output of the specified Relay.
 * @param id - The ID of the Relay (Relay_0 to Relay_F)
 * @returns HIGH or LOW
 */
uint8_t readRelay(RelayId id) {
    if (id > RELAY_F)
        return LOW;
    return (relayState & (1U << id)) ? HIGH : LOW;
}

/**
 * Toggles the state of specified Relay.
 * @param id - The ID of the Relay (Relay_0 to Relay_F)
 */
void toggleRelay(RelayId id) {
    if (id > RELAY_F)
        return;
    relayState ^= (1U << id);
    writeOutputs(relayState);
}

// =========================== //
// =      I2C FUNCTIONS      = //
// =========================== //

static void mcpWrite8(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(I2C_MCP_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

static uint8_t mcpRead8(uint8_t reg) {
    Wire.beginTransmission(I2C_MCP_ADDR);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(I2C_MCP_ADDR, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0xFF;
}

static uint16_t mcpReadGPIO16() {
    uint8_t a = mcpRead8(MCP_GPIOA);
    uint8_t b = mcpRead8(MCP_GPIOB);
    return ((uint16_t)b << 8) | a;
}

uint16_t getButtonStates() {
    // MCP pull-ups: idle HIGH = 1, pressed LOW = 0
    // Return mask where 1 = pressed
    uint16_t raw = mcpReadGPIO16();
    return (~raw) & 0xFFFF;
}

uint8_t getButtonState(RelayId id) {
    if (id > RELAY_F) return LOW;
    return (getButtonStates() & (1U << id)) ? HIGH : LOW; // HIGH means pressed
}

void pollButtonsToRelays(bool toggle) {
    static uint16_t lastPressed = 0;
    static uint16_t latched     = 0; // for toggle mode

    uint16_t pressed = getButtonStates();
    printBin(pressed, 16);


    if (!toggle) {
        // Hold-to-activate: pressed -> relay ON, released -> OFF
        // Our relayState uses 1=ON, so push mask directly
        relayState = pressed;       // reuse your existing relayState
        writeOutputs(relayState);   // reuse your existing writeOutputs
    } else {
        // Toggle on press (edge triggered)
        uint16_t rising = pressed & ~lastPressed; // newly pressed buttons
        latched ^= rising; // flip relays that had a rising edge
        relayState = latched;
        writeOutputs(relayState);
    }

    lastPressed = pressed;
}

// Init MCP23017: all inputs with pull-ups enabled
static void mcpBegin() {
    delay(10);
    mcpWrite8(MCP_IODIRA, 0xFF); // Port A inputs
    mcpWrite8(MCP_IODIRB, 0xFF); // Port B inputs
    mcpWrite8(MCP_GPPUA,  0xFF); // Port A pull-ups enabled
    mcpWrite8(MCP_GPPUB,  0xFF); // Port B pull-ups enabled
}

// =========================== //
// =     WiFi Functions      = //
// =========================== //

static void startAP() {
    WiFi.mode(WIFI_AP);
    String apSsid = String(MDNS_HOST);
    const char* apPass = AP_PASS;  // 8+ chars; change if you like
    WiFi.softAPConfig(apIP, apIP, netMsk);     // make ESP the gateway/DNS target
    WiFi.softAP(apSsid.c_str(), apPass);
    Serial.printf("AP SSID: %s  PASS: %s\n", apSsid.c_str(), apPass);
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());
    // Wildcard DNS: send all hostnames to our AP IP (captive portal)
    dns.setErrorReplyCode(DNSReplyCode::NoError);
    dns.start(53, "*", apIP);                   // wildcard DNS -> our AP IP
    Serial.println("Started capture portal");
}

static void startMDNS() {
    MDNS.end();  // stop any previous responder
    IPAddress ip = (WiFi.getMode() & WIFI_STA) && (WiFi.status() == WL_CONNECTED)
                       ? WiFi.localIP()
                       : WiFi.softAPIP();

    if (MDNS.begin(MDNS_HOST, ip)) {
        MDNS.addService("http", "tcp", 80);  // advertise HTTP
        Serial.printf("mDNS started: http://%s.local/ (%s)\n", MDNS_HOST,
                      ip.toString().c_str());
    } else {
        Serial.println("mDNS failed to start");
    }
}

// If the Host header doesn’t match our AP IP, redirect to it
static bool captivePortalRedirect() {
    if (WiFi.getMode() & WIFI_AP) {
        String host = server.hostHeader();
        if (host.length() && host != apIP.toString()) {
            server.sendHeader(
                "Location", String("http://") + apIP.toString() + "/index.html", true);
            server.send(302, "text/plain", "");
            return true;
        }
    }
    return false;
}

// Serve the portal page for OS connectivity checks
void handleCaptive() {
    if (captivePortalRedirect())
        return;
    File f = LittleFS.open("/index.html", "r");
    if (f) {
        server.streamFile(f, "text/html");
        f.close();
    } else {
        server.send(200, "text/html",
                    "<!doctype html><html><meta http-equiv='refresh' content='0; "
                    "url=/index.html'>"
                    "<body>Redirecting…</body></html>");
    }
}

static bool beginWiFi() {
    // WiFi.hostname(MDNS_HOST);
    // WiFi.mode(WIFI_STA);
    // WiFi.begin(STA_SSID, STA_PASS);
    // Serial.printf("Attempting to connect to WiFi SSID '%s' ...\n", STA_SSID);

    // uint32_t start = millis();
    // while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
    //     delay(250);
    //     Serial.print(".");
    // }
    // Serial.println();


    // Serial.println("WiFi connect failed. Starting AP...");
    startAP();

    if (WiFi.status() == WL_CONNECTED) {
        startMDNS();
    }
    return false;
}

static void setupWebServer() {
    // Serve static files directly by path
    server.serveStatic("/index.html", LittleFS, "/index.html");
    // Optional: other assets (CSS/JS/images) under /assets
    server.serveStatic("/assets", LittleFS, "/assets");

    // Redirect root to index.html
    server.on("/", HTTP_GET, [](){
        if (captivePortalRedirect()) return;
        server.sendHeader("Location", "/index.html", true);
        server.send(302, "text/plain", "");
    });

    // API: state
    server.on("/api/state", HTTP_GET, [](){
        String json = "{";
        json += "\"buttons\":" + String(lastButtonsMask);
        json += ",\"relays\":" + String(relayState);
        json += ",\"enabled\":" + String(buttonsDriveRelays ? "true" : "false");
        json += "}";
        server.send(200, "application/json", json);
    });

    // API: set relay (id=0..15, level=0/1)
    server.on("/api/relay", HTTP_POST, [](){
        if (!server.hasArg("id") || !server.hasArg("level")) {
            server.send(400, "text/plain", "id and level required");
            return;
        }
        int id = strtol(server.arg("id").c_str(), nullptr, 0);
        int level = strtol(server.arg("level").c_str(), nullptr, 0);
        if (id < 0 || id > 15) {
            server.send(400, "text/plain", "bad id");
            return;
        }
        writeRelay((RelayId)id, level ? HIGH : LOW);
        server.send(200, "text/plain", "OK");
    });

    // API: enable/disable buttons driving relays (value=0/1 or true/false)
    server.on("/api/enable", HTTP_POST, [](){
        if (!server.hasArg("value")) {
            server.send(400, "text/plain", "value required");
            return;
        }
        String v = server.arg("value");
        buttonsDriveRelays = (v == "1" || v == "true" || v == "on");
        server.send(200, "text/plain", buttonsDriveRelays ? "ENABLED" : "DISABLED");
    });

    // Captive portal urls
    server.on("/generate_204", HTTP_ANY, handleCaptive);       // Android
    server.on("/gen_204",      HTTP_ANY, handleCaptive);       // older Android
    server.on("/hotspot-detect.html", HTTP_ANY, handleCaptive); // iOS/macOS
    server.on("/ncsi.txt",     HTTP_ANY, handleCaptive);       // Windows
    server.on("/connecttest.txt", HTTP_ANY, handleCaptive);    // Windows 10/11
    server.on("/fwlink",       HTTP_ANY, handleCaptive);       // Windows captive

    // Fallback: try to serve any other static file; otherwise 404
    server.onNotFound([](){
        if (captivePortalRedirect()) return;

        String uri = server.uri();
        if (uri == "/") {  // just in case
            server.sendHeader("Location", "/index.html", true);
            server.send(302, "text/plain", "");
            return;
        }
        if (LittleFS.exists(uri)) {
            File f = LittleFS.open(uri, "r");
            if (f) {
                server.streamFile(
                    f, "text/plain");  // content-type guess skipped for brevity
                f.close();
                return;
            }
        }
        server.send(404, "text/plain", "Not found");
    });

    server.begin();
    Serial.println("HTTP server started");
}

// =========================== //
// =     Game Functions      = //
// =========================== //

/**
 * Debug print: X = mine, . = safe. Rows 1..12, Cols 1..4
 */
void printMinefield() {
    Serial.println("Minefield (rows x cols): X=mine . =safe");
    for (uint8_t r = 0; r < GRID_ROWS; r++) {
        Serial.printf("%2u: ", r + 1);
        for (uint8_t c = 0; c < GRID_COLS; c++) {
            Serial.print(mines[r][c] ? 'X' : '.');
        }
        Serial.println();
    }
    Serial.printf("Current section: %u (rows %u..%u)\n", currentSection + 1,
                  currentSection * SECTION_ROWS + 1,
                  currentSection * SECTION_ROWS + SECTION_ROWS);
}

// Section helpers
void setSection(uint8_t s) {
    if (s >= NUM_SECTIONS)
        s = NUM_SECTIONS - 1;
    currentSection = s;
    Serial.printf("Section set to %u (rows %u..%u)\n", currentSection + 1,
                  currentSection * SECTION_ROWS + 1,
                  currentSection * SECTION_ROWS + SECTION_ROWS);
}
void nextSection() {
    if (currentSection + 1 < NUM_SECTIONS)
        setSection(currentSection + 1);
}
void prevSection() {
    if (currentSection > 0)
        setSection(currentSection - 1);
}

/**
 * Map a 4x4 pad pin (0..15) to global (row, col) in the 4x12 minefield for a given
 * section pin 0 -> (row1,col1), pin 1 -> (row1,col2), pin 4 -> (row2,col1), pin 15 ->
 * (row4,col4) Section 1 adds +4 to the row (so pin 0 -> (row5,col1)), etc. All indices
 * here are 0-based internally; for printing, add +1.
 * @param pin - The pin to map
 * @param row - The reference to the (global) row to return
 * @param col - The reference to the column to return
 */
static void pinToRowCol(uint8_t pin, uint8_t& row, uint8_t& col) {
    col = pin % GRID_COLS;                              // 0..3
    uint8_t localRow = pin / GRID_COLS;                 // 0..3
    row = currentSection * SECTION_ROWS + localRow;     // 0..11
}

/**
 * Populates the minefield with the set mineratio. Ensures each row has at least 1 free safe cell.
 */
void generateMinefield(float mineRatio) {
    // Seed RNG (lightweight)
    randomSeed(ESP.getChipId() ^ micros());
    for (uint8_t r = 0; r < GRID_ROWS; r++) {
        uint8_t safeCount = 0;
        for (uint8_t c = 0; c < GRID_COLS; c++) {
            bool m = (random(1000) < (int)(mineRatio * 1000.0f));
            mines[r][c] = m;
            if (!m)
                safeCount++;
        }
        if (safeCount == 0) {
            // Force at least one safe cell in this row
            uint8_t safeCol = random(GRID_COLS);
            mines[r][safeCol] = false;
        }
    }
    printMinefield();
}

/**
 * Resets/regenerates the minefield.
 */
void resetMinefield() {
    generateMinefield(DEFAULT_MINE_RATIO);
    setSection(0);
}

// Start a timed pulse on a relay (non-blocking)
void triggerRelayPulse(uint8_t relayIndex, uint16_t ms) {
    if (relayIndex > 15)
        return;
    writeRelay((RelayId)relayIndex, HIGH);
    relayOffAt[relayIndex] = millis() + ms;
}

// Turn off any relays whose pulse expired
void serviceRelayPulses() {
    uint32_t now = millis();
    for (uint8_t i = 0; i < 16; i++) {
        if (relayOffAt[i] != 0 && (int32_t)(now - relayOffAt[i]) >= 0) {
            writeRelay((RelayId)i, LOW);
            relayOffAt[i] = 0;
        }
    }
}

// Handle button presses for minefield logic:
// On rising edge of a button in the current section, if that (row,col) is a mine ->
// pulse relay 500 ms.
void handleMinefieldButtons(uint16_t pressedMask) {
    static uint16_t last = 0;
    uint16_t rising = pressedMask & ~last;  // new presses only
    last = pressedMask;

    for (uint8_t pin = 0; pin < 16; pin++) { // Go through all buttons
        if (rising & (1U << pin)) { // Check if the button is active
            uint8_t row, col;
            pinToRowCol(pin, row, col);
            if (row < GRID_ROWS && col < GRID_COLS) { // Sanity check.
                if (mines[row][col]) {
                    // Mine: fire solenoid for 500 ms
                    triggerRelayPulse(pin, SOLENOID_FIRE_TIME);
                    Serial.printf(
                        "Mine HIT: pin %u -> (row %u, col %u) -> pulse relay %u\n", pin,
                        row + 1, col + 1, pin);
                } else {
                    // Safe: do nothing (ensure relay stays off)
                    Serial.printf("Safe: pin %u -> (row %u, col %u)\n", pin, row + 1,
                                  col + 1);
                }
            }
        }
    }
}



// =========================== //
// =          SETUP          = //
// =========================== //

void setup() {
    Serial.begin(9600);
    delay(50);

    // PinModes
    pinMode(PIN_74HC595_DATA, OUTPUT);
    pinMode(PIN_74HC595_CLOCK, OUTPUT);
    pinMode(PIN_74HC595_LATCH, OUTPUT);
    pinMode(PIN_74HC595_OUTPUTENABLE, OUTPUT);

    // Safe Startup
    digitalWrite(PIN_74HC595_OUTPUTENABLE, HIGH);  // disable outputs
    digitalWrite(PIN_74HC595_DATA, LOW);
    digitalWrite(PIN_74HC595_CLOCK, LOW);
    digitalWrite(PIN_74HC595_LATCH, LOW);
    relayState = 0x0000;       // all OFF logically
    writeOutputs(relayState);  // push initial state

    // I2C Startup
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    mcpBegin();

    // Filesystem Startup
    if (!LittleFS.begin()) {
        Serial.println("LittleFS mount failed. Did you upload data/?");
    }

    // WiFi Startup
    beginWiFi();
    setupWebServer();

    Serial.println("Generating minefield...");
    resetMinefield();

    Serial.println("READY.");
}

// =========================== //
// =        MAIN LOOP        = //
// =========================== //

void loop() {
    // HTTP handling
    dns.processNextRequest();
    server.handleClient();
    MDNS.update();

    // Poll buttons ~50–100 Hz
    static uint32_t lastPoll = 0;
    if (millis() - lastPoll >= 10) {
        lastPoll = millis();
        uint16_t pressed = getButtonStates();  // 1=pressed for bit i
        lastButtonsMask = pressed;
        if (buttonsDriveRelays) {
            // Hold-to-activate: pressed => relay ON
            relayState = pressed;
            writeOutputs(relayState);
        } else {
            handleMinefieldButtons(pressed);
            // Turn off relays whose pulses expired
            serviceRelayPulses();
        }
    }


    // // Demo: blink RELAY_0, keep RELAY_F ON for 3s then OFF for 3s
    // static uint32_t t = 0;
    // static bool blink = false;

    // if (millis() - t >= 1000) {
    //     t = millis();
    //     blink = !blink;

    //     writeRelay(RELAY_0, blink ? HIGH : LOW);
    //     Serial.printf("RELAY_0 -> %s\n", blink ? "ON" : "OFF");
    // }

    // Example usage:
    // writeRelay(RELAY_F, HIGH); delay(3000);
    // writeRelay(RELAY_F, LOW);  delay(3000);
}