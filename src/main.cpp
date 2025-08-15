// =========================== //
// =         IMPORTS         = //
// =========================== //

#include <Arduino.h>
#include <Wire.h>

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

    // Safe startup
    digitalWrite(PIN_74HC595_OUTPUTENABLE, HIGH);  // disable outputs
    digitalWrite(PIN_74HC595_DATA, LOW);
    digitalWrite(PIN_74HC595_CLOCK, LOW);
    digitalWrite(PIN_74HC595_LATCH, LOW);
    relayState = 0x0000;       // all OFF logically
    writeOutputs(relayState);  // push initial state

    // I2C Startup
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    mcpBegin();

    Serial.println("READY");
}

// =========================== //
// =        MAIN LOOP        = //
// =========================== //

void loop() {
    static uint32_t lastPoll = 0;
    if (millis() - lastPoll >= 10) { // ~100 Hz poll
        lastPoll = millis();
        pollButtonsToRelays(false); // false = hold-to-activate
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