// =========================== //
// =         IMPORTS         = //
// =========================== //

#include <Arduino.h>
#include <Wire.h>

// test script
// I2C pin selection (avoid GPIO5 which you use for 74HC595 OE)
const uint8_t SDA_PIN = 4;  // D2
const uint8_t SCL_PIN = 2;  // D4

// MCP23017 address range (A2..A0 select between 0x20..0x27)
const uint8_t MCP_MIN = 0x20;
const uint8_t MCP_MAX = 0x27;

// MCP23017 registers (BANK=0/default)
enum {
    MCP_IODIRA = 0x00,
    MCP_IODIRB = 0x01,
    MCP_GPPUA = 0x0C,
    MCP_GPPUB = 0x0D,
    MCP_GPIOA = 0x12,
    MCP_GPIOB = 0x13,
    MCP_OLATA = 0x14,
    MCP_OLATB = 0x15,
    MCP_IOCON = 0x0A  // IOCON (also at 0x0B in BANK=0)
};

// Pretty-print helpers
static void printBin(uint32_t v, uint8_t bits) {
    for (int i = bits - 1; i >= 0; --i) {
        Serial.write((v >> i) & 1 ? '1' : '0');
        if ((i % 4) == 0 && i != 0)
            Serial.write('_');
    }
}
static void printBin8(uint8_t v) { printBin(v, 8); }
static void printBin16(uint16_t v) { printBin(v, 16); }

// I2C helpers with error reporting
static bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    uint8_t e = Wire.endTransmission();
    if (e) {
        Serial.printf("I2C write error addr 0x%02X reg 0x%02X code %u\n", addr, reg, e);
        return false;
    }
    return true;
}

static bool i2cRead8(uint8_t addr, uint8_t reg, uint8_t& out) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    uint8_t e = Wire.endTransmission(false);  // repeated start
    if (e) {
        Serial.printf("I2C set-reg error addr 0x%02X reg 0x%02X code %u\n", addr, reg,
                      e);
        return false;
    }
    uint8_t n = Wire.requestFrom(addr, (uint8_t)1);
    if (n != 1) {
        Serial.printf("I2C read error addr 0x%02X reg 0x%02X got %u bytes\n", addr, reg,
                      n);
        return false;
    }
    out = Wire.read();
    return true;
}

static bool i2cPing(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}

// Scan bus and print addresses
static void scanI2C() {
    Serial.println("I2C scan start");
    uint8_t found = 0;
    for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
        Wire.beginTransmission(addr);
        uint8_t err = Wire.endTransmission();
        if (err == 0) {
            Serial.printf("  Found device at 0x%02X\n", addr);
            found++;
        } else if (err == 4) {
            Serial.printf("  Unknown error at 0x%02X\n", addr);
        }
    }
    if (!found)
        Serial.println("  No I2C devices found.");
    Serial.println("I2C scan done\n");
}

static bool mcpWriteVerify(uint8_t addr, uint8_t reg, uint8_t val) {
    if (!i2cWrite8(addr, reg, val))
        return false;
    uint8_t rd = 0xFF;
    if (!i2cRead8(addr, reg, rd))
        return false;
    if (rd != val) {
        Serial.printf("Verify mismatch reg 0x%02X wrote 0x%02X read 0x%02X\n", reg, val,
                      rd);
        return false;
    }
    return true;
}

static bool mcpInitInputsPullups(uint8_t addr) {
    Serial.printf("Initializing MCP23017 at 0x%02X\n", addr);
    uint8_t iocon = 0;
    if (!i2cRead8(addr, MCP_IOCON, iocon))
        return false;
    Serial.print("  IOCON = 0b");
    printBin8(iocon);
    Serial.println();

    if (!mcpWriteVerify(addr, MCP_IODIRA, 0xFF))
        return false;
    if (!mcpWriteVerify(addr, MCP_IODIRB, 0xFF))
        return false;
    if (!mcpWriteVerify(addr, MCP_GPPUA, 0xFF))
        return false;
    if (!mcpWriteVerify(addr, MCP_GPPUB, 0xFF))
        return false;

    uint8_t ga = 0xFF, gb = 0xFF;
    if (!i2cRead8(addr, MCP_GPIOA, ga))
        return false;
    if (!i2cRead8(addr, MCP_GPIOB, gb))
        return false;
    Serial.print("  After init GPIOA = 0b");
    printBin8(ga);
    Serial.print("  GPIOB = 0b");
    printBin8(gb);
    Serial.println();

    return true;
}

static uint16_t mcpReadButtons(uint8_t addr) {
    uint8_t ga = 0xFF, gb = 0xFF;
    if (!i2cRead8(addr, MCP_GPIOA, ga))
        return 0x0000;
    if (!i2cRead8(addr, MCP_GPIOB, gb))
        return 0x0000;
    // Pull-ups: idle HIGH, pressed LOW -> invert so 1 = pressed
    uint16_t raw = ((uint16_t)gb << 8) | ga;
    return (~raw) & 0xFFFF;
}

static int foundMcpAddr = -1;
static uint16_t lastButtons = 0;

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println();
    Serial.println("MCP23017 debug tool");
    Serial.printf("Using SDA=GPIO%d, SCL=GPIO%d\n", SDA_PIN, SCL_PIN);

    // Optional quick check for stuck lines (read levels)
    pinMode(SDA_PIN, INPUT);
    pinMode(SCL_PIN, INPUT);
    Serial.printf("Initial line levels: SDA=%d SCL=%d (expect 1/1)\n",
                  digitalRead(SDA_PIN), digitalRead(SCL_PIN));

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);  // 100 kHz

    delay(50);
    scanI2C();

    // Find first MCP23017 in 0x20..0x27
    for (uint8_t a = MCP_MIN; a <= MCP_MAX; ++a) {
        if (i2cPing(a)) {
            foundMcpAddr = a;
            break;
        }
    }

    if (foundMcpAddr < 0) {
        Serial.println("No MCP23017 found (0x20-0x27). Check wiring:");
        Serial.println("- VDD=3.3V, VSS=GND (common with ESP)");
        Serial.println("- SDA->GPIO4(D2), SCL->GPIO2(D4), both pulled up to 3.3V");
        Serial.println("- RESET tied to 3.3V");
        Serial.println("- A0/A1/A2 set for desired address (0x20 if all GND)");
        Serial.println(
            "- If MCP is at 5V, that will pull I2C to 5V and can damage ESP. Use "
            "3.3V.");
        return;
    }

    Serial.printf("MCP23017 detected at 0x%02X\n", foundMcpAddr);

    if (!mcpInitInputsPullups((uint8_t)foundMcpAddr)) {
        Serial.println("MCP init failed. Re-check address pins/bus pull-ups/RESET.");
        return;
    }

    lastButtons = mcpReadButtons((uint8_t)foundMcpAddr);
    Serial.print("Initial buttons (1=pressed): 0b");
    printBin16(lastButtons);
    Serial.println();
    Serial.println("Press buttons; state changes will be printed.");
}

void loop() {
    if (foundMcpAddr < 0) {
        // Re-scan periodically if nothing found
        static uint32_t last = 0;
        if (millis() - last > 3000) {
            last = millis();
            scanI2C();
        }
        return;
    }

    static uint32_t lastPoll = 0;
    if (millis() - lastPoll >= 50) {  // ~20 Hz poll
        lastPoll = millis();
        uint16_t btn = mcpReadButtons((uint8_t)foundMcpAddr);
        if (btn != lastButtons) {
            Serial.print("Buttons: 0b");
            printBin16(btn);
            Serial.print("  changed: 0b");
            printBin16(btn ^ lastButtons);
            Serial.println();
            lastButtons = btn;
        }
    }
}



// // =========================== //
// // =     PIN DEFINITIONS     = //
// // =========================== //

// // 74HC595s
// const uint8_t PIN_74HC595_DATA = 14;        // DS  -> 74HC595 pin 14
// const uint8_t PIN_74HC595_CLOCK = 13;       // SH_CP/SRCLK -> pin 11
// const uint8_t PIN_74HC595_LATCH = 12;       // ST_CP/RCLK  -> pin 12
// const uint8_t PIN_74HC595_OUTPUTENABLE = 5; // OE (active LOW) -> pin 13
// // I2C pins (match wiring above)
// const uint8_t PIN_I2C_SDA = 4;              // IO2
// const uint8_t PIN_I2C_SCL = 2;              // IO4 (Weird its swapped but whatever)

// // =========================== //
// // =     HELPER FUNCTIONS     = //
// // =========================== //
// static void printBin(uint32_t v, uint8_t bits) {
//     for (int i = bits - 1; i >= 0; --i) {
//         Serial.write((v >> i) & 1 ? '1' : '0');
//         if ((i % 8) == 0 && i != 0) Serial.write('_'); // group bytes
//     }
//     Serial.println();
// }

// // =========================== //
// // =     I2C Definitions     = //
// // =========================== //

// // MCP23017 base address (A2..A0 = 000)
// const uint8_t I2C_MCP_ADDR = 0x20;

// // MCP23017 registers (BANK=0/default)
// enum {
//     MCP_IODIRA = 0x00, MCP_IODIRB = 0x01,
//     MCP_GPPUA  = 0x0C, MCP_GPPUB  = 0x0D,
//     MCP_GPIOA  = 0x12, MCP_GPIOB  = 0x13
// };

// // =========================== //
// // =     RELAY FUNCTIONS     = //
// // =========================== //

// // Relay board settings
// bool RELAY_ACTIVE_LOW = false; 
// bool MSB_FIRST_ORDER = true;
// bool HIGH_BYTE_FIRST = true;

// // Relay ENUM
// enum RelayId : uint8_t {
//     RELAY_0 = 0,
//     RELAY_1,
//     RELAY_2,
//     RELAY_3,
//     RELAY_4,
//     RELAY_5,
//     RELAY_6,
//     RELAY_7,
//     RELAY_8,
//     RELAY_9,
//     RELAY_A,
//     RELAY_B,
//     RELAY_C,
//     RELAY_D,
//     RELAY_E,
//     RELAY_F
// };

// static uint16_t relayState = 0;  // 1 bit per relay: 1=ON (logical), 0=OFF

// // Push current relayState to the shift registers
// static void writeOutputs(uint16_t value) {
//     // Map logical ON/OFF to actual output levels
//     uint16_t v = RELAY_ACTIVE_LOW ? ~value : value;

//     // Optionally blank during update to avoid glitches
//     digitalWrite(PIN_74HC595_OUTPUTENABLE, HIGH);  // disable outputs (OE is active-low)
//     digitalWrite(PIN_74HC595_LATCH, LOW);

//     uint8_t hi = (v >> 8) & 0xFF;
//     uint8_t lo = v & 0xFF;

//     if (HIGH_BYTE_FIRST) {
//         shiftOut(PIN_74HC595_DATA, PIN_74HC595_CLOCK, MSB_FIRST_ORDER ? MSBFIRST : LSBFIRST, hi);
//         shiftOut(PIN_74HC595_DATA, PIN_74HC595_CLOCK, MSB_FIRST_ORDER ? MSBFIRST : LSBFIRST, lo);
//     } else {
//         shiftOut(PIN_74HC595_DATA, PIN_74HC595_CLOCK, MSB_FIRST_ORDER ? MSBFIRST : LSBFIRST, lo);
//         shiftOut(PIN_74HC595_DATA, PIN_74HC595_CLOCK, MSB_FIRST_ORDER ? MSBFIRST : LSBFIRST, hi);
//     }

//     digitalWrite(PIN_74HC595_LATCH, HIGH);
//     digitalWrite(PIN_74HC595_OUTPUTENABLE, LOW);  // enable outputs
// }

// /**
//  * Writes to a specified relay.
//  * @param id - The ID of the Relay (Relay_0 to Relay_F)
//  * @param level - HIGH or LOW
//  */
// void writeRelay(RelayId id, uint8_t level) {
//     if (id > RELAY_F)
//         return;
//     bool on = (level == HIGH);  // HIGH means "turn ON"
//     if (on)
//         relayState |= (1U << id);
//     else
//         relayState &= ~(1U << id);
//     writeOutputs(relayState);
// }

// /**
//  * Reads the current output of the specified Relay.
//  * @param id - The ID of the Relay (Relay_0 to Relay_F)
//  * @returns HIGH or LOW
//  */
// uint8_t readRelay(RelayId id) {
//     if (id > RELAY_F)
//         return LOW;
//     return (relayState & (1U << id)) ? HIGH : LOW;
// }

// /**
//  * Toggles the state of specified Relay.
//  * @param id - The ID of the Relay (Relay_0 to Relay_F)
//  */
// void toggleRelay(RelayId id) {
//     if (id > RELAY_F)
//         return;
//     relayState ^= (1U << id);
//     writeOutputs(relayState);
// }

// // =========================== //
// // =      I2C FUNCTIONS      = //
// // =========================== //

// static void mcpWrite8(uint8_t reg, uint8_t val) {
//     Wire.beginTransmission(I2C_MCP_ADDR);
//     Wire.write(reg);
//     Wire.write(val);
//     Wire.endTransmission();
// }

// static uint8_t mcpRead8(uint8_t reg) {
//     Wire.beginTransmission(I2C_MCP_ADDR);
//     Wire.write(reg);
//     Wire.endTransmission();
//     Wire.requestFrom(I2C_MCP_ADDR, (uint8_t)1);
//     return Wire.available() ? Wire.read() : 0xFF;
// }

// static uint16_t mcpReadGPIO16() {
//     uint8_t a = mcpRead8(MCP_GPIOA);
//     uint8_t b = mcpRead8(MCP_GPIOB);
//     return ((uint16_t)b << 8) | a;
// }

// uint16_t getButtonStates() {
//     // MCP pull-ups: idle HIGH = 1, pressed LOW = 0
//     // Return mask where 1 = pressed
//     uint16_t raw = mcpReadGPIO16();
//     return (~raw) & 0xFFFF;
// }

// uint8_t getButtonState(RelayId id) {
//     if (id > RELAY_F) return LOW;
//     return (getButtonStates() & (1U << id)) ? HIGH : LOW; // HIGH means pressed
// }

// void pollButtonsToRelays(bool toggle) {
//     static uint16_t lastPressed = 0;
//     static uint16_t latched     = 0; // for toggle mode

//     uint16_t pressed = getButtonStates();
//     printBin(pressed, 16);


//     if (!toggle) {
//         // Hold-to-activate: pressed -> relay ON, released -> OFF
//         // Our relayState uses 1=ON, so push mask directly
//         relayState = pressed;       // reuse your existing relayState
//         writeOutputs(relayState);   // reuse your existing writeOutputs
//     } else {
//         // Toggle on press (edge triggered)
//         uint16_t rising = pressed & ~lastPressed; // newly pressed buttons
//         latched ^= rising; // flip relays that had a rising edge
//         relayState = latched;
//         writeOutputs(relayState);
//     }

//     lastPressed = pressed;
// }

// // Init MCP23017: all inputs with pull-ups enabled
// static void mcpBegin() {
//     delay(10);
//     mcpWrite8(MCP_IODIRA, 0xFF); // Port A inputs
//     mcpWrite8(MCP_IODIRB, 0xFF); // Port B inputs
//     mcpWrite8(MCP_GPPUA,  0xFF); // Port A pull-ups enabled
//     mcpWrite8(MCP_GPPUB,  0xFF); // Port B pull-ups enabled
// }

// // =========================== //
// // =          SETUP          = //
// // =========================== //

// void setup() {
//     Serial.begin(9600);
//     delay(50);

//     // PinModes
//     pinMode(PIN_74HC595_DATA, OUTPUT);
//     pinMode(PIN_74HC595_CLOCK, OUTPUT);
//     pinMode(PIN_74HC595_LATCH, OUTPUT);
//     pinMode(PIN_74HC595_OUTPUTENABLE, OUTPUT);

//     // Safe startup
//     digitalWrite(PIN_74HC595_OUTPUTENABLE, HIGH);  // disable outputs
//     digitalWrite(PIN_74HC595_DATA, LOW);
//     digitalWrite(PIN_74HC595_CLOCK, LOW);
//     digitalWrite(PIN_74HC595_LATCH, LOW);
//     relayState = 0x0000;       // all OFF logically
//     writeOutputs(relayState);  // push initial state

//     // I2C Startup
//     Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
//     mcpBegin();

//     Serial.println("READY");
// }

// // =========================== //
// // =        MAIN LOOP        = //
// // =========================== //

// void loop() {
//     static uint32_t lastPoll = 0;
//     if (millis() - lastPoll >= 10) { // ~100 Hz poll
//         lastPoll = millis();
//         pollButtonsToRelays(false); // false = hold-to-activate
//     }


//     // // Demo: blink RELAY_0, keep RELAY_F ON for 3s then OFF for 3s
//     // static uint32_t t = 0;
//     // static bool blink = false;

//     // if (millis() - t >= 1000) {
//     //     t = millis();
//     //     blink = !blink;

//     //     writeRelay(RELAY_0, blink ? HIGH : LOW);
//     //     Serial.printf("RELAY_0 -> %s\n", blink ? "ON" : "OFF");
//     // }

//     // Example usage:
//     // writeRelay(RELAY_F, HIGH); delay(3000);
//     // writeRelay(RELAY_F, LOW);  delay(3000);
// }