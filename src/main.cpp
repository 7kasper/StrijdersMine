#include <Arduino.h>
// ESP8266 GPIO numbers (not Dx aliases)
const uint8_t PIN_DATA = 14;   // DS  -> 74HC595 pin 14
const uint8_t PIN_CLOCK = 13;  // SH_CP/SRCLK -> pin 11
const uint8_t PIN_LATCH = 12;  // ST_CP/RCLK  -> pin 12
const uint8_t PIN_OE = 5;      // OE (active LOW) -> pin 13

// Tuning flags (adjust if behavior isn’t right)
bool ACTIVE_LOW = true;       // try flipping to false if all relays are inverted
bool MSB_FIRST_ORDER = true;  // try false (LSBFIRST) if order seems reversed
bool HIGH_BYTE_FIRST = true;  // try false to swap which 8 relays are first

uint16_t relayState = 0;

void writeOutputs(uint16_t value) {
    // Apply desired logic polarity
    uint16_t v = ACTIVE_LOW ? ~value : value;

    // Blank the outputs while we shift to avoid glitches (optional)
    digitalWrite(PIN_OE, HIGH);  // disable outputs (OE is active low)
    digitalWrite(PIN_LATCH, LOW);

    // Choose byte order
    uint8_t hi = (v >> 8) & 0xFF;
    uint8_t lo = v & 0xFF;

    if (HIGH_BYTE_FIRST) {
        shiftOut(PIN_DATA, PIN_CLOCK, MSB_FIRST_ORDER ? MSBFIRST : LSBFIRST, hi);
        shiftOut(PIN_DATA, PIN_CLOCK, MSB_FIRST_ORDER ? MSBFIRST : LSBFIRST, lo);
    } else {
        shiftOut(PIN_DATA, PIN_CLOCK, MSB_FIRST_ORDER ? MSBFIRST : LSBFIRST, lo);
        shiftOut(PIN_DATA, PIN_CLOCK, MSB_FIRST_ORDER ? MSBFIRST : LSBFIRST, hi);
    }

    digitalWrite(PIN_LATCH, HIGH);
    digitalWrite(PIN_OE, LOW);  // enable outputs
}

void setRelay(uint8_t index, bool on) {
    if (index > 15)
        return;
    if (on)
        relayState |= (1U << index);
    else
        relayState &= ~(1U << index);
    writeOutputs(relayState);
}

void setAll(bool on) {
    relayState = on ? 0xFFFF : 0x0000;
    writeOutputs(relayState);
}

void setup() {
    Serial.begin(115200);
    delay(100);

    pinMode(PIN_DATA, OUTPUT);
    pinMode(PIN_CLOCK, OUTPUT);
    pinMode(PIN_LATCH, OUTPUT);
    pinMode(PIN_OE, OUTPUT);

    // Known-safe startup: outputs disabled, lines low
    digitalWrite(PIN_OE, HIGH);  // disable outputs
    digitalWrite(PIN_DATA, LOW);
    digitalWrite(PIN_CLOCK, LOW);
    digitalWrite(PIN_LATCH, LOW);
    delay(5);
    // Push all OFF before enabling outputs
    relayState = 0x0000;
    writeOutputs(relayState);

    Serial.println("Relay test starting...");
}

void loop() {
    // Phase A: quick polarity check (once)
    static bool didPolarity = false;
    if (!didPolarity) {
        Serial.println("All OFF for 2s...");
        setAll(false);
        delay(2000);
        Serial.println("All ON for 2s...");
        setAll(true);
        delay(2000);
        Serial.println("All OFF...");
        setAll(false);
        delay(1000);
        didPolarity = true;
    }

    // Phase B: walking one across 16 outputs
    for (uint8_t i = 0; i < 16; i++) {
        relayState = 0;  // only one ON at a time
        relayState |= (1U << i);
        writeOutputs(relayState);
        Serial.printf("Relay index %u ON\n", i);
        delay(800);
    }
}