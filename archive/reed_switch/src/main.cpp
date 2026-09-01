#include <Arduino.h>

#define REED_PIN 8

bool state = false;
bool last_state = false;

void reed_init(void) {
    pinMode(REED_PIN, INPUT_PULLUP);
}

void setup() {
    Serial.begin(115200);
    reed_init();
}

void loop() {
    state = !digitalRead(REED_PIN);
    if (state != last_state) {
        if (state) {
            Serial.println("REED ON");
        } else {
            Serial.println("REED OFF");
        }
        last_state = state;
    }
}

