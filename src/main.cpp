#include <Arduino.h>

#define ZC_PIN 2
#define TRIAC_PIN 4

static volatile bool zc_event_flag = false;

//ZC
static void zc_isr() {
    zc_event_flag = true;
}

void setup() {
    Serial.begin(115200);
    pinMode(ZC_PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ZC_PIN), zc_isr, RISING);
}

void loop() {
    noInterrupts();
    bool event = zc_event_flag;
    zc_event_flag = false;
    interrupts();
}

