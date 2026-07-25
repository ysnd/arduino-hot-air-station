#include "HardwareSerial.h"
#include <Arduino.h>
#include <avr/interrupt.h>

#define ENC_A 3 
#define ENC_B 5
#define ENC_SW 4
volatile int encPos = 0;
bool swPressed = false;
volatile unsigned long lastIntrTime = 0;

void readEnc() {
    if (millis() - lastIntrTime < 5) return;
    lastIntrTime = millis();
    int a = digitalRead(ENC_A);
    int b = digitalRead(ENC_B);

    if (a == b) {
        encPos++;
    } else {
        encPos--;
    }
} 

void setup() {
    Serial.begin(115200);
    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);
    pinMode(ENC_SW, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_A), readEnc, CHANGE);
}

void loop() {
    noInterrupts();
    int pos = encPos;
    interrupts();

    Serial.println(pos);

    if (!digitalRead(ENC_SW)) {
        delay(50);
        if (!digitalRead(ENC_SW)) {
            Serial.println("SW Pressed");
            while (!digitalRead(ENC_SW)); 
        }
    }
    delay(10);
}
