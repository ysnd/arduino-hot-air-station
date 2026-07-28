#include <Arduino.h>

#define THERMOCOUPLE_PIN A0 

void setup() {
    Serial.begin(115200); 
}

void loop() {
    uint16_t adcVal = analogRead(THERMOCOUPLE_PIN);
    Serial.print("ADC VAL:" );
    Serial.println(adcVal);
    delay(200);
}
