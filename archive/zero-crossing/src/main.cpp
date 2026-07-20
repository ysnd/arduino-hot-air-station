#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

const uint8_t ZC_PIN = 2; //zero crossing int0
const uint8_t TRIAC_PIN = 7; // heater 
const uint8_t BUZZER_PIN = 6;

const uint8_t PERIOD = 100; //100half cycle 
const uint32_t ZC_TIMEOUT = 1500;

volatile bool zc_flag = false;
volatile uint8_t zc_isr_count = 0;

uint8_t zc_cnt = 0;
uint32_t last_zc_time = 0;
bool zc_lost = false;

void ZeroCrossISR();
void handleHalfCycle();
void failedBeep();

void ZeroCrossISR() {
    zc_flag = true;
    zc_isr_count++;
}

void readTemp();

void setup() {
    Serial.begin(115200);
    pinMode(ZC_PIN, INPUT);
    pinMode(TRIAC_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(TRIAC_PIN, LOW);

    attachInterrupt(digitalPinToInterrupt(ZC_PIN), ZeroCrossISR, RISING);
    last_zc_time = millis();
    Serial.println("Hot Air Gun Controller");
    Serial.println("Waiting for AC sync...");
}

void loop() {
    if (zc_flag) {
        zc_flag = false;
        last_zc_time = millis();

        if (zc_lost) {
            zc_lost = false;
            zc_cnt=0;                
            Serial.println("AC sync lanjut");
        }
        handleHalfCycle();
    }
    
    //zc wdt
    if (!zc_lost && (millis() - last_zc_time > ZC_TIMEOUT)) {
        zc_lost = true;
        zc_cnt = 0;
        digitalWrite(TRIAC_PIN, LOW);
        failedBeep();
        Serial.println("ERROR ZeroCross Tidak terdeteksi heater off!");
    }
    static uint32_t last_debug = 0;
    if (millis() - last_debug >=1000) {
        last_debug = millis();
        uint8_t count = zc_isr_count;
        zc_isr_count = 0;
        Serial.print("ZC rate:");
        Serial.println(count);
    }
}

void handleHalfCycle() {
    zc_cnt++;
    if (zc_cnt >= PERIOD) {
        zc_cnt = 0;
    }
}

void failedBeep(void) { 
    digitalWrite(BUZZER_PIN, HIGH); 
    delay(170); 
    digitalWrite(BUZZER_PIN, LOW); 
    delay(10);
    digitalWrite(BUZZER_PIN, HIGH); 
    delay(80);
    digitalWrite(BUZZER_PIN, LOW); 
    delay(100);
    digitalWrite(BUZZER_PIN, HIGH); 
    delay(80);  
    digitalWrite(BUZZER_PIN, LOW);
}



