#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

const uint8_t ZC_PIN = 2; //zero crossing int0
const uint8_t TRIAC_PIN = 7; // heater 
const uint8_t TEMP_PIN = A0; // ternicouple/ntc gun
const uint8_t BUZZER_PIN = 6;

const uint8_t PERIOD = 100; //100half cycle 
const uint8_t MAX_POWER = 100; //100% = full period
const uint32_t ZC_TIMEOUT = 1500;

volatile bool zc_flag = false;
volatile uint8_t zc_isr_count = 0;

uint8_t zc_cnt = 0;
uint32_t last_zc_time = 0;
bool zc_lost = false;

uint16_t temp_raw = 0; //last adc read
uint8_t power_level = 0; //dipakai saat window berjalan
uint8_t power_level_next = 0; // hasil untuk window next
bool adc_triger = false; //flag read adc 
int16_t temp_setpoint = 500;


void ZeroCrossISR();
void handleHalfCycle();
void readTemp();
void keepTemp();
void failedBeep();

void ZeroCrossISR() {
    zc_flag = true;
    zc_isr_count++;
}

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

    //adc read diluar isr timing
    if (adc_triger) {
        adc_triger = false;
        readTemp();
    }
    
    //zc wdt
    if (!zc_lost && (millis() - last_zc_time > ZC_TIMEOUT)) {
        zc_lost = true;
        zc_cnt = 0;
        power_level = power_level_next = 0;
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
        Serial.print(count);
        Serial.print(" ADC RAW:");
        Serial.println(temp_raw);
    }
}

void handleHalfCycle() {
    zc_cnt++;
    if (zc_cnt >= PERIOD) {
        zc_cnt = 0;
        power_level = power_level_next; //pindah buffer
        keepTemp(); //hitung untuk next window
    }

    if (zc_cnt < power_level) {
        digitalWrite(TRIAC_PIN, HIGH);
        delayMicroseconds(120);
        digitalWrite(TRIAC_PIN, LOW);
    } else {
        digitalWrite(TRIAC_PIN, LOW);
        adc_triger = true;
    }
}

void readTemp() {
    temp_raw = analogRead(TEMP_PIN);
}

void keepTemp() {
    //dummy pid simple p only for testing
    int16_t error = temp_setpoint - (int16_t)temp_raw;

    if (error > 50) {
        power_level_next = MAX_POWER; //full jika jauh temp_setpoint
    } else if (error > 10) {
        power_level_next = 50; //half 
    } else if (error > -10) {
        power_level_next = 20; //low to maintain 
    } else {
        power_level_next = 0; //off if above
    }
    power_level_next = constrain(power_level_next, 0, MAX_POWER);

    Serial.print("ADC:");
    Serial.print(temp_raw);
    Serial.print(" Err:");
    Serial.print(error);
    Serial.print(" Pwr:");
    Serial.print(power_level);
    Serial.print(" Next:");
    Serial.println(power_level_next);
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




