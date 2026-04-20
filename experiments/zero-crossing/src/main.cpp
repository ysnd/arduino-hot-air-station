#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>


const uint8_t ZC_PIN = 2; //zero crossing int0
const uint8_t TRIAC_PIN = 7; // heater 
const uint8_t TEMP_PIN = A0; // ternicouple/ntc gun
const uint8_t FAN_PIN = 9;  // timer1 pwm (hardcoded)

const uint8_t PERIOD = 100; //100half cycle 
const uint8_t MAX_POWER = 100; //100% = full period

volatile bool zc_flag = false;
volatile uint8_t zc_cnt = 0;

uint16_t temp_raw = 0; //last adc read
uint8_t power_level = 0; //0-100 output triac
bool adc_triger = false; //flag read adc 
int16_t temp_setpoint = 500;
int16_t pid_last_error = 0;
int32_t pid_integral = 0;

void ZeroCrossISR();
void handleHalfCycle();
void readTemp();
void keepTemp();
void fanInit();
void fanSet(uint8_t speed);

void ZeroCrossISR() {
    zc_flag = true;
}

void setup() {
    Serial.begin(115200);
    pinMode(ZC_PIN, INPUT);
    pinMode(TRIAC_PIN, OUTPUT);
    digitalWrite(TRIAC_PIN, LOW);

    fanInit();
    fanSet(64);

    attachInterrupt(digitalPinToInterrupt(ZC_PIN), ZeroCrossISR, RISING);
}

void fanInit() {
    pinMode(FAN_PIN, OUTPUT);
    digitalWrite(FAN_PIN, LOW);

    noInterrupts();
    TCNT1 = 0;
    TCCR1B = _BV(WGM13);
    TCCR1A = 0;
    ICR1 = 256;
    TCCR1B = _BV(WGM13) | _BV(CS10);
    TCCR1A |= _BV(COM1A1);
    OCR1A = 0;
    interrupts();
}

void fanSet(uint8_t speed) {
    OCR1A = speed;
}

void loop() {
    if (zc_flag) {
        zc_flag = false;
        handleHalfCycle();
    }

    //adc read outside isr timing
    if (adc_triger) {
        adc_triger = false;
        readTemp();
    }
}

void handleHalfCycle() {
    zc_cnt++;

    if (zc_cnt >= PERIOD) {
        zc_cnt = 0;
        keepTemp(); //pid update once per detik
    }

    if (zc_cnt < power_level) {
        digitalWrite(TRIAC_PIN, HIGH);
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
        power_level = MAX_POWER; //full jika jauh temp_setpoint
    } else if (error > 10) {
        power_level = 50; //half 
    } else if (error > -10) {
        power_level = 20; //low to maintain 
    } else {
        power_level = 0; //off if above
    }
}
