#include <Arduino.h>

#define ZC_PIN 2
#define TRIAC_PIN 4
#define THERMOCOUPLE_PIN A0
#define HEATER_PERIOD 100
#define HISTORY_SIZE 16 

uint16_t history[HISTORY_SIZE];
uint8_t history_count = 0;
uint8_t history_idx = 0;

const uint16_t adc_ambient = 9;
const uint16_t adc_200 = 587;
const uint16_t adc_300 = 751;
const uint16_t adc_400 = 850;
const uint16_t temp_ambient = 25;
const uint16_t temp_200 = 200;
const uint16_t temp_300 = 300;
const uint16_t temp_400 = 400;


static volatile bool zc_event_flag = false;

uint16_t clamp_u16(uint16_t val, uint16_t min, uint16_t max) {
    if (val < min) {
        return min;
    }
    if (val > max) {
        return max;
    }
    return val;
}

int16_t interpolate(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max) {
    return out_min + ((x - in_min) * (out_max - out_min)) / (in_max - in_min);
}

void history_put(uint16_t val) {
    history[history_idx] = val;
    history_idx++;

    if (history_idx >= HISTORY_SIZE) {
        history_idx = 0;
    }
    if (history_count < HISTORY_SIZE) {
        history_count++;
    }
}

uint16_t history_avg(void) {
    if (history_count == 0) {
        return 0;
    }
    uint32_t sum = 0;
    for (uint8_t i=0; i<history_count ; i++) {
        sum += history[i];
    }
    sum += history_count >> 1;//integer rounding = sum+len/2
    sum /= history_count;
    return uint16_t(sum);
}

uint16_t history_last(void) {
    if (history_count == 0) {
        return 0;
    }
    uint8_t last_idx;
    if (history_idx == 0) {
        last_idx = HISTORY_SIZE - 1;
    } else {
        last_idx = history_idx - 1;
    }
    return history[last_idx];
}

uint16_t history_top(void) {
    if (history_count == 0) {
        return 0;
    }
    return history[0];
}

float history_dispersion(void) {
    if (history_count < 3) {
        return 1000;
    }

    uint32_t sum = 0;
    uint32_t avg = history_avg();

    for (uint8_t i=0 ; i<history_count ; i++) {
        long q = history[i];
        q -= avg;
        q *= q;

        sum += q;
    }
    sum += history_count << 1; //sum+=history_count*2
    float d = (float)sum / (float)history_count;
    return d;
}

uint16_t adc_to_temp(uint16_t adc) {
    uint16_t temp;
    if (adc <= adc_ambient) {
        temp = temp_ambient;
    } else if (adc < adc_200) {
        temp = interpolate(adc, adc_ambient, adc_200, temp_ambient, temp_200);
    } else if (adc < adc_300) {
        temp = interpolate(adc, adc_200, adc_300, temp_200, temp_300);
    } else {
        temp = interpolate(adc, adc_300, adc_400, temp_300, temp_400);
    }
    return temp;
}

uint16_t temp_to_adc(uint16_t temp) {
    temp = clamp_u16(temp, 150, 500);
    uint16_t adc;

    if (temp >= temp_300){
        adc = interpolate(temp + 1, temp_300, temp_400, adc_300, adc_400);
    } else {
        adc = interpolate(temp + 1, temp_200, temp_300, adc_200, adc_300);
    } 
    for (uint8_t i=0 ; i<10 ; i++) {
        if (adc_to_temp(adc) <= temp)
            break;

        adc--;
    }
    return adc;
}

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

    if (event) {

    }
}

