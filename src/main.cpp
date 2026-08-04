#include <Arduino.h>

#define ZC_PIN 2
#define TRIAC_PIN 4
#define THERMOCOUPLE_PIN A0
#define HEATER_PERIOD 100
#define HISTORY_SIZE 16 

uint16_t history[HISTORY_SIZE];
uint8_t history_count = 0;
uint8_t history_idx = 0;
uint32_t last_zc_ms = 0;

const uint16_t adc_ambient = 9;
const uint16_t adc_200 = 587;
const uint16_t adc_300 = 751;
const uint16_t adc_400 = 850;
const uint16_t temp_ambient = 25;
const uint16_t temp_200 = 200;
const uint16_t temp_300 = 300;
const uint16_t temp_400 = 400;
const byte PID_DENOMINATOR = 11;

typedef struct {
    int16_t temp_h0;
    int16_t temp_h1;

    int32_t power;
    int32_t i_sum;

    int32_t kp;
    int32_t ki;
    int32_t kd;

    bool iterate;
} pid_t;

typedef struct {
    uint16_t temp_set;
    uint8_t actual_power;
    uint8_t count;
    bool active;
    bool chill;
    bool on;
    bool error;
} gun_t;

static volatile bool zc_event_flag = false;
gun_t gun;
pid_t pid;

int32_t clamp(int32_t val, int32_t min, int32_t max) {
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
    temp = clamp(temp, 150, 500);
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

//PID
void pid_reset(pid_t *pid, int16_t temp) {
    pid->temp_h0 = 0;
    pid->power = 0;
    pid->i_sum = 0;
    pid->iterate = false;

    if ((temp > 0) && (temp < 1000)) {
        pid->temp_h1 = temp;
    } else {
        pid->temp_h1 = 0;
    }
}

void pid_init(pid_t *pid) {
    pid->kp = 638;
    pid->ki = 196;
    pid->kd = 1;

    pid_reset(pid, -1);
}

int32_t pid_round(int32_t power) {
    power+= (1L << (PID_DENOMINATOR - 1));

    return power >> PID_DENOMINATOR;
}

int32_t pid_req_power(pid_t *pid, int16_t temp_set, int16_t temp_curr) {
    if (!pid->iterate) {
        if ((temp_set - temp_curr) < 30) {
            pid->iterate = true;
            pid->power = 0;
            pid->i_sum = 0;
        }
        pid->i_sum += temp_set - temp_curr;//error
        pid->power = pid->kp * (temp_set - temp_curr) + pid->ki * pid->i_sum;
    } else {
        int32_t kp = pid->kp * (pid->temp_h1 - temp_curr);
        int32_t ki = pid->ki * (temp_set - temp_curr);
        int32_t kd = pid->kd * (pid->temp_h0 + temp_curr - (2 * pid->temp_h1));
        pid->power += kp + ki + kd; 
    }
    if (pid->iterate) {
        pid->temp_h0 = pid->temp_h1;
    }
    pid->temp_h1 = temp_curr;

    return pid_round(pid->power);
}

//ZC
static void zc_isr() {
    zc_event_flag = true;
}

void gun_init(gun_t *gun) {
    gun->temp_set = temp_to_adc(300);
    gun->actual_power = 0;
    gun->count = 0;
    gun->active = false;
    gun->chill = false;
    gun->on = true;
    gun->error = false;
}

bool gun_sync(gun_t *gun){
    if (++gun->count >= HEATER_PERIOD) {
        gun->count = 0;

        if ((!gun->active) && (gun->actual_power > 0)) {
            digitalWrite(TRIAC_PIN, HIGH);
            gun->active = true;
        }
    } else if (gun->count >= gun->actual_power) {
        if (gun->active) {
            digitalWrite(TRIAC_PIN, LOW);
            gun->active = false;
        }
    }
    return (gun->count == 0);
}

void keep_temp(gun_t *gun, pid_t *pid) {
    uint16_t temp = analogRead(THERMOCOUPLE_PIN);
    history_put(temp);

    if (!gun->chill && gun->on && temp > gun->temp_set + 20) {
        gun->actual_power = 0;
        gun->chill = true;
    }
    if (gun->chill) {
        if (temp < gun->temp_set - 8) {
            gun->chill = false;
            pid_reset(pid, temp);
        } else {
            gun->actual_power = 0;
            return;
        }
    }
    int32_t power = pid_req_power(pid, gun->temp_set, temp);

    gun->actual_power = (uint16_t)clamp(power, 0, HEATER_PERIOD);
}

void setup() {
    Serial.begin(115200);
    pinMode(ZC_PIN, INPUT_PULLUP);
    pinMode(TRIAC_PIN, OUTPUT);
    digitalWrite(TRIAC_PIN, LOW);
    gun_init(&gun);
    pid_init(&pid);
    attachInterrupt(digitalPinToInterrupt(ZC_PIN), zc_isr, RISING);
}

void loop() {
    noInterrupts();

    bool event = zc_event_flag;
    zc_event_flag = false;
    
    interrupts();

    if (event) {
        last_zc_ms = millis();
        bool end_of_period = gun_sync(&gun);
        if (end_of_period) {
            keep_temp(&gun, &pid);
        }
    }
    if (millis() - last_zc_ms > 1000) {
        gun.error = true;
        gun.actual_power = 0;
    }
}

