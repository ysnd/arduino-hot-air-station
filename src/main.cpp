#include <Arduino.h>

#define ZC_PIN 2
#define TRIAC_PIN 4
#define FAN_PIN 9
#define THERMOCOUPLE_PIN A0
#define HEATER_PERIOD 100
#define HISTORY_SIZE 16
#define MAX_POWER 70
#define MAX_COOL_FAN 255
#define MIN_FAN_SPEED 99
#define TEMP_GUN_COLD 90
#define INT_TEMP_MAX 900
#define ZC_TIMEOUT_MS 1500UL

const uint16_t adc_ambient = 9;
const uint16_t adc_200 = 587;
const uint16_t adc_300 = 751;
const uint16_t adc_400 = 850;
const uint16_t temp_ambient = 25;
const uint16_t temp_200 = 200;
const uint16_t temp_300 = 300;
const uint16_t temp_400 = 400;
const byte PID_DENOMINATOR = 11;

typedef enum {
    POWER_OFF,
    POWER_ON,
    POWER_FIXED,
    POWER_COOLING
} power_mode_t;

typedef struct {
    uint8_t k;
    int32_t data;
} emp_avg_t;

typedef struct {
    uint16_t queue[HISTORY_SIZE];
    uint8_t len;
    uint8_t index;
} history_t;

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
    power_mode_t mode;
    emp_avg_t sensor;
    history_t temp_history;
    history_t power_history;
    uint16_t temp_set;
    uint8_t actual_power;
    uint8_t fan_speed;
    uint8_t actual_fan;
    uint8_t fix_power;
    uint8_t count;
    bool active;
    bool chill;
    bool error;
} gun_t;

static volatile bool zc_event_flag = false;
uint32_t last_period = 0;

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

void emp_init(emp_avg_t *emp, uint8_t length) {
    emp->k = length;
    emp->data = 0;
}

void emp_update(emp_avg_t *emp, int32_t val) {
    uint8_t round = emp->k >> 1;
    emp->data += val - (emp->data + round) / emp->k;
}

int32_t emp_read(emp_avg_t *emp) {
    uint8_t round = emp->k >> 1;
    return (emp->data + round) / emp->k;
}

void history_init(history_t *h) {
    h->len = 0;
    h->index = 0;
}

void history_put(history_t *h, uint16_t val) {
    if (h->len < HISTORY_SIZE) {
        h->queue[h->len++] = val;
    } else {
        h->queue[h->index] = val;
        if (++h->index >= HISTORY_SIZE) {
        h->index = 0;
        }
    }
}

uint16_t history_avg(history_t *h) {
    if (h->len == 0) {
        return 0;
    }
    uint32_t sum = 0;
    for (uint8_t i=0; i < h->len; i++) {
        sum += h->queue[i];
    }
    sum += h->len >> 1;//integer rounding = sum+len/2
    sum /= h->len;
    return uint16_t(sum);
}

uint16_t history_last(history_t *h) {
    if (h->len == 0) {
        return 0;
    }
    uint8_t i = h->len - 1;
    if (h->index) {
        i = h->index - 1;
    }
    return h->queue[i];
}

uint16_t history_top(history_t *h) {
    if (h->len == 0) {
        return 0;
    }
    return h->queue[0];
}

float history_dispersion(history_t *h) {
    if (h->len < 3) {
        return 1000;
    }

    uint32_t sum = 0;
    uint32_t avg = history_avg(h);

    for (uint8_t i=0 ; i<h->len ; i++) {
        long q = h->queue[i];
        q -= avg;
        q *= q;

        sum += q;
    }
    sum += h->len << 1; //sum+=history_count*2
    float d = (float)sum / (float)h->len;
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
    pid->kp = 250;
    pid->ki = 16;
    pid->kd = 60;

    pid_reset(pid, -1);
}

int32_t pid_round(int32_t power) {
    power+= (1L << (PID_DENOMINATOR - 1));

    return power >> PID_DENOMINATOR;
}

int32_t pid_req_power(pid_t *pid, int16_t temp_set, int16_t temp_curr) {
    if (!pid->iterate) {
        if ((temp_set - temp_curr) < 20) {
            if (!pid->iterate) {
                pid->iterate = true;
                pid->power = 0;
                pid->i_sum = 0;
            }
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

bool zc_is_alive(void) {
    return (millis() - last_period) < ZC_TIMEOUT_MS;
}

//fan
void fan_init(void) {
    pinMode(FAN_PIN, OUTPUT);
    digitalWrite(FAN_PIN, LOW);
    noInterrupts();
    TCNT1 = 0;
    TCCR1A = 0;
    TCCR1B = _BV(WGM13);
    ICR1 = 256;
    TCCR1A |= _BV(COM1A1);
    TCCR1B |= _BV(CS10);
    OCR1A = 0;
    interrupts();
}

void fan_set(gun_t *gun, uint8_t duty) {
    OCR1A = duty;
    gun->actual_fan = duty;
}

void gun_init(gun_t *gun) {
    gun->mode = POWER_OFF;
    gun->temp_set = temp_to_adc(300);
    gun->fan_speed = 120;
    gun->actual_fan = 0;
    gun->actual_power = 0;
    gun->fix_power = 0;
    gun->count = 0;
    gun->active = false;
    gun->chill = false;
    gun->error = false;
    history_init(&gun->temp_history);
    history_init(&gun->power_history);
    emp_init(&gun->sensor, 40);
}

bool gun_sync(gun_t *gun){
    if (++gun->count >= HEATER_PERIOD) {
        gun->count = 0;

        //end of power period 
        last_period = millis();

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
    if (!gun->active) {
        emp_update(&gun->sensor, analogRead(THERMOCOUPLE_PIN));
    }
    return (gun->count == 0);
}

bool gun_is_cold(gun_t *gun) {
    return history_avg(&gun->temp_history) < TEMP_GUN_COLD;
}

void gun_shutdown(gun_t *gun) {
    digitalWrite(TRIAC_PIN, LOW);
    fan_set(gun, 0);
    gun->mode = POWER_OFF;
    gun->actual_power = 0;
    gun->active = false;
    gun->chill = false;
}

bool gun_is_connected(void) {
    return true;
}

void gun_switch_power(gun_t *gun, bool on) {
    switch (gun->mode) {
        case POWER_OFF :
            if (gun-> actual_fan == 0) {
                if (on) {
                    gun->mode = POWER_ON;
                }
            } else {
                if (on) {
                    if (gun_is_connected()) {
                        gun->mode = POWER_ON;
                    } else {
                        gun_shutdown(gun);
                    }
                } else {
                    if (gun_is_connected()) {
                        if (gun_is_cold(gun)) {
                            gun_shutdown(gun);
                        } else {
                            gun->mode = POWER_COOLING;
                        }
                    }
                }
            }
            break;
        case POWER_ON:
            if (!on) {
                gun->mode = POWER_COOLING;
            }
            break;

        case POWER_FIXED:
            if (gun->actual_fan) {
                if (on) {
                    gun->mode = POWER_ON;
                } else {
                    if (gun_is_connected()) {
                        if (gun_is_cold(gun)) {
                            gun_shutdown(gun);
                        } else {
                            gun->mode = POWER_COOLING;
                        }
                    }
                }
            } else {
                if (!on) {
                    gun_shutdown(gun);
                }
            }
            break;

        case POWER_COOLING:
            if (gun->actual_fan) {
                if (on) {
                    if (gun_is_connected()) {
                        gun->mode = POWER_ON;
                    } else {
                        gun_shutdown(gun);
                    }
                } else {
                    if (gun_is_connected()) {
                        if (gun_is_cold(gun)) {
                            gun_shutdown(gun);
                        }
                    } else {
                        gun_shutdown(gun);
                    }
                }
            } else {
                if (on) {
                    gun->mode = POWER_ON;
                }
            }
            break;
    }
    history_init(&gun->power_history);
}

void keep_temp(gun_t *gun, pid_t *pid) {
    uint16_t temp_adc = emp_read(&gun->sensor);
    history_put(&gun->temp_history, temp_adc);
    int32_t power = 0;

    //safety
    if ((temp_adc >= INT_TEMP_MAX + 30) || (temp_adc > gun->temp_set + 100)) {
        if (gun->mode == POWER_ON) {
            gun->chill = true;
        }
    }

    switch (gun->mode) {
        case POWER_OFF:
            break;

        case POWER_ON:
            fan_set(gun, gun->fan_speed);
            if (gun->chill) {
                if (temp_adc < gun->temp_set - 8) {
                    gun->chill = false;
                    pid_reset(pid, temp_adc);
                }
                else {
                    break;
                }
            }
            power = pid_req_power(pid, gun->temp_set, temp_adc);
            power = clamp(power, 0, MAX_POWER);
            break;

        case POWER_FIXED:
            power = gun->fix_power;
            fan_set(gun, gun->fan_speed);
            break;

        case POWER_COOLING:
            if (gun->actual_fan < MIN_FAN_SPEED) {
                gun_shutdown(gun);
            } else {
                if (gun_is_connected()) {
                    if (gun_is_cold(gun)) {
                        gun_shutdown(gun);
                    } else {
                        fan_set(gun, MAX_COOL_FAN);
                    }
                } else {
                    gun_shutdown(gun);
                }
            }
            break;
    }
    gun->actual_power = power;

    if (power == 0) {
        digitalWrite(TRIAC_PIN, LOW);
        gun->active = false;
    }
    history_put(&gun->power_history, gun->actual_power);
}

void setup() {
    Serial.begin(115200);
    pinMode(ZC_PIN, INPUT_PULLUP);
    pinMode(TRIAC_PIN, OUTPUT);
    digitalWrite(TRIAC_PIN, LOW);
    gun_init(&gun);
    pid_init(&pid);
    fan_init();
    attachInterrupt(digitalPinToInterrupt(ZC_PIN), zc_isr, RISING);
}

void loop() {
    noInterrupts();

    bool event = zc_event_flag;
    zc_event_flag = false;
    
    interrupts();

    if (event) {
        bool end_of_period = gun_sync(&gun);
        if (end_of_period) {
            keep_temp(&gun, &pid);
        }
    }
    if (last_period != 0 && !zc_is_alive()) {
        gun.error = true;
        gun.actual_power = 0;
        digitalWrite(TRIAC_PIN, LOW);
    }
}

