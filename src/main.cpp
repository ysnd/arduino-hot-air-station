#include "HardwareSerial.h"
#include <Arduino.h>
#include <LCD_I2C.h>

#define ZC_PIN 2
#define TRIAC_PIN 7
#define REED_PIN 8
#define FAN_PIN 9
#define BUZZER_PIN 6
#define ENC_A 3 
#define ENC_B 5 
#define ENC_SW 4 
#define THERMOCOUPLE_PIN A0
#define HEATER_PERIOD 100
#define HISTORY_SIZE 16
#define MAX_POWER 70
#define MAX_FIXED_POWER 70
#define MAX_FAN_SPEED 255
#define MIN_FAN_SPEED 99
#define TEMP_GUN_COLD 90
#define INT_TEMP_MAX 900
#define ZC_TIMEOUT_MS 1500UL
#define ENC_FAST_TIMEOUT 300
#define ENC_OVER_PRESS 1000
#define BTN_LONG_TIME 900
#define BTN_DEBOUNCE 50
#define BTN_TICK_TIME 200
#define BTN_OVER_PRESS 3000
#define TEMP_MIN 150
#define TEMP_MAX 500

const uint16_t adc_ambient = 9;
const uint16_t adc_200 = 587;
const uint16_t adc_300 = 751;
const uint16_t adc_400 = 850;
const uint16_t temp_ambient = 25;
const uint16_t temp_200 = 200;
const uint16_t temp_300 = 300;
const uint16_t temp_400 = 400;
const byte PID_DENOMINATOR = 11;

LCD_I2C lcd(0x27, 16, 2);

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

typedef struct { 
    uint8_t pin;
    bool state;
    bool last_state;
} reed_t;

typedef struct {
    int32_t min_pos;
    int32_t max_pos;
    uint8_t a_pin;
    uint8_t b_pin;
    bool loop_en;
    uint8_t increment;
    uint8_t fast_increment;
    uint32_t a_low_start_time;
    uint32_t last_change_time;
    bool ch_b_state;
    volatile int16_t pos;
} encoder_t;

typedef enum {
    BUTTON_NONE,
    BUTTON_SHORT,
    BUTTON_LONG
} button_evt_t;

typedef struct {
    uint8_t pin;
    uint32_t press_time;
    uint32_t tick_time;
    bool pressed;
} button_t;

typedef enum {
    UI_MAIN,
    UI_CONFIG,
    UI_CALIB,
    UI_TUNE,
    UI_WORK,
    UI_ERROR
} ui_page_t;

typedef enum {
    MAIN_MODE_TEMP,
    MAIN_MODE_FAN
} main_mode_t;

typedef enum {
    WORK_MODE_TEMP,
    WORK_MODE_FAN
} work_mode_t;

typedef enum {
    CONFIG_CALIB,
    CONFIG_TUNE,
    CONFIG_SAVE,
    CONFIG_CANCEL,
    CONFIG_DEFAULTS
} config_mode_t;

typedef enum {
    CALIB_TEMP_MIN,
    CALIB_TEMP_MID,
    CALIB_TEMP_MAX
} calib_point_t;

typedef struct {
    ui_page_t current;
    main_mode_t main_mode;
    work_mode_t work_mode;
    config_mode_t config_mode;
    calib_point_t calib_point;
    uint16_t temp_set;
    uint8_t fan_set;
    bool tune_on;
    uint8_t tune_power;
    bool work_ready;
    int16_t encoder_last_pos;
    bool used;
    bool cool_notified;
    uint32_t clear_used_ms;
    bool display_dirty;
} ui_t;

typedef struct {
    bool full_second_line;
    char temp_units;
} display_t;

reed_t reed;
encoder_t encoder;
button_t enc_button;
ui_t ui;
display_t display;
gun_t gun;
pid_t pid;

static const uint8_t custom_symbols[6][8] = {
    {
        0b00110,
        0b01001,
        0b01001,
        0b00110,
        0b00000,
        0b00000,
        0b00000,
        0b00000
    },
    {
        0b00000,
        0b11001,
        0b01011,
        0b00100,
        0b11010,
        0b10011,
        0b00000,
        0b00000
    },
    {
        0b00011,
        0b00110,
        0b01100,
        0b11111,
        0b00110,
        0b01100,
        0b01000,
        0b10000
    },
    {
        0b00100,
        0b01010,
        0b01010,
        0b01010,
        0b10001,
        0b10001,
        0b01110,
        0b00000
    },
    {
        0b01110,
        0b01010,
        0b01010,
        0b01010,
        0b11011,
        0b01110,
        0b00100,
        0b00000
    },
    {
        0b10111,
        0b10101,
        0b10101,
        0b10101,
        0b10101,
        0b10101,
        0b11101,
        0b00000
    }
};

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
    return out_min + ((int32_t)(x - in_min) * (out_max - out_min)) / (in_max - in_min);
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
    TCCR1B = _BV(WGM13) | _BV(CS12) | _BV(CS10);
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

void gun_fix_power(gun_t *gun, uint8_t power) {
    if (power == 0) {
        gun_switch_power(gun, false);
        return;
    }
    if (power > MAX_POWER) {
        power = MAX_POWER;
    }

    gun->mode = POWER_FIXED;
    gun->fix_power = power;
}

uint8_t gun_avg_power_percent(gun_t *gun) {
    uint8_t percent = 0;
    if (gun->mode == POWER_FIXED) {
        percent = interpolate(gun->fix_power, 0, MAX_POWER, 0, 100);
    } else {
        percent = interpolate(history_avg(&gun->power_history), 0, MAX_POWER, 0, 100);
    }

    if (percent > 100) {
        percent = 100;
    }
    return percent;
}

void gun_set_temp(gun_t *gun, uint16_t temp_c) {
    gun->temp_set = temp_to_adc(temp_c);
}

void gun_set_fan(gun_t *gun, uint8_t fan) {
    gun->fan_speed = fan;
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
                        fan_set(gun, MAX_FAN_SPEED);
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

//BUZZER 
void buzzer_short_beep(void) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(80);
    digitalWrite(BUZZER_PIN, LOW);
}

void buzzer_low_beep(void) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(160);
    digitalWrite(BUZZER_PIN, LOW);
}

void buzzer_double_beep(void) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(160);
    digitalWrite(BUZZER_PIN, LOW);
    delay(150);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(160);
    digitalWrite(BUZZER_PIN, LOW);
}

void buzzer_failed_beep(void) {
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

//Display 
void display_init(void) {
    lcd.begin();
    lcd.backlight();
    lcd.clear();
    for (uint8_t i = 0; i < 6; i++) {
        lcd.createChar(i + 1, (uint8_t *)custom_symbols[i]);
    }
}

void display_t_set(uint16_t t) {
    char buff[10];
    lcd.setCursor(0, 0);
    snprintf(buff, sizeof(buff), "%c%3u%cC", 4, t, 1);
    lcd.print(buff);
}

void display_t_curr(uint16_t t) {
    char buff[10];

    lcd.setCursor(0, 1);
    if (t < 1000) {
        snprintf(buff, sizeof(buff), "%c%3u%cC ", 5, t, 1);
        lcd.print(buff);
    } else {
        lcd.print("xxx");
    }
}

void display_fan(uint8_t s) {
    s = interpolate(s, 0, 255, 0, 99);
    lcd.setCursor(6, 0);
    lcd.print(" ");
    lcd.write(6);
    if (s < 10) {
        lcd.print(s);
    } 
    lcd.print(s);
    lcd.print("%");
}

void display_fan_curr(uint8_t s) {
    s = interpolate(s, 0, 255, 0, 99);
    lcd.setCursor(6, 1);
    lcd.print(" ");
    lcd.write(2);

    if (s < 10) {
        lcd.print(" ");
    }
    lcd.print(s);
    lcd.print("%");
}

void display_power(uint8_t p, bool show_zero) {
    if (p > 99) {
        p = 99;
    }
    lcd.setCursor(11, 1);
    if ((p == 0) && !show_zero) {
        lcd.print("     ");
        return;
    }
    lcd.print(" ");
    lcd.write(3);

    if (p < 10) {
        lcd.print(" ");
    }
    lcd.print(p);
    lcd.print("%");
}

void display_msg_on(void) {
    lcd.setCursor(11, 0);
    lcd.print("   ON");
}

void display_msg_off(void) {
    lcd.setCursor(11, 0);
    lcd.print("  OFF");
}

void display_msg_ready(void) {
    lcd.setCursor(11, 0);
    lcd.print("READY");
}

void display_msg_cold(void) {
    lcd.setCursor(11, 0);
    lcd.print(" COLD");
}

void display_msg_fail(void) {
    lcd.setCursor(0, 1);
    lcd.print("     FAILED     ");
}

void display_msg_tune(void) {
    lcd.setCursor(0, 0);
    lcd.print("TUNE");
}

//Encoder
void encoder_init(encoder_t *enc, uint8_t a_pin, uint8_t b_pin, int16_t init_pos) {
    enc->a_low_start_time = 0;
    enc->a_pin = a_pin;
    enc->b_pin = b_pin;
    enc->pos = init_pos;
    enc->min_pos = -32767;
    enc->max_pos = 32766;
    enc->ch_b_state = false;
    enc->increment = 1;
    enc->fast_increment = 2;
    enc->last_change_time = 0;
    enc->loop_en = false;
    pinMode(enc->a_pin, INPUT_PULLUP);
    pinMode(enc->b_pin, INPUT_PULLUP);
}

int16_t encoder_read(encoder_t *enc) {
    int16_t pos;
    noInterrupts();
    pos = enc->pos;
    interrupts();
    return pos;
}

bool encoder_write(encoder_t *enc, int16_t pos) {
    if (pos >= enc->min_pos && pos <= enc->max_pos) {
        enc->pos = pos;
        return true;
    }
    return false;
}

void encoder_config(encoder_t *enc, int16_t init_pos, int16_t min_pos, int16_t max_pos, uint8_t inc, uint8_t fast_inc, bool looped) {
    enc->min_pos = min_pos;
    enc->max_pos = max_pos;
    if (!encoder_write(enc, init_pos)) {
        enc->pos = min_pos;
    }
    enc->increment = inc;
    enc->fast_increment = inc;
    if (fast_inc > inc) {
        enc->fast_increment = fast_inc;
    }
    enc->loop_en = looped;
}

void encoder_change_isr(encoder_t *enc) {
    bool a = digitalRead(enc->a_pin);
    uint32_t now = millis();

    if (!a) {
        if ((enc->a_low_start_time == 0) || (now-enc->a_low_start_time > ENC_OVER_PRESS)) {
            enc->a_low_start_time = now;
            enc->ch_b_state = digitalRead(enc->b_pin);
        }
    } else {
        if (enc->a_low_start_time > 0) {
            uint8_t step = enc->increment;
            uint32_t duration = now - enc->a_low_start_time;

            if (duration < ENC_OVER_PRESS) {
                if ((now - enc->last_change_time) < ENC_FAST_TIMEOUT) {
                    step = enc->fast_increment;
                }
                enc->last_change_time = now;

                if (enc->ch_b_state) {
                    enc->pos -= step;
                } else {
                    enc->pos += step;
                }
                if (enc->pos > enc->max_pos) {
                    if (enc->loop_en) {
                        enc->pos = enc->min_pos;
                    } else {
                        enc->pos = enc->max_pos;
                    }
                }
                if (enc->pos < enc->min_pos) {
                    if (enc->loop_en) {
                        enc->pos = enc->max_pos;
                    } else {
                        enc->pos = enc->min_pos;
                    }
                }
            }
            enc->a_low_start_time = 0;
        }
    }
}

void encoder_irq(void) {
    encoder_change_isr(&encoder);
}

//Button
void button_init(button_t *btn, uint8_t pin) {
    btn->pin = pin;
    btn->press_time = 0;
    btn->tick_time = 0;
    btn->pressed = false;
    pinMode(btn->pin, INPUT_PULLUP);
}

button_evt_t button_check(button_t *btn) {
    uint32_t now = millis();

    if (!digitalRead(btn->pin)) {
        //tombol diteken
        if (!btn->pressed) {
            btn->pressed = true;
            btn->press_time = now;
        } else if ((now - btn->press_time) > BTN_OVER_PRESS) {
            btn->press_time = now;
        }
        return BUTTON_NONE;
    }
    //tombol tdk diteken
    if (!btn->pressed) {
        return BUTTON_NONE;
    }
    uint32_t dur = now - btn->press_time;
    btn->pressed = false;
    btn->tick_time = 0;

    if (dur < BTN_DEBOUNCE) {
        return BUTTON_NONE;
    }
    if (dur > BTN_LONG_TIME) {
        return BUTTON_LONG;
    }
    return BUTTON_SHORT;
}

bool button_tick(button_t *btn) {
    uint32_t now = millis();
    if (!digitalRead(btn->pin) && btn->pressed && (now - btn->press_time > BTN_LONG_TIME)) {
        if (now - btn->tick_time > BTN_TICK_TIME) {
            btn->tick_time = now;
            return true;
        }
        return false;
    }
    if (!btn->pressed) {
        btn->tick_time = 0;
    }
    return false;
}

//Reed
void reed_init(reed_t *reed, uint8_t pin) {
    reed->pin = pin;
    reed->state = false;
    reed->last_state = false;
    pinMode(reed->pin, INPUT_PULLUP);
}

bool reed_read(reed_t *reed) {
    reed->state = !digitalRead(reed->pin);
    return reed->state;
}

//UI 
void ui_sync_encoder(ui_t *ui) {
    switch (ui->current) {
        case UI_MAIN:
            if (ui->main_mode == MAIN_MODE_TEMP) {
                encoder_config(&encoder, ui->temp_set, TEMP_MIN, TEMP_MAX, 1, 1, false);
            } else {
                encoder_config(&encoder, ui->fan_set, MIN_FAN_SPEED, MAX_FAN_SPEED, 5, 5, false);
            }
            break;

        case UI_CONFIG:
            encoder_config(&encoder, ui->config_mode, CONFIG_CALIB, CONFIG_DEFAULTS, 1, 1, false);
            break;

        case UI_TUNE:
            encoder_config(&encoder, ui->tune_power, 0, MAX_FIXED_POWER, 1, 1, false);
            break;

        case  UI_WORK:
            if (ui->work_mode == WORK_MODE_TEMP) {
                encoder_config(&encoder, ui->temp_set, TEMP_MIN, TEMP_MAX, 1, 1, false);
            } else {
                encoder_config(&encoder, ui->fan_set, MIN_FAN_SPEED, MAX_FAN_SPEED, 5, 5, false);
            }
            break;

        default:
            break;
    }
    ui->encoder_last_pos = encoder_read(&encoder);
}

void ui_init(ui_t *ui) {
    ui->current = UI_MAIN;
    ui->main_mode = MAIN_MODE_TEMP;
    ui->work_mode = WORK_MODE_FAN;
    ui->config_mode = CONFIG_CALIB;
    ui->calib_point = CALIB_TEMP_MIN;
    ui->temp_set = 300;
    ui->fan_set = MIN_FAN_SPEED;
    ui->tune_on = false;
    ui->tune_power = MAX_FIXED_POWER >> 2;
    ui->work_ready = false;
    ui->encoder_last_pos = 0;
    ui_sync_encoder(ui);
}

//Main Init
void main_init(ui_t *ui) {
    ui->main_mode = MAIN_MODE_TEMP;
    ui->used = !gun_is_cold(&gun);
    ui->cool_notified = !ui->used;
    ui->clear_used_ms = 0;
    ui->display_dirty = true;
    ui_sync_encoder(ui);
    Serial.println("MAIN: INIT");
    lcd.clear();
}

//Work Init 
void work_init(ui_t *ui) {
    ui->work_mode = WORK_MODE_FAN;
    ui->work_ready = false;
    gun_set_temp(&gun, ui->temp_set);
    gun_set_fan(&gun, ui->fan_set);
    ui_sync_encoder(ui);
    Serial.println("WORK: INIT");
    Serial.println("WORK: FAN MODE");
    lcd.clear();
}

void ui_set_screen(ui_t *ui, ui_page_t next) {
    ui->current = next;

    switch (next) {
        case UI_MAIN:
            main_init(ui);
            break;

        case UI_WORK:
            work_init(ui);
            break;

        default:
            ui_sync_encoder(ui);
            break;
    }
}

ui_page_t ui_get_screen(ui_t *ui) {
    return ui->current;
}

//Rotate
void main_rotate(ui_t *ui, int16_t delta) {
    if (ui->main_mode == MAIN_MODE_TEMP) {
        int16_t temp = ui->temp_set + delta;
        ui->temp_set = clamp(temp, TEMP_MIN, TEMP_MAX);
        ui->display_dirty = true;
        Serial.print("MAIN TEMP delta = ");
        Serial.println(ui->temp_set);
    } else {
        int16_t fan = ui->fan_set + delta;
        ui->fan_set = clamp(fan, MIN_FAN_SPEED, MAX_FAN_SPEED);
        ui->display_dirty = true;
        Serial.print("MAIN FAN delta = ");
        Serial.println(ui->fan_set);
    }
}

void work_rotate(ui_t *ui, int16_t delta) {
    if (ui->work_mode == WORK_MODE_TEMP) {
        int16_t temp = ui->temp_set + delta;
        ui->temp_set = clamp(temp, TEMP_MIN, TEMP_MAX);
        ui->display_dirty = true;
        gun_set_temp(&gun, ui->temp_set);
        Serial.print("WORK TEMP = ");
        Serial.println(ui->temp_set);
    } else {
        int16_t fan = ui->fan_set + delta;
        ui->fan_set = clamp(fan, MIN_FAN_SPEED, MAX_FAN_SPEED);
        ui->display_dirty = true;
        gun_set_fan(&gun, ui->fan_set);
        Serial.print("WORK FAN = ");
        Serial.println(ui->fan_set);
    }
}

void config_rotate(ui_t *ui, int16_t delta) {
    int16_t val = (int16_t)ui->config_mode + delta;
    ui->config_mode = (config_mode_t)clamp(val, CONFIG_CALIB, CONFIG_DEFAULTS);
    Serial.print("CONFIG selection = ");
    Serial.println(ui->config_mode);
}

void tune_rotate(ui_t *ui, int16_t delta) {
    int16_t pwr = ui->tune_power + delta;
    ui->tune_power = clamp(pwr, 0, MAX_FIXED_POWER);
    Serial.print("TUNE PWR = ");
    Serial.println(ui->tune_power);
}

void ui_rotate(ui_t *ui, int16_t delta) {
    switch (ui->current) {
        case UI_MAIN:
            main_rotate(ui, delta)  ;
            break;

        case UI_CONFIG:
            config_rotate(ui, delta);
            break;

        case UI_TUNE:
            tune_rotate(ui, delta);
            break;

        case UI_WORK:
            work_rotate(ui, delta);
            break;

        default:
            break;
    }
}

//Short Press
void main_short_press(ui_t *ui) {
    if (ui->main_mode == MAIN_MODE_TEMP) {
        ui->main_mode = MAIN_MODE_FAN;
        Serial.println("MAIN: TEMP -> FAN");
    } else {
        ui->main_mode = MAIN_MODE_TEMP;
        Serial.println("MAIN: FAN -> TEMP");
    }
    ui->display_dirty = true;
    ui_sync_encoder(ui);
}

void work_short_press(ui_t *ui) {
    if (ui->work_mode == WORK_MODE_TEMP) {
        ui->work_mode = WORK_MODE_FAN;
        Serial.println("WORK: TEMP -> FAN");
    } else {
        ui->work_mode = WORK_MODE_TEMP;
        Serial.println("WORK: FAN -> TEMP");
    }
    ui->display_dirty = true;
    ui_sync_encoder(ui);
}

void config_short_press(ui_t *ui) {
    switch (ui->config_mode) {
        case CONFIG_CALIB:
            ui_set_screen(ui, UI_CALIB);
            Serial.println("CONFIG -> CALIB");
            break;

        case CONFIG_TUNE:
            ui_set_screen(ui, UI_TUNE);
            Serial.println("CONFIG -> TUNE");
            break;

        case CONFIG_SAVE:
            Serial.println("CONFIG: Save selected");
            break;

        case CONFIG_CANCEL:
            ui_set_screen(ui, UI_MAIN);
            Serial.println("CONFIG -> MAIN");
            break;

        case CONFIG_DEFAULTS:
            Serial.println("CONFIG: DEFAULTS selected");
            break;
    }
}

void tune_short_press(ui_t *ui) {
    if (ui->tune_on) {
        ui->tune_on = false;
        Serial.println("TUNE: OFF");
    } else {
        ui->tune_on = true;
        Serial.print("TUNE: ON, power = ");
        Serial.println(ui->tune_power);
    }
}

void ui_short_press(ui_t *ui) {
    Serial.println("UI SHORTPRESS");
    switch (ui->current) {
        case UI_MAIN:
            main_short_press(ui);
            break;

        case UI_CONFIG:
            config_short_press(ui);
            break;

        case UI_TUNE:
            tune_short_press(ui);
            break;

        case UI_WORK:
            work_short_press(ui);
            break;

        default:
            break;
    }
}

//Long Press
void tune_long_press(ui_t *ui) {
    ui->tune_on = false;
    Serial.print("TUNE: OFF");
    Serial.println("TUNE -> MAIN");
    ui_set_screen(ui, UI_MAIN);
}

void ui_long_press(ui_t *ui) {
    switch (ui->current) {
        case UI_MAIN:
            ui_set_screen(ui, UI_CONFIG);
            Serial.println("MAIN -> CONFIG");
            break;

        case UI_TUNE:
            tune_long_press(ui);
            break;

        default:
            break;
    }
}

void ui_reed_event(ui_t *ui, bool on) {
    switch (ui->current) {
        case UI_MAIN:
            if (!on) {
                Serial.println("REED: MAIN -> WORK");
                gun_switch_power(&gun, true);
                ui_set_screen(ui, UI_WORK);
            }
            break;

        case UI_WORK:
            if (on) {
                Serial.println("REED: WORK -> MAIN");
                gun_switch_power(&gun, false);
                ui_set_screen(ui, UI_MAIN);
            }
            break;

        default:
            break;
    }
}

void main_show(ui_t *ui) {
    static uint32_t last_update = 0;
    static uint16_t last_temp_set;
    static uint16_t last_temp_curr;
    static uint8_t last_fan_set;
    static uint8_t last_fan_curr;
    static uint8_t last_power;
    static bool last_cold;
    static bool last_used;

    uint32_t now = millis();

    bool periodic = (now - last_update >= 500);
    
    if (!ui->display_dirty && !periodic) {
        return;
    }

    last_update = now;

    uint16_t temp_curr = adc_to_temp(history_avg(&gun.temp_history));
    uint8_t fan_curr = gun.actual_fan;
    uint8_t power = gun_avg_power_percent(&gun);
    bool cold = gun_is_cold(&gun);

    if (ui->display_dirty) {
        display_t_set(ui->temp_set);
        display_t_curr(temp_curr);
        display_fan(ui->fan_set);
        display_fan_curr(fan_curr);
        display_power(power, false);
        
        if (cold && ui->used) {
            display_msg_cold();
        } else {
            display_msg_off();
        }
        last_temp_set = ui->temp_set;
        last_temp_curr = temp_curr;
        last_fan_set = ui->fan_set;
        last_fan_curr = fan_curr;
        last_power = power;
        last_cold = cold;
        last_used = ui->used;

        ui->display_dirty = false;
    } else {
        if (ui->temp_set != last_temp_set) {
            display_t_set(ui->temp_set);
            last_temp_set = ui->temp_set;
        }
        if (temp_curr != last_temp_curr) {
            display_t_curr(temp_curr);
            last_temp_curr = temp_curr;
        }
        if (ui->fan_set != last_fan_set) {
            display_fan(ui->fan_set);
            last_fan_set = ui->fan_set;
        }
        if (fan_curr != last_fan_curr) {
            display_fan_curr(fan_curr);
            last_fan_curr = fan_curr;
        }
        if (power != last_power) {
            display_power(power, false);
            last_power = power;
        }
        if (cold != last_cold || ui->used != last_used) {
            if (cold && ui->used) {
                display_msg_cold();
            } else {
                display_msg_off();
            }
            last_cold = cold;
            last_used = ui->used;
        }
    }

    if (cold && ui->used && !ui->cool_notified) {
        buzzer_low_beep();
        ui->cool_notified = true;            
        ui->clear_used_ms = now + 120000UL;
    }
    if (ui->clear_used_ms != 0 && (int32_t)(now - ui->clear_used_ms) >=0) {
        ui->used = false;
        ui->clear_used_ms = 0;
        display_msg_off();
        last_used = false;
    }
}

void debug_gun(void) {
    static uint32_t last = 0;
    if (millis() - last < 500) {
        return;
    }

    Serial.print("UI=");
    Serial.print(ui.current);

    Serial.print(" TEMP SET=");
    Serial.print(ui.temp_set);

    Serial.print(" FAN SET=");
    Serial.print(ui.fan_set);

    Serial.print(" | GUN_MODE=");
    Serial.print(gun.mode);

    Serial.print(" GUN_TEMP_SET=");
    Serial.print(gun.temp_set);

    Serial.print(" FAN=");
    Serial.print(gun.actual_fan);

    Serial.print(" POWER=");
    Serial.print(gun.actual_power);

    Serial.print(" DOCKED=");
    Serial.print(reed.state);

    uint16_t raw = analogRead(THERMOCOUPLE_PIN);
    uint16_t filtered = emp_read(&gun.sensor);
    uint16_t temp = adc_to_temp(filtered);

    Serial.print(" | RAW=");
    Serial.print(raw);
    Serial.print(" FILTER=");
    Serial.print(filtered);
    Serial.print(" TEMP=");
    Serial.println(temp);
}

void setup() {
    Serial.begin(115200);
    pinMode(ZC_PIN, INPUT_PULLUP);
    pinMode(TRIAC_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(TRIAC_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    reed_init(&reed, REED_PIN);
    encoder_init(&encoder, ENC_A, ENC_B, 0);
    button_init(&enc_button, ENC_SW);
    ui_init(&ui);
    gun_init(&gun);
    pid_init(&pid);
    fan_init();
    display_init();
    attachInterrupt(digitalPinToInterrupt(ZC_PIN), zc_isr, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_A), encoder_irq, CHANGE);
}

void loop() {
    noInterrupts();

    bool event = zc_event_flag;
    zc_event_flag = false;  
    
    interrupts();

    int16_t pos = encoder_read(&encoder);
    
    if (pos != ui.encoder_last_pos) {
        //Serial.println(pos);
        int16_t delta = pos - ui.encoder_last_pos;
        ui.encoder_last_pos = pos;
        ui_rotate(&ui, delta);
    } 
    bool reed_state = reed_read(&reed);
    if (reed_state != reed.last_state) {
        reed.last_state = reed_state;
        ui_reed_event(&ui, reed_state);
    }
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

    button_evt_t evt = button_check(&enc_button);

    switch (evt) {
        case BUTTON_SHORT:
            //Serial.println("SHORT");
            ui_short_press(&ui);
            break;

        case BUTTON_LONG:
            //Serial.println("LONG");
            ui_long_press(&ui);
            break;

        default: 
            break;
    }
    if (button_tick(&enc_button)) {
        Serial.println("TICK");
    }
    if (ui.current == UI_MAIN) {
    main_show(&ui);
}
    //debug_gun();
}

