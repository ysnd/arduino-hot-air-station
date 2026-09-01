#include <Arduino.h>

#define ENC_A 3 
#define ENC_B 5 
#define ENC_SW 4 
#define ENC_FAST_TIMEOUT 300
#define ENC_OVER_PRESS 1000
#define BTN_LONG_TIME 900
#define BTN_DEBOUNCE 50
#define BTN_TICK_TIME 200
#define BTN_OVER_PRESS 3000
#define MAX_FIXED_POWER 70
#define TEMP_MIN 150
#define TEMP_MAX 500
#define FAN_MIN 0
#define FAN_MAX 254

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
} ui_t;

encoder_t encoder;
button_t enc_button;
ui_t ui;

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

//UI 
void ui_sync_encoder(ui_t *ui) {
    switch (ui->current) {
        case UI_MAIN:
            if (ui->main_mode == MAIN_MODE_TEMP) {
                encoder_config(&encoder, ui->temp_set, TEMP_MIN, TEMP_MAX, 1, 1, false);
            } else {
                encoder_config(&encoder, ui->fan_set, FAN_MIN, FAN_MAX, 5, 5, false);
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
                encoder_config(&encoder, ui->fan_set, FAN_MIN, FAN_MAX, 5, 5, false);
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
    ui->fan_set = 50;
    ui->tune_on = false;
    ui->tune_power = MAX_FIXED_POWER >> 2;
    ui->work_ready = false;
    ui->encoder_last_pos = 0;
    ui_sync_encoder(ui);
}

//Work Init 
void work_init(ui_t *ui) {
    ui->work_mode = WORK_MODE_FAN;
    ui->work_ready = false;
    //TODO: gun 
    //aktifkan sistem hot air gun
    ui_sync_encoder(ui);
    Serial.println("WORK: INIT");
    Serial.println("WORK: FAN MODE");
    //TODO:Display
    //clear display dan redraw work screen
}

void ui_set_screen(ui_t *ui, ui_page_t next) {
    ui->current = next;
    if (next == UI_WORK) {
        work_init(ui);
    } else {
        ui_sync_encoder(ui);
    }
}

ui_page_t ui_get_screen(ui_t *ui) {
    return ui->current;
}

//Rotate
void main_rotate(ui_t *ui, int16_t delta) {
    if (ui->main_mode == MAIN_MODE_TEMP) {
        int16_t temp = ui->temp_set + delta;
        if (temp < TEMP_MIN) {
            temp = TEMP_MIN;
        } 
        if (temp > TEMP_MAX) {
            temp = TEMP_MAX;
        }
        ui->temp_set = temp;
        Serial.print("MAIN TEMP delta = ");
        Serial.println(ui->temp_set);
    } else {
        int16_t fan = ui->fan_set + delta;
        if (fan < FAN_MIN) {
            fan = FAN_MIN;
        } 
        if (fan > FAN_MAX) {
            fan = FAN_MAX;
        }
        ui->fan_set = fan;
        Serial.print("MAIN FAN delta = ");
        Serial.println(ui->fan_set);
    }
}

void work_rotate(ui_t *ui, int16_t delta) {
    if (ui->work_mode == WORK_MODE_TEMP) {
        int16_t temp = ui->temp_set + delta;
        if (temp < TEMP_MIN) {
            temp = TEMP_MIN;
        } 
        if (temp > TEMP_MAX) {
            temp = TEMP_MAX;
        }
        ui->temp_set = temp;
        //TODO:gun
        //konversi hum temp->internal temp
        Serial.print("WORK TEMP = ");
        Serial.println(ui->temp_set);
    } else {
        int16_t fan = ui->fan_set + delta;
        if (fan < FAN_MIN) {
            fan = FAN_MIN;
        } 
        if (fan > FAN_MAX) {
            fan = FAN_MAX;
        }
        ui->fan_set = fan;
        //TODO: gun
        //->gun fan control
        Serial.print("WORK FAN = ");
        Serial.println(ui->fan_set);
    }
}

void config_rotate(ui_t *ui, int16_t delta) {
    int16_t val = (int16_t)ui->config_mode + delta;
    if (val < CONFIG_CALIB) {
        val = CONFIG_CALIB;
    }
    if (val > CONFIG_DEFAULTS) {
        val = CONFIG_DEFAULTS;
    }
    ui->config_mode = (config_mode_t)val;
    Serial.print("CONFIG selection = ");
    Serial.println(val);
}

void tune_rotate(ui_t *ui, int32_t delta) {
    int16_t pwr = ui->tune_power + delta;
    if (pwr < 0) {
        pwr = 0;
    }
    if (pwr > MAX_FIXED_POWER) {
        pwr = MAX_FIXED_POWER;
    }

    ui->tune_power = pwr;
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

void setup() {
    Serial.begin(115200);
    encoder_init(&encoder, ENC_A, ENC_B, 0);
    button_init(&enc_button, ENC_SW);
    ui_init(&ui);
    Serial.print("Current screen ");
    Serial.println(ui_get_screen(&ui));
    attachInterrupt(digitalPinToInterrupt(ENC_A), encoder_irq, CHANGE);
}

void loop() {
    int16_t pos = encoder_read(&encoder);
    
    if (pos != ui.encoder_last_pos) {
        //Serial.println(pos);
        int16_t delta = pos - ui.encoder_last_pos;
        ui.encoder_last_pos = pos;
        ui_rotate(&ui, delta);
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
}
