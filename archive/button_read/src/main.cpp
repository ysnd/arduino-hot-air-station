#include <Arduino.h>

#define ENC_SW 4
#define BTN_LONG_TIME 900
#define BTN_DEBOUNCE 50
#define BTN_TICK_TIME 200
#define BTN_OVER_PRESS 3000

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

button_t enc_button;

void setup() {
    Serial.begin(115200);
    button_init(&enc_button, ENC_SW);
}

void loop() {
    button_evt_t evt = button_check(&enc_button);

    switch (evt) {
        case BUTTON_SHORT:
            Serial.println("SHORT");
            break;

        case BUTTON_LONG:
            Serial.println("LONG");
            break;

        default: 
            break;
    }
    if (button_tick(&enc_button)) {
        Serial.println("TICK");
    }
}


