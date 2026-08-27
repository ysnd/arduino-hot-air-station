#include <Arduino.h>
#include <avr/interrupt.h>

#define ENC_A 3 
#define ENC_B 5
#define ENC_SW 4
#define ENC_FAST_TIMEOUT 300
#define ENC_OVER_PRESS 1000

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

encoder_t encoder;

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

void encoder_reset(encoder_t *enc, int16_t init_pos, int16_t min_pos, int16_t max_pos, uint8_t inc, uint8_t fast_inc, bool looped) {
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

void setup() {
    Serial.begin(115200);
    pinMode(ENC_SW, INPUT_PULLUP);
    encoder_init(&encoder, ENC_A, ENC_B, 0);
    attachInterrupt(digitalPinToInterrupt(ENC_A), encoder_irq, CHANGE);
}

void loop() {
    static int16_t old_pos = 0;
    int16_t pos = encoder_read(&encoder);

    if (pos != old_pos) {
        Serial.println(pos);
        old_pos = pos;
    }
}
