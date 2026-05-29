#ifndef MT6701_H
#define MT6701_H

#include "driver/pcnt.h"

class EncoderPCNT {
private:
    pcnt_unit_t unit;
    gpio_num_t pinA;
    gpio_num_t pinB;

public:
    EncoderPCNT(pcnt_unit_t unit_num, gpio_num_t a, gpio_num_t b)
        : unit(unit_num), pinA(a), pinB(b) {}

    void begin() {

        // Channel 0 (A)
        pcnt_config_t pcntA = {};
        pcntA.pulse_gpio_num = pinA;
        pcntA.ctrl_gpio_num  = pinB;
        pcntA.channel        = PCNT_CHANNEL_0;
        pcntA.unit           = unit;
        pcntA.pos_mode       = PCNT_COUNT_INC;
        pcntA.neg_mode       = PCNT_COUNT_DEC;
        pcntA.lctrl_mode     = PCNT_MODE_REVERSE;
        pcntA.hctrl_mode     = PCNT_MODE_KEEP;
        pcntA.counter_h_lim  = 32767;
        pcntA.counter_l_lim  = -32768;

        pcnt_unit_config(&pcntA);

        // Channel 1 (B)
        pcnt_config_t pcntB = {};
        pcntB.pulse_gpio_num = pinB;
        pcntB.ctrl_gpio_num  = pinA;
        pcntB.channel        = PCNT_CHANNEL_1;
        pcntB.unit           = unit;
        pcntB.pos_mode       = PCNT_COUNT_INC;
        pcntB.neg_mode       = PCNT_COUNT_DEC;
        pcntB.lctrl_mode     = PCNT_MODE_KEEP;
        pcntB.hctrl_mode     = PCNT_MODE_REVERSE;
        pcntB.counter_h_lim  = 32767;
        pcntB.counter_l_lim  = -32768;

        pcnt_unit_config(&pcntB);

        pcnt_filter_disable(unit);

        pcnt_counter_pause(unit);
        pcnt_counter_clear(unit);
        pcnt_counter_resume(unit);
    }

    int32_t getCount() {
        int16_t value = 0;
        pcnt_get_counter_value(unit, &value);
        return (int32_t)value;
    }

    void clear() {
        pcnt_counter_clear(unit);
    }
};


#endif