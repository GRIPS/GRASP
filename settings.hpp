#ifndef _SETTINGS_HPP_
#define _SETTINGS_HPP_

#include <stdint.h>

struct settings
{
    uint8_t PY_rate; //frames per second
    uint8_t PY_gain; //dB
    uint16_t PY_exposure; //microseconds

    uint8_t R_rate; //frames per second
    uint8_t R_gain; //dB
    uint16_t R_exposure; //microseconds

    uint8_t cadence_housekeeping; //seconds
    uint8_t cadence_a2d; //seconds
    uint8_t cadence_science; //seconds
};

int load_settings(uint8_t table);
void save_settings(); // always saves to table 255

extern struct settings current_settings;
extern uint8_t current_table;

#endif
