#ifndef _oeb_h_
#define _oeb_h_

#include <stdint.h>

int oeb_init(); //Returns 0 if communication with the OEB can be established, otherwise return -1
uint64_t oeb_get_clock();
void oeb_set_clock(uint64_t value); //The clock is not actually set to this value until the OEB receives the synchronization signal

uint64_t oeb_get_irs();
uint64_t oeb_get_pyc();
uint64_t oeb_get_rc();

#endif
