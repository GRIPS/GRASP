#ifndef _oeb_h_
#define _oeb_h_

#include <stdint.h>

int oeb_init(); //Returns 0 if communication with the OEB can be established, otherwise return -1
uint64_t oeb_get_clock();
uint64_t oeb_set_clock(); //The clock is not actually set to this value until the OEB receives the synchronization signal

uint64_t oeb_ir();

#endif
