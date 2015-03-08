#include <stdio.h>
#include <stdint.h>
#include <sys/io.h>

#include "oeb.h"

#define BASE_ADDR   0x380
#define BLOCK_SIZE  0x040
#define CLOCK_READ  0x380
#define CLOCK_LOW   0x384
#define CLOCK_MID   0x386
#define CLOCK_HIGH  0x388
#define IR_LOW      0x390
#define IR_MID      0x392
#define IR_HIGH     0x394

#define get_word(value) inw(value)
#define put_byte(addr, value) outb(value, addr)
#define put_word(addr, value) outw(value, addr)

uint64_t get_48(uint32_t addr_low, uint32_t addr_mid, uint32_t addr_high)
{
    uint16_t low = get_word(addr_low);
    uint32_t mid = get_word(addr_mid);
    uint64_t high = get_word(addr_high);
    return (high << 32) + (mid << 16) + low;
}

int oeb_init()
{
    if(ioperm(BASE_ADDR, BLOCK_SIZE, -1) != 0) {
        fprintf(stderr, "Run with root permissions\n");
        return -1;
    }
    return 0;
}

uint64_t oeb_clock()
{
    put_byte(CLOCK_READ, 0); //latches the value of the clock for reading
    return get_48(CLOCK_LOW, CLOCK_MID, CLOCK_HIGH);
}

uint64_t oeb_ir()
{
    put_byte(IR_LOW, 0); //latches something
    return get_48(IR_LOW, IR_MID, IR_HIGH);
}
