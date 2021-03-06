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
#define SYNC_LOW    0x3B0
#define SYNC_MID    0x3B2
#define SYNC_HIGH   0x3B4

#define IRS_LOW     0x390
#define IRS_MID     0x392
#define IRS_HIGH    0x394
#define PYC_LOW     0x396
#define PYC_MID     0x398
#define PYC_HIGH    0x39A
#define RC_LOW     0x39C
#define RC_MID     0x39E
#define RC_HIGH    0x3A0

#define get_word(value) inw(value)
#define put_byte(addr, value) outb(value, addr)
#define put_word(addr, value) outw(value, addr)

pthread_mutex_t mutex_oeb;

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

    pthread_mutex_init(&mutex_oeb, NULL);

    return 0;
}

void oeb_uninit()
{
    pthread_mutex_destroy(&mutex_oeb);
}

uint64_t oeb_get_clock()
{
    pthread_mutex_lock(&mutex_oeb);
    put_byte(CLOCK_READ, 0); //latches the value of the clock for reading
    uint64_t result = get_48(CLOCK_LOW, CLOCK_MID, CLOCK_HIGH);
    pthread_mutex_unlock(&mutex_oeb);
    return result;
}

void oeb_set_clock(uint64_t value)
{
    pthread_mutex_lock(&mutex_oeb);
    put_word(SYNC_LOW, value & 0xFFFF);
    put_word(SYNC_MID, (value >> 16) & 0xFFFF);
    put_word(SYNC_HIGH, (value >> 32) & 0xFFFF);
    pthread_mutex_unlock(&mutex_oeb);
}

uint64_t oeb_get_irs()
{
    pthread_mutex_lock(&mutex_oeb);
    put_byte(IRS_LOW, 0); //latches something
    uint64_t result = get_48(IRS_LOW, IRS_MID, IRS_HIGH);
    pthread_mutex_unlock(&mutex_oeb);
    return result;
}

uint64_t oeb_get_pyc()
{
    pthread_mutex_lock(&mutex_oeb);
    put_byte(PYC_LOW, 0); //latches something
    uint64_t result = get_48(PYC_LOW, PYC_MID, PYC_HIGH);
    pthread_mutex_unlock(&mutex_oeb);
    return result;
}

uint64_t oeb_get_rc()
{
    pthread_mutex_lock(&mutex_oeb);
    put_byte(RC_LOW, 0); //latches something
    uint64_t result = get_48(RC_LOW, RC_MID, RC_HIGH);
    pthread_mutex_unlock(&mutex_oeb);
    return result;
}
