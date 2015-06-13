#ifndef _MAIN_HPP_
#define _MAIN_HPP_

#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))

extern sig_atomic_t volatile g_running;

int usleep_force(uint64_t microseconds);

#endif
