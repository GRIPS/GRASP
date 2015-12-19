#ifndef _MAIN_HPP_
#define _MAIN_HPP_

#include <signal.h>

#include "Telemetry.hpp"

#undef MAX
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#undef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))

// global variables found in main.cpp
extern sig_atomic_t volatile g_running_camera_main;
extern TelemetryPacketQueue tm_packet_queue;
extern volatile uint8_t py_image_counter;
extern volatile uint8_t roll_image_counter;
extern float temp_py, temp_roll;
extern pthread_mutex_t mutexAnalysis;

extern char save_locations[3][100];

extern bool MODE_AUTOMATIC, MODE_COMPRESS, MODE_DECIMATE, MODE_MOCK, MODE_TIMING, MODE_VERBOSE;

int usleep_force(uint64_t microseconds);

#endif
