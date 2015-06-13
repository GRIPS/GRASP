#ifndef _MAIN_HPP_
#define _MAIN_HPP_

#include <signal.h>

#include "Telemetry.hpp"

#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))

// global variables found in main.cpp
extern sig_atomic_t volatile g_running_camera_main;
extern TelemetryPacketQueue tm_packet_queue;
extern volatile uint8_t py_image_counter;
extern volatile uint8_t roll_image_counter;
extern float temp_py, temp_roll;

int usleep_force(uint64_t microseconds);

#endif
