#ifndef _CAMERA_HPP_
#define _CAMERA_HPP_

int camera_main();
int arm_timer();
int disarm_timer();

//global variables found in camera.cpp
extern volatile bool TRANSMIT_NEXT_PY_IMAGE, TRANSMIT_NEXT_R_IMAGE;

#endif
