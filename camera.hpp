#ifndef _CAMERA_HPP_
#define _CAMERA_HPP_

#define MAXNUMOFCAMERAS 2

#define FRAMESCOUNT 5

#include <stdint.h>

#include <PvApi.h>

#include "analysis.hpp"

int camera_main();
int arm_timer();
int disarm_timer();

unsigned int stopwatch(unsigned int &watch);

/* =============================================================================================
   Structure for camera data. The universal ID is unique to each camera and the handle allows
   the program to interact with the camera. The frame(s) are filled with images as they are
   taken.

            The struct handles all error and diagnostic info
   ========================================================================================== */
struct tCamera
{
    //Camera handle and parameters
    tPvHandle        Handle;
    unsigned long    UID;
    unsigned long    ExposureLength;
    unsigned long    Gain;

    timespec         ClockPC[FRAMESCOUNT];
    uint64_t         ClockOEB[FRAMESCOUNT]; //from the odds & ends board

    tPvFrame         Frames[FRAMESCOUNT];
    //valarray<unsigned char> imarr[FRAMESCOUNT]; //each valarray is sized 0, resized in setup
    //auto_ptr<CCfits::FITS> pFits[FRAMESCOUNT]; //vector of pointers
    volatile bool    NewFlags[FRAMESCOUNT];
    unsigned int     Rate; //in Hz
    unsigned int     BufferIndex; //FIXME: not thread-safe!
    unsigned int     FrameHeight;
    unsigned int     FrameWidth;
    volatile bool    PauseCapture;
    volatile bool    WantToSave;
    int idx;
    pthread_t thread[FRAMESCOUNT];        //threads for snapping/processing

    //error flags
    volatile bool waitFlag;
    int requeueFlag;
    volatile bool frameandqueueFlag;
    volatile bool triggerFlag;
    volatile bool requeueCallFlag;
    volatile bool timeoutFlag;

    //diagnostics - which ones do i really still need?
    int snapcount;
    int savecount;
    int unsuccount;    //frames that complete but are marked unsuccessful
    int zerobitcount;    //number of frames with zero bit depth
    int pcount;        //processed frames count
    int framecount;
    tPvErr queueStatus;
    int frameandqueueErrors;
    int queueErrors;
    int TimeoutCount;
    int queueError;
    int to1;
};
// _____________________________________________________________________________________________

//global variables found in camera.cpp
extern volatile bool TRANSMIT_NEXT_PY_IMAGE, TRANSMIT_NEXT_R_IMAGE;
extern tCamera CAMERAS[MAXNUMOFCAMERAS];
extern struct info PY_ANALYSIS, R_ANALYSIS;

#endif
