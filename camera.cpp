#define SAVE_1_OF_EVERY_N 1

#define TIMEOUT1 125
#define TIMEOUT2 60

#define PY_CAM_ID 158434
#define R_CAM_ID 142974

/* =============================================================================================

   Includes
   ========================================================================================== */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <cstdlib>
#include <sys/time.h>
#include <sys/syscall.h>
#include <sys/stat.h>
#include <fstream>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <arpa/inet.h>
#include <pthread.h>

#include "camera.hpp"
#include "analysis.hpp"
#include "main.hpp"
#include "oeb.h"
#include "Image.hpp"
#include "settings.hpp"

//PvApi libraries
#include <PvApi.h>
#include <ImageLib.h>

#include <CCfits>            //This file includes all the headers to build the CCfits namespace classes
#include <cmath>

// _____________________________________________________________________________________________

/* =============================================================================================
   Declaration of namespaces and definitions
   ========================================================================================== */
using namespace std; //the global namespace, CCfits used locally in saveim

//#define    _STDCALL
// _____________________________________________________________________________________________


/* =============================================================================================
    Global Variables
   ========================================================================================== */
unsigned long NUMOFCAMERAS = 0;        // Actual number of cameras detected.
unsigned int TICKS_PER_SECOND; // Total number of ticks per second
volatile unsigned int CURRENT_TICK = 0;
bool PAUSEPROGRAM = false;
volatile bool TRANSMIT_NEXT_PY_IMAGE = false, TRANSMIT_NEXT_R_IMAGE = false;
tCamera CAMERAS[MAXNUMOFCAMERAS];
timer_t TIMER; //timer identifier
struct info PY_ANALYSIS, R_ANALYSIS;
// __________________________________________________________________________________________end


/* =============================================================================================
   Function declarations.
   ========================================================================================== */
void PrintError(int errCode);
void CameraEventCB(void* Context, tPvInterface Interface, tPvLinkEvent Event,
                   unsigned long UniqueId);

bool CameraGrab();
bool CameraSetup(tCamera *Camera);
bool CameraStart(tCamera *Camera);
bool getTemp(tCamera *Camera);
void CameraUnsetup(tCamera *Camera);

bool is_pyc(tCamera *Camera);

int create_timer();
int arm_timer();
int disarm_timer();

void tick_handler(int x);
void spawn_thread(tCamera *Camera);
void *snap_thread(void *cam);
void handle_wait(tCamera *Camera, tPvErr &errCode, int timeout2);
void handle_timeout(tCamera *Camera);
void RestartImCap(tCamera *Camera);
void queueErrorHandling(tCamera *Camera);

void Process(tCamera *Camera);
bool saveim(tCamera *Camera, valarray<unsigned char> &imarr, const char* filename);

void transmit_image(params &val, info &im, valarray<unsigned char> &imarr);

int readfits(const char* filename, valarray<unsigned char>& contents, unsigned int &width, unsigned int &height);

//not currently used
void CameraStop(tCamera *Camera);
void RestartAcq(tCamera *Camera);
void DisplayParameters();
void FrameStats(tCamera *Camera);
void tester(int x, timeval& t1, int i);
// __________________________________________________________________________________________end


/* =============================================================================================
   Main Program
   Based on the AVT GigE SDK examples named Stream and Snap.
   ========================================================================================== */
int camera_main()
{
    //create handler
    struct sigaction act, oldact;
    memset(&act, 0, sizeof(struct sigaction));
    act.sa_handler = tick_handler;
    sigaction(10, &act, &oldact);

    memset(&PY_ANALYSIS, 0, sizeof(struct info));
    memset(&R_ANALYSIS, 0, sizeof(struct info));

    // Initialize the API.
    if(!PvInitialize()) {

        PvLinkCallbackRegister(CameraEventCB, ePvLinkAdd, NULL);
        PvLinkCallbackRegister(CameraEventCB, ePvLinkRemove, NULL);

            // Grab cameras.
            if(CameraGrab()) {
                synchronize_settings();
                // Set up cameras.
                bool setupSuccess = true;
                for(unsigned int i = 0; i < NUMOFCAMERAS; i++) {
                    if(!CameraSetup(&CAMERAS[i]))
                        setupSuccess = false;
                }
                if(setupSuccess) {
                    // Start streaming from cameras.
                    bool startSuccess = true;
                    for(unsigned int j = 0; j < NUMOFCAMERAS; j++) {
                        if(!CameraStart(&CAMERAS[j]))
                            startSuccess = false;
                    }
                    if(!startSuccess) {
                        printf("Failed to start streaming from cameras.\n");
                    } else {
                        create_timer();
                        arm_timer();
                        while(g_running_camera_main) {
                            //Wait for interrputs which generate triggers to snap and process images
                            usleep_force(5000);
                        }
                        disarm_timer();
                    }

                    // Unsetup the cameras.
                    for(unsigned int k = 0; k < NUMOFCAMERAS; k++) {
                        CameraUnsetup(&CAMERAS[k]);
                    }
                } else
                    printf("Failed to set up cameras.\n");
            } else
                printf("Failed to grab cameras.\n");

        PvLinkCallbackUnRegister(CameraEventCB, ePvLinkAdd);
        PvLinkCallbackUnRegister(CameraEventCB, ePvLinkRemove);

        // Uninitialize the API.
        PvUnInitialize();

        printf("\nEnd of camera main reached.\n\n");

    } else
        printf("\nFailed to initialize the API. Program terminating.\n\n");

    sigaction(10, &oldact, NULL);

    return 0;
}
// __________________________________________________________________________________________end


/* =============================================================================================
   Very simple function to check whether a camera is the pitch-yaw camera
   Could be made fancier by checking for the roll camera or unknown cameras
   ========================================================================================== */
bool is_pyc(tCamera *Camera)
{
    return Camera->UID == PY_CAM_ID;
}
// __________________________________________________________________________________________end


/* =============================================================================================
   Stopwatch function
   You need to pass in the clock (rather than a static variable) so that it can be thread-safe
   Returns the time elapsed in microseconds since the last time it was called
   ========================================================================================== */
unsigned int stopwatch(unsigned int &watch)
{
    unsigned int old_watch = watch;
    timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    watch = now.tv_sec * 1000000 + now.tv_nsec / 1000;
    return watch - old_watch;
}
// __________________________________________________________________________________________end


/* =============================================================================================
   get the camera temperature
            ***move into control.h
   ========================================================================================== */
bool getTemp(tCamera *Camera)
{
    tPvFloat32 T_MB = 0;
    //tPvFloat32 T_CCD = 0;

    const char* whichfile;
    ofstream tempfile;
    if(is_pyc(Camera)) {
        whichfile = "PY_temps.txt";
    } else {
        whichfile = "H_temps.txt";
    }
    tempfile.open(whichfile, ios::app);

    if(PvAttrFloat32Get(Camera->Handle, "DeviceTemperatureMainboard", &T_MB) == 0) {
        //PvAttrFloat32Get(Camera->Handle, "DeviceTemperatureSensor", &T_CCD);
        //cout<<"T_MB = "<<T_MB<<endl;
        //cout<<"T_CCD = "<<T_CCD<<endl;
        tempfile << T_MB <<endl;

        if(is_pyc(Camera)) {
            temp_py = T_MB;
        } else {
            temp_roll = T_MB;
        }

        return true;
    } else {
        cout<<"Temp error\n";
        tempfile << "Temp error"<< endl;
        //PrintError(PvAttrFloat32Get(Camera->Handle, "DeviceTemperatureMainboard", &T_MB));
        //PrintError(PvAttrFloat32Get(Camera->Handle, "DeviceTemperatureSensor", &T_CCD));
        //cout<<"T_MB = "<<T_MB<<endl;
        //cout<<"T_CCD = "<<T_CCD<<endl;
        return false;
    }
}
// _________________________________________________________________________________________end


/* =============================================================================================
   Callback called when a camera is plugged or unplugged.
   From AVT GigE SDK example named Stream.
        ** I don't think this is working!!!!!!!!!! for cameras plugged in during running
   ========================================================================================== */
void CameraEventCB(void* Context, tPvInterface Interface, tPvLinkEvent Event,
                   unsigned long UniqueId)
{
    switch(Event) {
        case ePvLinkAdd: {
            printf("\nCamera %lu is now plugged in.\n", UniqueId);
            break;
        }
        case ePvLinkRemove: {
            printf("\nCamera %lu has been unplugged.\n", UniqueId);
            break;
        }
        default:
            break;
    }

    //If a camera has been plugged in or unplugged since cameras have been grabbed,
    //the main function should be stopped
    g_running_camera_main = 0;
}
// __________________________________________________________________________________________end


/* =============================================================================================
   Grab cameras and establish default parameters.
   ========================================================================================== */
bool CameraGrab()
{
    int i = 0;
    const char* IP1 = "169.254.100.2";
    const char* IP2 = "169.254.200.2";

    //try 1st camera
    memset(&CAMERAS[i], 0, sizeof(tCamera));
    if(!PvCameraOpenByAddr(inet_addr(IP1), ePvAccessMaster, &(CAMERAS[i].Handle))) {
        CAMERAS[i].UID = PY_CAM_ID;
        i++;
    } else {
        cout<<"couldn't open PY camera\n";
    }
    //try 2nd camera
    memset(&CAMERAS[i], 0, sizeof(tCamera));
    if(!PvCameraOpenByAddr(inet_addr(IP2), ePvAccessMaster, &(CAMERAS[i].Handle))) {
        CAMERAS[i].UID = R_CAM_ID;
        i++;
    } else {
        cout<<"couldn't open R camera\n";
    }

    NUMOFCAMERAS = i;

    g_running_camera_main = 1;

    return i > 0;
}
// __________________________________________________________________________________________end


/* =============================================================================================
   Synchronize camera settings from a different global...
   ========================================================================================== */
void synchronize_settings()
{
    for(unsigned int i = 0; i < NUMOFCAMERAS; i++) {
        if(is_pyc(&CAMERAS[i])) {
            CAMERAS[i].Rate = current_settings.PY_rate;
            CAMERAS[i].ExposureLength = current_settings.PY_exposure;
            CAMERAS[i].Gain = current_settings.PY_gain;
        } else {
            CAMERAS[i].Rate = current_settings.R_rate;
            CAMERAS[i].ExposureLength = current_settings.R_exposure;
            CAMERAS[i].Gain = current_settings.R_gain;
        }
    }
}
// __________________________________________________________________________________________end


/* =============================================================================================
   Opens a camera and fills in parameters.
   ========================================================================================== */
bool CameraSetup(tCamera *Camera)
{
    Camera->WantToSave = true;

    printf("%s camera settings: %u Hz, %lu us, %lu dB gain, %s\n",
           (is_pyc(Camera) ? "Pitch-yaw" : "Roll"),
           Camera->Rate, Camera->ExposureLength, Camera->Gain,
           (Camera->WantToSave ? "saving" : "NOT saving"));

    //define the pixel format. see camera and driver attributes p.15
    PvAttrEnumSet(Camera->Handle, "PixelFormat", "Mono8");

    //set the output
    //PvAttrEnumSet(Camera->Handle, "SyncOut1Mode", "off");
    //PvAttrEnumSet(Camera->Handle, "SyncOut3Mode", "FrameTrigger");
    PvAttrEnumSet(Camera->Handle, "SyncOut3Mode", "Exposing");
    //PvAttrEnumSet(Camera->Handle, "SyncOut3Mode", "Acquiring");

    //playing with camera frame options
    //PvAttrUint32Set(Camera->Handle, "Height", 144129/449 );
    //PvAttrUint32Set(Camera->Handle, "Width", 449 );
    //define ROI
    /*if(PvAttrUint32Set(Camera->Handle, "RegionX", 0)||
        PvAttrUint32Set(Camera->Handle, "RegionY", 0)||
        PvAttrUint32Set(Camera->Handle, "Height", 400)||
        PvAttrUint32Set(Camera->Handle, "Width", 400)){
        cout<<"The ROI isn't defined correctly\n"<<"\n";
    }*/

    return true;
}
// __________________________________________________________________________________________end


/* =============================================================================================
   Create the interrupt timer for snap
   ========================================================================================== */
int create_timer()
{
    //set the sigevent notification structure
    struct sigevent timersigevent; //create signal event
    memset(&timersigevent, 0, sizeof timersigevent); //initialize the struct with zeros
    timersigevent.sigev_notify = SIGEV_THREAD_ID | SIGEV_SIGNAL; //send a signal upon expiration of timer & only to a certain thread
    timersigevent.sigev_signo = 10; //set to SIGUSR1 number 10
    timersigevent._sigev_un._tid = syscall(SYS_gettid); //notify the thread that creates the timer. This will be the main thread.

    //create the timer
    return timer_create(CLOCK_REALTIME, &timersigevent, &TIMER) == 0;
}
// __________________________________________________________________________________________end


/* =============================================================================================
   Arm the interrupt timer for snap after determining the rate of ticks
   ========================================================================================== */
int arm_timer()
{
    TICKS_PER_SECOND = 1;
    for(unsigned int i = 0; i < NUMOFCAMERAS; i++) {
        TICKS_PER_SECOND *= CAMERAS[i].Rate;
    }
    cout << "This is the ticks per second: " << TICKS_PER_SECOND << "\n";

    struct itimerspec cadence;
    memset(&cadence, 0, sizeof(struct itimerspec));
    if (TICKS_PER_SECOND > 1) {
        //time until first tick
        cadence.it_value.tv_nsec = 1000000000 / TICKS_PER_SECOND;
        //time between ticks
        cadence.it_interval.tv_nsec = 1000000000 / TICKS_PER_SECOND;
    } else {
        cadence.it_value.tv_sec = 1;
        cadence.it_interval.tv_sec = 1;
    }

    return timer_settime(TIMER, 0, &cadence, NULL);
}
// __________________________________________________________________________________________end


/* =============================================================================================
   Disarm the interrupt timer for snap
   ========================================================================================== */
int disarm_timer()
{
    struct itimerspec zero;
    memset(&zero, 0, sizeof(struct itimerspec));
    return timer_settime(TIMER, 0, &zero, NULL);
}
// __________________________________________________________________________________________end


/* =============================================================================================
   Finishes setting up a camera and starts streaming.
   ========================================================================================== */
bool CameraStart(tCamera *Camera)
{
    unsigned long FrameSize = 0;

    printf("Starting camera with ID %lu\n", Camera->UID);

    // Auto adjust the packet size to the maximum supported by the network; usually 8228,
    // but 6000 for the current hardware.
    if(PvCaptureAdjustPacketSize(Camera->Handle, 6000)== ePvErrSuccess )
        cout << "Packet Size sucessfully determined.\n";
    else
        cout << "Possible Packet Size issue.\n";

    // Determine how big the frame buffers should be
    if(!PvAttrUint32Get(Camera->Handle, "TotalBytesPerFrame", &FrameSize)) {

        // Calculate the frame dimensions from the frame size assuming 4:3 aspect ratio
        Camera->FrameHeight = (unsigned int)fabs(sqrt(.75 * FrameSize));
        Camera->FrameWidth = FrameSize / Camera->FrameHeight;
        printf("\nFrame buffer size: %lu; %d by %d\n", FrameSize, Camera->FrameWidth,
               Camera->FrameHeight);

        bool failed = false;

        // Allocate the buffer for multiple frames.
        for(int i = 0; i < FRAMESCOUNT; i++) {
            Camera->Frames[i].ImageBuffer = new unsigned char[FrameSize];
            if(Camera->Frames[i].ImageBuffer)
                Camera->Frames[i].ImageBufferSize = FrameSize;
            else
                failed = true;
        }

        if(!failed) {
            // Prepare the camera for capturing pictures.
            if(!PvCaptureStart(Camera->Handle)) {
                // Set the trigger mode to software for controlled acquisition.
                if(!PvAttrEnumSet(Camera->Handle, "AcquisitionMode", "Continuous")
                   && !PvAttrEnumSet(Camera->Handle, "FrameStartTriggerMode", "Software")) {
                    //set packet size
                    if(PvAttrUint32Set(Camera->Handle, "PacketSize", 8228)) { //
                        // Begin acquisition.
                        if(PvCommandRun(Camera->Handle, "AcquisitionStart")) {
                            // If that fails, reset the camera to non-capture mode.
                            PvCaptureEnd(Camera->Handle);
                        } else {
                            printf("Camera with ID %lu is now acquiring images.\n", Camera->UID);
                            Camera->PauseCapture = false;
                            return true;
                        }
                    }
                }
            }
        }
    }

    return false;
}
// __________________________________________________________________________________________end


/* =============================================================================================
   Stop streaming from a camera.
   ========================================================================================== */
void CameraStop(tCamera *Camera)
{
    printf("\nStopping the stream for camera with ID %lu.\n", Camera->UID);
    PvCommandRun(Camera->Handle, "AcquisitionAbort");
    PvCaptureEnd(Camera->Handle);
}
// _________________________________________________________________________________________end


/*=============================================================================================
  Handle ticks by calling the thread spawner at the appropriate multiples
  ========================================================================================== */
void tick_handler(int x)
{
    switch(NUMOFCAMERAS) {
        case 1:
            spawn_thread(&CAMERAS[0]);
        case 2:
        default:
            if((CURRENT_TICK % CAMERAS[1].Rate) == 0) spawn_thread(&CAMERAS[0]);
            if((CURRENT_TICK % CAMERAS[0].Rate) == 0) spawn_thread(&CAMERAS[1]);
    }

    //update CURRENT_TICK
    CURRENT_TICK = ((CURRENT_TICK + 1) % TICKS_PER_SECOND);
}
// _________________________________________________________________________________________end


/*=============================================================================================
  spawn threads for snapping/processing
  ========================================================================================== */
void spawn_thread(tCamera *Camera)
{
    if(!PAUSEPROGRAM && g_running_camera_main) {
        int thread_err;
        pthread_attr_t attr;                                            //attribute object
        pthread_attr_init(&attr);
        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED); //create thread in detached state
        thread_err = pthread_create(&Camera->thread[Camera->idx], &attr, snap_thread, (void *)Camera);
        pthread_attr_destroy(&attr);
        if(thread_err) {
            cout<<"didn't create thread for camera "<<Camera->UID<<" frame "<<Camera->idx<<endl;
            cout<<"Error: "<<thread_err<<endl;
            return;
        }

        //cout<<"Camera "<<Camera->UID<<" frame "<<Camera->idx<<endl;
        if(MODE_VERBOSE) cout << (is_pyc(Camera) ? "..\n" : "++\n");
        //cout<<" \n"; //without this the transmission time is 50ms, wiht it its .05ms.. something is wrong

        //update thread and buffer indexes
        Camera->BufferIndex = Camera->idx; //the active buffer = this thread buffer
        Camera->idx = ((Camera->idx + 1) % FRAMESCOUNT);
    }
}
// _________________________________________________________________________________________end


/*=============================================================================================
  spawned thread for snapping/processing
  ========================================================================================== */
void *snap_thread(void *cam)
{
    tCamera *Camera = (tCamera *)cam;

    //initalize variables & housekeeping
    tPvErr errCode;

    //start snap only if we're done waiting for another frame to return (but doesn't protect frame buffer)
    if(Camera->Handle != NULL && !Camera->PauseCapture && !Camera->waitFlag) {
        //set the exposure length and gain
        //TODO: check for failure?
        PvAttrUint32Set(Camera->Handle, "ExposureValue", Camera->ExposureLength);
        PvAttrUint32Set(Camera->Handle, "GainValue", Camera->Gain);

        //requeue a frame & snap (if successful requeue) - then process (if no timeout & done waiting)
        Camera->queueStatus = PvCaptureQueueFrame(Camera->Handle,&(Camera->Frames[Camera->BufferIndex]),NULL);
        if(Camera->queueStatus == ePvErrSuccess) {
            //update flags
            Camera->requeueCallFlag = false;
            Camera->NewFlags[Camera->BufferIndex] = true;
            Camera->frameandqueueFlag = false;

            //trigger, wait and queue processing if successful
            clock_gettime(CLOCK_REALTIME, &Camera->ClockPC[Camera->BufferIndex]);
            Camera->snapcount++;
            Camera->waitFlag= true;

            Camera->ClockOEB1[Camera->BufferIndex] = oeb_get_clock();

            if(PvCommandRun(Camera->Handle, "FrameStartTriggerSoftware") != ePvErrSuccess) {
                cout<<"Trigger Software Error: ";
                PrintError(errCode);
            }

            errCode = PvCaptureWaitForFrameDone(Camera->Handle, &(Camera->Frames[Camera->BufferIndex]), TIMEOUT1);

            Camera->ClockOEB2[Camera->BufferIndex] = oeb_get_clock();

            handle_wait(Camera, errCode, TIMEOUT2); //checks and updates waitflag and errCode

            //Process: if done waiting, no timeout, successful frame & non-zero bitdepth
            //cout<<"image size: "<<Camera->Frames[Camera->BufferIndex].ImageSize<<endl;
            if(Camera->waitFlag== false && errCode == ePvErrSuccess) {
                if( Camera->Frames[Camera->BufferIndex].Status == ePvErrSuccess && Camera->Frames[Camera->BufferIndex].BitDepth != 0) {
                    Process(Camera);
                } else {
                    cout<<"CurrBuffer: "<<Camera->BufferIndex<<endl;
                    if(Camera->Frames[Camera->BufferIndex].Status != ePvErrSuccess) {
                        cout<<"unsuccessful frame\n\n";
                        ++Camera->unsuccount;
                    }
                    if(Camera->Frames[Camera->BufferIndex].BitDepth ==0) {
                        cout<<"BitDepth: "<<Camera->Frames[Camera->BufferIndex].BitDepth<<"\n\n";
                        ++Camera->zerobitcount;
                    }
                }
            } else {
                cout<<"wait flag or timeout error\n\n";
            }
        } else {
            cout<<"PvCaptureQueueFrame err\n";
            queueErrorHandling(Camera);
        }//requeueframe

        //housekeeping
        getTemp(Camera);
    } else { //handle&&pausecap&&flag
        cout<<Camera->UID<<" handle, pausecap or flag error\n";
    }

    //probably don't need this b/c its explicity created detached... test later
    //pthread_detach(pthread_self());
    pthread_exit(NULL);
}
// _________________________________________________________________________________________end


/*=============================================================================================
  Handle timeout and waitforframedone
  ========================================================================================== */
void handle_wait(tCamera *Camera, tPvErr &errCode, int timeout2)
{
    if (errCode == ePvErrSuccess) { //if returns in time, release flag
        Camera->waitFlag= false;
        //++Camera->compcount;
    } else if (errCode == ePvErrTimeout) { //if timeout try again with less time
        //cout<<"Timeout 1.\n";
        Camera->to1++;
        //try again
        errCode = PvCaptureWaitForFrameDone(Camera->Handle,&(Camera->Frames[Camera->BufferIndex]), timeout2);
        if (errCode == ePvErrSuccess) {  //if returns in time, release flag
            Camera->waitFlag= false;
                    //++Camera->compcount;
        } else if (errCode == ePvErrTimeout) { //if still timeout, assume trigerror and restart acquisition stream, don't process
            handle_timeout(Camera);
            Camera->waitFlag= false;
        }
    } else {
        cout<<"Unknown return code for WaitForFrameDone. Releasing camera.\n";
        Camera->waitFlag= false;
    }
}
// _________________________________________________________________________________________end


/*=============================================================================================
        error handling for timeout - 1st time just abort frame and restart acq, next restart imcap
  ========================================================================================== */
void handle_timeout(tCamera *Camera)
{
    //abort frame and clear queue each timeout
    //for every nth timeout restart imagecapture stream - not sure if this actually makes a difference
    Camera->TimeoutCount++;

    if (Camera->TimeoutCount % 10 == 0 ) { //make this a parameter on table!!!
        cout<<"10th timeout. Restart Image capture stream\n";
        RestartImCap(Camera);
    } else {
        if(PvCaptureQueueClear(Camera->Handle)==ePvErrSuccess)
            printf("Image aborted and frame buffer queue cleared. \n");
    }
}
// _________________________________________________________________________________________end


/*=============================================================================================
  Process and optionally save the current image buffer from a camera
  ========================================================================================== */
void Process(tCamera *Camera)
{
    bool py = is_pyc(Camera);
    unsigned int watch;

    ++Camera->pcount;
    unsigned int localIndex = Camera->BufferIndex;

    if(py) {
        Camera->ClockTrigger[localIndex] = oeb_get_pyc();
        py_image_counter++;
    } else {
        Camera->ClockTrigger[localIndex] = oeb_get_rc();
        roll_image_counter++;
    }

    char directory[128];
    char filename[128];
    struct stat st = {0};

    if(Camera->WantToSave) {
        char timestamp[14];
        struct tm *capturetime;
        capturetime = gmtime(&Camera->ClockPC[localIndex].tv_sec);
        strftime(timestamp, 14, "%y%m%d_%H%M%S", capturetime);

        strftime(directory, 128, "images/%Y%m%d_%H", capturetime);
        // Make the appropriate directory if it doesn't exist
        if (stat(directory, &st) == -1) {
            mkdir(directory, 0755);
        }

        sprintf(filename, "%s/%s_%s_%06ld_%012lx.fits",
                directory,
                (py ? "py" : "r"), timestamp,
                Camera->ClockPC[localIndex].tv_nsec/1000l,
                Camera->ClockTrigger[localIndex]);
    }

    if(MODE_TIMING) stopwatch(watch);
    // create copy of image buffer
    valarray<unsigned char> imarr((unsigned char*)Camera->Frames[localIndex].ImageBuffer,
                                  Camera->FrameHeight * Camera->FrameWidth);

    //1. init stucts and variables
    info im;
    memset(&im, 0, sizeof(info));
    params val;
    init_params(val, Camera->FrameWidth, Camera->FrameHeight);
    val.UID = Camera->UID;
    val.clock = Camera->ClockTrigger[localIndex];

    //timeval t;
    //2. analyze live or test image?
    if(MODE_MOCK) {
        valarray<unsigned char> imarr2;

        // read file and introduce a shift to the image
        uint8_t row_shift = 0, col_shift = 0; // for shorter code, these shifts must be nonnegative
        if(py) {
            const char* filename1 = "mock_py.fits";        //sun is 330 pix
            readfits(filename1, imarr2, val.width, val.height);
            row_shift = 25. * (1 - cos(5. * Camera->pcount * M_PI / 180.)); // 5 deg CW per frame
            col_shift = 25. * (1 + sin(5. * Camera->pcount * M_PI / 180.)); // 5 deg CW per frame
        } else {
            const char* filename1 = "mock_r.fits";
            readfits(filename1, imarr2, val.width, val.height);
            row_shift = 25. * (1 - cos(5. * Camera->pcount * M_PI / 180.)); // 5 deg CW per frame
            col_shift = 25. * (1 + sin(5. * Camera->pcount * M_PI / 180.)); // 5 deg CW per frame
        }
        if((val.width == Camera->FrameWidth) && (val.height == Camera->FrameHeight)) {
            uint32_t start = row_shift * val.width + col_shift;
            memcpy(&imarr[start], &imarr2[0], val.width * val.height - start);
            memcpy(&imarr[0], &imarr2[val.width * val.height - start], start);
        } else {
            std::cerr << "Specified mock image does not match sensor dimentions\n";
        }
    }
    if(MODE_TIMING) cout << (py ? "Pitch-yaw" : "Roll")
                         << " image ready in " << stopwatch(watch) << " us\n";

    // Only proceed with analysis on one frame each second (per camera)
    if((Camera->pcount % Camera->Rate) == 0) {

    if(MODE_TIMING) stopwatch(watch);
    if(py) {
        analyzePY(im, val, imarr);
    } else {
        analyzeR(im, val, imarr);
    }
    if(MODE_TIMING) cout << "  Analysis took " << stopwatch(watch) << " us\n";

    if(MODE_AUTOMATIC) {
        transmit_image(val, im, imarr);
    }
    if(py) {
        if(TRANSMIT_NEXT_PY_IMAGE) {
            TRANSMIT_NEXT_PY_IMAGE = false;
            transmit_image(val, im, imarr);
        }
    } else {
        if(TRANSMIT_NEXT_R_IMAGE) {
            TRANSMIT_NEXT_R_IMAGE = false;
            transmit_image(val, im, imarr);
        }
    }
    if(MODE_VERBOSE) {
        if(py) {
            reportPY(val, im);
        } else {
            reportR(val, im);
        }
    }

    pthread_mutex_lock(&mutexAnalysis);
    if(py) {
        memcpy(&PY_ANALYSIS, &im, sizeof(info));
    } else {
        memcpy(&R_ANALYSIS, &im, sizeof(info));
    }
    pthread_mutex_unlock(&mutexAnalysis);

    //drawline at centers? This will be saved in the image below
    if(py && val.drawline) {
        drawline(imarr, val, im);
    }

    } // end of analysis

    //3. save?
    if(Camera->WantToSave) {
        if((Camera->savecount % SAVE_1_OF_EVERY_N) == 0) {
            if(MODE_TIMING) stopwatch(watch);
            if(MODE_VERBOSE) cout << "Saving to " << filename << endl;
            saveim(Camera, imarr, filename);
            if(MODE_TIMING) cout << "  Saving took " << stopwatch(watch) << " us\n";
        }
        Camera->savecount++;
     }
}
// __________________________________________________________________________________________end


/*=============================================================================================
  Transmit a whole image
  ========================================================================================== */
void transmit_image(params &val, info &im, valarray<unsigned char> &imarr)
{
    int id = (val.UID == PY_CAM_ID ? 0 : 1);
    cout << "Transmitting image from " << (id == 0 ? "pitch-yaw" : "roll") << " camera\n";

    ImagePacketQueue ipq;

    ipq.add_partial_array(id, val.height, val.width, 0, val.height, 0, val.width,
                          (uint8_t *)&(imarr[0]), val.clock, true);

    ImagePacket ip(NULL);
    cout << ipq.size() << " image packets to add to queue\n";
    while(!ipq.empty()) {
        ipq >> ip;
        tm_packet_queue << ip;
    }
}
// __________________________________________________________________________________________end


/* =============================================================================================
   Saves a FITS file
   ========================================================================================== */
bool saveim(tCamera *Camera, valarray<unsigned char> &imarr, const char* filename)
{
    using namespace CCfits;
    //using std::valarray;

    FITS::setVerboseMode(true);

    //create pointer to fits object
    std::auto_ptr<FITS> pFits(0);
    //std::valarray<unsigned char> imarr((char*)Camera->Frames[j].ImageBuffer, nelements);

    unsigned int localIndex = Camera->BufferIndex;

    //for running loop tests
    remove(filename);

    try {
        //create new fits file
        try {
            pFits.reset( new FITS(filename , BYTE_IMG , 0 , 0 ) ); //reset=deallocate object and reset its value. change to BYTE_IMG?
        } catch (FITS::CantCreate) {
            std::cout<<"Couldn't create FITS file. The file might already exist. \n";
            return false;
        }

        if(MODE_COMPRESS) pFits->setCompressionType(RICE_1);

        //append keys to the primary header
        //long exposure(1500);
        pFits->pHDU().addKey("Camera",(long)Camera->UID, "Camera UID");
        pFits->pHDU().addKey("Save Count",(int)Camera->savecount, "Num of Saved Images");
        pFits->pHDU().addKey("Snap Count",(int)Camera->snapcount, "Num of SNAPS");
        pFits->pHDU().addKey("Exposure",(long)Camera->ExposureLength, "for cameanalyzera");
        pFits->pHDU().addKey("Gain",(int)Camera->Gain, "Gain (dB)");
        pFits->pHDU().addKey("Rate",(int)Camera->Rate, "Number of frames per second");
        pFits->pHDU().addKey("filename", filename, "Name of the file");

        pFits->pHDU().addKey("PC_SEC", (long)Camera->ClockPC[localIndex].tv_sec, "PC time (seconds)");
        pFits->pHDU().addKey("PC_NSEC", (long)Camera->ClockPC[localIndex].tv_nsec, "PC time (nanoseconds)");
        pFits->pHDU().addKey("GT_TRIG", (long)Camera->ClockTrigger[localIndex], "Gondola time of camera trigger pulse");
        pFits->pHDU().addKey("GT_OEB1", (long)Camera->ClockOEB1[localIndex], "Gondola time before snap");
        pFits->pHDU().addKey("GT_OEB2", (long)Camera->ClockOEB2[localIndex], "Gondola time after snap");

        //switch to make the appropriate ascii tables for PY or H from their info structs
        //pFits->pHDU().addKey("xp", im.xp, "x coordinates");
        //pFits->pHDU().addKey("yp", im.yp, "y coordinates");
        //pFits->pHDU().addKey("thresh", im.thresh, "Thresholds");
        //pFits->pHDU().addKey("Time Stamp",Camera->TimeStamps[j], "prog timestamp"); //gives error
    } catch (FitsException&) {
     // will catch all exceptions thrown by CCfits, including errors
     // found by cfitsio (status != 0).
        std::cerr << " Fits Exception Thrown.  \n";
    }

    //store data in an extension HDU
    //its customary to have compressed images in 1st Ext
    ExtHDU* imageExt;
    std::vector<long> extAx;
    extAx.push_back(Camera->FrameWidth);
    extAx.push_back(Camera->FrameHeight);

    string newName ("Raw Frame");
    long fpixel(1);

    try {
        imageExt = pFits->addImage(newName, BYTE_IMG, extAx, 1);
        imageExt->write(fpixel, Camera->FrameWidth*Camera->FrameHeight, imarr);    //write extension
        //std::cout << pFits->pHDU() << std::endl;
    } catch (FitsException) {
        std::cout<<"Couldn't write image to extension\n";
    }

    return true;
}
// __________________________________________________________________________________________end


/*==============================================================================================
    Test fuction for diagnostics
    ========================================================================================= */
void tester(int x, timeval& t1, int i)
{
    //get time
    timeval highrestime;
    char filename[17];
    time_t rawtime;
    struct tm * timeinfo;
    gettimeofday(&highrestime, NULL);
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime (filename, 17, "%Y%m%d_%H%M%S_", timeinfo);
    float sec;
    float us;

    //tests
    switch (x) {
        case 0 : {
            cout<<"SNAP "<< CAMERAS[i].snapcount <<" : "<<filename << highrestime.tv_usec<<"\n";
            break;
        }
        case 1 : {
            //cout<<"START Saving : "<<filename << highrestime.tv_usec<<"\n";
            t1 = highrestime;
            break;
        }
        case 2 : {
            //cout<<"End Saving : "<<filename << highrestime.tv_usec<<"\n";
            sec = highrestime.tv_sec - t1.tv_sec;
            if (sec == 0) {
                            us = highrestime.tv_usec - t1.tv_usec;
            } else {
                            us = (1000000 - t1.tv_usec) + highrestime.tv_usec;
                            us = us + ((sec - 1) * 1000);
            }
         cout<<"dt = "<< (us/1000)<<" ms"<<endl;
            break;
        }
        case 3 : {
            cout<<"Save Status : ";
            /*if( y == 1 ){
                cout<< "success.\n";
            }else{
                cout<< "failure.\n";
            } */
            cout<<"-------------------------------------------------------- \n";
            break;
        }
        case 4 : {
            cout<<"** Frame "<< CAMERAS[i].framecount<< " Done : "<<filename << highrestime.tv_usec<<"\n";
            CAMERAS[i].framecount ++;
            break;
        }
        default: {
            break;
        }
    }
}
//_____________________________________________________________________________________________


/*=============================================================================================
    This returns statistics on each camera's frames
  ============================================================================================*/
void FrameStats(tCamera *Camera)
{
    unsigned long Ncomp=0;        //Num of frames acquired
    unsigned long Ndrop=0;        //Num of frames unsuccessfully acquired
    unsigned long Nerr=0;        //Num of erraneous packets
    unsigned long Nmiss=0;        //Num packets sent by camera not received by host
    unsigned long Nrec=0;        //Num packets sent by camera and received by host
    unsigned long Nreq=0;        //Num of missing packets requested by camera for resend
    unsigned long Nres=0;        //Num of missing packets resent by camera and receieved by host

    PvAttrUint32Get(Camera->Handle, "StatFramesCompleted", &Ncomp);
    PvAttrUint32Get(Camera->Handle, "StatFramesDropped", &Ndrop);
    PvAttrUint32Get(Camera->Handle, "StatPacketsErroneous", &Nerr);
    PvAttrUint32Get(Camera->Handle, "StatPacketsMissed", &Nmiss);
    PvAttrUint32Get(Camera->Handle, "StatPacketsReceived", &Nrec);
    PvAttrUint32Get(Camera->Handle, "StatPacketsRequested", &Nreq);
    PvAttrUint32Get(Camera->Handle, "StatPacketsResent", &Nres);

    cout<<"\nStatistics for camera: "<<Camera->UID<<"\n";
    cout<<"SNAP count: "<<Camera->snapcount<<"\n";
    cout<<"Save count: "<<Camera->savecount<<"\n";
    cout<<"Frames Completed: "<< Ncomp <<"\n";
    cout<<"Frames Dropped: "<<Ndrop<<"\n";
    cout<<"Num of erroneous packets received: "<<Nerr<<"\n";
    cout<<"Num of packets sent by camera and NOT received by host : "<<Nmiss<<"\n";
    cout<<"Num of packets sent by camera and received by host: "<<Nrec<<"\n";
    cout<<"Num of missing packets requested to camera for resend: "<<Nreq<<"\n";
    cout<<"Num of missing packets resent by camera and received by host: "<<Nres<<"\n\n";
}
//______________________________________________________________________________________________


/* =============================================================================================
   Read a fits file
   ========================================================================================== */
int readfits(const char* filename, valarray<unsigned char>& contents, unsigned int &width, unsigned int &height)
{
    using namespace CCfits;

    //FITS::setVerboseMode(true);

    std::auto_ptr<FITS> pInfile(new FITS( filename ,Read,true));
    //PHDU& image = pInfile->pHDU();
    ExtHDU& image = pInfile->extension("Raw Frame");

    // read all user-specifed, coordinate, and checksum keys in the image
    //image.readAllKeys();
    image.read(contents);

    //axes
    width = image.axis(0);
    height = image.axis(1);

    //cout<<"width: "<<width<<endl;
    //cout<<"height: "<<height<<endl;
/*    cout<<"Max pixel value: "<<(int)contents.max()<<endl;
    cout<<"Min pixel value: "<<(int)contents.min()<<endl;*/

 // this doesn't print the data, just header info.
    //std::cout << image << std::endl;
    //this prints out elements from the image array
    //for(int i = 0; i < nelements+1; i+=nelements/height) std::cout<< "Fits "<<i<<" : "<< contents[i]<<"        ";

    return 0;
}
// __________________________________________________________________________________________end


/*==============================================================================================
    Error Handling for queue frame
==============================================================================================*/
void queueErrorHandling(tCamera *Camera)
{
    PrintError(Camera->queueStatus);
    if(PvCaptureQueueClear(Camera->Handle)==ePvErrSuccess) //clear buffer if we have any issues queueing frames
        printf("Frame buffer queue cleared. \n");

    //if the requeue results in error 3 times, restart image capture
    //infinite requeue errors were seen in testing. Restarting image capture fixes the problem.
    if(Camera->snapcount == Camera->requeueFlag + 1) {
        Camera->queueError++;
        cout<<"Successive queue Failure: "<<Camera->queueError+1<<"\n\n";
    } else {
        Camera->queueError = 0;
    }
    if(Camera->queueError >= 2) {
        cout<<"Restarting Image Stream: successive requeue errors.\n";
        Camera->queueErrors++;
        RestartImCap(Camera);
        Camera->queueError = 0;
    }
    Camera->requeueFlag = Camera->snapcount;

    //if a frame returns error 16 followed by a frame requeue failure, restart Image Capture
    //This pattern of frame and queue errors was seen to preceed system crashes in testing.
/*    if(Camera->frameandqueueFlag){
        cout<<"Restarting Image Stream: frame and queue errors\n";
        RestartImCap(i);
        Camera->frameandqueueFlag=false;
        Camera->frameandqueueErrors++;
    } */
}
//______________________________________________________________________________________________


/* =============================================================================================
   Unsetup the camera.
   From AVT GigE SDK example named Stream.
   ========================================================================================== */
void CameraUnsetup(tCamera *Camera)
{
    printf("Preparing to unsetup camera with ID %lu\n", Camera->UID);
    printf("\nClearing the queue.\n");
    // Dequeue all the frames still queued (causes a block until dequeue finishes).
    PvCaptureQueueClear(Camera->Handle);
    // Close the camera.
    printf("Closing the camera.\n");
    PvCameraClose(Camera->Handle);

    // Delete the allocated buffer(s).
    for(int i = 0; i < FRAMESCOUNT; i++)
        delete [] (unsigned char*)Camera->Frames[i].ImageBuffer;

    Camera->Handle = NULL;
}
// __________________________________________________________________________________________end


/*===========================================================================================
    Restart Acquisition
=============================================================================================*/
void RestartAcq(tCamera *Camera)
{
    //pause program
    PAUSEPROGRAM = true;

    if(PvCommandRun(Camera->Handle, "AcquisitionAbort") == ePvErrSuccess) {
        cout<<"Acquisition Stopped.\n";
    } else {
        cout<<"Couldn't stop acquisition.\n";
    }
    if(PvCaptureQueueClear(Camera->Handle) == ePvErrSuccess)
        printf("Frame buffer queue cleared. \n");
    if(PvCommandRun(Camera->Handle, "AcquisitionStart") == ePvErrSuccess)
        cout<<"Acquisition Started. \n\n";

    //restart program
    PAUSEPROGRAM=false;
}
//____________________________________________________________________________________________


/*============================================================================================
    Restart the image capture stream
==============================================================================================*/
void RestartImCap(tCamera *Camera)
{
    //pause program
    PAUSEPROGRAM = true;

    if(PvCaptureQueueClear(Camera->Handle) == ePvErrSuccess)
        printf("Frame buffer queue cleared. \n");
    if(PvCommandRun(Camera->Handle, "AcquisitionAbort") == ePvErrSuccess) {
        cout<<"Acquisition Stopped.\n";
    }else{
        cout<<"Couldn't stop Acquisition.\n";
    }
    //if(PvCaptureQueueClear(Camera->Handle)==ePvErrSuccess)
    //    printf("Frame buffer queue cleared. \n");
    if(PvCaptureEnd(Camera->Handle) == ePvErrSuccess)
        printf("Image capture stream terminated. \n");
    if(PvCaptureStart(Camera->Handle) == ePvErrSuccess)
        printf("Image capture stream restarted. \n");
    if(PvCommandRun(Camera->Handle, "AcquisitionStart") == ePvErrSuccess)
        cout<<"Acquisition Started. \n\n";

    //restart Program
    PAUSEPROGRAM=false;
}
//______________________________________________________________________________________________


/* =============================================================================================
   Displays the parameters chosen for the camera.
   ========================================================================================== */
void DisplayParameters()
{
    printf("\n");
    for(unsigned int i = 0; i < NUMOFCAMERAS; i++) {
        printf("Displaying settings for camera with ID %lu\n", CAMERAS[i].UID);
        printf("----------\n");
        printf("Images per second: %d\n", CAMERAS[i].Rate);
        printf("Exposure time in microseconds: %lu\n", CAMERAS[i].ExposureLength);
        printf("Gain (dB): %lu\n", CAMERAS[i].Gain);
        if(CAMERAS[i].PauseCapture == true)
            printf("Camera capture paused? true\n");
        else
            printf("Camera capture paused? false\n");
        if(CAMERAS[i].WantToSave == true)
            printf("Saving images taken by this camera? true\n");
        else
            printf("Saving images taken by this camera? false\n");
        printf("\n");
    }
}
// __________________________________________________________________________________________end


/* =============================================================================================

   An error message printer for translations of numerical error codes to their
   corresponding errors, as noted by the Prosilica PvAPI Manual. Good for debugging.
   ========================================================================================== */
void PrintError(int errCode)
{
    printf("Error encountered. Error code: %d\n", errCode);
    switch(errCode) {
        case 2: {
            printf("ePvErrCameraFault: Unexpected camera fault.\n");
            break;
        }
        case 3: {
            printf("ePvErrInternalFault: Unexpected fault in PvAPI or driver.\n");
            break;
        }
        case 4: {
            printf("ePvErrBadHandle: Camera handle is bad.\n");
            break;
        }
        case 5: {
            printf("ePvErrBadParameter: Function parameter is bad.\n");
            break;
        }
        case 6: {
            printf("ePvErrBadSequence: Incorrect sequence of API calls. For example,\n");
            printf("queuing a frame before starting image capture.\n");
            break;
        }
        case 7: {
            printf("ePvErrNotFound: Returned by PvCameraOpen when the requested camera\n");
            printf("is not found.\n");
            break;
        }
        case 8: {
            printf("ePvErrAccessDenied: Returned by PvCameraOpen when the camera cannot be\n");
            printf("opened in the requested mode, because it is already in use by another\n");
            printf("application.\n");
            break;
        }
        case 9: {
            printf("ePvErrUnplugged: Returned when the camera has been unexpectedly\n");
            printf("unplugged.\n");
            break;
        }
        case 10: {
            printf("ePvErrInvalidSetup: Returned when the user attempts to capture images,\n");
            printf("but the camera setup is incorrect.\n");
            break;
        }
        case 11: {
            printf("ePvErrResources: Required system or network resources are unavailable.\n");
            break;
        }
        case 12: {
            printf("ePvErrQueueFull: The frame queue is full.\n");
            break;
        }
        case 13: {
            printf("ePvErrBufferTooSmall: The frame buffer is too small to store the image.\n");
            break;
        }
        case 14: {
            printf("ePvErrCancelled: Frame is cancelled. This is returned when frames are\n");
            printf("aborted using PvCaptureQueueClear.\n");
            break;
        }
        case 15: {
            printf("ePvErrDataLost: The data for this frame was lost. The contents of the\n");
            printf("image buffer are invalid.\n");
            break;
        }
        case 16: {
            printf("ePvErrDataMissing: Some of the data in this frame was lost.\n");
            break;
        }
        case 17: {
            printf("ePvErrTimeout: Timeout expired. This is returned only by functions with\n");
            printf("a specified timeout.\n");
            break;
        }
        case 18: {
            printf("ePvErrOutOfRange: The attribute value is out of range.\n");
            break;
        }
        case 19: {
            printf("ePvErrWrongType: This function cannot access the attribute, because the\n");
            printf("attribute type is different.\n");
            break;
        }
        case 20: {
            printf("ePvErrForbidden: The attribute cannot be written at this time.\n");
            break;
        }
        case 21: {
            printf("ePvErrUnavailable: The attribute is not available at this time.\n");
            break;
        }
        case 22: {
            printf("ePvErrFirewall: A firewall is blocking the streaming port.\n");
            break;
        }
        default: {
            printf("Unrecognizable error. Check for unintended calls to this function.\n");
            break;
        }
    }
}
// __________________________________________________________________________________________end
