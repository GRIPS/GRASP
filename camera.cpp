#define SAVE_1_OF_EVERY_N 10

#define MAXNUMOFCAMERAS 2
#define FRAMESCOUNT 5

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
#include <fstream>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <arpa/inet.h>
#include <pthread.h>

#include "analysis.hpp"
#include "control.h"
#include "main.hpp"
#include "oeb.h"
#include "Image.hpp"

//PvApi libraries
#include <PvApi.h>
#include <ImageLib.h>

#include <CCfits>            //This file includes all the headers to build the CCfits namespace classes
#include <cmath>

// _____________________________________________________________________________________________

/* =============================================================================================
   Declaration of namespaces and definitions
   ========================================================================================== */
using namespace std; //the global namespace, CCfits used loacally in savefits

//#define    _STDCALL
// _____________________________________________________________________________________________

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
    uint64_t         Clock; //from the odds & ends board

    tPvFrame         Frames[FRAMESCOUNT];
    //valarray<unsigned char> imarr[FRAMESCOUNT]; //each valarray is sized 0, resized in setup
    //auto_ptr<CCfits::FITS> pFits[FRAMESCOUNT]; //vector of pointers
    char             TimeStamps[FRAMESCOUNT][23];
    volatile bool    NewFlags[FRAMESCOUNT];
    unsigned int     Cadence;
    unsigned int     BufferIndex; //FIXME: not thread-safe
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


/* =============================================================================================
    Global Variables
   ========================================================================================== */
unsigned long NUMOFCAMERAS = 0;        // Actual number of cameras detected.
unsigned int TICKS_PER_SECOND; // Total number of ticks per second
volatile unsigned int CURRENT_TICK = 0;
bool PAUSEPROGRAM = false;
volatile bool TRANSMIT_NEXT_PY_IMAGE = false, TRANSMIT_NEXT_R_IMAGE = false;
tCamera CAMERAS[MAXNUMOFCAMERAS];
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
void timestamp(tCamera *Camera);

void set_cadence();
void set_timer(int x);
int next_camera();

void tester(int x, timeval& t1, int i);

void spawn_thread(int x);
void *snap_thread(void *cam);
void handle_wait(tCamera *Camera, tPvErr &errCode, int timeout2);
void handle_timeout(tCamera *Camera);
void RestartImCap(tCamera *Camera);
void queueErrorHandling(tCamera *Camera);

void Process(tCamera *Camera);
bool saveim(tCamera *Camera, valarray<unsigned char> imarr, const char* filename);
void transmit_image(tCamera *Camera, valarray<unsigned char> imarr);

int readfits(const char* filename, valarray<unsigned char>& contents, int &nelements, int &width);

//not currently used
void CameraStop(tCamera *Camera);
void RestartAcq(tCamera *Camera);
void DisplayParameters();
void FrameStats(tCamera *Camera);
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
    act.sa_handler = spawn_thread;
    sigaction(10, &act, &oldact);

    // Initialize the API.
    if(!PvInitialize()) {

        PvLinkCallbackRegister(CameraEventCB, ePvLinkAdd, NULL);
        PvLinkCallbackRegister(CameraEventCB, ePvLinkRemove, NULL);

            // Grab cameras.
            if(CameraGrab()) {
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
                        set_cadence();
                        while(g_running_camera_main) {
                            //Wait for interrputs which generate triggers to snap and process images
                            usleep_force(5000);
                        }
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

    return i > 0;
}
// __________________________________________________________________________________________end


/* =============================================================================================
   Opens a camera and fills in parameters.
   ========================================================================================== */
bool CameraSetup(tCamera *Camera)
{
    prog_c con;
    init_prog_c(con);

    // Default settings are hard-coded, but should load from parameter table (FIXME)
    Camera->WantToSave = true;
    if(is_pyc(Camera)) {
        Camera->Cadence = 1;
        Camera->ExposureLength = 20000;
        Camera->Gain = 0;
    } else {
        Camera->Cadence = 1;
        Camera->ExposureLength = 4000;
        Camera->Gain = 0;
    }

    printf("%s camera settings: %u Hz, %lu us, %lu dB gain, %s\n",
           (is_pyc(Camera) ? "Pitch-yaw" : "Roll"),
           Camera->Cadence, Camera->ExposureLength, Camera->Gain,
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
   Sets cadence and initiates alarm
   ========================================================================================== */
void set_cadence()
{
    bool tempBool = PAUSEPROGRAM;
    PAUSEPROGRAM = true;

    for(unsigned int i = 0; i < NUMOFCAMERAS; i++) {
        TICKS_PER_SECOND += CAMERAS[i].Cadence;
    }

    //print TICKS_PER_SECOND
    cout<<"This is the ticks per second: "<<TICKS_PER_SECOND<<"\n";
    set_timer(0);     //create and arm the interrupt timer

    PAUSEPROGRAM = tempBool;
}
// __________________________________________________________________________________________end


/*============================================================================================
 Handles timers for the camera SNAP
 timer(0); create timer
 timer(1); disable timer //FIXME: I don't think this is implemented correctly
==============================================================================================*/
void set_timer(int x)
{
    //set the sigevent notification structure
    struct sigevent timersigevent; //create signal event
    memset(&timersigevent, 0, sizeof timersigevent); //initialize the struct with zeros
    timersigevent.sigev_notify = SIGEV_THREAD_ID | SIGEV_SIGNAL; //send a signal upon expiration of timer & only to a certain thread
    timersigevent.sigev_signo = 10; //set to SIGUSR1 number 10
    timersigevent._sigev_un._tid = syscall(SYS_gettid); //notify the thread that creates the timer. This will be the main thread.

    //create the timer
    timer_t timer1; //timer identifier
    if(timer_create(CLOCK_REALTIME, &timersigevent, &timer1) == 0) {
        printf("timer created correctly\n");
    } else {
        printf("timer not created \n");
    }

    //sleep to allow dd to register the handler before arming timer
    //struct timespec tim, tim2;
    //tim.tv_sec=0;
    //tim.tv_nsec=100;
    //nanosleep(&tim, &tim2);

    if(x == 0) {
        //Set timer values
        struct itimerspec cadence;
        memset(&cadence, 0, sizeof cadence);
        if (TICKS_PER_SECOND != 1) {
            cadence.it_value.tv_sec= 0;         //value is time from set until first tick
            cadence.it_value.tv_nsec =  1000000000/ TICKS_PER_SECOND;
            cadence.it_interval.tv_sec=0;         //interval resets the timer to this value
            cadence.it_interval.tv_nsec= 1000000000/ TICKS_PER_SECOND;
        } else {
            cadence.it_value.tv_sec= 1;
            cadence.it_value.tv_nsec =  0;
            cadence.it_interval.tv_sec= 1;
            cadence.it_interval.tv_nsec= 0;
        }

        //arm the timer
        if(timer_settime(timer1, 0, &cadence, NULL) == 0) {
            printf("timer armed correctly\n");
        } else {
            printf("timer not armed.\n");
        }
    } else if(x == 1) {
        //disable the timer
        struct itimerspec pause;
        memset(&pause, 0, sizeof pause);
        if(timer_settime(timer1, 0, &pause, NULL) == 0)
            printf("SNAP timer disabled.\n");
    }
    cout<<"\n";
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

    // Determine how big the frame buffers should be and set the exposure value.
    if(!PvAttrUint32Get(Camera->Handle, "TotalBytesPerFrame", &FrameSize)
       && !PvAttrUint32Set(Camera->Handle, "ExposureValue", Camera->ExposureLength)
       && !PvAttrUint32Set(Camera->Handle, "GainValue", Camera->Gain)) {

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


/* =============================================================================================
   timestamp an image
   ========================================================================================== */
void timestamp(tCamera *Camera)
{
    ostringstream os;
    ostringstream imageIndex;
    time_t rawtime;
    timeval highrestime;
    struct tm * timeinfo;
    char filename[17];
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime (filename, 17, "%Y%m%d_%H%M%S_", timeinfo);
    gettimeofday(&highrestime, NULL);
    imageIndex << highrestime.tv_usec;
    os << filename << imageIndex.str();
    strcpy(Camera->TimeStamps[Camera->BufferIndex], os.str().c_str());
}
// _________________________________________________________________________________________end


/*=============================================================================================
  Determines which camera to snap next
  Currently assumes two cameras and alternates between them until one camera is "done"
  Returns 0 or 1
  ========================================================================================== */
int next_camera()
{
    if(CURRENT_TICK < 2 * MIN(CAMERAS[0].Cadence, CAMERAS[1].Cadence)) {
        return CURRENT_TICK % 2;
    }
    return CAMERAS[0].Cadence > CAMERAS[1].Cadence ? 0 : 1;
}
// _________________________________________________________________________________________end


/*=============================================================================================
  spawn threads for snapping/processing
  ========================================================================================== */
void spawn_thread(int x)
{
    if(!PAUSEPROGRAM && g_running_camera_main) {
        //which camera snaps?
        tCamera *Camera = &CAMERAS[next_camera()];

        //spawn thread based on camera and idx
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
        cout << (is_pyc(Camera) ? "..\n" : "++\n");
        //cout<<" \n"; //without this the transmission time is 50ms, wiht it its .05ms.. something is wrong

        //update thread and buffer indexes
        Camera->BufferIndex = Camera->idx; //the active buffer = this thread buffer
        Camera->idx = ((Camera->idx + 1) % FRAMESCOUNT);

        //update CURRENT_TICK
        CURRENT_TICK = ((CURRENT_TICK + 1) % TICKS_PER_SECOND);
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
    //int count = 0;
    prog_c con;
    init_prog_c(con);

    //housekeeping
    getTemp(Camera);

    //start snap only if we're done waiting for another frame to return (but doesn't protect frame buffer)
    if(Camera->Handle != NULL && !Camera->PauseCapture && !Camera->waitFlag) {
        //snap on time? output to screen
        timeval t;
        //tester(0, t, i);

        //requeue a frame & snap (if successful requeue) - then process (if no timeout & done waiting)
        Camera->queueStatus = PvCaptureQueueFrame(Camera->Handle,&(Camera->Frames[Camera->BufferIndex]),NULL);
        if(Camera->queueStatus == ePvErrSuccess) {
            //update flags
            Camera->requeueCallFlag = false;
            Camera->NewFlags[Camera->BufferIndex] = true;
            Camera->frameandqueueFlag = false;

            //trigger, wait and queue processing if successful
            timestamp(Camera);
            Camera->snapcount++;
            Camera->waitFlag= true;
            if(PvCommandRun(Camera->Handle, "FrameStartTriggerSoftware") != ePvErrSuccess) {
                cout<<"Trigger Software Error: ";
                PrintError(errCode);
            }
            if(con.c_timer)
                tester(1,t,0);
            errCode = PvCaptureWaitForFrameDone(Camera->Handle,&(Camera->Frames[Camera->BufferIndex]),con.timeout1);
            handle_wait(Camera, errCode, con.timeout2); //checks and updates waitflag and errCode
            if(con.c_timer) {
                cout<<Camera->UID<<" ";
                tester(2,t,0);
            }

            //Process: if done waiting, no timeout, successful frame & non-zero bitdepth
            //cout<<"image size: "<<Camera->Frames[Camera->BufferIndex].ImageSize<<endl;
            if(Camera->waitFlag== false && errCode == ePvErrSuccess) {
                if( Camera->Frames[Camera->BufferIndex].Status == ePvErrSuccess && Camera->Frames[Camera->BufferIndex].BitDepth != 0) {
                    if(con.c_timer)
                        tester(1,t,0);
                    Process(Camera);
                    if(con.c_timer) {
                        cout<<"Processing "; //timer
                        tester(2,t,0);
                        cout<<"\n\n";
                    }
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
    ++Camera->pcount;
    if(is_pyc(Camera)) {
        Camera->Clock = oeb_get_pyc();
        py_image_counter++;
    } else {
        Camera->Clock = oeb_get_rc();
        roll_image_counter++;
    }

    // create copy of image buffer
    valarray<unsigned char> imarr((unsigned char*)Camera->Frames[Camera->BufferIndex].ImageBuffer, Camera->FrameHeight*Camera->FrameWidth);

    //1. init stucts and variables
    prog_c con;
    init_prog_c(con);

    // FIXME: should not do both, but can't fix here
    // Stuff for pitch-yaw camera
    params val;
    init_params(val, (int)Camera->FrameWidth, (int)(Camera->FrameHeight*Camera->FrameWidth));
    //init_params(val, 449, 144129); //for small live frame
    info im;
    init_im(im);
    // Stuff for roll camera
    params_H val_H;
    init_params_H(val_H, (int)Camera->FrameWidth, (int)(Camera->FrameHeight*Camera->FrameWidth));
    info_H im_H;
    init_H(im_H);

    timeval t;

    //2. analyze live or test image?
    if(is_pyc(Camera) && !con.live) {
        const char* filename1 = "~/GRASPcode/tstimages/tstim2.fits";        //sun is 330 pix
        //const char* filename1 = "~/GRASPcode/tstimages/dimsun1_960x1290_bw.fits";    //sun is 195 pixels
        //const char* filename1 = "~/GRASPcode/tstimages/dimsun1.fits";                        //sun is        <80
        readfits(filename1, imarr, val.nel, val.width);
    }

    if(con.c_timer)
        tester(1,t,0);
    if(is_pyc(Camera)) {
        analyzePY(im, val, imarr);
        if(TRANSMIT_NEXT_PY_IMAGE) {
            transmit_image(Camera, imarr);
            TRANSMIT_NEXT_PY_IMAGE = false;
        }
    } else {
        //analyzeH(im_H, val_H, imarr);
        if(TRANSMIT_NEXT_R_IMAGE) {
            transmit_image(Camera, imarr);
            TRANSMIT_NEXT_R_IMAGE = false;
        }
    }
    if(con.c_timer) {
        cout<<"Analysis ";
        tester(2,t,0);
    }
    if(con.diag) {
        if(is_pyc(Camera)) {
            diagnostics(val, im);
        } else {
            diag_H(val_H, im_H);
        }
    }

    //drawline at centers? This will be saved in the image below
    if(is_pyc(Camera) && val.drawline) {
        drawline(imarr, val, im);
    }

    //3. save?
    if(Camera->WantToSave) {
        //create filename
        ostringstream filename;
        filename << "images/" << Camera->UID << "_" << Camera->TimeStamps[Camera->BufferIndex]<<"_"<< Camera->savecount<<".fits";
        //filename << "images/" << Camera->UID << "_" << Camera->BufferIndex<<".fits"; //circular filename buffer
        if(con.c_timer)
            tester(1,t,0);
        if((Camera->savecount % SAVE_1_OF_EVERY_N) == 0) {
            saveim(Camera, imarr, filename.str().c_str());
        }
        Camera->savecount++;
        if(con.c_timer) {
            cout<<"Saving ";
            tester(2,t,0);
        }
        filename.seekp(0);
     }
}
// __________________________________________________________________________________________end


/*=============================================================================================
  Transmit a whole image
  ========================================================================================== */
void transmit_image(tCamera *Camera, valarray<unsigned char> imarr)
{
    cout << "Transmitting image from camera " << Camera->UID << endl;

    ImagePacketQueue ipq;

    ipq.add_partial_array(is_pyc(Camera) ? 0 : 1, Camera->FrameHeight, Camera->FrameWidth,
                          0, Camera->FrameHeight, 0, Camera->FrameWidth / 2, (uint8_t *)&(imarr[0]),
                          Camera->Clock, false);
    ipq.add_partial_array(is_pyc(Camera) ? 0 : 1, Camera->FrameHeight, Camera->FrameWidth,
                          0, Camera->FrameHeight, 0, Camera->FrameWidth / 2, (uint8_t *)&(imarr[0]),
                          Camera->Clock, true);

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
bool saveim(tCamera *Camera, valarray<unsigned char> imarr, const char* filename)
{
    using namespace CCfits;
    //using std::valarray;

    FITS::setVerboseMode(true);

    //create pointer to fits object
    std::auto_ptr<FITS> pFits(0);
    //std::valarray<unsigned char> imarr((char*)Camera->Frames[j].ImageBuffer, nelements);

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

        //append keys to the primary header
        //long exposure(1500);
        pFits->pHDU().addKey("Camera",(long)Camera->UID, "Camera UID");
        pFits->pHDU().addKey("Save Count",(int)Camera->savecount, "Num of Saved Images");
        pFits->pHDU().addKey("Snap Count",(int)Camera->snapcount, "Num of SNAPS");
        pFits->pHDU().addKey("Exposure",(long)Camera->ExposureLength, "for cameanalyzera");
        pFits->pHDU().addKey("Gain",(long)Camera->Gain, "Gain (dB)");
        pFits->pHDU().addKey("filename", filename, "Name of the file");

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

    string newName ("Raw Image");
    long fpixel(1);

    try {
        imageExt = pFits->addImage(newName,BYTE_IMG,extAx, 1);
        imageExt->write(fpixel,Camera->FrameWidth*Camera->FrameHeight,imarr);    //write extension
        //std::cout << pFits->pHDU() << std::endl;
    } catch (FitsException) {
        std::cout<<"Couldn't write image to extension\n";
    }

    return true;
}
// __________________________________________________________________________________________end


/* =============================================================================================
   Saves a fits file
   ========================================================================================== */
bool saveim_H(tCamera *Camera, valarray<unsigned char> imarr, const char* filename, info_H im, params_H val)
{
    using namespace CCfits;
    using std::valarray;

    FITS::setVerboseMode(true);

    //create pointer to fits object
    std::auto_ptr<FITS> pFits(0);
    //std::valarray<unsigned char> imarr((char*)Camera->Frames[j].ImageBuffer, nelements);

    //for running loop tests
    remove(filename);
    //std::cout<<"here\n";

    try {
        //create new fits file
        try {
           pFits.reset( new FITS(filename , BYTE_IMG , 0 , 0 ) ); //reset=deallocate object and reset its value. change to BYTE_IMG?
        } catch (FITS::CantCreate) {
          std::cout<<"Couldn't create FITS file. The file might already exist. \n";
          return false;
        }

        //append keys to the primary header
        //long exposure(1500);
        pFits->pHDU().addKey("Camera",(long)Camera->UID, "Camera UID");
        pFits->pHDU().addKey("Save Count",(int)Camera->savecount, "Num of Saved Images");
        pFits->pHDU().addKey("Snap Count",(int)Camera->snapcount, "Num of SNAPS");
        pFits->pHDU().addKey("Exposure",(long)Camera->ExposureLength, "for cameanalyzera");
        pFits->pHDU().addKey("filename", filename, "Name of the file");
    } catch (FitsException&) {
        std::cerr << " Fits Exception Thrown.  \n";
    }

    //store data in an extension HDU
    //its customary to have compressed images in 1st Ext
    ExtHDU* imageExt;
    std::vector<long> extAx;
    extAx.push_back(val.width);
    extAx.push_back(val.nel/val.width);
    string newName ("Raw Image");
    long fpixel(1);

    try {
        imageExt = pFits->addImage(newName,BYTE_IMG,extAx, 1);
        imageExt->write(fpixel,val.nel,imarr);    //write extension
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
int readfits(const char* filename, valarray<unsigned char>& contents, int &nelements, int &width)
{
    using namespace CCfits;
    using std::valarray;

    //FITS::setVerboseMode(true);

    std::auto_ptr<FITS> pInfile(new FITS( filename ,Read,true));
    PHDU& image = pInfile->pHDU();

    // read all user-specifed, coordinate, and checksum keys in the image
     image.readAllKeys();
     image.read(contents);

    //axes
    int idx = 0;
    width = (int)image.axis(idx);
    int height = (int)image.axis(idx+1);
    nelements = width * height;

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
        printf("Images per second: %d\n", CAMERAS[i].Cadence);
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
