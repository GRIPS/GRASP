/* =============================================================================================
   Program Description

   Version 0 of the GRASP program does not compress the images. Many of the
   functions used are made available in the Linux version of the AVT GigE SDK, which can be
   found online.

   This program was written for use in a Linux environment. Follow the prompts in the program
   to advance it. Pressing CTRL + 'c' will allow changing of parameters, pausing of the program,
   or termination of the program.

   Please pay attention to the FRAMESCOUNT parameter and recompile if it needs to be changed.

   Development versions of the level zero software are appended with _dvN.cpp 
   Where N is the version.

   ========================================================================================== */


/* =============================================================================================
			dv27: 			re-introducing processing chain. keep thread idx and BufferIndex separate but linked

			dv26: 			implemented error handling for timeout. Actual processing is still commented out

			v1_dv25: restructured code to use waitforframedone instead of callbacks. This works reasonably well in this
					code implementation. 

			v1_dv24: implement camera temperature sensors and voltage output for exposure (on the connector). Last version of code
												using the callback. Next version will try to use threadblocking via waitforframedone for analysis

			v1_dv23:	only uses openbyaddr. Does not use listcameras or wait for cameras. Tries both IP addys. Getting rid of 
												now obsolete code. 

			v1_dv22: introduce separate .h/.cpp files for Horizon sensor and control chains. openbyaddr works but stil uses 
												listcameras to discover cameras

			v1_dv21: change ticks so that we never snap at the same time. This version is used to test the flight computer. 
												use with a_PY_dv21.cpp/.h

			v1_dv20: test parameter to analyze dimsun images or live images, diagnostics and timing. sorting only 1/4 image

			v1_dv19: changing to unsigned char. Use a_PY_dv19.h/.cpp Fixed the centroiding issue in find_centroid of a_PY.cpp code

			v1_dv18: changing to unsigned short. Use a_PY.h/.cpp

			v1_dv17: changing to char. Use a_PY.h/.cpp. The problem with char is that I can't use the sort on char and I can't
												typecast a valarray for sorting. So, the sort doesn't come out right. 

			v1_dv16: branching off v1_dv14_test_live. switching over to floats for framebuffer. Use analysis_PY_float.h/.cpp

			v1_dv15: Chaning analysis code from int --> char. Using analysis_PY.h/.cpp and Makefile_dv15		

			v1_dv14_test_live: branching off v1_dv14_test. analysis is in <int>, changed acquire from char --> int in the framebuffer

			v1_dv14_test: This is a branch of v1_dv14 that replicates the analysis code from analyzePY into analyzeH. This is for a 
												functional test of the aspect code on the flight computer hardware, at this stage.

			v1_dv14:	Started integrating the analysis code, still using the type int. Using analysis_PY_int.h/.cpp and Makefile_dv14

			v1_dv13:	Added keys to the Fits file. restructured the saveim, got rid of savefits_char. This is compiled with Makefile_dv13

			v1_dv12: cleaned up the code and removed errors. No major changes.

			v1_dv11: Segmentation fault issue resolved. Needed to rebuild the cfitsio and CCfits libraries to be threadsafe.
												Also in this version: tried to change how the program deals with extracting the image from the camera's image buffer. goes into
												a valarray which is in the camera struct. each framebuffer has a corresponding valarray in imarr.	Also tried assoc
												each framebuffer with a unique pFits pointer.
												Neither of these strategies fixed the segmentation fault issue. 
												This version still has those code snippets, but commented out. 
												Currently, we create a valarray and pFits pointer in the save thread, they are not assoc uniquely with a buffer		

			v1_dv10: runs. fixed threading issue by detaching threads before it exits. This releases all system 
											resources upon exit.   
		
			v1_dv9: runs. snaps from both cameras. can snap at ~same speed and with the same amount of 
											dropped frames as the example code. Issues with saving images, get successive queue errors
											core dumps, something where the snap occurs and the tickcount isn't right. 

											implemented threads for camera[0]. have some things hardcoded for camera[0] in the framedone
											and processimages. threading issue, can't create more threads


   ========================================================================================== */



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
#include "a_PY.h"
#include "a_H.h" 
#include "control.h"
#include "oeb.h"

//PvApi libraries
#include <PvApi.h>
#include <ImageLib.h>

//from CCFits 
#ifdef _MSC_VER
#include "MSconfig.h" // for truncation warning
#endif

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <CCfits>			//This file includes all the headers to build the CCfits namespace classes
#include <cmath>


// _____________________________________________________________________________________________

/* =============================================================================================
   Declaration of namespaces and definitions
   ========================================================================================== */
using namespace std; //the global namespace, CCfits used loacally in savefits

//#define	_STDCALL
#define	TRUE		0
#define	FRAMESCOUNT	5
// _____________________________________________________________________________________________

/* =============================================================================================
   Structure for camera data. The universal ID is unique to each camera and the handle allows
   the program to interact with the camera. The frame(s) are filled with images as they are
   taken.

			The struct handles all error and diagnostic info
   ========================================================================================== */
struct tCamera {

	unsigned long	UID;
	tPvHandle		Handle;
	tPvFrame		Frames[FRAMESCOUNT];
	unsigned long IP;			//the IP addy of the camera
	//valarray<unsigned char> imarr[FRAMESCOUNT]; //each valarray is sized 0, resized in setup
	//auto_ptr<CCfits::FITS> pFits[FRAMESCOUNT]; //vector of pointers
	char			TimeStamps[FRAMESCOUNT][23];
	volatile bool	NewFlags[FRAMESCOUNT];
	int				TicksPerSec;
	int				TicksUntilCapture;
	unsigned long	ExposureLength;
	int				BufferIndex;
	float			FrameHeight;
	float			FrameWidth;
	volatile bool	PauseCapture;
	volatile bool	WantToSave;
	int idx;
	pthread_t thread[FRAMESCOUNT];		//threads for snapping/processing

	//error flags
	volatile bool waitFlag;
	int requeueFlag;
	volatile bool frameandqueueFlag;
	volatile bool triggerFlag;
	int queueClearFlag;
	volatile bool requeueCallFlag;
	volatile bool timeoutFlag;

	//diagnostics - which ones do i really still need?
	int snapcount;
	int savecount;
	int unsuccount;	//frames that complete but are marked unsuccessful
	int zerobitcount;	//number of frames with zero bit depth
	int pcount;		//processed frames count
	int framecount;
	int noSignocount;
	int Signocount;
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

const int MAXNUMOFCAMERAS = 2;	// Specifies the maximum number of cameras used. Can be changed.
unsigned long NUMOFCAMERAS = 2;		// Actual number of cameras detected.
volatile bool TERMINATE = false;
volatile bool CAMERASFOUND = false;
int TICKSLCM;
int a_cad;
int TICKCOUNT = 0;
bool PAUSEPROGRAM = false;
tCamera GCamera;
tCamera CAMERAS[MAXNUMOFCAMERAS];

unsigned long PY_cam_ID = 158434;
unsigned long H_cam_ID = 142974;
// __________________________________________________________________________________________end


/* =============================================================================================
   Function declarations.
   ========================================================================================== */
void PrintError(int errCode);
void Sleep(unsigned int time);
void SetConsoleCtrlHandler(void (*func)(int), int junk);
void CameraEventCB(void* Context, tPvInterface Interface, tPvLinkEvent Event,
							unsigned long UniqueId);
bool YesOrNo();
int LCM(int x, int y);
bool WaitForCamera();
bool CameraGrab();
bool CameraSetup(tCamera* Camera, int cam);
void TicksLCM();
bool CameraStart(tCamera* Camera);
void CameraStop(tCamera* Camera);
void CameraSnap(int x);
void CameraSnap2(int x);
void spawn_thread(int x);
void *snap_thread(void *cam);
void handle_wait(int i, tPvErr &errCode, int timeout2);
void handle_timeout(int i);
void Process(int i, int CurrBuffer);
void CameraUnsetup(tCamera* Camera);
void CtrlCHandler(int Signo);
void DisplayParameters();
void ProcessPY(int i, valarray<unsigned char> imarr);
void ProcessH(int i, valarray<unsigned char> imarr);
void tester(int x, timeval& t1, int i);
void FrameDone0(tPvFrame* pFrame);
void FrameDone1(tPvFrame* pFrame);
void FrameStats(tCamera* Camera);
void RestartImCap(int j);
void RestartAcq(int j);
void timer(int x);
void queueErrorHandling(int i);
bool saveim(int i, valarray<unsigned char> imarr, const char* filename, info im, params val);
bool saveim_H(int i, valarray<unsigned char> imarr, const char* filename, info_H im, params_H val);
int readfits(const char* filename, valarray<unsigned char>& contents, int &nelements, int &width);
void set_timer(int x);
void set_cadence();
void timestamp(int i);
int evenodd();
int whichcamera();
void init_cam_struct(int i);
bool getTemp(tCamera* Camera);
// __________________________________________________________________________________________end


/* =============================================================================================
   Main Program
   Based on the AVT GigE SDK examples named Stream and Snap.
   ========================================================================================== */
int main(int argc, char* argv[]) {

	// Initialize the API.
	if(!PvInitialize()) {

		SetConsoleCtrlHandler(CtrlCHandler, TRUE);

		PvLinkCallbackRegister(CameraEventCB, ePvLinkAdd, NULL);
		PvLinkCallbackRegister(CameraEventCB, ePvLinkRemove, NULL);

			// Grab cameras.
			if(CameraGrab()) {
				// Set up cameras.
				bool setupSuccess = true;
				for(unsigned int i = 0; i < NUMOFCAMERAS; i++) {
					if(!CameraSetup(&CAMERAS[i], i))
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
						//TicksLCM(); //must have ticks after the signal handler is setup
						set_cadence();
						while(!TERMINATE) {
							//Wait for interrputs which generate triggers to snap and process images
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

		printf("\nEnd of program reached.\n\n");

	} else
		printf("\nFailed to initialize the API. Program terminating.\n\n");

    return 0;

}
// __________________________________________________________________________________________end


/* =============================================================================================
   get the camera temperature
			***move into control.h
   ========================================================================================== */
bool getTemp(tCamera* Camera){

	tPvFloat32 T_MB = 0;
	tPvFloat32 T_CCD = 0;

	const char* whichfile;
	ofstream tempfile;
	if(Camera->UID == PY_cam_ID){
		whichfile = "PY_temps.txt";
	}else{
		whichfile = "H_temps.txt";
	}
	tempfile.open(whichfile, ios::app);

	if(PvAttrFloat32Get(Camera->Handle, "DeviceTemperatureMainboard", &T_MB) == 0){	
				//PvAttrFloat32Get(Camera->Handle, "DeviceTemperatureSensor", &T_CCD);
				//cout<<"T_MB = "<<T_MB<<endl;
				//cout<<"T_CCD = "<<T_CCD<<endl;
				tempfile << T_MB <<endl;

                                //Send temperatures on a loopback to be received by main process
                                TelemetrySender telSender("127.0.0.1", 44444);
                                TelemetryPacket tp(0x4F, Camera->UID == PY_cam_ID ? 0 : 1, 0,
                                                   Camera->UID == PY_cam_ID ? oeb_get_pyc() : oeb_get_rc());
                                tp << (float)T_MB;
                                telSender.send(&tp);

				return true;
	}	else {
				cout<<"Temp error\n";
				tempfile << "Temp error"<< endl;
				//PrintError(PvAttrFloat32Get(Camera->Handle, "DeviceTemperatureMainboard", &T_MB));
			//	PrintError(PvAttrFloat32Get(Camera->Handle, "DeviceTemperatureSensor", &T_CCD));
				//cout<<"T_MB = "<<T_MB<<endl;
			//	cout<<"T_CCD = "<<T_CCD<<endl;
				return false;
	}
}
// _________________________________________________________________________________________end



/* =============================================================================================
   Handler for CTRL key
   ========================================================================================== */
void SetConsoleCtrlHandler(void (*func)(int), int junk) {

	signal(SIGINT, func);

}
// __________________________________________________________________________________________end



/* =============================================================================================
   Callback called when a camera is plugged or unplugged.
   From AVT GigE SDK example named Stream.
		** I don't think this is working!!!!!!!!!! for cameras plugged in during running
   ========================================================================================== */
void CameraEventCB(void* Context, tPvInterface Interface, tPvLinkEvent Event,
							unsigned long UniqueId) {

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
   Wait for the user to choose between "yes" or "no" when prompted.
   ========================================================================================== */
bool YesOrNo() {

	char choice;

	do {
		choice = getc(stdin);
	} while(choice != 'y' && choice != 'n');

	return choice == 'y';

}
// __________________________________________________________________________________________end



/* =============================================================================================
   Grab cameras and establish default parameters.
   ========================================================================================== */
bool CameraGrab() {

	int i = 0;
	const char* IP1 = "169.254.100.2";
	const char* IP2 = "169.254.200.2";
	

	//try 1st camera
	memset(&CAMERAS[i], 0, sizeof(tCamera));
	if(!PvCameraOpenByAddr(inet_addr(IP1), ePvAccessMaster, &(CAMERAS[i].Handle))) {
				CAMERAS[i].UID = PY_cam_ID;
				CAMERAS[i].IP =  inet_addr(IP1);
				i+=1;
	}else{
				cout<<"couldn't open PY camera\n";
	}
	//try 2nd camera
	memset(&CAMERAS[i], 0, sizeof(tCamera));
	if(!PvCameraOpenByAddr(inet_addr(IP2), ePvAccessMaster, &(CAMERAS[i].Handle))) {
				CAMERAS[i].UID = H_cam_ID;
				CAMERAS[i].IP =  inet_addr(IP2);
				i+=1;
	}else{
				cout<<"couldn't open H camera\n";
	}

	NUMOFCAMERAS = i;
	if(i > 0)
				CAMERASFOUND = true;

	//set-up camera flags and indexes
	for(unsigned int i = 0; i < NUMOFCAMERAS; i++) {
			init_cam_struct(i);
		}
		
	if(i > 0)
			return true;
	else
			return false;
}
// __________________________________________________________________________________________end

/* =============================================================================================
   initialize camera struct
   ========================================================================================== */
void init_cam_struct(int i){

			CAMERAS[i].BufferIndex = 0;
			CAMERAS[i].PauseCapture = false;
			CAMERAS[i].WantToSave = false;
			CAMERAS[i].idx=0;

			//initialize flags
			CAMERAS[i].waitFlag = false;
			CAMERAS[i].requeueFlag=0;
			CAMERAS[i].frameandqueueFlag= false;
			CAMERAS[i].triggerFlag=false;
			CAMERAS[i].queueClearFlag = 1;
			CAMERAS[i].requeueCallFlag=false;
			CAMERAS[i].timeoutFlag=false;
			for(int j = 0; j < FRAMESCOUNT; j++) {
				CAMERAS[i].NewFlags[j] = false; // =true for a populated frame that hasn't been processed
			}

			//initialize diagnostics
			CAMERAS[i].snapcount=0;
			CAMERAS[i].savecount=0;
			CAMERAS[i].unsuccount=0;
			CAMERAS[i].zerobitcount=0;
			CAMERAS[i].framecount=0;
			CAMERAS[i].pcount=0;
			CAMERAS[i].noSignocount=0;
			CAMERAS[i].Signocount=0;
			//CAMERAS[i].queueStatus=; //initialize to nothing for now
			CAMERAS[i].frameandqueueErrors=0;
			CAMERAS[i].queueErrors=0;
			CAMERAS[i].TimeoutCount =0;
			CAMERAS[i].queueError=0;
			CAMERAS[i].to1=0;

}

/* =============================================================================================
   Opens a camera and fills in parameters.
   ========================================================================================== */
bool CameraSetup(tCamera* Camera, int cam) {

		prog_c con;
		init_prog_c(con);

		if(con.def){
				cout<<"Default settings: 5 im/s, 20000us, saving\n";
				Camera->TicksPerSec = 5;
				Camera->ExposureLength = 20000;
				//Camera->WantToSave = false;
				Camera->WantToSave = true;
		} else {
				printf("Now setting up camera with ID %lu\n", Camera->UID);
				printf("\nHow quickly should pictures be processed?\n");
				printf("Pictures per second: ");
				scanf("%d", &Camera->TicksPerSec); 
		
				printf("\nHow long should the exposure value be in microseconds?\n");
				printf("Upper limit based on pictures per second: %d\n", 1000000 / Camera->TicksPerSec);
				printf("Exposure time: ");
				scanf("%lu", &Camera->ExposureLength); 

				printf("\nWould you like to save the pictures to the hard drive? (y/n): ");
				if(YesOrNo())
					Camera->WantToSave = true;
				else
					Camera->WantToSave = false;
				printf("\n");
		}

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
void set_cadence(){

	bool tempBool = PAUSEPROGRAM;
	PAUSEPROGRAM = true;

	for(unsigned int i=0;i<NUMOFCAMERAS;i++){
		a_cad += CAMERAS[i].TicksPerSec;
	}

	//print a_cad
	cout<<"This is the a_cad: "<<a_cad<<"\n";
	set_timer(0); 	//create and arm the timer which generates an interrupts at a cadence of TICKSLCM
	
	PAUSEPROGRAM = tempBool;

}


// __________________________________________________________________________________________end



/*============================================================================================
 Handles timers for the camera SNAP
 timer(0); create timer
 timer(1); disable timer
==============================================================================================*/
void set_timer(int x){
	//create handler
	struct sigaction act;  
	struct sigaction oldact;
	//act.sa_handler = CameraSnap2;
	act.sa_handler = spawn_thread;
	sigaction(10, &act, &oldact);	

	//set the sigevent notification structure
	struct sigevent timersigevent; //create signal event
	memset(&timersigevent, 0, sizeof timersigevent); //initialize the struct with zeros
	timersigevent.sigev_notify = SIGEV_THREAD_ID | SIGEV_SIGNAL; //send a signal upon expiration of timer & only to a certain thread
	timersigevent.sigev_signo = 10; //set to SIGUSR1 number 10
	timersigevent._sigev_un._tid = syscall(SYS_gettid); //notify the thread that creates the timer. This will be the main thread.

	//create the timer
	timer_t timer1; //timer identifier	
	if(timer_create(CLOCK_REALTIME, &timersigevent, &timer1) == 0){
		printf("timer created correctly\n");
	}else{
		printf("timer not created \n");
	} 

	//sleep to allow dd to register the handler before arming timer
	struct timespec tim, tim2;
	tim.tv_sec=0;
	tim.tv_nsec=100;
	nanosleep(&tim, &tim2);


	if(x == 0){
		//Set timer values
		struct itimerspec cadence;
		memset(&cadence, 0, sizeof cadence);
		if (a_cad != 1){
			cadence.it_value.tv_sec= 0; 		//value is time from set until first tick
			cadence.it_value.tv_nsec =  1000000000/ a_cad; 
 			cadence.it_interval.tv_sec=0; 		//interval resets the timer to this value
			cadence.it_interval.tv_nsec= 1000000000/ a_cad;
		} else{
			cadence.it_value.tv_sec= 1; 
			cadence.it_value.tv_nsec =  0; 
	 		cadence.it_interval.tv_sec= 1; 
			cadence.it_interval.tv_nsec= 0;
		}

		//arm the timer
		if(timer_settime(timer1, 0, &cadence, NULL) == 0){
			printf("timer armed correctly\n");
		}else{
			printf("timer not armed.\n");
		}
	}else if(x == 1){
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
bool CameraStart(tCamera* Camera) {

	unsigned long FrameSize = 0;

	printf("Starting camera with ID %lu\n", Camera->UID);

	// Auto adjust the packet size to the maximum supported by the network; usually 8228,
	// but 6000 for the current hardware.
	if(PvCaptureAdjustPacketSize(Camera->Handle, 6000)== ePvErrSuccess )
			cout<< "Packet Size sucessfully determined.\n\n";
	else
			cout<<"Possible Packet Size issue.\n\n";

	// Determine how big the frame buffers should be and set the exposure value.
	if(!PvAttrUint32Get(Camera->Handle, "TotalBytesPerFrame", &FrameSize)
	   && !PvAttrUint32Set(Camera->Handle, "ExposureValue", Camera->ExposureLength)) {

		Camera->FrameHeight = fabs(sqrt(.75 * FrameSize));
		Camera->FrameWidth = FrameSize / Camera->FrameHeight;
		printf("\nFrame buffer size: %lu; %f by %f\n", FrameSize, Camera->FrameWidth,
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
					if(PvAttrUint32Set(Camera->Handle, "PacketSize", 8228)){ //
						// Begin acquisition.
						if(PvCommandRun(Camera->Handle, "AcquisitionStart")) {
							// If that fails, reset the camera to non-capture mode.
							PvCaptureEnd(Camera->Handle);
							return false;
							} else {
									printf("Camera with ID %lu is now acquiring images.\n", Camera->UID);
									Camera->PauseCapture = false;
									return true;
							}
					} else
							return false;
				} else
						return false;
			}	else //
					return false;
		} else
				return false;
	} else
			return false;
}
// __________________________________________________________________________________________end



/* =============================================================================================
   Stop streaming from a camera.
   ========================================================================================== */
void CameraStop(tCamera* Camera) {

	printf("\nStopping the stream for camera with ID %lu.\n", Camera->UID);
	PvCommandRun(Camera->Handle, "AcquisitionAbort");
	PvCaptureEnd(Camera->Handle);

}
// _________________________________________________________________________________________end




/* =============================================================================================
   timestamp an image
   ========================================================================================== */
void timestamp(int i){

			
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
							strcpy(CAMERAS[i].TimeStamps[CAMERAS[i].BufferIndex], os.str().c_str());
		
}
// _________________________________________________________________________________________end



/*=============================================================================================
		return the even odd index
  ========================================================================================== */
int evenodd(){
		int i=0;
		if (!PAUSEPROGRAM && !TERMINATE){
				if(TICKCOUNT % 2 == 0){
								i=0;
				}else{
								i=1;
				}
		} 
		return i; 
}
// _________________________________________________________________________________________end



/*=============================================================================================
		which camera to snap from?
  ========================================================================================== */
int whichcamera(){

		int i=0;
		if (!PAUSEPROGRAM && !TERMINATE){
				int diff = CAMERAS[0].TicksPerSec - CAMERAS[1].TicksPerSec;
				//alternate btwn the two cameras for even/odd at first and all the rest get tacked on at the end for now
				if( diff == 0 || abs(diff) ==1){
								i=evenodd();
				}else{
						int gc = CAMERAS[0].TicksPerSec > CAMERAS[1].TicksPerSec ? 0 : 1; 
						if(TICKCOUNT < a_cad - abs(diff)) {
								i=evenodd();
						} else {
								i = gc;
						}
				}
		} 
		return i;
}
// _________________________________________________________________________________________end


/*=============================================================================================
  Check error flags before requeue and snap
  ========================================================================================== */
void checkerr(int i){
	
			//got rid of triggerflag and requeueCallFlag - so now this function isn't used
}
// _________________________________________________________________________________________end


/*=============================================================================================
  spawn threads for snapping/processing
  ========================================================================================== */
void spawn_thread(int x){

	if(!PAUSEPROGRAM && !TERMINATE) {

			//code heartbeat
			if(TICKCOUNT % a_cad == 0)
				heartbeat_code();


			//which camera snaps?
			int i = whichcamera();

			//spawn thread based on camera and idx
			int thread_err;
			pthread_attr_t attr;											//attribute object				
			pthread_attr_init(&attr);
			pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED); //create thread in detached state
			thread_err = pthread_create(&CAMERAS[i].thread[CAMERAS[i].idx], &attr, snap_thread, (void *)i);
			pthread_attr_destroy(&attr);
			if(thread_err){
					cout<<"didn't create thread for camera "<<CAMERAS[i].UID<<" frame "<<CAMERAS[i].idx<<endl;
					cout<<"Error: "<<thread_err<<endl;
			}
	
			//cout<<"Camera "<<CAMERAS[i].UID<<" frame "<<CAMERAS[i].idx<<endl;
			cout<<"..\n";
			//cout<<" \n"; //without this the transmission time is 50ms, wiht it its .05ms.. something is wrong

			//update thread and buffer indexes 
			CAMERAS[i].BufferIndex = CAMERAS[i].idx; //the active buffer = this thread buffer
			++CAMERAS[i].idx;
			if(CAMERAS[i].idx >= FRAMESCOUNT)
					CAMERAS[i].idx = 0;

			//update TICKCOUNT 
			TICKCOUNT++;
			if(TICKCOUNT >= a_cad)
					TICKCOUNT = 0;		
	}
}
// _________________________________________________________________________________________end


/*=============================================================================================
  spawned thread for snapping/processing
  ========================================================================================== */
void *snap_thread(void *cam){

 int i = *(int *)(&cam);


	if(!PAUSEPROGRAM && !TERMINATE) {

			//initalize variables & housekeeping
			tPvErr errCode;
			int count = 0;
			prog_c con;
			init_prog_c(con); 
			
			//housekeeping
			getTemp(&CAMERAS[i]);

			//start snap only if we're done waiting for another frame to return (but doesn't protect frame buffer)
			if(CAMERAS[i].Handle != NULL && !CAMERAS[i].PauseCapture && !CAMERAS[i].waitFlag) { 
					//snap on time? output to screen
					timeval t;
					//tester(0, t, i);

					//check error flags
					//checkerr(i);

					//requeue a frame & snap (if successful requeue) - then process (if no timeout & done waiting)
					CAMERAS[i].queueStatus = PvCaptureQueueFrame(CAMERAS[i].Handle,&(CAMERAS[i].Frames[CAMERAS[i].BufferIndex]),NULL);
					if(CAMERAS[i].queueStatus == ePvErrSuccess){ 
								//update flags
								CAMERAS[i].requeueCallFlag = false;
								CAMERAS[i].NewFlags[CAMERAS[i].BufferIndex] = true;
								CAMERAS[i].frameandqueueFlag = false;

								//trigger, wait and queue processing if successful
								timestamp(i);
								CAMERAS[i].snapcount++;	
								CAMERAS[i].waitFlag= true;
								if(PvCommandRun(CAMERAS[i].Handle, "FrameStartTriggerSoftware") != ePvErrSuccess){
											cout<<"Trigger Software Error: ";
											PrintError(errCode);
								} 
								if(con.c_timer)
											tester(1,t,0);
								errCode = PvCaptureWaitForFrameDone(CAMERAS[i].Handle,&(CAMERAS[i].Frames[CAMERAS[i].BufferIndex]),con.timeout1);
								handle_wait(i, errCode, con.timeout2); //checks and updates waitflag and errCode
								if(con.c_timer){
											cout<<CAMERAS[i].UID<<" "; 
											tester(2,t,0);
								}

								//Process: if done waiting, no timeout, successful frame & non-zero bitdepth
								//cout<<"image size: "<<CAMERAS[i].Frames[CAMERAS[i].BufferIndex].ImageSize<<endl;
								if(CAMERAS[i].waitFlag== false && errCode == ePvErrSuccess){
									if( CAMERAS[i].Frames[CAMERAS[i].BufferIndex].Status == ePvErrSuccess && CAMERAS[i].Frames[CAMERAS[i].BufferIndex].BitDepth != 0){ 
										if(con.c_timer)
											tester(1,t,0);
										Process(i, CAMERAS[i].BufferIndex);
										if(con.c_timer){
											cout<<"Processing "; //timer
											tester(2,t,0);
											cout<<"\n\n";
										}
									} else{
										cout<<"CurrBuffer: "<<CAMERAS[i].BufferIndex<<endl;
										if(CAMERAS[i].Frames[CAMERAS[i].BufferIndex].Status != ePvErrSuccess){
											cout<<"unsuccessful frame\n\n";
											++CAMERAS[i].unsuccount;
										}
										if(CAMERAS[i].Frames[CAMERAS[i].BufferIndex].BitDepth ==0){
											cout<<"BitDepth: "<<CAMERAS[i].Frames[CAMERAS[i].BufferIndex].BitDepth<<"\n\n";
											++CAMERAS[i].zerobitcount;
										}
									}
								} else {
									cout<<"wait flag or timeout error\n\n";
								}

						} else {
								cout<<"PvCaptureQueueFrame err\n";
								queueErrorHandling(i);
						}//requeueframe


			} else { //handle&&pausecap&&flag
						cout<<CAMERAS[i].UID<<" handle, pausecap or flag error\n";
			} 
	}  else {//!pause&&terminate
					cout<<CAMERAS[i].UID<<" pause or terminate error\n";
			}
		//probably don't need this b/c its explicity created detached... test later
		pthread_detach(pthread_self());
		pthread_exit(NULL); 
}
// _________________________________________________________________________________________end




/*=============================================================================================
  Handle timeout and waitforframedone
  ========================================================================================== */
void handle_wait(int i, tPvErr &errCode, int timeout2){

		if (errCode == ePvErrSuccess){ //if returns in time, release flag 
				CAMERAS[i].waitFlag= false;
				//++CAMERAS[i].compcount;
		} else if (errCode == ePvErrTimeout) { //if timeout try again with less time
				//cout<<"Timeout 1.\n";
				CAMERAS[i].to1++;
				//try again
				errCode = PvCaptureWaitForFrameDone(CAMERAS[i].Handle,&(CAMERAS[i].Frames[CAMERAS[i].BufferIndex]), timeout2);
				if (errCode == ePvErrSuccess){  //if returns in time, release flag 
							CAMERAS[i].waitFlag= false;
							//++CAMERAS[i].compcount;
				} else if (errCode == ePvErrTimeout) { //if still timeout, assume trigerror and restart acquisition stream, don't process
							handle_timeout(i);
							CAMERAS[i].waitFlag= false;
				}
		} else {
				cout<<"Unknown return code for WaitForFrameDone. Releasing camera.\n";
				CAMERAS[i].waitFlag= false;
		}
}
// _________________________________________________________________________________________end


/*=============================================================================================
		error handling for timeout - 1st time just abort frame and restart acq, next restart imcap
  ========================================================================================== */
void handle_timeout(int i){
	
	//abort frame and clear queue each timeout
	//for every nth timeout restart imagecapture stream - not sure if this actually makes a difference
	CAMERAS[i].TimeoutCount++;

	if (CAMERAS[i].TimeoutCount % 10 == 0 ){ //make this a parameter on table!!!
			cout<<"10th timeout. Restart Image capture stream\n";
			RestartImCap(i);		
	} else {
			if(PvCaptureQueueClear(CAMERAS[i].Handle)==ePvErrSuccess) 
						printf("Image aborted and frame buffer queue cleared. \n");
	}
}
// _________________________________________________________________________________________end



/*=============================================================================================
  Get image out of framebuffer, determine which type of processing to do
  completely done with camera framebuffer after this function
  ========================================================================================== */
void Process(int i, int CurrBuffer){
		

	++CAMERAS[i].pcount;

	valarray<unsigned char> imarr((unsigned char*)CAMERAS[i].Frames[CurrBuffer].ImageBuffer, CAMERAS[i].FrameHeight*CAMERAS[i].FrameWidth);	

	if(CAMERAS[i].UID == PY_cam_ID){
				//cout<<"ProcessPY\n";
				ProcessPY(i, imarr);
	} else if(CAMERAS[i].UID == H_cam_ID){
				//cout<<"ProcessH\n";
				ProcessH(i, imarr);
	} else {
				cout<<"Unkown UID. No processing.\n";
	} 		
	
}
// _________________________________________________________________________________________end




/* =============================================================================================
   Set-up structs/vars for analysis. Determine if live or test image. Save.
  ========================================================================================== */
void ProcessPY(int i, valarray<unsigned char> imarr) {


	//1. init stucts and variables
	prog_c con;
	init_prog_c(con);
	params val;						
	init_params(val, (int)CAMERAS[i].FrameWidth, (int)(CAMERAS[i].FrameHeight*CAMERAS[i].FrameWidth));
	//init_params(val, 449, 144129); //for small live frame
	info im;
	init_im(im);
	timeval t;

	//2. analyze live or test image?
	if(con.live){
		if(con.c_timer)
			tester(1,t,0);
		analyzePY(im, val, imarr); 
		if(con.c_timer){
			cout<<"Analysis ";
			tester(2,t,0);
		}
		if(con.diag)
			diagnostics(val, im);	
	} else {
		const char* filename1 = "~/GRASPcode/tstimages/tstim2.fits";		//sun is 330 pix
		//const char* filename1 = "~/GRASPcode/tstimages/dimsun1_960x1290_bw.fits";	//sun is 195 pixels
		//const char* filename1 = "~/GRASPcode/tstimages/dimsun1.fits";						//sun is		<80
		readfits(filename1, imarr, val.nel, val.width);
		if(con.c_timer)
			tester(1,t,0);
		analyzePY(im, val, imarr);
		if(con.c_timer){
			cout<<"Test Im Analysis ";
			tester(2,t,0);
		}
		if(con.diag)
			diagnostics(val, im);
	}

	//drawline at centers? This will be saved in the image below
	if(val.drawline){
		drawline(imarr, val, im);
	} 

	//3. save?
	if(CAMERAS[i].WantToSave) {
		//create filename					
		ostringstream filename;
		//filename << "images/" << CAMERAS[i].UID << "_" << CAMERAS[i].TimeStamps[j]<<"_"<< CAMERAS[i].savecount<<".fits";	
		filename << "images/" << CAMERAS[i].UID << "_" << CAMERAS[i].BufferIndex<<".fits"; //circular filename buffer
		if(con.c_timer)
			tester(1,t,0);										
		saveim(i, imarr, filename.str().c_str(), im, val);
		CAMERAS[i].savecount++;
		if(con.c_timer){
			cout<<"Saving ";		
			tester(2,t,0);
			}
		filename.seekp(0);
	 }

	//	pthread_detach(pthread_self());
	//	pthread_exit(NULL);
}
// __________________________________________________________________________________________end





/* =============================================================================================
   Process Horizon Sensor Data
			Needs: some diagnostic to show that the horizon sensor is operating correctly
   ========================================================================================== */
void ProcessH(int i, valarray<unsigned char> imarr){

	//1. init stucts and variables
	prog_c con;
	init_prog_c(con);		
	params_H val;				
	init_params_H(val, (int)CAMERAS[i].FrameWidth, (int)(CAMERAS[i].FrameHeight*CAMERAS[i].FrameWidth));
 	info_H im;
	init_H(im);
	timeval t;

	//2. analyze live or test image?
	if(con.live){
		if(con.c_timer)
			tester(1,t,0);
		//analyzeH(im, val, imarr); 
		if(con.c_timer){
			cout<<"Analysis ";
			tester(2,t,0);
		}
		if(con.diag)
			diag_H(val, im);	
	} else {
		//const char* filename1 = "~/GRASPcode/tstimages/dimsun2.fits";							
		//readfits(filename1, imarr, val.nel, val.width);
		if(con.c_timer)
			tester(1,t,0);
		analyzeH(im, val, imarr);
		if(con.c_timer){
			cout<<"Analysis ";
			tester(2,t,0);
		}
		if(con.diag)
			diag_H(val, im);
	} 


	//3. save image
	if(CAMERAS[i].WantToSave) {
		//create filename					
		ostringstream filename;
		CAMERAS[i].savecount++;
		//filename << "images/" << CAMERAS[i].UID << "_" << CAMERAS[i].TimeStamps[j]
		//									<<"_"<< CAMERAS[i].savecount<<".fits";	
		filename << "images/" << CAMERAS[i].UID << "_" << CAMERAS[i].BufferIndex<<".fits";
		if(con.c_timer)
				tester(1,t,0);
		saveim_H(i, imarr, filename.str().c_str(), im, val); //set-up as bool, return true if ok, error msg if not
		if(con.c_timer){
				cout<<"Saving ";		
				tester(2,t,0);
		}
		filename.seekp(0);	
	 }


	//	pthread_detach(pthread_self());
	//	pthread_exit(NULL);
}
// __________________________________________________________________________________________end





/* =============================================================================================
   Saves a fits file
   ========================================================================================== */
bool saveim(int i, valarray<unsigned char> imarr, const char* filename, info im, params val){ 
	using namespace CCfits;
	using std::valarray;

	FITS::setVerboseMode(true);

	//create pointer to fits object
	std::auto_ptr<FITS> pFits(0);
	//std::valarray<unsigned char> imarr((char*)CAMERAS[i].Frames[j].ImageBuffer, nelements);

	//for running loop tests
	remove(filename);

	try{


		//create new fits file 
		try{
        	pFits.reset( new FITS(filename , BYTE_IMG , 0 , 0 ) ); //reset=deallocate object and reset its value. change to BYTE_IMG?								
		}catch (FITS::CantCreate){
         	std::cout<<"Couldn't create FITS file. The file might already exist. \n";
          	return false;       
		} 


		//append keys to the primary header
		//long exposure(1500);
		pFits->pHDU().addKey("Camera",(long)CAMERAS[i].UID, "Camera UID");
		pFits->pHDU().addKey("Save Count",(int)CAMERAS[i].savecount, "Num of Saved Images");
		pFits->pHDU().addKey("Snap Count",(int)CAMERAS[i].snapcount, "Num of SNAPS");
		pFits->pHDU().addKey("Exposure",(long)CAMERAS[i].ExposureLength, "for cameanalyzera");
		pFits->pHDU().addKey("filename", filename, "Name of the file");

		//switch to make the appropriate ascii tables for PY or H from their info structs
		//pFits->pHDU().addKey("xp", im.xp, "x coordinates");
		//pFits->pHDU().addKey("yp", im.yp, "y coordinates");
		//pFits->pHDU().addKey("thresh", im.thresh, "Thresholds");
		//pFits->pHDU().addKey("Time Stamp",CAMERAS[i].TimeStamps[j], "prog timestamp"); //gives error

		
    }catch (FitsException&){ 
     // will catch all exceptions thrown by CCfits, including errors
     // found by cfitsio (status != 0).     
        std::cerr << " Fits Exception Thrown.  \n";               
    }

	//store data in an extension HDU
	//its customary to have compressed images in 1st Ext
	ExtHDU* imageExt;
	std::vector<long> extAx;
	extAx.push_back(val.width);
	extAx.push_back(val.nel/val.width);

    string newName ("Raw Image");
	long  fpixel(1);
				 
	try{
		imageExt = pFits->addImage(newName,BYTE_IMG,extAx, 1);
		imageExt->write(fpixel,val.nel,imarr);	//write extension
		//std::cout << pFits->pHDU() << std::endl;												
	}catch (FitsException){
		std::cout<<"Couldn't write image to extension\n";
	}

	return true;
}
// __________________________________________________________________________________________end



/* =============================================================================================
   Saves a fits file
   ========================================================================================== */
bool saveim_H(int i, valarray<unsigned char> imarr, const char* filename, info_H im, params_H val){ 
	using namespace CCfits;
	using std::valarray;

	FITS::setVerboseMode(true);

	//create pointer to fits object
	std::auto_ptr<FITS> pFits(0);
	//std::valarray<unsigned char> imarr((char*)CAMERAS[i].Frames[j].ImageBuffer, nelements);

	//for running loop tests
	remove(filename);
	//std::cout<<"here\n";

	try{

		//create new fits file 
		try{
       		pFits.reset( new FITS(filename , BYTE_IMG , 0 , 0 ) ); //reset=deallocate object and reset its value. change to BYTE_IMG?								
		} catch (FITS::CantCreate){
          	std::cout<<"Couldn't create FITS file. The file might already exist. \n";
          	return false;       
		} 

		//append keys to the primary header
		//long exposure(1500);
		pFits->pHDU().addKey("Camera",(long)CAMERAS[i].UID, "Camera UID");
		pFits->pHDU().addKey("Save Count",(int)CAMERAS[i].savecount, "Num of Saved Images");
		pFits->pHDU().addKey("Snap Count",(int)CAMERAS[i].snapcount, "Num of SNAPS");
		pFits->pHDU().addKey("Exposure",(long)CAMERAS[i].ExposureLength, "for cameanalyzera");
		pFits->pHDU().addKey("filename", filename, "Name of the file");
		
    }catch (FitsException&){      
        std::cerr << " Fits Exception Thrown.  \n";               
    }

		//store data in an extension HDU
		//its customary to have compressed images in 1st Ext
		ExtHDU* imageExt;
		std::vector<long> extAx;
		extAx.push_back(val.width);
		extAx.push_back(val.nel/val.width);
    	string newName ("Raw Image");
		long  fpixel(1);
			
		 
		try{
			imageExt = pFits->addImage(newName,BYTE_IMG,extAx, 1);
			imageExt->write(fpixel,val.nel,imarr);	//write extension
			//std::cout << pFits->pHDU() << std::endl;													
		}catch (FitsException){
			std::cout<<"Couldn't write image to extension\n";
		}

	return true;
}
// __________________________________________________________________________________________end




/*==============================================================================================
	Test fuction for diagnostics
	========================================================================================= */
void tester(int x, timeval& t1, int i){

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
	switch (x){
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
void FrameStats(tCamera* Camera){

	unsigned long Ncomp=0;		//Num of frames acquired
	unsigned long Ndrop=0;		//Num of frames unsuccessfully acquired
	unsigned long Nerr=0;		//Num of erraneous packets
	unsigned long Nmiss=0;		//Num packets sent by camera not received by host
	unsigned long Nrec=0;		//Num packets sent by camera and received by host	
	unsigned long Nreq=0;		//Num of missing packets requested by camera for resend
	unsigned long Nres=0;		//Num of missing packets resent by camera and receieved by host

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
int readfits(const char* filename, valarray<unsigned char>& contents, int &nelements, int &width){
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
/*	cout<<"Max pixel value: "<<(int)contents.max()<<endl;
	cout<<"Min pixel value: "<<(int)contents.min()<<endl;*/

 // this doesn't print the data, just header info.
	//std::cout << image << std::endl;
	//this prints out elements from the image array
	//for(int i = 0; i < nelements+1; i+=nelements/height) std::cout<< "Fits "<<i<<" : "<< contents[i]<<"		";
	
	
	return 0;

}
// __________________________________________________________________________________________end

/*=============================================================================================
  Called by the timer interrupt 
  Triggers image acquisition and timestamps the image.
  ========================================================================================== */
void CameraSnap2(int x) {

if(!PAUSEPROGRAM && !TERMINATE) {

	prog_c con;
	init_prog_c(con);

		int i = whichcamera();
		if(con.temp)
				getTemp(&CAMERAS[i]);

			if(CAMERAS[i].Handle != NULL && !CAMERAS[i].PauseCapture) {

				//diagnostic to see if we snap on time
				timeval t;
				//timeval tt;
				//tester(1, tt, 0);
				tester(0, t, i);
				//cout<<"tester ";
				//tester(2, tt, 0);

				//check error flags
				checkerr(i);
				
				//requeue a frame buffer and set the CB
				/*if(CAMERAS[i].UID == PY_cam_ID){				
									CAMERAS[i].queueStatus=PvCaptureQueueFrame(CAMERAS[i].Handle,
									&(CAMERAS[i].Frames[CAMERAS[i].BufferIndex]),
									FrameDone0); //Place an image buffer onto the queue frame and send to the 0th callback
				}else if (CAMERAS[i].UID == H_cam_ID){
									CAMERAS[i].queueStatus=PvCaptureQueueFrame(CAMERAS[i].Handle,
									&(CAMERAS[i].Frames[CAMERAS[i].BufferIndex]),
									FrameDone1); //Place an image buffer onto the queue frame and send to 1st callback
				}else{				
									cout<<"requeue error: unknown camera UID\n";
				} */
				CAMERAS[i].requeueCallFlag=false; //what if it doesn't come back?? then the requeue flag is still set false...

				//Snap picture if requeue success
				if(CAMERAS[i].queueStatus == ePvErrSuccess){
					CAMERAS[i].NewFlags[CAMERAS[i].BufferIndex] = true;
					CAMERAS[i].frameandqueueFlag = false;

					//trigger command and set flags
					CAMERAS[i].triggerFlag= true;
					//cout<<"Frame queued successfully. Trigger SNAP. \n"; //only print if frame queue isn't successful
					timestamp(i);
					PvCommandRun(CAMERAS[i].Handle, "FrameStartTriggerSoftware"); //Software trigger to SNAP			
					CAMERAS[i].triggerFlag= false;
					CAMERAS[i].snapcount++;


				//if the queue isn't successful, print error messages
				}else if(CAMERAS[i].queueStatus == ePvErrUnplugged){
					cout<<"Queue frame returns Camera is unplugged. No Snap. \n";
					CAMERAS[i].frameandqueueFlag= false;
					
				}else if(CAMERAS[i].queueStatus != ePvErrSuccess){
					cout<<"Failed to requeue frame. No Snap. \n";
					queueErrorHandling(i);
				}else{
					cout<<"Queue return status is unknown!!!!!.\n";
					CAMERAS[i].frameandqueueFlag=false;
				}
			
				//Change active buffer for a camera
				CAMERAS[i].BufferIndex++;
				if (CAMERAS[i].BufferIndex >= FRAMESCOUNT)
					CAMERAS[i].BufferIndex = 0;
				//timestamp(i,1);
				//os.seekp(0);			// Rewind os so we overwrite value next time
				//imageIndex.seekp(0);	// Rewind for the same reason as above
			}else{
				//Diagnostics
				printf("skipped the SNAP if statement because: "); 
				if(CAMERAS[i].Handle == NULL)
					printf("CAMERAS[i].Handle == NULL \n");
				if(TICKCOUNT % CAMERAS[i].TicksUntilCapture != 0)
					printf("Remainder of TICKCOUNT & CAMERAS[i].TicksUntilCapture != 0 \n");
				if(CAMERAS[i].PauseCapture)
					printf("CAMERAS[i].PauseCapture == true \n");
				if(CAMERAS[i].NewFlags[CAMERAS[i].BufferIndex])
					printf("CAMERAS[i].NewFlags[CAMERAS[i].BufferIndex] == true \n");
			}
		
		//advance and reset TICKCOUNT 
		TICKCOUNT++;
		if(TICKCOUNT >= a_cad)
			TICKCOUNT = 0;
	}

	
}
// __________________________________________________________________________________________end





/*==============================================================================================
	Error Handling for queue frame
==============================================================================================*/
void queueErrorHandling(int i){

	PrintError(CAMERAS[i].queueStatus);
	if(PvCaptureQueueClear(CAMERAS[i].Handle)==ePvErrSuccess) //clear buffer if we have any issues queueing frames
				printf("Frame buffer queue cleared. \n");

	//if the requeue results in error 3 times, restart image capture
	//infinite requeue errors were seen in testing. Restarting image capture fixes the problem.
	if(CAMERAS[i].snapcount == CAMERAS[i].requeueFlag + 1){ 
		CAMERAS[i].queueError++;
		cout<<"Successive queue Failure: "<<CAMERAS[i].queueError+1<<"\n\n";
	}else{
		CAMERAS[i].queueError = 0;	
	}
	if(CAMERAS[i].queueError >= 2){
		cout<<"Restarting Image Stream: successive requeue errors.\n";
		CAMERAS[i].queueErrors++;
		RestartImCap(i);
		CAMERAS[i].queueError = 0;
	}
	CAMERAS[i].requeueFlag = CAMERAS[i].snapcount;


	//if a frame returns error 16 followed by a frame requeue failure, restart Image Capture
	//This pattern of frame and queue errors was seen to preceed system crashes in testing.
/*	if(CAMERAS[i].frameandqueueFlag){
		cout<<"Restarting Image Stream: frame and queue errors\n";
		RestartImCap(i);
		CAMERAS[i].frameandqueueFlag=false;
		CAMERAS[i].frameandqueueErrors++;
	} */
}
//______________________________________________________________________________________________


/* =============================================================================================
   Unsetup the camera.
   From AVT GigE SDK example named Stream.
   ========================================================================================== */
void CameraUnsetup(tCamera* Camera) {

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



/* =============================================================================================
   CTRL-C handler for changing parameters, checking stats and restarting streams
   ========================================================================================== */
void CtrlCHandler(int Signo) {

	if(CAMERASFOUND) {

		PAUSEPROGRAM = true;
		DisplayParameters();

		// Print Camera Statistics
		for(unsigned int i = 0; i < NUMOFCAMERAS; i++){			
			cout<<"Performance Statistics for Camera "<<CAMERAS[i].UID<<"\n";
			FrameStats(&CAMERAS[i]);
			cout<<"\nTimeout Count: "<<CAMERAS[i].TimeoutCount<<"\n";
			cout<<"t1 count: "<<CAMERAS[i].to1<<endl;
			cout<<"Frames Processed: "<<CAMERAS[i].pcount<<endl;
			cout<<"Frames completed but unsuccessful: "<<CAMERAS[i].unsuccount<<endl;
			cout<<"Frames completed with zero BitDepth: "<<CAMERAS[i].zerobitcount<<endl;			
			cout<<"Successive Queue failures: "<<CAMERAS[i].queueErrors<<"\n";
			cout<<"Frame and Queue errors: "<<CAMERAS[i].frameandqueueErrors<<"\n\n";
		}

		// Options
		printf("Would you like to end the program? (y/n): ");
		if(YesOrNo()) {

			for(unsigned int i = 0; i < NUMOFCAMERAS; i++) {
				CAMERAS[i].PauseCapture = true;
				CAMERAS[i].WantToSave = false;
				CameraStop(&CAMERAS[i]);
			}
			TERMINATE = true;

		} else {

			for(unsigned int j = 0; j < NUMOFCAMERAS; j++) {

				printf("Dealing with settings for camera with ID %lu\n", CAMERAS[j].UID);
				printf("----------\n");

				printf("Would you like to change the pause option? (y/n): ");
				if(YesOrNo()) {
					if(CAMERAS[j].PauseCapture)
						CAMERAS[j].PauseCapture = false;
					else
						CAMERAS[j].PauseCapture = true;
				}

				printf("Would you like to change the save option? (y/n): ");
				if(YesOrNo()) {
					if(CAMERAS[j].WantToSave)
						CAMERAS[j].WantToSave = false;
					else
						CAMERAS[j].WantToSave = true;
				}

				printf("Would you like to change some parameters? (y/n): ");
				//This doesn't work right now
				if(YesOrNo()) {
					CAMERAS[j].PauseCapture = true;
					CameraStop(&CAMERAS[j]);
					CameraUnsetup(&CAMERAS[j]);
					printf("Restarting camera with ID %lu\n", CAMERAS[j].UID);
					CameraSetup(&CAMERAS[j], j);
					CameraStart(&CAMERAS[j]);
				}
			}
		}

		PAUSEPROGRAM = false;

	} else {
		printf("Would you like to stop looking for cameras and end the program? (y/n): ");
		if(YesOrNo()) {
			TERMINATE = true;
		}
	}
}
// __________________________________________________________________________________________end



/*===========================================================================================
	Restart Acquisition
=============================================================================================*/
void RestartAcq(int j){

	//pause program
	PAUSEPROGRAM= true;

	if(PvCommandRun(CAMERAS[j].Handle, "AcquisitionAbort")==ePvErrSuccess){
		cout<<"Acquisition Stopped.\n";
	} else {
		cout<<"Couldn't stop acquisition.\n";
	}
	if(PvCaptureQueueClear(CAMERAS[j].Handle)==ePvErrSuccess)
		printf("Frame buffer queue cleared. \n");		
	if(PvCommandRun(CAMERAS[j].Handle, "AcquisitionStart")==ePvErrSuccess)
		cout<<"Acquisition Started. \n\n";

	//restart program
	PAUSEPROGRAM=false;
}
//____________________________________________________________________________________________



/*============================================================================================
	Restart the image capture stream
==============================================================================================*/
void RestartImCap(int j){

	//pause program
	PAUSEPROGRAM = true;

	if(PvCaptureQueueClear(CAMERAS[j].Handle)==ePvErrSuccess)
		printf("Frame buffer queue cleared. \n");
	if(PvCommandRun(CAMERAS[j].Handle, "AcquisitionAbort")==ePvErrSuccess){
		cout<<"Acquisition Stopped.\n";
	}else{
		cout<<"Couldn't stop Acquisition.\n";
	}
	//if(PvCaptureQueueClear(CAMERAS[j].Handle)==ePvErrSuccess)
	//	printf("Frame buffer queue cleared. \n");
	if(PvCaptureEnd(CAMERAS[j].Handle)==ePvErrSuccess)
		printf("Image capture stream terminated. \n");
	if(PvCaptureStart(CAMERAS[j].Handle)==ePvErrSuccess)
		printf("Image capture stream restarted. \n");
	if(PvCommandRun(CAMERAS[j].Handle, "AcquisitionStart")==ePvErrSuccess)
		cout<<"Acquisition Started. \n\n";

	//restart Program
	PAUSEPROGRAM=false;
}
//______________________________________________________________________________________________



/* =============================================================================================
   Displays the parameters chosen for the camera.
   ========================================================================================== */
void DisplayParameters() {

	printf("\n");
	for(unsigned int i = 0; i < NUMOFCAMERAS; i++) {
		printf("Displaying settings for camera with ID %lu\n", CAMERAS[i].UID);
		printf("----------\n");
		printf("Images per second: %d\n", CAMERAS[i].TicksPerSec);
		printf("Exposure time in microseconds: %lu\n", CAMERAS[i].ExposureLength);
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
void PrintError(int errCode) {

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


