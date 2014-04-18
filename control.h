
/*----------------------------------------------------------------------------------------------
		These functions and structs control the operation of the program and cameras
		--------------------------------------------------------------------------------------------*/


#ifndef ___control___
#define ___control___


#include <valarray>
using namespace std;


/* =============================================================================================
   Define structures
   ========================================================================================== */

struct prog_c	{

		bool live;						//analyze live images or test images?
		//int thresh_strat;	//strategy for finding threshold
		bool diag; 					//print diagnostics for image processing
		bool c_timer;			//print timers for control
		bool temp;						//get temperature of camera 
		bool trig_out;		//output from SyncOut1
		bool def;							//use default camera settings

		int timeout1;			//initial timeout for return frame
		int timeout2;   //secondary timeout for return frame

};
//______________________________________________________________________________________________





/* =============================================================================================
   Function declarations
   ========================================================================================== */

bool init_prog_c(prog_c& con);



#endif
