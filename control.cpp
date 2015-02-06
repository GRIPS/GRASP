

#include "control.h" 


// Standard libs
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <cstdlib>
#include <sys/time.h>
#include <fstream>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>
using namespace std;


/* =============================================================================================
			Initialize the info struct
   ========================================================================================== */
bool init_prog_c(prog_c& con){

	//program control
	//con.live = true; 					 	//change this to run in testing mode
	con.live = false;
	//con.diag = true;					 		//print diagnostics for analyzed frames
	con.diag = false;	
	//con.c_timer = true;					//elapsed time for analysis
	con.c_timer = false;
	//con.temp = true;								//query camera for temperature
	con.temp = false;
	con.trig_out = true;				//output signal from camera when exposing
	con.def = true;									//use default settings

	con.timeout1 = 5;						//this should go in parameter table
	con.timeout2 = 60;						//probably don't need 2 timeouts

	return true;

}
// __________________________________________________________________________________________end


