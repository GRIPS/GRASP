

#include "a_H.h" 

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
bool init_H(info_H im){

		for(int i=0; i<6; i++)
					im.vals[i]=0;

		return true;
}
// __________________________________________________________________________________________end



/* =============================================================================================
			Initialize parameter struct 
   ========================================================================================== */
bool init_params_H(params_H& val, int width, int nel){

	val.width = width;									
	val.nel = nel;											

	val.reject = 100;
	val.comp = 2;

	return true;
	
}
// __________________________________________________________________________________________end



/* =============================================================================================
			The main function to call for analysis of the H images
   ========================================================================================== */
bool analyzeH(info_H& im,  params_H val, valarray<unsigned char> imarr){

	//are we going to do any analysis? or just checks?
	
	//be able to downlink whole images or part of images on command
	//downlink a check to make sure we're reasonably still operating
	//AYS says that they weren't reasonably sure it was working until he saw clouds. Saturation is still white/black/white
	//we need detail in the horizon
	//implement a check for if saturated? 

	//sort and return the nth pixel?


	return true;
}
// __________________________________________________________________________________________end


/* =============================================================================================
			crude check
   ========================================================================================== */
bool sort_H(info_H& im, params_H val, valarray<unsigned char> imarr){

/*	valarray<unsigned char> cim(imarr[slice(0, (val.nel/val.comp), val.comp)]); 
	sort(&cim[0], &cim[(val.nel/val.comp)-1]);

	im.vals[0]=cim[(val.nel/val.comp)-1];	//max
	im.vals[1]=cim[(val.nel/val.comp)-val.reject];	//max brightness
	im.vals[2]=cim[(int)(val.nel/val.comp)(3/4.0)];
	im.vals[3]=cim[(int)(val.nel/val.comp)(2/4.0)];
	im.vals[4]=cim[(int)(val.nel/val.comp)(1/4.0)];
	im.vals[4]=cim[reject];
 */

	return true;


}
// __________________________________________________________________________________________end


bool diag_H(params_H val, info_H im){


	return true;

}






