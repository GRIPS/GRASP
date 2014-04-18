
/* =============================================================================================

			analysis_PY_int_2: code returns error for negative values of the xp or yp. using analysis_PY_int.h

			analysis_PY_int: first integrated version of the analysis code (a7) into the control program	

   ========================================================================================== */



#include "a_PY.h" 

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
			initialize the image structure
   ========================================================================================== */
bool init_im(info& im){

	for(int i=0;i<3;i++){
		//im.there[i]=false;									//true = its there
		//im.wsun[i]=false; 									//true = its a whole sun
		im.xp[i]=0; 			//pixel coordinates
		im.yp[i]=0;
		im.xs[i]=0;			//solar coordinates
		im.ys[i]=0;
		im.thresh[i]=0;
		im.ngt[i]=0;												//npixels gt thresh --> need to add to mask_centroid
	}
	//im.theta=0;							//orientation of relative roll
	//im.w=0;

	return true;
}
// __________________________________________________________________________________________end


/* =============================================================================================
			initialize the parameters and load a parameter table
   ========================================================================================== */
bool init_params(params& val, int w_in, int n_in){

	val.width=w_in;  
	val.nel=n_in;
	
	//change these to load from parameter table
	val.Rs= 100;
	//val.ns = 3; 										//number of suns to find, finds the brightest --> dimmest
	val.ns = 1;
	val.min = 50;									 	//min number of pixels for a sun to "be there"
	val.reject = 100;
	val.th[0] = .6;
	val.th[1] = .4;
	val.th[2] = .2;
	//val.box = 80;				//for dimsun
	val.box = 350;					//for tstim & tstim2
	//val.box = 210;						//for 960x1290_bw

	/*val.ix = 1;						//full image
	val.iy = 1;
	val.comp =1;*/ 
	/*val.ix = 14;				//for timetests
	val.iy = 14;
	val.comp = 4; */
	val.ix = 20;						//limits of test image
	val.iy = 20;
	val.comp = 50;


	//val.drawline=true;
	val.drawline = false;

	return true;
}
// __________________________________________________________________________________________end



/* =============================================================================================
			The main function to call for analysis of the PY images
			follows the code outline in a7.cpp
   ========================================================================================== */
bool analyzePY(info& im,  params val, valarray<unsigned char> imarr){

	//find solar centers
	Find_3_mask(imarr, val, im); 

	//find fiducials

	//coordinate transform to match-up with sun sensor

	return true;
}
// __________________________________________________________________________________________end


/* =============================================================================================
			Method 1: use centroiding to find all 3 suns			
			From a single image containing 3 suns at different intensities,
			This program finds each of their xy locations using the mask_centoid function

			This is probably more robust, but slower than Method 2
   ========================================================================================== */

bool Find_3_mask(valarray<unsigned char> imarr, params& val, info& im){

	//get thresholds
	timeval t;
	if(val.a_timer)
				timetest(1,t,0);
	find_thresh(imarr, im.thresh, val);
	if(val.a_timer){	
				cout<<"find_thresh ";
				timetest(2,t,0);
	}

	//operate on images
	for(int i=0; i<val.ns; i++){


			//threshold and mask
			if(val.a_timer)
						timetest(1,t,0);
			valarray<unsigned short> mask(val.nel);

			//mask entire imarr
			for (int p = 0; p < val.nel; p++){
						mask[p] = ((imarr[p] >= im.thresh[i] ) ? 1 : 0);
			}
			//mask only the rows we use in centroiding - possibly didn't implement correctly. ran 10x slower
		/*	int height = val.nel/val.width;
			for(int n=0; n<height;n+=val.ic){
						for (int p = n*val.width; p < (n+1)*val.width; p++){
								mask[p] = ((imarr[p] >= im.thresh[i] ) ? 255 : 0);
						}
			} 
			for(int m=0; m<val.width;m+=val.ic){

					for (int p = m; p < (m+1)*height; p++){     //this rewrites where we've already compared with x 
						mask[p] = ((imarr[p] >= im.thresh[i] ) ? 255 : 0);  
					}

			} */

			if(val.a_timer){
						cout<<"d_mask ";
						timetest(2,t,0);
			}
		
			//centroid the mask
				centroid(mask, val, im.xp[i], im.yp[i], im.there[i]);
				~mask;

			//crop and black out the current sun from the mask
			if(im.there[i] == true){
					const char* fnc= "!dimsun_crop.fits";  //right now this calls all of them the same name, need to dynamically assign
					crop(imarr, fnc, im.xp[i], im.yp[i], val); //crops and blacks out sun > thresh
			}
	}
	
			//other function options to add in
			//drawline(imarr, val.nel, width, x, y);  //draws a line at centroid location
			//const char* fn= "!dimsun1_imarr1.fits";
			//savefits(imarr, fn, val.nel, width);  	//good diagnostic to check if the suns are blacked out

	return true;
}
// __________________________________________________________________________________________end



/* =============================================================================================
			Find Thresholds
   ========================================================================================== */
bool find_thresh(valarray<unsigned char> imarr, unsigned char thresh[], params val){

	//for a single sun should be .25*max, since we have 3 suns, thresh must be >2nd brightest sun

	//sort
	//cout<<"Sorting times: ";
	/*timetest(1,t,0);
	sort(&imarr[0], &imarr[val.nel-1]); //sorts the values of the image array in ascending order
	cout<<"imarr ";
	timetest(2,t,0);
	//timetest(2); */

	//sort comppressed array
	valarray<unsigned char> cim(imarr[slice(0, (val.nel/val.comp), val.comp)]); 
	sort(&cim[0], &cim[(val.nel/val.comp)-1]);

	/*thresh[0]=th1*(int)imarr[val.nel-reject];
	thresh[1]=th2*(int)imarr[val.nel-reject];
	thresh[2]=th3*(int)imarr[val.nel-reject];*/

	//use the compressed thresh
	thresh[0]=val.th[0]*(int)cim[(val.nel/val.comp)-val.reject];
	thresh[1]=val.th[1]*(int)cim[(val.nel/val.comp)-val.reject];
	thresh[2]=val.th[2]*(int)cim[(val.nel/val.comp)-val.reject]; 

	//check thresholds are similar
/*	cout<<"th = "<<th1*(int)imarr[val.nel-reject]<<" , "<<th2*(int)imarr[val.nel-reject]<<" , "<<th3*(int)imarr[val.nel-reject]<<endl;
	//cout<<"c_th = "<<th1*(int)cim[(val.nel/2)-reject]<<" , "<<th2*(int)cim[(val.nel/2)-reject]<<" , "<<th3*(int)cim[(val.nel/2)-reject]<<endl;
	cout<<"c_th = "<<th1*(int)cim[(val.nel/4)-reject]<<" , "<<th2*(int)cim[(val.nel/4)-reject]<<" , "<<th3*(int)cim[(val.nel/4)-reject]<<endl;
	*/
	

	return true;
}
// __________________________________________________________________________________________end

/* =============================================================================================
   Time Testing different methods
			don't need this and timer from v1_dvx. But until I make and include those files here, I just have two of them
   ========================================================================================== */
void timetest(int x, timeval& t1, int i){

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
			cout<<"Time: "<<filename << highrestime.tv_usec<<"\n";
			break;
		}
		case 1 : {
			t1 = highrestime;
			break;
		}
		case 2 : {
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
		default: {
			break;
		}
	}
}
//_____________________________________________________________________________________________


/* =============================================================================================
			Mask image and get rough centroid
   ========================================================================================== */
void centroid(valarray<unsigned short>& mask, params val, float& xloc, float& yloc, bool& there){
	
	int height = val.nel/val.width;
	unsigned long sumy = 0;
	unsigned long sumx = 0;
	unsigned long weighted_sumx =0; //unsigned long ok so long as sun diameter = 330 and the mask is 0 & 1 only
	unsigned long weighted_sumy =0;



	timeval t;
	if(val.a_timer)
				timetest(1,t,0);

	//strips along y
	for(int m=0; m<val.width;m+=val.iy){
				valarray<unsigned short> stripx(mask[slice(m, height, val.width)]);
				weighted_sumx += stripx.sum()*m; 
				sumx += stripx.sum();
				//~stripx;
	}


	//strips along x
	for(int n=0; n<height;n+=val.ix){
				valarray<unsigned short> stripy(mask[slice(n*val.width, val.width, 1)]);
				weighted_sumy += stripy.sum()*n; 
				sumy += stripy.sum();
				//~stripy; 
	} 

	//strips along y
/*	for(int m=0; m<val.width;m+=val.iy){
				valarray<unsigned short> stripx(mask[slice(m, height, val.width)]);
				weighted_sumx += stripx.sum()*m; 
				sumx += stripx.sum();
				//~stripx;
	} */


	if(val.a_timer){
				cout<<"summing ";
				timetest(2,t,0);
	}

	~mask;
/*	cout<<"wx: "<<weighted_sumx<<endl;
	cout<<"wy: "<<weighted_sumy<<endl;	
	cout<<"sumx: "<<sumx<<endl;
	cout<<"sumy: "<<sumy<<endl;
	cout<<"nx: "<<nx<<endl;
	cout<<"ny: "<<ny<<endl; */
	xloc = (float)weighted_sumx/sumx;
	yloc = (float)weighted_sumy/sumy;


	//is the sun there?
	if((sumy/1) > val.min) 
			there = true;

}
// __________________________________________________________________________________________end


/* =============================================================================================
			Crop the image & black-out the cropped image in imarr

   ========================================================================================== */
bool crop(valarray<unsigned char>& imarr, const char* fn,  float x, float y, params val){

	//This code assumes x & y are the approximate center coordinates of the box
	int bx = x - val.box/2;
	int by = y - val.box/2;

	//adjust for cropped area extending outside of the image area
	if(bx<0) bx=0;  
	if(bx > (val.width - val.box)) bx = (val.width - val.box);
	if(by<0) by=0;
	if(by > (val.nel/val.width - val.box)) by = ((val.nel/val.width) - val.box);
	//cout<<"bx, by: "<<bx<<" , "<<by<<endl;

	//for blacking out & saving
	slice slicex;
	valarray<unsigned char> cropped(255, val.box*val.box); //initally set to white so we see problem 
	//size_t size = sizeof(unsigned short); //change this if we change int -> char
	size_t size = sizeof(unsigned char);
	valarray<unsigned char> sliced(val.box);


	for(int ny=0; ny<val.box;ny++){
			slicex = slice (((by+ny)*val.width+bx),val.box,1);

			//save cropped data
			if(val.savecrop){
					sliced = imarr[slicex];
					memcpy(&cropped[ny*val.box], &sliced[0] , val.box*size );
					~sliced;
			}

			//black out region that is cropped
			imarr[slicex]=0;
	} 
	
	if(val.savecrop){
				//savefits(cropped, fn, box*box, box); //saved cropped images //need saveim available here
	}

	return true;
}
// __________________________________________________________________________________________end


/* =============================================================================================
			Analysis results
   ========================================================================================== */
void diagnostics(params val, info im){
	//diagnostics
	cout<<"width: "<<val.width<<endl;
	cout<<"nel: "<<val.nel<<endl;
	//cout<<"image struct:\n";
	cout<<"xp: "<<im.xp[0]<<" ,"<<im.xp[1]<<" , "<<im.xp[2]<<endl;
	cout<<"yp: "<<im.yp[0]<<" ,"<<im.yp[1]<<" , "<<im.yp[2]<<endl;
	cout<<"thresh: "<<(int)im.thresh[0]<<" ,"<<(int)im.thresh[1]<<" , "<<(int)im.thresh[2]<<endl;
	cout<<"there: "<<im.there[0]<<" ,"<<im.there[1]<<" , "<<im.there[2]<<endl;
	//cout<<"wsun: "<<im.wsun[0]<<" ,"<<im.wsun[1]<<" , "<<im.wsun[2]<<endl;
	//cout<<"theta: "<<im.theta<<endl;
	cout<<"\n\n";
}
// __________________________________________________________________________________________end

/* =============================================================================================
			Draw line in image where the xy location is			
			========================================================================================== */
void drawline(valarray<unsigned char>& imarr, params val, info im){

	slice slicex;
	slice slicey;
	for(int i =0; i<val.ns;i++){
			slicex = slice (im.yp[i]*val.width,val.width,1);
			slicey = slice (im.xp[i], val.nel/val.width, val.width);
			imarr[slicex] = 255;
			imarr[slicey] = 255;
	}



}
// __________________________________________________________________________________________end


