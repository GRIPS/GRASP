
#ifndef ___a_H___
#define ___a_H___


#include <valarray>
using namespace std;


/* =============================================================================================
   Define structures
   ========================================================================================== */


struct info_H {						//info are data products from analysis

		int vals[6];							//crude check for saturation, look @ range. top pixel, top-reject, 3/4,  mid, 1/4,  min


};



struct params_H {				//parameters control the program and set variables

		int width;									//width of the image 
		int nel;											//nelements

		int reject;							//for sorting/crude dyanmic range check
		int comp;									//compression for saving
};

//______________________________________________________________________________________________





/* =============================================================================================
   Function declarations
   ========================================================================================== */

bool analyzeH(info_H& im,  params_H val, valarray<unsigned char> imarr);
bool sort_H(info_H& im, params_H val, valarray<unsigned char> imarr);
bool init_params_H(params_H& val, int width, int nel);
bool init_H(info_H im);
bool diag_H(params_H val, info_H im);


#endif
