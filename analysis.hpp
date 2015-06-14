#ifndef _ANALYSIS_HPP_
#define _ANALYSIS_HPP_

#include <valarray>
using namespace std;

/* =============================================================================================
   Define structures
   ========================================================================================== */
struct info            //info are data products
{
    float histogram[256];

    // Pitch-yaw image analysis
    bool there[3];                                    //true = its there
    //bool wsun[3];                                     //true = too close to edge. change to esun if needed
    float xp[3], yp[3];             //pixel coordinates
    float xs[3], ys[3];                //solar coordinates
    unsigned char thresh[3];
    int ngt[3];                                                //npixels gt thresh --> need to add to mask_centroid

    //double theta;                            //orientation of relative roll
    //float w;                                            //the rotation rate of grid (using previous theta)
    //int param_tbl;         //which parameter table is loaded
    //int            snap;                        //snap number, should be on the file name anyway
    //char? time;                        //what format? need to get from the original fits file & control program. h m s us all diff vars?

    // Roll image analysis
    float mean[3]; // left/middle/right strips
    float stdev[3]; // left/middle/right strips
};

struct params        //parameters control the program and define variables
{
    unsigned int width;
    unsigned int height;

    //analysis control
    int Rs;                                                //pixel radius of the sun
    int ns;                    //number of suns we're to find the centroids of - moved over from info
    int min;                //min number of pixels for a sun to be considered "there" in the frame
    int ix;                        //iterator for chords along x. num_chords = height/ix
    int iy;                        //iterator for chords along y. num_chords = width/iy
    int comp;                //how much to compress the sorting by (4 = use every 4th pixel)
    int reject;        //to account for saturated pixels in find_thresh
    float th[3];    //multiplier to set threshold from highest pixel value
    int box;                    //pixel size of box for cropping/blacking out sun

    bool a_timer;         //timers for analysis
    bool savecrop;        //save the cropped solar image

    bool drawline;        //analysis tool, draws a line through the solar centers
};

//______________________________________________________________________________________________


/* =============================================================================================
   Function declarations
   ========================================================================================== */

bool init_params(params& val, unsigned int w_in, unsigned int h_in);

bool analyzePY(info &im, params val, valarray<unsigned char> &imarr);
bool Find_3_mask(valarray<unsigned char> &imarr, params& val, info& im);
bool find_thresh(valarray<unsigned char> &imarr, unsigned char thresh[], params val);
bool crop(valarray<unsigned char>& imarr, const char* fn,float x, float y, params val);
void centroid(valarray<unsigned short>& mask, params val, float& xloc, float& yloc, bool& there);
void drawline(valarray<unsigned char>& imarr, params val, info im);

bool analyzeR(info &im, params val, valarray<unsigned char> &imarr);

void reportPY(params val, info im);
void reportR(params val, info im);

void timetest(int x, timeval& t1, int i);
//bool savefits(valarray<unsigned char>& newarr, const char* filename, int nelements, int width);

#endif
