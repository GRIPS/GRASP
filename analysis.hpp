#ifndef _ANALYSIS_HPP_
#define _ANALYSIS_HPP_

#include <valarray>
using namespace std;

/* =============================================================================================
   Define structures
   ========================================================================================== */
struct info            //info are data products
{


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

};

struct params        //parameters control the program and define variables
{
    int width;                                    //width of the image
    int nel;                                            //nelements

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

struct info_H                        //info are data products from analysis
{
    int vals[6];                            //crude check for saturation, look @ range. top pixel, top-reject, 3/4,  mid, 1/4,  min
};

struct params_H                //parameters control the program and set variables
{
    int width;                                    //width of the image
    int nel;                                            //nelements
    int reject;                            //for sorting/crude dyanmic range check
    int comp;                                    //compression for saving
};

//______________________________________________________________________________________________


/* =============================================================================================
   Function declarations
   ========================================================================================== */
bool Find_3_mask(valarray<unsigned char> &imarr, params& val, info& im);
bool find_thresh(valarray<unsigned char> &imarr, unsigned char thresh[], params val);
bool crop(valarray<unsigned char>& imarr, const char* fn,float x, float y, params val);
void timetest(int x, timeval& t1, int i);
bool init_im(info& im); //changed from a7
bool init_params(params& val, int w_in, int n_in); //changed from a7
bool analyzePY(info& im,  params val, valarray<unsigned char> &imarr);
void diagnostics(params val, info im);
bool savefits(valarray<unsigned char>& newarr, const char* filename, int nelements, int width);
void centroid(valarray<unsigned short>& mask, params val, float& xloc, float& yloc, bool& there);
void drawline(valarray<unsigned char>& imarr, params val, info im);

bool analyzeH(info_H& im,  params_H val, valarray<unsigned char> &imarr);
bool sort_H(info_H& im, params_H val, valarray<unsigned char> &imarr);
bool init_params_H(params_H& val, int width, int nel);
bool init_H(info_H im);
bool diag_H(params_H val, info_H im);

#endif
