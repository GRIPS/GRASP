
/* =============================================================================================

   ========================================================================================== */
#include "analysis.hpp"

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
#include <opencv.hpp>

#include "main.hpp"
#include "camera.hpp"

using namespace std;

struct XYZ {
    XYZ(int _x = 0, int _y = 0, float _z = 0);

    int x, y;
    float z;

    friend bool operator<(XYZ& a, XYZ &b);
};

XYZ::XYZ(int _x, int _y, float _z) : x(_x), y(_y), z(_z)
{
}

bool operator<(XYZ& a, XYZ &b)
{
    return a.z < b.z;
}

/* =============================================================================================
   initialize the parameters and load a parameter table
   ========================================================================================== */
bool init_params(params& val, unsigned int w_in, unsigned int h_in)
{
    val.width = w_in;
    val.height = h_in;

    val.a_timer=false;

    //change these to load from parameter table
    val.Rs= 100;
    val.ns = 3;                                         //number of suns to find, finds the brightest --> dimmest
    val.min = 50;                                         //min number of pixels for a sun to "be there"
    val.reject = 100;
    val.th[0] = .6;
    val.th[1] = .4;
    val.th[2] = .2;
    //val.box = 80;                //for dimsun
    val.box = 350;                    //for tstim & tstim2
    //val.box = 210;                        //for 960x1290_bw

    /*val.ix = 1;                        //full image
    val.iy = 1;
    val.comp =1;*/
    /*val.ix = 14;                //for timetests
    val.iy = 14;
    val.comp = 4; */
    val.ix = 20;                        //limits of test image
    val.iy = 20;
    val.comp = 50;

    val.drawline = false;

    return true;
}
// __________________________________________________________________________________________end


/* =============================================================================================
   The main function to call for analysis of the PY images
   follows the code outline in a7.cpp
   ========================================================================================== */
bool analyzePY(info& im, params val, valarray<unsigned char> &imarr)
{
    //find solar centers
    //Find_3_mask(imarr, val, im);
    fancy_stuff(imarr, val, im);

    //find fiducials

    //coordinate transform to match-up with sun sensor

    return true;
}
// __________________________________________________________________________________________end


/* =============================================================================================
   Find the local peaks (by brute force) in a float cv::Mat
   ========================================================================================== */
list<XYZ> find_local_peaks(cv::Mat float_array, float threshold)
{
    list<XYZ> peaks;

    for(int i = 1; i < float_array.rows - 1; i++) {
        for(int j = 1; j < float_array.cols - 1; j++) {
            float z = float_array.at<float>(i, j);
            if((z > threshold) &&
               (float_array.at<float>(i - 1, j - 1) < z) &&
               (float_array.at<float>(i - 1, j    ) < z) &&
               (float_array.at<float>(i - 1, j + 1) < z) &&
               (float_array.at<float>(i    , j - 1) < z) &&
               (float_array.at<float>(i    , j + 1) < z) &&
               (float_array.at<float>(i + 1, j - 1) < z) &&
               (float_array.at<float>(i + 1, j    ) < z) &&
               (float_array.at<float>(i + 1, j + 1) < z)) {
                peaks.push_back(XYZ(j, i, z));
            }
        }
    }

    return peaks;
}
// __________________________________________________________________________________________end


/* =============================================================================================
   Using OpenCV to process images via convolution
   ========================================================================================== */
bool fancy_stuff(valarray<unsigned char> imarr, params& val, info& im)
{
    static bool templates_initialized = false;
    static cv::Mat template_sun_fine(141, 141, CV_32FC1);
    static cv::Mat template_sun_coarse(15, 15, CV_32FC1);
    static cv::Mat template_fiducial(11, 11, CV_32FC1);

    // Generate the templates if they haven't already been generated
    if(!templates_initialized) {
        for(int i = -70; i <= 70; i++) {
            for(int j = -70; j <= 70; j++) {
                float temp = 1 - cosh(sqrt(i * i + j * j) / 10) / 200;
                template_sun_fine.at<float>(i + 70, j + 70) = MAX(temp, 0);
            }
        }
        for(int i = -7; i <= 7; i++) {
            for(int j = -7; j <= 7; j++) {
                float temp = 1 - cosh(sqrt(i * i + j * j)) / 200;
                template_sun_coarse.at<float>(i + 7, j + 7) = MAX(temp, 0);
            }
        }
        for(int i = -5; i <= 5; i++) {
            for(int j = -5; j <= 5; j++) {
                float temp = 1 - cosh(sqrt(i * i + j * j) * 1.25) / 200;
                template_fiducial.at<float>(i + 5, j + 5) = MAX(temp, 0);
            }
        }

        templates_initialized = true;
    }

    cv::Mat full_frame(val.height, val.width, CV_8UC1, &imarr[0]);

    // calculate histogram
    cv::Mat hist;
    int size[] = {256};
    float range[] = {0, 256};
    const float *ranges[] = {range};
    cv::calcHist(&full_frame, 1, //just one matrix
                 0,       //just the first channel
                 cv::Mat(), //no mask
                 hist,      //output histogram
                 1, size,   //number of dimensions and bins
                 ranges,  //range of histogram
                 true,      //uniform?
                 false);    //accumulate?

    // Normalize histogram
    for(int bin = 0; bin < 255; bin++) {
        im.histogram[bin] = hist.at<float>(bin) / (val.width * val.height);
    }

    cv::Mat coarse_frame(val.height / 10, val.width / 10, CV_8UC1);
    for(int i = 0; i < coarse_frame.rows; i++) {
        for(int j = 0; j < coarse_frame.cols; j++) {
            coarse_frame.at<unsigned char>(i, j) = full_frame.at<unsigned char>(i * 10 + 5, j * 10 + 5);
        }
    }

    cv::Mat correlation(coarse_frame.rows, coarse_frame.cols, CV_32FC1);
    cv::filter2D(coarse_frame, correlation, CV_32F, template_sun_coarse);
    list<XYZ> coarse_suns = find_local_peaks(correlation, 1000); // FIXME: magic number!

    if(!coarse_suns.empty()) {
        // Trim down to the top three matches ordered by decreasing intensity
        coarse_suns.sort();
        coarse_suns.reverse();
        if(coarse_suns.size() > 3) coarse_suns.resize(3);

        int index = 0;
        // Step through each Sun
        for(list<XYZ>::iterator it = coarse_suns.begin(); it != coarse_suns.end(); ++it) {
            // Crop the full frame to around the Sun
            int anchor_x = (*it).x * 10 + 5 - 80;
            int anchor_y = (*it).y * 10 + 5 - 80;
            cv::Mat sub_frame(full_frame,
                              cv::Range(anchor_y, anchor_y + 161),
                              cv::Range(anchor_x, anchor_x + 161));

            // Perform the correlation with the Sun template
            cv::Mat sub_correlation(161, 161, CV_32FC1);
            cv::filter2D(sub_frame, sub_correlation, CV_32F, template_sun_fine);

            // Find the center of the Sun
            double minVal, maxVal;
            cv::Point minLoc, maxLoc;
            cv::minMaxLoc(sub_correlation, &minVal, &maxVal, &minLoc, &maxLoc);
            // TODO: sub-pixel positioning?
            im.there[index] = true;
            im.xp[index] = anchor_x + maxLoc.x;
            im.yp[index] = anchor_y + maxLoc.y;
            index++;

            // Crop even further
            anchor_x = (*it).x * 10 + 5 - 40;
            anchor_y = (*it).y * 10 + 5 - 40;
            cv::Mat subsub_frame(full_frame,
                                 cv::Range(anchor_y, anchor_y + 81),
                                 cv::Range(anchor_x, anchor_x + 81));
            cv::Mat adjusted_frame(81, 81, CV_8UC1);
            cv::subtract(255, subsub_frame, adjusted_frame);

            // Perform the correlation with the fiducial template
            cv::Mat subsub_correlation(81, 81, CV_32FC1);
            cv::filter2D(adjusted_frame, subsub_correlation, CV_32F, template_fiducial);
            cv::minMaxLoc(subsub_correlation, &minVal, &maxVal, &minLoc, &maxLoc);
            //cout << minVal << " " << minLoc << " " << maxVal << " " << maxLoc << endl;

            // Find the fiducials
            list<XYZ> this_fiducials = find_local_peaks(subsub_correlation, 0); // FIXME: magic number!
            if(!this_fiducials.empty()) {
                // Keep the four strongest matches
                this_fiducials.sort();
                this_fiducials.reverse();
                if(this_fiducials.size() > 4) this_fiducials.resize(4);

                // Store the fiducial location in the 
                for(list<XYZ>::iterator it2 = this_fiducials.begin(); it2 != this_fiducials.end(); ++it2) {
                    im.xfid[im.nfid] = anchor_x + (*it2).x;
                    im.yfid[im.nfid] = anchor_y + (*it2).y;
                    //cout << "(" << im.xfid[im.nfid] << "," << im.yfid[im.nfid] << ")\n";
                    im.nfid++;
                }
            }
        }
    }

    // Calculate the grid orientation based on the two brightest Suns
    if(im.there[1]) {
        im.theta = atan2(im.yp[1] - im.yp[0], im.xp[1] - im.xp[0]) * 180 / 3.14159265358979;
    }

    return true;
}
// __________________________________________________________________________________________end


/* =============================================================================================
   Method 1: use centroiding to find all 3 suns
   From a single image containing 3 suns at different intensities,
   This program finds each of their xy locations using the mask_centoid function

   This is probably more robust, but slower than Method 2
   ========================================================================================== */

bool Find_3_mask(valarray<unsigned char> imarr, params& val, info& im)
{
    unsigned int watch;
    //get thresholds
    timeval t;
    if(val.a_timer)
        timetest(1,t,0);
    if(MODE_TIMING) stopwatch(watch);
    find_thresh(imarr, im.thresh, val);
    if(MODE_TIMING) cout << "|find_thresh took " << stopwatch(watch) << " us\n";
    if(val.a_timer) {
        cout<<"find_thresh ";
        timetest(2,t,0);
    }

    //operate on images
    for(int i=0; i<val.ns; i++) {
        //threshold and mask
        if(val.a_timer)
            timetest(1,t,0);
        if(MODE_TIMING) stopwatch(watch);
        valarray<unsigned short> mask(val.width * val.height);

        //mask entire imarr
        for (unsigned int p = 0; p < val.width * val.height; p++) {
            mask[p] = ((imarr[p] >= im.thresh[i] ) ? 1 : 0);
            if(i == 0) im.histogram[imarr[p]] += 1; //accumulate histogram
        }
        if(MODE_TIMING) cout << "||mask took " << stopwatch(watch) << " us\n";
        //mask only the rows we use in centroiding - possibly didn't implement correctly. ran 10x slower
        /*for(int n=0; n<val.height;n+=val.ic){
            for (int p = n*val.width; p < (n+1)*val.width; p++){
                mask[p] = ((imarr[p] >= im.thresh[i] ) ? 255 : 0);
            }
        }
        for(int m=0; m<val.width;m+=val.ic){
            for (int p = m; p < (m+1)*height; p++){     //this rewrites where we've already compared with x
                mask[p] = ((imarr[p] >= im.thresh[i] ) ? 255 : 0);
            }
        } */

        if(val.a_timer) {
            cout<<"d_mask ";
            timetest(2,t,0);
        }

        //centroid the mask
        if(MODE_TIMING) stopwatch(watch);
        centroid(mask, val, im.xp[i], im.yp[i], im.there[i]);
        if(MODE_TIMING) cout << "|||centroid took " << stopwatch(watch) << " us\n";
        //~mask;

        //crop and black out the current sun from the mask
        if(im.there[i] == true) {
            if(MODE_TIMING) stopwatch(watch);
            const char* fnc= "!dimsun_crop.fits";  //right now this calls all of them the same name, need to dynamically assign
            crop(imarr, fnc, im.xp[i], im.yp[i], val); //crops and blacks out sun > thresh
            if(MODE_TIMING) cout << "||||crop took " << stopwatch(watch) << " us\n";
        }
    }

    //normalize histogram
    for(int bin = 0; bin < 255; bin++) {
        im.histogram[bin] /= val.width * val.height;
    }

    //other function options to add in
    //drawline(imarr, val.nel, width, x, y);  //draws a line at centroid location
    //const char* fn= "!dimsun1_imarr1.fits";
    //savefits(imarr, fn, val.nel, width);      //good diagnostic to check if the suns are blacked out

    return true;
}
// __________________________________________________________________________________________end



/* =============================================================================================
   Find Thresholds
   ========================================================================================== */
bool find_thresh(valarray<unsigned char> &imarr, unsigned char thresh[], params val)
{
    int subset = val.width * val.height / val.comp;
    //for a single sun should be .25*max, since we have 3 suns, thresh must be >2nd brightest sun

    //sort
    //cout<<"Sorting times: ";
    /*timetest(1,t,0);
    sort(&imarr[0], &imarr[val.nel-1]); //sorts the values of the image array in ascending order
    cout<<"imarr ";
    timetest(2,t,0);
    //timetest(2); */

    //sort comppressed array
    valarray<unsigned char> cim(imarr[slice(0, subset, val.comp)]);
    sort(&cim[0], &cim[subset-1]);

    /*thresh[0]=th1*(int)imarr[val.nel-reject];
    thresh[1]=th2*(int)imarr[val.nel-reject];
    thresh[2]=th3*(int)imarr[val.nel-reject];*/

    //use the compressed thresh
    thresh[0]=val.th[0]*(int)cim[subset-val.reject];
    thresh[1]=val.th[1]*(int)cim[subset-val.reject];
    thresh[2]=val.th[2]*(int)cim[subset-val.reject];

    //check thresholds are similar
/*    cout<<"th = "<<th1*(int)imarr[val.nel-reject]<<" , "<<th2*(int)imarr[val.nel-reject]<<" , "<<th3*(int)imarr[val.nel-reject]<<endl;
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
void timetest(int x, timeval& t1, int i)
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
void centroid(valarray<unsigned short>& mask, params val, float& xloc, float& yloc, bool& there)
{
    unsigned long sumy = 0;
    unsigned long sumx = 0;
    unsigned long weighted_sumx =0; //unsigned long ok so long as sun diameter = 330 and the mask is 0 & 1 only
    unsigned long weighted_sumy =0;

    timeval t;
    if(val.a_timer)
        timetest(1,t,0);

    //strips along y
    for(unsigned int m = 0; m < val.width; m += val.iy){
        valarray<unsigned short> stripx(mask[slice(m, val.height, val.width)]);
        weighted_sumx += stripx.sum()*m;
        sumx += stripx.sum();
        //~stripx;
    }

    //strips along x
    for(unsigned int n = 0; n < val.height; n += val.ix){
        valarray<unsigned short> stripy(mask[slice(n*val.width, val.width, 1)]);
        weighted_sumy += stripy.sum()*n;
        sumy += stripy.sum();
        //~stripy;
    }

    //strips along y
/*    for(int m=0; m<val.width;m+=val.iy){
        valarray<unsigned short> stripx(mask[slice(m, val.height, val.width)]);
        weighted_sumx += stripx.sum()*m;
        sumx += stripx.sum();
        //~stripx;
    } */

    if(val.a_timer){
        cout<<"summing ";
        timetest(2,t,0);
    }

    //~mask;
/*    cout<<"wx: "<<weighted_sumx<<endl;
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
bool crop(valarray<unsigned char>& imarr, const char* fn,  float x, float y, params val)
{
    //This code assumes x & y are the approximate center coordinates of the box
    int bx = x - val.box/2;
    int by = y - val.box/2;

    //adjust for cropped area extending outside of the image area
    if(bx<0) bx=0;
    if(bx > (val.width - val.box)) bx = (val.width - val.box);
    if(by<0) by=0;
    if(by > (val.height - val.box)) by = (val.height - val.box);
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
            //~sliced;
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
   Print out pitch-yaw analysis results
   ========================================================================================== */
void reportPY(params val, info im)
{
    cout<<"width: "<<val.width<<endl;
    cout<<"height: "<<val.height<<endl;
    //cout<<"image struct:\n";
    cout<<"xp: "<<im.xp[0]<<" ,"<<im.xp[1]<<" , "<<im.xp[2]<<endl;
    cout<<"yp: "<<im.yp[0]<<" ,"<<im.yp[1]<<" , "<<im.yp[2]<<endl;
    cout<<"thresh: "<<(int)im.thresh[0]<<" ,"<<(int)im.thresh[1]<<" , "<<(int)im.thresh[2]<<endl;
    cout<<"there: "<<im.there[0]<<" ,"<<im.there[1]<<" , "<<im.there[2]<<endl;
    //cout<<"wsun: "<<im.wsun[0]<<" ,"<<im.wsun[1]<<" , "<<im.wsun[2]<<endl;
    //cout<<"theta: "<<im.theta<<endl;
    cout<<"\n";
}
// __________________________________________________________________________________________end


/* =============================================================================================
   Draw line in image where the xy location is
   ========================================================================================== */
void drawline(valarray<unsigned char>& imarr, params val, info im)
{
    slice slicex;
    slice slicey;
    for(int i =0; i<val.ns;i++){
        slicex = slice (im.yp[i]*val.width,val.width,1);
        slicey = slice (im.xp[i], val.height, val.width);
        imarr[slicex] = 255;
        imarr[slicey] = 255;
    }
}
// __________________________________________________________________________________________end


/* =============================================================================================
   The main function to call for analysis of roll images
   Currently, it calculates the mean and standard deviation of three patches of the image
   ========================================================================================== */
bool analyzeR(info &im, params val, valarray<unsigned char> &imarr)
{
    unsigned int subheight = 100;
    unsigned int subwidth = val.width / 10;
    unsigned long total[3] = {0, 0, 0};
    unsigned long total_squares[3] = {0, 0, 0};

    unsigned long index = val.width * (val.height - subheight) / 2;
    for(unsigned int row = 0; row < subheight; row++) {
        for(unsigned int col = 0; col < val.width; col++) {
            im.histogram[imarr[index]] += 1;
            if(col < subwidth) {
                total[0] += imarr[index];
                total_squares[0] += imarr[index] * imarr[index];
            } else if(col > val.width - subwidth) {
                total[2] += imarr[index];
                total_squares[2] += imarr[index] * imarr[index];
            } else if(col >= (val.width - subwidth) / 2 && col < (val.width + subwidth) / 2) {
                total[1] += imarr[index];
                total_squares[1] += imarr[index] * imarr[index];
            }
            index++;
        }
    }

    //mormalize histogram
    for(unsigned int i = 0; i < 256; i++) {
        im.histogram[i] /= subheight * val.width;
    }

    unsigned long n = subheight * subwidth;
    for(unsigned int i = 0; i < 3; i++) {
        im.mean[i] = total[i] / (float)n;
        im.stdev[i] = sqrt((total_squares[i] - n * im.mean[i] * im.mean[i]) / (n - 1));
    }

    im.good_contrast = (im.mean[0] - 3 * im.stdev[0]) + (im.mean[2] - 3 * im.stdev[2]) > 2 * (im.mean[1] + 3 * im.stdev[1]);
    im.good_black_level = (im.mean[1] + 3 * im.stdev[1]) < 16;

    return true;
}
// __________________________________________________________________________________________end


/* =============================================================================================
   Print out roll analysis results
   ========================================================================================== */
void reportR(params val, info im)
{
    cout << val.width << "x" << val.height << endl;
    cout<<"mean: "<<im.mean[0]<<", "<<im.mean[1]<<" , "<<im.mean[2]<<endl;
    cout<<"stdev: "<<im.stdev[0]<<", "<<im.stdev[1]<<" , "<<im.stdev[2]<<endl;
    cout<<"contrast is "<< (im.good_contrast ? "GOOD" : "BAD") <<endl;
    cout<<"black is "<< (im.good_black_level ? "GOOD" : "BAD") <<endl;
    cout<<"\n";
}
// __________________________________________________________________________________________end
