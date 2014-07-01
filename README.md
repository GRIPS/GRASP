GRASP
=====

GRips ASPect code 

When setting up this code on different computers
1. Include the AVT PvAPI ARCH file in the location of the GRASP
2. The ARCH file must be edited for the computer architecture. Default is x64.
3. The ARCH file in the depository has a default hack which omits -ltiff
4. The way CCfits and cfitsio is set-up on computers may differ, change the includes in the Makefile, not the ARCH file
5. ld linker had some issues finding the libCCfits.so.0 so its copied into the GRASP working directory locally, but not in the repository

