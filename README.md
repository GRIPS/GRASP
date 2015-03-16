GRASP
=====

GRips ASPect code 

Dependencies
------------
When setting up this code on different computers

0. Download and install AVT PVAPI, CCfits and cfitsio. 

1. Include the AVT PvAPI ARCH file in the location of the GRASP. This file is not inlcuded in the repository, copy from the AVT download and modify it to fit the filepaths to AVT.

2. The way CCfits and cfitsio is set-up on computers may differ, change the includes in the Makefile, not the ARCH file

3. ld linker had some issues finding the libCCfits.so.0 so its copied into the GRASP working directory locally, but not in the repository

Installation
------------
The main program makes use of the GRIPS network code (https://github.com/GRIPS/network) as a git submodule.  When first cloning this repository, you need to initialize this submodule:

    git submodule init

Then, to actually update the submodule, you need to call:

    git submodule update

You should not actually edit any of the files in the `network` subdirectory, because they belong to the other repository.

