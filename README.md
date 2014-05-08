GRASP
=====

GRips ASPect code 


Installation
------------
The main program makes use of the GRIPS network code (https://github.com/GRIPS/network) as a git submodule.  When first cloning this repository, you need to initialize this submodule:

    git submodule init

Then, to actually update the submodule, you need to call:

    git submodule update

You should not actually edit any of the files in the `network` subdirectory, because they belong to the other repository.
