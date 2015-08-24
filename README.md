GRASP
=====

GRips ASPect code 

Dependencies
------------
### `cfitsio`
```
./configure --enable-reentrant --prefix=/usr/local
make
make install
```

### `CCfits`
```
export CXXFLAGS="$CXXFLAGS -D_REENTRANT=1"
export LIBS="$LIBS -lpthread"
./configure --with-cfitsio=/usr/local
make
make install
```

### AVT PvAPI
The `ARCH` file assumes that the SDK can be found in a directory called `AVT GigE SDK` at the same level as the repository directory.  The actual library, `libPvAPI.so`, may need to be copied into the repository directory.

Installation
------------
The main program makes use of the GRIPS network code (https://github.com/GRIPS/network) as a git submodule.  When first cloning this repository, you need to initialize this submodule:

    git submodule init

Then, to actually update the submodule, you need to call:

    git submodule update

You should not actually edit any of the files in the `network` subdirectory, because they belong to the other repository.

Execution
---------
`main` needs to be run with root privileges.  `main -?` shows available command-line options.
