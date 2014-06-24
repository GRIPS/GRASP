include ARCH

#CCfits = /usr/local/include/CCfits

#ASPECT makefile settings
CCfitslib = /usr/local/lib
CCfits=/usr/local/include/CCfits
cfitsio=/usr/local/include
cfitsiolib=~/CCfits

#GRASI makefile settings
#CCfits=/home/grips/CCfits
#cfitsio=/home/grips/cfitsio

all: program

# Executable, Change this for new code names
EXE = grasp
#EXE = a_rel0
#EXE = a7
#EXE= fitsplay

#include all .cpp files here
#SOURCES = analysis_PY.cpp $(EXE).cpp
SOURCES = a_PY.cpp $(EXE).cpp a_H.cpp control.cpp

clean:
	rm $(EXE)

program:  $(SOURCES)
	$(CC)  $(RPATH) $(TARGET) $(CFLAGS) $(SOURCES) -g -o $(EXE) $(SOLIB) $(PVLIB)  -I$(CCfits) -I$(cfitsio) -I$(cfitsiolib) -L$(CCfitslib) -lCCfits $(IMLIB) 

install:
	cp -f $(EXE) $(BIN_DIR) 


