include ARCH

#CCfits = /usr/local/include/CCfits

#ASPECT makefile settings
CCfitslib = /usr/local/lib
CCfits=/usr/local/include/CCfits
cfitsio=/usr/local/include
#cfitsiolib=~/cfitsio

#GRASI makefile settings
#CCfits=/home/grips/CCfits
#cfitsio=/home/grips/cfitsio

all: program

# Executable, Change this for new code names
EXE = grasp
EXE_pb = grasp_pb
#EXE = a_rel0
#EXE = a7
#EXE= fitsplay

#include all .cpp files here
#SOURCES = analysis_PY.cpp $(EXE).cpp
SOURCES = a_PY.cpp $(EXE).cpp a_H.cpp control.cpp
SOURCES_pb = a_PY.cpp $(EXE_pb).cpp a_H.cpp control.cpp

clean:
	rm $(EXE)

grasp_pb: $(SOURCES)
	$(CC)  $(RPATH) $(TARGET) $(CFLAGS) $(SOURCES_pb) -g -o $(EXE_pb) $(SOLIB) $(PVLIB)  -I$(CCfits) -I$(cfitsio) -I$(cfitsiolib) -L$(CCfitslib) -lCCfits $(IMLIB) 

program:  $(SOURCES)
	$(CC)  $(RPATH) $(TARGET) $(CFLAGS) $(SOURCES) -g -o $(EXE) $(SOLIB) $(PVLIB)  -I$(CCfits) -I$(cfitsio) -I$(cfitsiolib) -L$(CCfitslib) -lCCfits $(IMLIB) 

install:
	cp -f $(EXE) $(BIN_DIR) 

main: main.cpp
	make -C network all
	$(CC) -o main main.cpp -Inetwork network/*.o -lpthread

fake_tm: main.cpp
	make -C network all
	$(CC) -o fake_tm -DFAKE_TM main.cpp -Inetwork network/*.o -lpthread
