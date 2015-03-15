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

#Used for implicit compiling of C++ files
CXXFLAGS = -Inetwork -Idmm -Wall

all: program

# Executable, Change this for new code names
EXE = grasp
EXE_pb = grasp_pb
#EXE = a_rel0
#EXE = a7
#EXE= fitsplay

#include all .cpp files here
#SOURCES = analysis_PY.cpp $(EXE).cpp
SOURCES = a_PY.cpp $(EXE).cpp a_H.cpp control.cpp oeb.o
SOURCES_pb = a_PY.cpp $(EXE_pb).cpp a_H.cpp control.cpp oeb.o

clean:
	rm -f $(EXE)
	rm -f *.o
	rm -f main fake_tm

grasp_pb: $(SOURCES)
	$(CC)  $(RPATH) $(TARGET) $(CFLAGS) $(SOURCES_pb) -g -o $(EXE_pb) $(SOLIB) $(PVLIB)  -I$(CCfits) -I$(cfitsio) -I$(cfitsiolib) -L$(CCfitslib) -lCCfits $(IMLIB) 

program:  $(SOURCES)
	$(CC)  $(RPATH) $(TARGET) $(CFLAGS) $(SOURCES) -g -o $(EXE) $(SOLIB) $(PVLIB)  -I$(CCfits) -I$(cfitsio) -I$(cfitsiolib) -L$(CCfitslib) -lCCfits $(IMLIB) 

install:
	cp -f $(EXE) $(BIN_DIR) 

dmm.o: dmm/dmm.c dmm/dmm.h
	$(CC) $(CXXFLAGS) -c $<

main: main.o oeb.o dmm.o
	make -C network all
	$(CC) -o main main.o oeb.o -Inetwork -Idmm network/*.o dmm.o -lpthread

main_fake_tm.o: main.cpp
	$(CC) -c -o main_fake_tm.o main.cpp -DFAKE_TM $(CXXFLAGS)

fake_tm: main_fake_tm.o oeb.o dmm.o
	make -C network all
	$(CC) -o fake_tm -DFAKE_TM main_fake_tm.o oeb.o -Inetwork -Idmm network/*.o dmm.o -lpthread
