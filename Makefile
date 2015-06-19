include ARCH

#ASPECT makefile settings
CCfitslib = /usr/local/lib
CCfits=/usr/local/include/CCfits
cfitsio=/usr/local/include
#cfitsiolib=~/cfitsio

#Used for implicit compiling of C++ files
CXXFLAGS = -Inetwork -Idmm -Wall -pthread -I$(INC_DIR) $(FLAGS) -I$(CCfits)

default: main

all: program main

#Variables for Nicole's executable
EXE = grasp
SOURCES = a_PY.cpp $(EXE).cpp a_H.cpp control.cpp oeb.o network/*.o

clean:
	make -C network clean
	rm -f $(EXE)
	rm -f *.o
	rm -f main quick

program:  $(SOURCES)
	$(CC)  $(RPATH) $(TARGET) $(CFLAGS) $(SOURCES) -g -o $(EXE) $(SOLIB) $(PVLIB)  -I$(CCfits) -I$(cfitsio) -I$(cfitsiolib) -L$(CCfitslib) -lCCfits $(IMLIB)

install:
	cp -f $(EXE) $(BIN_DIR)

quick: quick.o
	$(CC) -o $@ $^ network/*.o -pthread

dmm.o: dmm/dmm.c dmm/dmm.h
	$(CC) $(CXXFLAGS) -c $<

main: main.o oeb.o dmm.o camera.o analysis.o Image.o settings.o
	make -C network all
	$(CC) -o $@ $^ network/*.o -pthread -L$(CCfitslib) -lCCfits $(PVLIB) $(SOLIB) $(RPATH)
