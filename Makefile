include ARCH

#CCfits = /usr/local/include/CCfits
CCfitslib = /usr/local/lib
#CCfitslib=/home/grips/desktop
CCfits=/home/grips/CCfits
cfitsio=/home/grips/cfitsio


# Modification of Makefile found in GRASPv0 folder.
# Including the CCfits and cfitsio directories  

all: program

# Executable, Change this for new code names
EXE = dv29
#EXE = a_rel0
#EXE = a7
#EXE= fitsplay

#include all .cpp files here
#SOURCES = analysis_PY.cpp $(EXE).cpp
SOURCES = a_PY.cpp $(EXE).cpp a_H.cpp control.cpp

clean:
	rm $(EXE)

program:  $(SOURCES)
	$(CC)  $(RPATH) $(TARGET) $(CFLAGS) $(SOURCES) -g -o $(EXE) $(SOLIB) $(PVLIB)  -I$(CCfits) -I$(cfitsio) -L$(CCfitslib) -lCCfits $(IMLIB) 

install:
	cp -f $(EXE) $(BIN_DIR) 

main: main.cpp
	make -C network all
	$(CC) -o main main.cpp -Inetwork network/*.o

