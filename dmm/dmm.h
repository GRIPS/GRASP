#include <unistd.h>
#include <stdint.h>
#include <pthread.h>
#include <stdio.h>
#include <sys/io.h>

#define N_AIN_CHANNELS 32
#define N_DIN_CHANNELS 31
#define N_AOUT_CHANNELS 4

#define BIT_0 1
#define BIT_1 2
#define BIT_2 4
#define BIT_3 8
#define BIT_4 16
#define BIT_5 32
#define BIT_6 64
#define BIT_7 128

#define PORT_A 0
#define PORT_B 1
#define PORT_C 2

#define DMMWAITTIME       50
#define DMMMAXWAITCOUNT   20

#define DMMPULSEDURATION_SMALL   100
#define DMMPULSEDURATION_MEDIUM  1000
#define DMMPULSEDURATION_LARGE   10000

#define MAX32BIT 4294967296
#define MAX16BIT 65535

struct dmminfo{
	unsigned char id,init;
	uint32_t base;	
	
	/* 0->port A, 1->port B, 2->port C */
	unsigned char dio_port[3];
	unsigned char dio_port_pulse_bits[3];
	
	//these two numbers define the range of channels for the A/D conversion
	//i.e. ADCLowChannel = 0 and AD	CHighChannel = 3 activates the first 4 channels for the conversion
	//it is important to activate only the channels that are not floating... if a channel is floating,
	//it can give weird values for the conversions on other channels.
	unsigned char ADCLowChannel;
	unsigned char ADCHighChannel;
	
	short int ain[32];
	short int aout[N_AOUT_CHANNELS];
	
	/* DMMMutex is used to control hardware access and data in this structure. */
	pthread_mutex_t DMMMutex;
};

void DMMSetup(struct dmminfo * DMM, uint32_t DMMBase);
void DMMSetDIOByte(struct dmminfo * DMM,uint8_t port,uint8_t byte);
int DMMLockMutex(struct dmminfo * DMM);
int DMMUnlockMutex(struct dmminfo * DMM);
void DMMUpdateDIO(struct dmminfo * DMM);
void DMMUpdateADC(struct dmminfo * DMM);
void DMMUpdateAll(struct dmminfo * DMM);
void DMMUpdateDIOPort(struct dmminfo * DMM, uint8_t port);
void DMMSetDIOBit(struct dmminfo * DMM,uint8_t port,uint8_t whichbit,uint8_t onoroff);
void DMMPulseDIOBit(struct dmminfo * DMM,uint8_t port,uint8_t whichbit,uint32_t pulse_duration);
void SendWord(struct dmminfo *  DMM, uint32_t dac, uint32_t reg, uint32_t data, uint32_t delay);
void DMMEnableVetoCount(struct dmminfo * DMM);
void DMMEnableDeadTimeCount(struct dmminfo * DMM);
void DMMConfigureCounters(struct dmminfo * DMM);
void DMMStartCounters(struct dmminfo * DMM);
void DMMLatchCounters(struct dmminfo * DMM);
uint32_t DMMReadoutCounters(struct dmminfo * DMM,uint32_t cnt);
int InitDMM1(void);
int32_t DMMSetADCLowChannel(struct dmminfo * DMM, uint8_t ch);
int32_t DMMSetADCHighChannel(struct dmminfo * DMM, uint8_t ch);

