#include "dmm.h"

extern struct dmminfo DMM1;

int32_t InitDMM1(void){
      
      //initializes the operating parameters of the DMM boards.  It is very important not to 
      //write to any of the digital ports here since this routine gets executed at startup,
      //and I don't think we would want to cycle the power to the card cages etc.
      
      if( pthread_mutex_init(&DMM1.DMMMutex,NULL) != 0 ){ //defaults to unlocked state
	    printf("DMMSetup: could not init DMM mutex.\n");
	    return -1;
      }
      
      //DMMLockMutex(&DMM1);
      
      DMM1.base = 0x340; //I/O address of DMM1 on the PC104 bus, set by jumpers on the DMM board
      
      /* Setup A/D channel range, voltage ranges, etc. */      
      DMMSetADCLowChannel(&DMM1,0);
      DMMSetADCHighChannel(&DMM1,31);  
      outb(0x28,DMM1.base+11);// sets all A/D channels to bipolar, +/- 10V range.
      
      /* Enable FIFO, reset FIFO contents, and enable scan mode */
      outb(0x0E,DMM1.base+7); 
      
      /* Sets the current page to 1, thus allowing for access to the DIO
       *      ports through addresses DMMBASE+12  -  DMMBASE+15 */
      outb(0x01,DMM1.base+8); 
      
      /* Sets ports A B and C as output ports under mode 0 (no handshaking) */
      outb(0x80,DMM1.base+15);
      
      /* Enable all bits on DIO ports for pulsing */
      DMM1.dio_port_pulse_bits[0] = 0xff;
      DMM1.dio_port_pulse_bits[1] = 0xff;
      DMM1.dio_port_pulse_bits[2] = 0xff;
      
      //read back current port values 
      DMMUpdateDIO(&DMM1);
      
      //Configure counters 
      DMMConfigureCounters(&DMM1);
      
      //Enable DMM1 to count shield box output
      DMMEnableVetoCount(&DMM1);
      
      // Mark DMM structure as initialized 
      DMM1.init = 1;
      
      //DMMUnlockMutex(&DMM1);
      return 0;
      
}

int32_t DMMSetADCLowChannel(struct dmminfo * DMM, uint8_t ch){
      
      //DMMLockMutex(DMM);
      DMM->ADCLowChannel = ch;
      outb(ch,DMM->base+2); // low channel       
      //DMMUnlockMutex(DMM);
      return 0;
      
}

int32_t DMMSetADCHighChannel(struct dmminfo * DMM, uint8_t ch){
      
      //DMMLockMutex(DMM);
      DMM->ADCHighChannel = ch;
      outb(ch,DMM->base+3); // hi  channel  
      //DMMUnlockMutex(DMM);
      return 0;
      
}

int32_t DMMLockMutex(struct dmminfo * DMM){
      /* Any thread that attempts to access the DMM hardware or data structure
       *      must call this function, then call DMMunlockmutex once it is done. */
      uint32_t waitcount;
    /*  
      waitcount = 0;
      while(pthread_mutex_trylock(&DMM->DMMMutex) != 0){
	    printf("DMMLockMutex: Could not lock mutex.\n");
	    usleep(DMMWAITTIME);
	    waitcount++;
	    if(waitcount == DMMMAXWAITCOUNT){
		  printf("DMMLockMutex: DMM mutex lock timeout.\n");
		  return 1;
	    }	
      }
      */
      return 0;
}

int32_t DMMUnlockMutex(struct dmminfo * DMM){
      
      if(pthread_mutex_unlock(&(DMM->DMMMutex)) == 0){
	    /* Successfully unlocked mutex */
	    return 0;
      } else {
	    /* Unsuccessful */
	    printf("DMMUnlockmutex: problem unlocking mutex");
	    return 1;
      }
}

void DMMConfigureCounters(struct dmminfo * DMM){
      
      
      DMMLockMutex(DMM);
      outb(0x00,DMM->base+8); //set to page zero to modify counter/timer configuration
      
      /* configure counters 0 & 2 with the following parameters */
      // 2 byte read/write cycles (as opposed to just reading the MSB or LSB)
      // Mode 0, traditional event down-counter mode
      //Binary counter rather than decimal decade counter
      
      /*configure CNT1 with the following params */
      // 2 byte read/write cycles (as opposed to just reading the MSB or LSB)
      // Mode 2, full range rate generator... each pulse on output acts as a clock in to CNT2.  This realizes a 32 bit counter.
      //Binary counter rather than decimal decade counter 
      
      outb(0x30,DMM->base+15); //config CNT0
      outb(0x74,DMM->base+15); //config CNT1
      outb(0xB0,DMM->base+15); //config CNT2
      
      DMMUnlockMutex(DMM);
      return;
      
}

void DMMEnableVetoCount(struct dmminfo * DMM){
      
      DMMLockMutex(DMM);
      outb(0x00,DMM->base+8); //set to page zero to modify counter/timer configuration
      
      
      /* The following line performs the following configurations */
      //10 MHz clock to CNT1-2 cascade
      //CNT2 output at J3, pin 42
      //CNT0 output at J3, pin 44
      //no gating for CNT0
      //input to CNT0 at J3, pin 48
      
      //>>>>>>>>>>>>>>>>>>>
      outb(0x30,DMM->base+10);
      //<<<<<<<<<<<<<<<<<<<
      
      DMMUnlockMutex(DMM);
      return;
      
}

void DMMEnableDeadTimeCount(struct dmminfo * DMM){
      
      DMMLockMutex(DMM);
      outb(0x00,DMM->base+8); //set to page zero to modify counter/timer configuration
      
      /* The following line performs the following configurations */
      // 10 MHz clock to CNT1-2 cascade
      //10 MHz to CNT0 input (as opposed to 10 kHz)
      //CNT2 output at J3, pin 42
      //CNT0 output at J3, pin 44
      //CNT0 gate enabled
      //CNT0 input is clock (as opposed to external signal)
      
      //>>>>>>>>>>>>>>>>>>>
      outb(0x36,DMM->base+10);
      //<<<<<<<<<<<<<<<<<<<
      
      DMMUnlockMutex(DMM);
      return;
      
}

void DMMUpdateDIO(struct dmminfo * DMM){
      
      outb(0x01,DMM->base+8); //set page to 1 to access DIO ports
      
      if( DMMLockMutex(DMM) == 1){
	    printf("DMMUpdateDIO: could not lock DMM mutex.\n");
	    return;
      }
      DMM->dio_port[0] = inb(DMM->base+12);
      DMM->dio_port[1] = inb(DMM->base+13);
      DMM->dio_port[2] = inb(DMM->base+14);	
      DMMUnlockMutex(DMM);
      
      return;
      
}


void DMMSetDIOByte(struct dmminfo * DMM, uint8_t port, uint8_t byte){
      
      outb(0x01,DMM->base+8); //set page to 1 to access DIO ports
      
      if(port > 2){
	    printf("DMMSetDIOBit: port must be = PORT_A, PORT_B, or PORT_C.\n");
	    return;
      }
      
      if( DMMLockMutex(DMM) == 1){
	    printf("DMMSetDIOBit: could not lock mutex.\n");
	    return;
      }
      
      /* mutex is locked before accessing DMM->init */
      if( DMM->init == 0 ){
	    printf("DMMSetDIOBit: DMM board is not initialized.\n");
	    DMMUnlockMutex(DMM);
	    return;
      }
      
      outb(byte,DMM->base + 12 + port); //write byte 
      DMM->dio_port[port] = inb(DMM->base+12+port); // read byte back in to structure		
      DMMUnlockMutex(DMM); //unlock DMM struct
      
      return;
}

void DMMUpdateADC(struct dmminfo * DMM){
      
      uint32_t waitcount,i;
      int16_t c;
      uint8_t ibyteMSB,ibyteLSB;
      
      if( DMMLockMutex(DMM) == 1){
	    printf("DMMUpdateADC: could not lock DMM mutex.\n");
	    return;
      }
      
      waitcount = 0;		
      while(inb(DMM->base+11) & 0x80){
	    usleep(DMMWAITTIME);
	    waitcount++;
	    if(waitcount == DMMMAXWAITCOUNT){
		  printf("DMMupdateADC: A/D settling timeout.\n");
		  break;
	    }
      }
      waitcount = 0;
      outb(0x0E,DMM->base+7); //reset FIFO
      outb(0x01,DMM->base+0); //start A/D conversion
      while(inb(DMM->base+8) & 0x80){
	    usleep(DMMWAITTIME);
	    waitcount++;
	    if(waitcount == DMMMAXWAITCOUNT){
		  printf("DMMupdateADC: A/D conversion in progress timeout.\n");
		  break;
	    }
      }		
      for(i=0;i<N_AIN_CHANNELS;i++){
	    if(!(inb(DMM->base+7) & 0x80)){				
		  ibyteLSB = inb(DMM->base+0);
		  ibyteMSB = inb(DMM->base+1);
		  c = (uint16_t)ibyteLSB | ((uint16_t)ibyteMSB << 8);
		  DMM->ain[i] = (int16_t)c;
		  
		  
	    } else {
		  printf("DMMupdate: FIFO is empty, no data to collect.\n");
	    }	
      }
      if(!(inb(DMM->base+7) & 0x80)){
	    printf("DMMupdate: FIFO is NOT empty after A/D readout.\n");
	    outb(0x0E,DMM->base+7); //reset FIFO
      }
      
      DMMUnlockMutex(DMM);
      return;
}

void DMMUpdateAll(struct dmminfo * DMM){
      /*DMM update...do A/D conversion and read back Digital Output settings*/
      
      DMMLockMutex(DMM);
      outb(0x01,DMM->base+8); //set page to 1 to access DIO ports
      
      DMMUpdateADC(DMM);
      DMMUpdateDIO(DMM);
      
      DMMUnlockMutex(DMM);
      return;
}

void DMMSetDIOBit(struct dmminfo * DMM, uint8_t port, uint8_t whichbit, uint8_t onoroff){
      
      
      outb(0x01,DMM->base+8); //set page to 1 to access DIO ports
      
      if(port > 2){
	    printf("DMMSetDIOBit: port must be = PORT_A, PORT_B, or PORT_C.\n");
	    return;
      }
      
      if( (whichbit == 0) || (whichbit>128) ){
	    printf("DMMSetDIOBit: Bit is out of range.\n");
	    return;
      }		
      
      if( DMMLockMutex(DMM) == 1){
	    printf("DMMSetDIOBit: could not lock mutex.\n");
	    return;
      }
      
      /* mutex is locked before accessing DMM->init */
      if( DMM->init == 0 ){
	    printf("DMMSetDIOBit: DMM board is not initialized.\n");
	    DMMUnlockMutex(DMM);
	    return;
      }
      
      if(onoroff == 1){
	    /* set bit on */
	    outb(DMM->dio_port[port] | whichbit,DMM->base + 12 + port);	
      } else if (onoroff == 0){
	    /* set bit off by doing a bitwise not - or - not */	
	    outb( ~( (~DMM->dio_port[port]) | whichbit ), DMM->base + 12 + port);
      }
      
      /* read DMM register values back in */
      DMM->dio_port[port] = inb(DMM->base+12+port);	
      
      DMMUnlockMutex(DMM);
      return;
}

void DMMPulseDIOBit(struct dmminfo * DMM, uint8_t port, uint8_t whichbit,uint32_t pulse_duration){
      
      uint8_t obyte;
      
      outb(0x01,DMM->base+8); //set page to 1 to access DIO ports
      
      
      if(port > 2){
	    printf("DMMPulseDIOBit: port must be = PORT_A, PORT_B, or PORT_C.\n");
	    return;
      }
      
      if( (whichbit ==0) || (whichbit>128) ){
	    printf("DMMPulseDIOBit: Bit is out of range.\n");
	    return;
      }		
      
      if( DMMLockMutex(DMM) == 1){
	    printf("DMMPulseDIOBit: could not lock mutex.\n");
	    return;
      }
      
      /* mutex is locked before accessing DMM->init */
      if( DMM->init == 0 ){
	    printf("DMMPulseDIOBit: DMM board is not initialized.\n");
	    DMMUnlockMutex(DMM);
	    return;
      }
      
      if( !(DMM->dio_port_pulse_bits[port] & whichbit) ){
	    printf("DMMPulseDIOBit: bit not enabled for pulsing.\n");
	    DMMUnlockMutex(DMM);
	    return;
      }
      
      /* I'm assuming all pulses will be positive going.  if it turns out
       *      to be the case that we will need some negative going pulses, then that
       *      may be implemented here.  For the time being, I will just make sure that
       *      the line is set to zero before the pulse commands are sent. */
      
      if( (DMM->dio_port[port] & whichbit) ){
	    /* port is high, set it low */
	    outb( ~( (~DMM->dio_port[port]) | whichbit ), DMM->base + 12 + port);
	    usleep(10000); //sleep for 10 ms
      }
      
      outb(DMM->dio_port[port] | whichbit, DMM->base + 12 + port); //set high
      usleep(pulse_duration); //sleep -> pulse width
      //outb( ~( (~DMM->dio_port[port]) | whichbit ), DMM->base + 12 + port); //set low
      outb(DMM->dio_port[port], DMM->base + 12 + port);
      DMMUnlockMutex(DMM);
      return;
}

void DMMStartCounters(struct dmminfo * DMM){
      
      DMMLockMutex(DMM);
      outb(0x00,DMM->base+8); //set to page zero to modify counter/timer configuration
      
      /* The counter actually starts immediately after the MSB is written.  In light of this,
       *      load the MSB to CNT2 before CNT1, since the output of CNT1 is cascaded into CNT2
       * 
       *      /*load max LSBs into all counters */
       
       outb(0xff,DMM->base+12); //CNT0
       outb(0xff,DMM->base+14); //CNT2
       outb(0xff,DMM->base+13); //CNT1
       
       /*load max MSBs into all counters */
       
       outb(0xff,DMM->base+12); //CNT0
       outb(0xff,DMM->base+14); //CNT2
       outb(0xff,DMM->base+13); //CNT1
       
       //At this point, the counters should all be actively counting down.
       
       DMMUnlockMutex(DMM);
       return;
       
}

void DMMLatchCounters(struct dmminfo * DMM){
      
      DMMLockMutex(DMM);
      outb(0x00,DMM->base+8); //set to page zero to modify counter/timer configuration	
      
      //0xde is the Read back command ... it latches all counters simultaneously at hardware level as opposed to 
      //individually latching each counter
      
      outb(0xde,DMM->base+15);
      
      //I tried using 0xce instead of 0xde, which would latch both the statuses and count values of the counters,
      //but this has implications on the order in which the counters/statuses are read out, so I'm not latching
      //the output value for now (don't really need to, although it would be useful for detecting rollovers).
      
      DMMUnlockMutex(DMM);
      return;
      
}

uint32_t DMMReadoutCounters(struct dmminfo * DMM,uint32_t cnt){
      
      uint32_t LSB,MSB;
      
      
      if( cnt > 2 ){
	    printf("DMMReadoutCounters: counter ID is out of range, should be 0,1, or 2.\n");
	    return 0;
      }
      
      DMMLockMutex(DMM);
      outb(0x00,DMM->base+8); //set to page zero to modify counter/timer configuration            
      LSB = inb(DMM->base + 12 + cnt); //LSB gets read in first
      MSB = inb(DMM->base + 12 + cnt);            
      DMMUnlockMutex(DMM);
      
      return MAX16BIT - ((MSB << 8) | LSB);
      
}

