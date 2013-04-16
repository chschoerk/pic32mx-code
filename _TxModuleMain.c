/*
 * File:   _TxModuleMain.c
 * Author: abu
 *
 * Created on 18. Dezember 2012, 10:01
 */

#include <stdio.h>
#include <stdlib.h>
#include <plib.h>
#include <p32xxxx.h>

#include "adf7023_mint.h"
#include "si5326.h"
#include "configandmux.h"
#include "timestamping.h"
#include "switching.h"
#include "smbus.h"
#include "measuring.h"
#include "smbus.h"
#include "tempsensor.h"
#include "_TxModuleMain.h"


#define tPREAMBEL_LEN        32  //byte
#define tPREAMBEL            0x55
#define tSYNCBYTE0           0x33
#define tSYNCBYTE1           0x33
#define tSYNCBYTE2           0xA6
#define tPAYLOADLEN          32   //240 //byte

#define VCO_FREQ             12288000
#define CNT_HIST_BUFFER_SIZE 64 //size of ringbuffer that keeps track of PID controller output history
#define CNT_STOP_THRESH      10
//#define CNT_STOP_SIGN_TRESH  40
#define BOUNCE_TRESH         250
#define LOST_PACKETS_TILL_FREERUNNING 20
#define TURNS_BUFFER_SIZE    8

volatile BOOL rxDetected;
volatile unsigned int counterValue = 0;
volatile unsigned int counterValueOld = 0;
volatile unsigned int counterOverflow = 0;
volatile UINT32 counterValue32 = 0;

UINT32 txferTxBuff[TXBUFFSZ];
unsigned char syncState = NOTSYNCED;
volatile UINT16 curDmaSrcPtr;
volatile UINT32 curTimestampWritten;
volatile UINT32 firstTimestampInBufferA;
volatile UINT32 firstTimestampInBufferB;
volatile UINT32 counter = 0;
volatile unsigned char fillBufferA = 0;
volatile unsigned char fillBufferB = 0;
volatile UINT32 nominalValue = REFEDGES;

volatile unsigned char sendData = 0; //for SMBus
volatile unsigned char recvData = 0; //for SMBus
volatile unsigned char smbusCmdReceived = 0; //for SMBus

int main(void) {

    BOOL bOk = TRUE;
    TyMCR MCRregisters;
    unsigned char dummyDat;
    unsigned int pbclockfreq;
    int skippedFirst = 0;
    INT32 pBuf[TMEAS_BUFFER_SIZE];
    INT32 bufSum=0;
    INT32 fDeviation;
    UINT32 turns;
    UINT32 ts = 0;
    UINT32 tsOld = 0;
    UINT32 edgeCount = 0;
    UINT32 timeStampIncrement;
    int sane;
    int ret;
    int i;
    UINT32 pwmValCurrent;
    INT32 out;
    UINT32 cntHist[CNT_HIST_BUFFER_SIZE] = { 0 };
    INT8 devSign = 0;
    UINT32 cntHistBufSum = 0;
    int cntHistIdx = 0;
    int controllerOn = 1;
    INT16 outOfBounceCount = 0;
    INT32 thisDeviationAbs = 0;
    INT32 tmpArr1[CNT_HIST_BUFFER_SIZE] = { 0 };
    INT32 timestampDivergence;
    UINT32 packetCounter = 0;
    INT32 lastTimestampDivergence = 0;
    UINT32 packetCount[3] = { 0 };
    UINT16 pcidx = 0;
    INT32 tsdiv[3] = { 0 };

    UINT32 t1counter = 0;
    UINT32 turnsBuffer[TURNS_BUFFER_SIZE] = { 0 };
    UINT32 turnsBufferSum = 0;
    UINT32 avgTurns = 0;
    int t1ix = 0;

    unsigned char temperature = 0;
    unsigned char syncStatusWord = SYNC_STATE_FREERUNNING;
    unsigned char rssiValue = 0;

    counterOverflow = 0;
    counterValue = 0;
    counterValueOld = 0;
    
    /*---SYSTEM CONFIG---*/
    pbclockfreq = SYSTEMConfig(GetSystemClock(), SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);
    //SYSTEMConfigWaitStatesAndPB()
    DDPCONbits.JTAGEN = 0; //disable JTAG
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);

    /*---PINMUXING (SWITCHING)---------------------------------------*/
    pinMux01();

    /*---SWITCHING---------------------------------------------------*/
    turnOffLED1;
    turnOffLED2;
    switchOnCounter; //enable clock division
    switch2ClockAnd(); //use buffer instead of AND
    //switchOffBuffer();
    //switchOffAnd();

    /*---SETUP------------------------------------------------------*/
    setupI2S();                             //I2S (TIMESTAMP OUT)   
    setupSMBus(pbclockfreq);              //I2C (SMBus slave)
    pwmValCurrent = setupPWM(pbclockfreq); //PWM (VCXO CONTROL)
    setupEdgeCount();                       //VCXCO EDGE COUNTING
    setupTempSensor();
    turnOnLED1;
    turnOnLED2;
    setupADF();                             //ADF7023
    turnOffLED1;
    ADF_MCRRegisterReadBack(&MCRregisters); //read back the MCRRegisters
    setupDetectInterrupt();                 //PREAMBEL DETECTED IRQ
    //setupRTCC();
    //tmLastPacket.l = RtccGetTime(); //debug
    OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_256,  39063); //timer overflow frequency to 4Hz (40MHz/(256*4) = 39063)

    /*---ENABLE INTERRUPTS------------------------------------------*/
    INTEnableInterrupts();
    
    /*start DMA transfer AFTER(!) enabling interrupts*/
    DmaChnStartTxfer(DMA_CHANNEL1, DMA_WAIT_NOT, 0);	// force the DMA transfer: the SPI TBE flag it's already been active

    /*---LOOP-------------------------------------------------------*/
    //stallRecover = 0;
    timeStampIncrement = (UINT32)(REFEDGES / 256); //245 = (12288000/48000)
    rxDetected = FALSE;
    bOk = bOk && ADF_GoToRxState(); //ADF: Go to RX state
    turnOffLED2;

    while(1){

        if (rxDetected){

            if (syncStatusWord !=  SYNC_STATE_SYNCED){
                syncStatusWord = SYNC_STATE_SYNCING;
            }

            toggleLED2;
            if (skippedFirst){

                i = 100;
                while(i--); //wait

                ts = readReceivedTimestamp();
                turns = (ts - tsOld)/timeStampIncrement;
                tsOld = ts;

                if (syncState == SYNCED){
                    packetCounter++;
                    /*if we think we're synced, timestamps should be in sync*/
                    timestampDivergence = ts - curTimestampWritten; //timestamp error
                    
                    if (timestampDivergence <= (lastTimestampDivergence+1) && timestampDivergence >= (lastTimestampDivergence-1)){
                        //everything's alright, all in good sync
                        timestampDivergence = 0; //temp debug
                    }else{
                        //we seem to be a bit off -> adjust VCXO slighly (how?)
                        lastTimestampDivergence = timestampDivergence;
                        packetCount[pcidx] = packetCounter;
                        tsdiv[pcidx] = lastTimestampDivergence;
                        packetCounter = 0;
                        pcidx++;
                        if (pcidx==3){
                            pcidx = 0;
                        }

                        if (timestampDivergence > 0){ //we're too slow
                            nominalValue += 30; //30 mal so zum probieren
                        }else{ //we're too fast
                            nominalValue -= 30; //30 mal so zum probieren
                        }
                        turnOffLED1;
                    }

                }

                if (counterOverflow > 0){
                    edgeCount = T45PR - counterValueOld + ((counterOverflow-1)*T45PR) + counterValue32 - 1;
                }else{
                    edgeCount = counterValue32 - counterValueOld - 1;
                }
                
                sane = sanityCheck(edgeCount, turns);
                if (sane){
                    ret = measureFrequency(edgeCount, pBuf, &bufSum, turns, &fDeviation);
                    if (controllerOn == 1){
                        out = PID(-fDeviation, 0, 399999); //max: 0 - 399999
                        if (out >= 0 && controllerOn == 1){
                            SetDCOC1PWM(out);
                        }

                        /*controller history ringbuffer*/
                        cntHistBufSum = cntHistBufSum - cntHist[cntHistIdx] + out;
                        cntHist[cntHistIdx] = out;

                        /*update outOfBounce Count (defines sync status)*/
                        if (tmpArr1[cntHistIdx] > BOUNCE_TRESH){
                            outOfBounceCount--;
                        }
                        devSign = ( (fDeviation > 0) - (fDeviation < 0) );
                        thisDeviationAbs = devSign * fDeviation;
                        if (thisDeviationAbs > BOUNCE_TRESH){
                            outOfBounceCount++;
                        }
                        tmpArr1[cntHistIdx] = thisDeviationAbs;                          


                        /*check if we can stop controlling*/
                        if (outOfBounceCount < CNT_STOP_THRESH && ret > 0  && syncState == NOTSYNCED){
                            turnOnLED1;
                            //out = cntHistBufSum / CNT_HIST_BUFFER_SIZE;
                            //SetDCOC1PWM(out);
                            //controllerOn = 0;
                            //switchOnCounter; //enable clock division
                            //switch2ClockAnd();

                            /*update timestamps - now they should be valid*/
                            updateTimestamp(ts);
                            syncState = SYNCED;
                            syncStatusWord = SYNC_STATE_SYNCED;
                        }

                        cntHistIdx++;
                        cntHistIdx &= (CNT_HIST_BUFFER_SIZE-1); //equals: cntHistIdx = cntHistIdx % CNT_HIST_BUFFER_SIZE if CNT_HIST_BUFFER_SIZE is 2^x


                    } else { // if (controllerOn == 1)

                        /*....*/

                    }                        
                    
                } //if sane

            }else{ //if(skippedFirst)

                ts = readReceivedTimestamp();
                updateTimestamp(ts);
                tsOld = ts;
                skippedFirst = 1;

            }
            
            /*reset counters*/
            counterOverflow = 0;
            counterValueOld = counterValue32;

            /*Clear ADF8023 Interrupt*/
            dummyDat = 0xFF;
            bOk = bOk & ADF_MMapWrite(MCR_interrupt_source_0_Adr, 0x1, &dummyDat); //clear all interrupts in source 0 by writing 1's

            /*update sync-status, ...*/
            //TODO

            /*Bring ADF back to RxState*/
            rxDetected = FALSE;
            bOk = bOk && ADF_GoToRxState();


            /*read temperature from sensor*/
            temperature = readTemperature();

            /*get RSSI info from ADF*/
            bOk = bOk & ADF_MMapRead(MCR_rssi_readback_Adr, 0x01, &rssiValue);

            /*reset t1counter, count until nex packet is received*/
            t1counter = 0;

            /*compute average package loss*/
            turnsBufferSum = turnsBufferSum - turnsBuffer[t1ix] + turns;
            avgTurns = turnsBufferSum>>3; //divide by 8 (=TURNS_BUFFER_SIZE)
            if (avgTurns > 255){
                avgTurns = 255; //to squeeze it in a single byte value
            }
            turnsBuffer[t1ix] = turns;
            t1ix++;
            t1ix &= (TURNS_BUFFER_SIZE-1); //wrap index


        } //if (rxDetected)


        if (fillBufferA || fillBufferB){  /*fill DMA buffer*/

            fillDMABufferHalf();

        }


        if (smbusCmdReceived){

            switch(recvData){
                case CMD_SYNCSTATUS:
                    sendData = (unsigned char)syncStatusWord;
                    break;
                case CMD_TEMPSENS:
                    sendData = (unsigned char)temperature;
                    break;
                case CMD_GETRSSI:
                    sendData = (unsigned char)rssiValue;
                case CMD_GETPACKETLOSS:
                    sendData = (unsigned char)avgTurns;
                default:
                    //should not happen
                    break;
            }
            smbusCmdReceived = 0;
            
        }

        if ( mT1GetIntFlag() ){
            t1counter++;
            //if (t1counter == (LOST_PACKETS_TILL_FREERUNNING<<2)){ //multiplay 4 because packet rate is 1Hz and timer1 irq rate is 4Hz
            if (t1counter == 40){
                syncStatusWord = SYNC_STATE_FREERUNNING;
                avgTurns = 0xff; //set to 0%
            }
            mT1ClearIntFlag();
        }

    } //while(1)

    

    return (EXIT_SUCCESS);
}


void __ISR(_EXTERNAL_1_VECTOR, ipl3) INT1Interrupt()
{
   //read and reset counter value TMR1
   /*
   while(T1CON & 0x0800); //check T1CON.TWIP and wait until theres no write to TMR1 in progess
   counterValue = TMR1;
   */
   counterValue32 = TMR4;

   //updateDMASourcePointer();
   curDmaSrcPtr = (DmaChnGetSrcPnt(DMA_CHANNEL1)) >> 2; //floor(srcPtr_byte/4) => index of the current buffer element (UINT32)

   if (curDmaSrcPtr < TXBUFFSZ_HALF){
        curTimestampWritten = curDmaSrcPtr + firstTimestampInBufferA;
   }else{
        curTimestampWritten = (curDmaSrcPtr - TXBUFFSZ_HALF) + firstTimestampInBufferB;
   }

   curTimestampWritten += (TXBUFFSZ_HALF-1); //otherwise timestampDeviation (= ts - curTimestampWritten) = TXBUFFSZ_HALF when in sync (???)

   resetSrcPtrOverruns();

   rxDetected = TRUE;
   mINT1ClearIntFlag();

}

/*
void __ISR(_TIMER_1_VECTOR, ipl1) T1Interrupt()
{
   counterOverflow++;
   //mPORTBToggleBits(BIT_10); //PIN 8
   mT1ClearIntFlag();
}
*/

void __ISR(_TIMER_5_VECTOR, ipl6) T5Interrupt()
{
   counterOverflow++;
   mT5ClearIntFlag();
}

/*
void __ISR(_TIMER_3_VECTOR, ipl2) T3Interrupt()
{
   //mPORTBToggleBits(BIT_13);
   mT3ClearIntFlag();
}*/
