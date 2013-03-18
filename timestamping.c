
#include <stdio.h>
#include <stdlib.h>
#include <plib.h>
#include <p32xxxx.h>

#include "configandmux.h"
#include "timestamping.h"

/*DEFINES---------------------------------------------*/
#define TXBUFFSZ                64
#define TXBUFFSZ_HALF           (TXBUFFSZ/2)
#define RXBUFFSZ                6
#define RXBUFFSZ_HALF           (RXBUFFSZ/2)



/*GLOBALS----------------------------------------------*/
UINT32                  txferTxBuff[TXBUFFSZ];
UINT8                   txferRxBuff[RXBUFFSZ];
static volatile int     RxBufferADone = 0;
static volatile int     RxBufferBDone = 0;


int setupI2S()
{
    SPI2_configI2S();
    initBuffers();
    startDMA1_TxBuffToSpi2();
    return 0;
}

int initBuffers()
{
    /*fill transmit buffer with dummy data-----------------------------*/
    int ix;
    UINT32 val;
    for(ix=0; ix<sizeof(txferTxBuff)/sizeof(*txferTxBuff); ix++)
    {
        if (ix%2 == 0)
            val = 0xffff0000; //dummy
        else
            val = 0;

        txferTxBuff[ix] = val;
    }

    return 0;
}

int startDMA1_TxBuffToSpi2(void)
{
	DmaChannel		dmaTxChn=DMA_CHANNEL1;	// DMA channel to use for our example
							// NOTE: the DMA ISR setting has to match the channel number
        DmaChnOpen(dmaTxChn, DMA_CHN_PRI2, DMA_OPEN_AUTO);

	// set the events: we want the SPI transmit buffer empty interrupt to start our transfer
	DmaChnSetEventControl(dmaTxChn, DMA_EV_START_IRQ_EN|DMA_EV_START_IRQ(_SPI2_TX_IRQ));

	// set the transfer:
	// source is our buffer, dest is the SPI transmit buffer
	// source size is the whole buffer, destination size is one byte
	// cell size is one byte: we want one byte to be sent per each SPI TXBE event
	DmaChnSetTxfer(dmaTxChn, txferTxBuff, (void*)&SPI2BUF, sizeof(txferTxBuff), 4, 4);

        DmaChnSetEvEnableFlags(dmaTxChn, DMA_EV_BLOCK_DONE | DMA_EV_SRC_HALF);	// enable the transfer done interrupt, when all buffer transferred
	//INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
	//INTEnableInterrupts();
	INTSetVectorPriority(INT_VECTOR_DMA(dmaTxChn), INT_PRIORITY_LEVEL_5);		// set INT controller priority
	INTSetVectorSubPriority(INT_VECTOR_DMA(dmaTxChn), INT_SUB_PRIORITY_LEVEL_3);		// set INT controller sub-priority
	INTEnable(INT_SOURCE_DMA(dmaTxChn), INT_ENABLED);		// enable the chn interrupt in the INT controller
	DmaChnStartTxfer(dmaTxChn, DMA_WAIT_NOT, 0);	// force the DMA transfer: the SPI TBE flag it's already been active
        
	return 1;
}



// handler for the DMA channel 1 interrupt
void __ISR(_DMA1_VECTOR, ipl5) DmaHandler1(void)
{
    
	int	evFlags;				// event flags when getting the interrupt
        int     mx = 0;
        static int counter = 0;

	INTClearFlag(INT_SOURCE_DMA(DMA_CHANNEL1));	// acknowledge the INT controller, we're servicing int

	evFlags=DmaChnGetEvFlags(DMA_CHANNEL1);	// get the event flags

    if(evFlags&DMA_EV_BLOCK_DONE)
    {
    	//DmaTxIntFlag=1;
 	DmaChnClrEvFlags(DMA_CHANNEL1, DMA_EV_BLOCK_DONE);

        mx = TXBUFFSZ_HALF;
        while(mx < TXBUFFSZ)
        {
            //txferTxBuff[mx] = 0xffff0000;
            
            txferTxBuff[mx] = counter;
            counter++;
            //if (counter < 0xfffffffe){
            //    counter++;
            //}
            mx++;
            
        }

    }

    if(evFlags&DMA_EV_SRC_HALF)
    {
        DmaChnClrEvFlags(DMA_CHANNEL1, DMA_EV_SRC_HALF);
        mx = 0;
        while(mx < TXBUFFSZ_HALF)
        {
            //txferTxBuff[mx] = 0xffff0000;
            
            txferTxBuff[mx] = counter;
            counter++;
            //if (counter < 0xfffffffe){
            //    counter++;
            //}
            mx++;
            
        }
    }


}
