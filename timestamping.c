
#include <stdio.h>
#include <stdlib.h>
#include <plib.h>
#include <p32xxxx.h>

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



int TS_initBuffers()
{
    /*fill transmit buffer with dummy data-----------------------------*/
    int ix;
    for(ix=0; ix<sizeof(txferTxBuff)/sizeof(*txferTxBuff); ix++)
    {
        txferTxBuff[ix] = 0xaaaaaaaa; //dummy
    }

    return 0;
}


int startDMA2_Spi1ToRxBuff(void)
{

	DmaChannel		dmaRxChn=DMA_CHANNEL2;	// DMA channel to use for our example
							// NOTE: the DMA ISR setting has to match the channel number

	// open and configure the SPI channel to use: slave, no frame mode, 8 bit mode.
	// won't use SS for communicating with the master
	// SPI clock is irrelevant in slve mode
	//SpiChnOpen(spiRxChn, SPI_OPEN_SLVEN|SPI_OPEN_MODE8|SPI_OPEN_CKP_HIGH, 4); //open in enhanced buffer mode?

	// open and configure the DMA channel.
	DmaChnOpen(dmaRxChn, DMA_CHN_PRI3, DMA_OPEN_AUTO);

	// set the events: we want the SPI receive buffer full interrupt to start our transfer
	DmaChnSetEventControl(dmaRxChn, DMA_EV_START_IRQ_EN|DMA_EV_START_IRQ(_SPI1_RX_IRQ));

	// set the transfer:
	// source is the SPI buffer, dest is our memory buffer
	// source size is one byte, destination size is the whole buffer
	// cell size is one byte: we want one byte to be sent per each SPI RXBF event
	DmaChnSetTxfer(dmaRxChn, (void*)&SPI1BUF, txferRxBuff, 1, sizeof(txferRxBuff), 1);


        //DmaChnSetEvEnableFlags(dmaRxChn, DMA_EV_CELL_DONE);
        DmaChnSetEvEnableFlags(dmaRxChn, DMA_EV_BLOCK_DONE|DMA_EV_DST_HALF);	// enable the transfer done interrupt, when all buffer transferred

	//INTEnableSystemMultiVectoredInt();			// enable system wide multi vectored interrupts

	//INTSetVectorPriority(INT_VECTOR_DMA(dmaRxChn), INT_PRIORITY_LEVEL_5);		// set INT controller priority


        INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);

	INTEnableInterrupts();

	INTSetVectorPriority(INT_VECTOR_DMA(dmaRxChn), INT_PRIORITY_LEVEL_5);		// set INT controller priority
	INTSetVectorSubPriority(INT_VECTOR_DMA(dmaRxChn), INT_SUB_PRIORITY_LEVEL_3);		// set INT controller sub-priority

	INTEnable(INT_SOURCE_DMA(dmaRxChn), INT_ENABLED);		// enable the chn interrupt in the INT controller

	DmaChnEnable(dmaRxChn);	// enable the DMA channel

	return 1;
}

 

// handler for the DMA channel 2 interrupt
void __ISR(_DMA2_VECTOR, ipl5) DmaHandler2(void)
{
	int	evFlags;				// event flags when getting the interrupt

	INTClearFlag(INT_SOURCE_DMA(DMA_CHANNEL2));	// acknowledge the INT controller, we're servicing int

	evFlags=DmaChnGetEvFlags(DMA_CHANNEL2);	// get the event flags

    if(evFlags&DMA_EV_BLOCK_DONE)
    {
        RxBufferBDone = 1;
 	DmaChnClrEvFlags(DMA_CHANNEL2, DMA_EV_BLOCK_DONE);
    }

    if(evFlags&DMA_EV_DST_HALF)
    {
        RxBufferADone = 1;
        DmaChnClrEvFlags(DMA_CHANNEL2, DMA_EV_DST_HALF);

    }
}


int startDMA1_TxBuffToSpi1(void)
{
	DmaChannel		dmaTxChn=DMA_CHANNEL1;	// DMA channel to use for our example
							// NOTE: the DMA ISR setting has to match the channel number
	//SpiChannel		spiTxChn=SPI_CHANNEL2;	// the transmitting SPI channel to use in our example


	// open and configure the SPI channel to use: master, no frame mode, 8 bit mode.
	// won't use SS for communicating with the slave
	// we'll be using 40MHz/4=10MHz SPI clock
	//SpiChnOpen(spiTxChn, SPI_OPEN_MSTEN|SPI_OPEN_SMP_END|SPI_OPEN_MODE8, 4);
        //open in slave mode
        //SpiChnOpen(spiTxChn, SPI_OPEN_SLVEN|SPI_OPEN_SMP_END|SPI_OPEN_MODE32, 4); //open in enhanced buffer mode?

	// open and configure the DMA channel.
	//DmaChnOpen(dmaTxChn, DMA_CHN_PRI2, DMA_OPEN_DEFAULT);
        DmaChnOpen(dmaTxChn, DMA_CHN_PRI2, DMA_OPEN_AUTO);

	// set the events: we want the SPI transmit buffer empty interrupt to start our transfer
	DmaChnSetEventControl(dmaTxChn, DMA_EV_START_IRQ_EN|DMA_EV_START_IRQ(_SPI1_TX_IRQ));

	// set the transfer:
	// source is our buffer, dest is the SPI transmit buffer
	// source size is the whole buffer, destination size is one byte
	// cell size is one byte: we want one byte to be sent per each SPI TXBE event
	DmaChnSetTxfer(dmaTxChn, txferTxBuff, (void*)&SPI1BUF, sizeof(txferTxBuff), 4, 4);

	//DmaChnSetEvEnableFlags(dmaTxChn, DMA_EV_BLOCK_DONE);
        DmaChnSetEvEnableFlags(dmaTxChn, DMA_EV_BLOCK_DONE | DMA_EV_SRC_HALF);	// enable the transfer done interrupt, when all buffer transferred

	INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
	INTEnableInterrupts();

	INTSetVectorPriority(INT_VECTOR_DMA(dmaTxChn), INT_PRIORITY_LEVEL_5);		// set INT controller priority
	INTSetVectorSubPriority(INT_VECTOR_DMA(dmaTxChn), INT_SUB_PRIORITY_LEVEL_3);		// set INT controller sub-priority

	INTEnable(INT_SOURCE_DMA(dmaTxChn), INT_ENABLED);		// enable the chn interrupt in the INT controller

        
	//DmaChnStartTxfer(dmaTxChn, DMA_WAIT_NOT, 0);	// force the DMA transfer: the SPI TBE flag it's already been active
        
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
            txferTxBuff[mx] = 0xaaaaaaaa;
            //txferTxBuff[mx] = counter;
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
            txferTxBuff[mx] = 0xaaaaaaaa;
            //txferTxBuff[mx] = counter;
            //if (counter < 0xfffffffe){
            //    counter++;
            //}
            //mx++;
        }
    }


}
