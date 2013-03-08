#include <stdio.h>
#include <stdlib.h>
#include <plib.h>
#include <p32xxxx.h>

#include "interrupts.h"


void enableExtPinIRQ()
{
    //...
    INTEnableInterrupts();
    INTEnable(INT_INT1, INT_ENABLED);
}

/*interrupt handler*/
//...