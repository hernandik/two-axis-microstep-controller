;*********************************************************************
;*
;*  This implements a generic library functionality to support UART.
;*  It adds additional functionality of Rx/Tx user defined circular 
;*  FIFO buffer
;*
;*********************************************************************
;* FileName:            UARTInt.asm
;* Dependencies:        16UARTI.asm
;*                      18UARTI.asm
;*                      P16xxx.inc
;*                      P18xxx.inc
;*                      UARTInt.inc
;* Processor:           PIC16xxxx & PIC18xxxxx
;* Assembler:           MPASMWIN 02.70.02 or higher
;* Linker:              MPLINK 2.33.00 or higher
;* Company:             Microchip Technology, Inc.
;*
;* Software License Agreement
;*
;* The software supplied herewith by Microchip Technology Incorporated
;* (the "Company") for its PICmicro® Microcontroller is intended and
;* supplied to you, the Company's customer, for use solely and
;* exclusively on Microchip PICmicro Microcontroller products. The
;* software is owned by the Company and/or its supplier, and is
;* protected under applicable copyright laws. All rights are reserved.
;* Any use in violation of the foregoing restrictions may subject the
;* user to criminal sanctions under applicable laws, as well as to
;* civil liability for the breach of the terms and conditions of this
;* license.
;*
;* THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
;* WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
;* TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
;* PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
;* IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
;* CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
;*
;*
;* Author               Date            Comment
;*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;* Gaurang Kavaiya      Nov 17, 2000    Initial Release 
;* Gaurang Kavaiya      Sep 27, 2002    Modified for Application Maestro
;* Gaurang Kavaiya      Feb 11, 2003    Enhancements for Maestro
;* Gaurang Kavaiya      Feb 21, 2003    Added support for Circular buf
;* Gaurang Kavaiya      Mar 12, 2003    Removed Liner FIFO Buf Suprt<V1.0>
;********************************************************************/


        errorlevel      -207    ;Hide found label after column 1 warning


#define USART_MODULE        ;Module definition to generate error message for
                            ;Processor which do not have this module.
#define _ADD_PROC_INC_FILE
#define GEN_MODULE_ERROR
       include  "P18xxx.inc"
       include  "P16xxx.inc"


#define UARTInt_Source
       include "UARTInt.inc"


        UDATA
vUARTIntStatus          RES     01      ;Used to store flags for different errors

vUARTIntTxBuffer        RES     UARTINT_TX_BUFFER_SIZE   ;Transmit Data FIFO buffer
vUARTIntTxBufDataCnt    RES     01          ;Transmit Buffer Write Position
vUARTIntTxBufWrPtr      RES     01
vUARTIntTxBufRdPtr      RES     01
                                          ;Indicator Pointer
vUARTIntRxBuffer        RES     UARTINT_RX_BUFFER_SIZE   ;Receive Data FIFO buffer
vUARTIntRxBufDataCnt    RES     01          ;Transmit Buffer Write Position
vUARTIntRxBufWrPtr      RES     01
vUARTIntRxBufRdPtr      RES     01



;   UDATA_OVR
;This data is temporarily only. It can be kept in over writtable RAM.
;But in that case linker will define same RAM address for other
;parameters. As these are used in ISR (Interrupt Service Routine),
;If interrupt occurs while other parameter is used then one will loose
;value of that parameter. So preferably keep it in UDATA only unless you
;are sure that ISR will not be invoked at the time of use of that
;parameters
temp1           RES     01
temp3           RES     01


ISR_FSR0L   RES         01      ;For saving FSR0L value


#ifdef  _PIC18xxx

ISR_FSR0H   RES         01      ;For saving FSR0H value

#endif




#define     BRGH_HIGH       ;Select for BRGH=1

SPBRG_V1 = CLOCK_FREQ/UARTINT_BAUDRATE;

SPBRG_V2 = SPBRG_V1 /.16   ;
SPBRG_V22 = (SPBRG_V1 * .10) /.16   ;
        if ( ((SPBRG_V2*.10) - SPBRG_V22) >= .5)
        SPBRG_V2 ++;
        endif


        if (SPBRG_V2 > 0xff)
        SPBRG_V3 = SPBRG_V2

        SPBRG_V2 = SPBRG_V3 / .4
        SPBRG_V22 = (SPBRG_V3 * .10) /.4   ;
        if ( ((SPBRG_V2*.10) - SPBRG_V22) >= .5)
                SPBRG_V2 ++;
        endif
        
        #undefine   BRGH_HIGH   ;BRGH = 0
        endif


SPBRG_VAL=SPBRG_V2 - D'1'   ;Calculated SPBRG register value

    if SPBRG_VAL > .255
        error "Calculated SPBRG register value is out of range"
    endif

    if SPBRG_VAL <= .10
        error "Calculated SPBRG register value is too low"
    endif



#ifdef  _PIC18xxx
    include "18UARTI.asm"
#endif

#ifdef  _PIC16xxx
    include "16UARTI.asm"
#endif



    END
