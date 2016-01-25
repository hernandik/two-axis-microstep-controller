;*********************************************************************
;*
;*  This contains definitions for UART Int library
;*
;*********************************************************************
;* FileName:            UARTInt.inc
;* Dependencies:
;* Processor:
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
;* Gaurang Kavaiya      Feb 11, 2003    Enhancements for Maestro (v1.0)
;*
;********************************************************************/

#ifndef __UARTInt_INC           ;Check if inc file already included

#define __UARTInt_INC

        include "UARTInt.def"
#ifndef UARTInt_Source

       include  "P18xxx.inc"
       include  "P16xxx.inc"


;****************************************************************************
;Definations of Shared Parameters
;****************************************************************************

    EXTERN  vUARTIntStatus         ;Defined in Usart.asm
    EXTERN  vUARTIntTxBuffer, vUARTIntTxBufDataCnt     ;Defined in Usart.asm
    EXTERN  vUARTIntRxBuffer, vUARTIntRxBufDataCnt     ;Defined in Usart.asm


    EXTERN  UARTIntInit, UARTIntISR     ;Defined in USART.asm


#ifdef  UARTINT_TXON
        EXTERN  UARTIntPutCh
#endif

#ifdef  UARTINT_RXON
        EXTERN  UARTIntGetCh
#endif

#else

;****************************************************************************
;Definations of Shared Parameters
;****************************************************************************

    GLOBAL  vUARTIntStatus         ;Defined in UsartDef.h
    GLOBAL  vUARTIntTxBuffer, vUARTIntTxBufDataCnt     ;Defined in UsartDef.h
    GLOBAL  vUARTIntRxBuffer, vUARTIntRxBufDataCnt     ;Defined in UsartDef.h


    GLOBAL  UARTIntInit, UARTIntISR     ;Defined in USART.asm

#ifdef  UARTINT_TXON
        GLOBAL  UARTIntPutCh
#endif

#ifdef  UARTINT_RXON
        GLOBAL  UARTIntGetCh
#endif


#endif

#define UARTIntTxBufFul    00      ;For Transmit Buffer Full Flag in
                    ;vUARTIntStatus
#define UARTIntRxBufFul    01      ;For Receive Buffer Full Flag in
                    ;vUARTIntStatus
#define UARTIntRxBufEmpty   02      ;For Receive Buffer Empty Flag in
                    ;vUARTIntStatus
#define UARTIntRxError 03      ;For Data Receive Error Flag in
                    ;vUARTIntStatus. It represents Error
#define UARTIntRxBufOF  04      ;For Data Buffer over flow indiaction
                    ;bit. It indicates that Data received
                    ;when Rxbuffer was full. So it shows
                    ;that in between data is missing.




#ifdef  _PIC18xxx

;##############################################################################
; For PIC18xxx
;##############################################################################

;****************************************************************************
; Macro:        mUARTRxIntDisable
;
; Overview:     Disables Receive interrupt.
;
; Input:        None
;
; Output:       None
;
; Side Effects: None
;****************************************************************************
mUARTRxIntDisable    macro
        bcf     PIE1, RCIE              ; Disable USART rx interrupt
        endm


;****************************************************************************
; Macro:        mUARTRxIntEnable
;
; Overview:     Enables Receive interrupt.
;
; Input:        None
;
; Output:       None
;
; Side Effects: None
;****************************************************************************
mUARTRxIntEnable     macro
        bsf     PIE1, RCIE              ; Enable USART rx interrupt
        endm




;****************************************************************************
; Macro:        mSetUARTRxIntHighPrior
;
; Overview:     Sets high priority for Receive interrupt.
;
; Input:        None
;
; Output:       None
;
; Side Effects:  None
;****************************************************************************
mSetUARTRxIntHighPrior macro
        bsf     IPR1,RCIP               ;High Priority for Rx Int.
        endm


;****************************************************************************
; Macro:        mSetUARTRxIntLowPrior
;
; Overview:     Sets low priority for Receive interrupt.
;
; Input:        None
;
; Output:       None
;
; Side Effects: None
;****************************************************************************
mSetUARTRxIntLowPrior  macro
        bsf     IPR1,RCIP               ;Low Priority for Rx Int.
        endm



;****************************************************************************
; Macro:        mSetUARTTxIntHighPrior
;
; Overview:     Sets high priority for transmit interrupt.
;
; Input:        None
;
; Output:       None
;
; Side Effects:  None
;****************************************************************************
mSetUARTTxIntHighPrior macro
        bsf     IPR1,TXIP               ;High Priority for Tx Int.
        endm


;****************************************************************************
; Macro:        mSetUARTTxIntLowPrior
;
; Overview:     Sets low priority for transmit interrupt.
;
; Input:        None
;
; Output:       None
;
; Side Effects: None
;****************************************************************************
mSetUARTTxIntLowPrior  macro
        bsf     IPR1,TXIP               ;Low Priority for Tx Int.
        endm




;****************************************************************************
; Macro:        mDisableUARTTxInt
;
; Overview:     Disables Transmit interrupt.
;
; Input:        None
;
; Output:       None
;
; Side Effects: None
;****************************************************************************
mDisableUARTTxInt    macro
        bcf     PIE1, TXIE              ; Disable USART Tx interrupt
        endm


;****************************************************************************
; Macro:        mEnableUARTTxInt
;
; Overview:     Enables Transmit interrupt.
;
; Input:        None
;
; Output:       None
;
; Side Effects: None
;****************************************************************************
mEnableUARTTxInt     macro
        bsf     PIE1, TXIE              ; Enable USART Tx interrupt
        endm



;****************************************************************************
; Macro:        mSetUART_BRGHHigh
;
; Overview:     Sets BRGH value to high
;
; Input:        None
;
; Output:       None
;
; Side Effects: None
;****************************************************************************
mSetUART_BRGHHigh         macro
        bsf     TXSTA,BRGH              ;Enable BRGH
        endm


;****************************************************************************
; Macro:        mSetUART_BRGHLow
;
; Overview:     Sets BRGH value to low.
;
; Input:        None
;
; Output:       None
;
; Side Effects: None
;****************************************************************************
mSetUART_BRGHLow  macro
        bcf     TXSTA,BRGH              ;Disable BRGH
        endm




;****************************************************************************
; Macro:        mSetUART_SPBRG  _SPBRGVal
;
; Overview:     Loads SPBRG register with content of W register
;
; Input:        SPBRG loading value as argument
;
; Output:       None
;
; Side Effects: W changed
;****************************************************************************
mSetUART_SPBRG         macro    _SPBRGVal
        bcf     RCSTA,SPEN              ;Disable Serial Port
        movlw   _SPBRGVal
        movwf   SPBRG                   ;Set SPBRG value
        bsf     RCSTA,SPEN              ;Enable Serial Port
        endm





;****************************************************************************
; Macro:        mSetUARTBaud _Baudrate
;
; Overview:     Loads SPBRG register with calculated value for reqd baud rate
;
; Input:        Baudrate value as argument
;
; Output:       None
;
; Side Effects: W changed
;****************************************************************************
mSetUARTBaud    macro   _Baudrate

#define         BRGH_HIGH_M      ;Select for BRGH=1


SPBRG_VM1 = CLOCK_FREQ/_Baudrate;

SPBRG_VM2 = SPBRG_VM1 /.16   ;
SPBRG_VM22 = (SPBRG_VM1 * .10) /.16   ;
        if ( ((SPBRG_VM2*.10) - SPBRG_VM22) >= .5)
        SPBRG_VM2 ++;
        endif

        if (SPBRG_VM2 > 0xff)
        SPBRG_VM2 / .16
        #undefine   BRGH_HIGH_M   ;BRGH = 0
        endif

SPBRG_MVAL=SPBRG_VM2 - D'1'   ;Calculated SPBRG register value

    if SPBRG_MVAL > .255
        error "Calculated SPBRG register value is out of range"
    endif

    if SPBRG_MVAL <= .10
        error "Calculated SPBRG register value is too low"
    endif


        bcf     RCSTA,SPEN              ;Disable Serial Port
        
        #ifdef  BRGH_HIGH_M
        mSetUART_BRGHHigh               ;Set BRGH high
        #else       
        mSetUART_BRGHLow                ;Set BRGH low
        #endif
        
        movlw   SPBRG_MVAL
        movwf   SPBRG                   ;Set SPBRG value

        bsf     RCSTA,SPEN              ;Enable Serial Port
        endm




#endif



;##############################################################################
; For PIC16xxx
;##############################################################################


#ifdef  _PIC16xxx

;****************************************************************************
; Macro:        mUARTRxIntDisable
;
; Overview:     Disables Receive interrupt.
;
; Input:        None
;
; Output:       None
;
; Side Effects: Databank changed
;****************************************************************************
mUARTRxIntDisable    macro
        banksel PIE1
        bcf     PIE1, RCIE                      ; Disable USART rx interrupt
        endm



;****************************************************************************
; Macro:        mUARTRxIntEnable
;
; Overview:     Enables Receive interrupt.
;
; Input:        None
;
; Output:       None
;
; Side Effects: Databank changed
;****************************************************************************
mUARTRxIntEnable     macro
        banksel PIE1
        bsf     PIE1, RCIE                      ; Enable USART rx interrupt
        endm



;****************************************************************************
; Macro:        mDisableUARTTxInt
;
; Overview:     Disables Transmit interrupt.
;
; Input:        None
;
; Output:       None
;
; Side Effects: Databank changed
;****************************************************************************
mDisableUARTTxInt    macro
        banksel PIE1
        bcf     PIE1, TXIE                      ; Disable USART Tx interrupt
        endm


;****************************************************************************
; Macro:        mEnableUARTTxInt
;
; Overview:     Enables Transmit interrupt.
;
; Input:        None
;
; Output:       None
;
; Side Effects: Databank changed
;****************************************************************************
mEnableUARTTxInt     macro
        banksel PIE1
        bsf     PIE1, TXIE                      ; Enable USART Tx interrupt
        endm



;****************************************************************************
; Macro:        mSetUART_BRGHHigh
;
; Overview:     Sets BRGH value to high
;
; Input:        None
;
; Output:       None
;
; Side Effects: Databank changed
;****************************************************************************
mSetUART_BRGHHigh         macro
        banksel TXSTA
        bsf     TXSTA,BRGH              ;Enable BRGH
        endm


;****************************************************************************
; Macro:        mSetUART_BRGHLow
;
; Overview:     Sets BRGH value to low.
;
; Input:        None
;
; Output:       None
;
; Side Effects: Databank changed
;****************************************************************************
mSetUART_BRGHLow          macro
        banksel TXSTA
        bcf     TXSTA,BRGH              ;Disable BRGH
        endm




;****************************************************************************
; Macro:        mSetUART_SPBRG _SPBRGVal
;
; Overview:     Loads SPBRG register with content of W register
;
; Input:        SPBRG value as argument
;
; Output:       None
;
; Side Effects: Databank, W changed
;****************************************************************************
mSetUART_SPBRG         macro   _SPBRGVal
        banksel RCSTA
        bcf     RCSTA,SPEN              ;Disable Serial Port
        movlw   _SPBRGVal
        bsf     STATUS,RP0              ;Bank-1
        movwf   SPBRG                   ;Set SPBRG value
        bcf     STATUS,RP0              ;Bank-0
        bsf     RCSTA,SPEN              ;Enable Serial Port
        endm




;****************************************************************************
; Macro:        mSetUARTBaud _Baudrate
;
; Overview:     Loads SPBRG register with caclulated value for required
;               baudrate        
;
; Input:        Baudrate value as argument
;
; Output:       None
;
; Side Effects: Databank, W changed
;****************************************************************************
mSetUARTBaud    macro   _Baudrate

#define         BRGH_HIGH_M      ;Select for BRGH=1


SPBRG_VM1 = CLOCK_FREQ/_Baudrate;

SPBRG_VM2 = SPBRG_VM1 /.16   ;
SPBRG_VM22 = (SPBRG_VM1 * .10) /.16   ;
        if ( ((SPBRG_VM2*.10) - SPBRG_VM22) >= .5)
        SPBRG_VM2 ++;
        endif

        if (SPBRG_VM2 > 0xff)
        SPBRG_VM2 / .16
        #undefine   BRGH_HIGH_M   ;BRGH = 0
        endif

SPBRG_MVAL=SPBRG_VM2 - D'1'   ;Calculated SPBRG register value

    if SPBRG_MVAL > .255
        error "Calculated SPBRG register value is out of range"
    endif

    if SPBRG_MVAL <= .10
        error "Calculated SPBRG register value is too low"
    endif


        banksel RCSTA
        bcf     RCSTA,SPEN              ;Disable Serial Port
        
        #ifdef  BRGH_HIGH_M
        mSetUART_BRGHHigh               ;Set BRGH high
        #else       
        mSetUART_BRGHLow                ;Set BRGH low
        #endif
        
        movlw   SPBRG_MVAL
        movwf   SPBRG                   ;Set SPBRG value

        bcf     STATUS,RP0              ;Bank-0
        bsf     RCSTA,SPEN              ;Enable Serial Port
        endm




#endif

#endif                          ;For inc file check.
