;############################################################################
;# TITLE   "USART General Purpose Library Checking Software"
;#
;#
;#      Program:        PIC16Tst.ASM
;#      Version:        1.0
;#      Revision Date:
;#      Author:         Gaurang Kavaiya
;#
;#
;# Program demonstrates use of USART general purpose library module.
;############################################################################




        list p=16f877A
        include <P16f877A.INC>

        include "UARTInt.inc"


UARTTstRAM      UDATA
ISR_STAT    RES 02			;For saving STATUS value
#define     ISR_PCLATH	ISR_STAT+1	;For saving PCLATH

;If device has shared RAM
UARTTstShr	 UDATA_SHR
ISR_W	    RES 01	               ;For Saving W reg. value


;If device does not have shared RAM then reserve one location in each bank
;UART_Shr       UDATA   0x7f
;ISR_W           RES     01

;UART_Shr_1     UDATA   0x7f
;ISR_W_1           RES     01

;UART_Shr_2     UDATA   0xff
;ISR_W_2           RES     01

;This is not required for most of the devices as bank

;UART_Shr_3     UDATA   0x17f
;ISR_W_3           RES     01

;UART_Shr_4     UDATA   0x1ff
;ISR_W_3           RES     01



STARThere       CODE    0x00
                goto    START



INTserv      CODE    0x04	     ;
	nop		             ;this is necessary because the Linker file only allows 1 instruction
				     ;if we use the goto instruction, the code might go to the wrong location
				     ;depending on PCLATH.  So using a nop allows the code to go to the next
				     ;location.  This is important for context saving.
;Else for better INT Latency

;	       movwf	ISR_W		     ;If INT latency is critical

InteruptServiceLocation     CODE    0x05
ISRoutine
;context savings (very important)
        movwf	ISR_W		     ;save Wreg in ISR_W, If NOP is used above
	swapf	STATUS,W
	banksel ISR_STAT
	movwf	ISR_STAT	     ;put STATUSreg (swapped) into ISR_STAT
	movf	PCLATH,W
	movwf	ISR_PCLATH	     ;put PCLATH into ISR_PCLATH

;call the interrupt function
	pagesel     UARTIntISR
	call	    UARTIntISR	     ;Call general purpose RTC interrupt service routine

;restore context
	banksel ISR_STAT
	movf	ISR_PCLATH,W	    ;put ISR_PCLATH back into PCLATH
	movwf	PCLATH
	swapf	ISR_STAT,W
	movwf	STATUS		    ;swap ISR_STAT back into STATUSreg
	swapf	ISR_W,f 	    ;swap ISR_W into itself
	swapf	ISR_W,W 	    ;swap ISR_W into Wreg.
	retfie






Main    CODE

START
        ;Define the required TRIS and PORT settings here
        
;Make sure that UART pins are defined as i/p
        pagesel UARTIntInit
        call    UARTIntInit


;Display Hello

        movlw   'H'
        Pagesel UARTIntPutCh
        call    UARTIntPutCh
        movlw   'E'
        call    UARTIntPutCh
        movlw   'L'
        call    UARTIntPutCh
        movlw   'L'
        call    UARTIntPutCh
        movlw   'O'
        call    UARTIntPutCh



;Change the pre-claculated baudrate, If required
;        mSetUARTBaud    .9600


        banksel vUARTIntStatus
WaitRxData
;Check if Receive buffer is full        
        btfss   vUARTIntStatus,UARTIntRxBufFul
        goto    WaitRxData


;If receive buffer is full then read the data
ReadAgain
        Pagesel UARTIntGetCh
        call    UARTIntGetCh

;check if Tx buffer is empty
        banksel vUARTIntStatus
WaitForTxBufEmpty
        btfsc   vUARTIntStatus,UARTIntTxBufFul
        goto    WaitForTxBufEmpty

;Echo back the received data                
        Pagesel UARTIntPutCh
        call    UARTIntPutCh

;Check if Rx Buffer is empty. If not keep reading it.
        banksel vUARTIntStatus
        btfss   vUARTIntStatus,UARTIntRxBufEmpty
        goto    ReadAgain

        goto    WaitRxData


        END
