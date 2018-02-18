;*********************************************************************
;*
;*  This implements a generic library functionality to support UART
;*  for PIC16 family
;*  It adds additional functionality of Rx/Tx user defined FIFO buffer
;*
;*********************************************************************
;* FileName:            UARTInt.asm
;* Dependencies:        P16xxx.inc
;*                      UARTInt.inc
;* Processor:           PIC16xxxx
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
;* Gaurang Kavaiya      Sep 27, 2002    Modified for Maestro 
;* Gaurang Kavaiya      Feb 11, 2003    Enhancements for MpAM
;* Gaurang Kavaiya      Mar 7,  2003    Optimization & clean-up
;* Gaurang Kavaiya      Mar 12, 2003    Removed Liner FIFO Buf Suprt<V1.0>
;********************************************************************/

        errorlevel      -302            ;Ignore Banksel warning

_UARTIntcode   code

;****************************************************************************
; Function: UARTIntInit
;
; PreCondition: None
;
; Overview:
;       This routine is used for Serial Port Initialization
;       It initializes Port according to compile time selection and
;       flushes the Rx and Tx buffer. It clears all USART errors
;
; Input: None
;
;
; Output: None
;
; Side Effects: Databank, W changed
;
; Stack requirement: 1 level deep
;
;****************************************************************************

UARTIntInit:

        GLOBAL      UARTIntInit

        movlw   SPBRG_VAL       ;Initial baud rate Value
        banksel SPBRG
        movwf   SPBRG           ;Set Baud Rate

        banksel TXSTA
        clrf    TXSTA           ;Clear Transmit Status Register

#ifdef  UARTINT_TXON
        bsf     TXSTA,TXEN      ;Tx. Enabled
#endif


#ifdef  BRGH_HIGH
        bsf     TXSTA,BRGH      ;Set BRGH value HIGH
#endif

#ifdef  UARTINT_TXON
        banksel PIE1
        bcf     PIE1,TXIE
#endif
        
#ifdef  UARTINT_RXON
        bsf     PIE1,RCIE       ;Enable receive interrupt
#endif

        banksel RCSTA
        clrf    RCSTA           ;Clear Receive status register
        bsf     RCSTA,SPEN      ;Enable Serial Port
        bsf     RCSTA,CREN      ;Enable Continuous Receive


#ifdef  UARTINT_TXON
        banksel vUARTIntTxBufDataCnt
        clrf    vUARTIntTxBufDataCnt
        clrf    vUARTIntTxBufRdPtr
        clrf    vUARTIntTxBufWrPtr
#endif

#ifdef  UARTINT_RXON
        banksel vUARTIntRxBufDataCnt        
        clrf    vUARTIntRxBufDataCnt
        clrf    vUARTIntRxBufRdPtr
        clrf    vUARTIntRxBufWrPtr
#endif
        
        banksel vUARTIntStatus
        clrf    vUARTIntStatus      ;Clear all the Errors



#ifdef  UARTINT_TXON

        banksel INTCON
        bsf     INTCON,GIE      ;Enable Global Interrupt
        bsf     INTCON,PEIE     ;Enable Peripheral Interrupt

#else

        #ifdef  UARTINT_RXON
        banksel INTCON
        bsf     INTCON,GIE      ;Enable Global Interrupt
        bsf     INTCON,PEIE     ;Enable Peripheral Interrupt      
        #endif  
#endif        


        return








#ifdef  UARTINT_RXON
;****************************************************************************
; Function: UARTIntGetCh
;
; PreCondition: None
;
; Overview:
;       It reads data in Receive Buffer. If vUARTIntRxBuffer is already empty
;       then it will set UARTIntRxBufEmpty bit in vUARTIntStatus.
;       Otherwise returns received data in Wreg and accordingly
;       adjusts vUARTIntRxBufDataCnt.
	;
; Input:    None
;
; Output:   If success UARTIntRxBufEmpty=0 else UARTIntRxBufEmpty=1,
;       UARTIntRxBufEmpty is defined in vUARTIntStatus
;       If success Received byte in W register
;
;
; Side Effects: Databank, FSR, W , STATUS value changed
;
; Stack requirement: 1 level deep
;
;****************************************************************************

UARTIntGetCh:

        GLOBAL  UARTIntGetCh        ;

        banksel vUARTIntRxBufDataCnt
        movf    vUARTIntRxBufDataCnt,W
        btfss   STATUS,Z                ;Check if data is availabe in Rx buf
        goto    TransferRecdData

        bsf     vUARTIntStatus,UARTIntRxBufEmpty   ;Set UARTIntRxBufEmpty flag
        retlw   0

TransferRecdData    
        bankisel vUARTIntRxBuffer       ;The UDATA segments ensures that whole
        movf    vUARTIntRxBufRdPtr,W    ;buffer will be in one bank. Therefore
        addlw   low(vUARTIntRxBuffer)   ;setting IRP bit based on buffer
                                        ;start address is OK
        movwf   FSR                     ;Point FSR to Wrtie location
        incf    vUARTIntRxBufRdPtr,F    ;Increment Read pointer
        movlw   UARTINT_RX_BUFFER_SIZE  ;If read pointer has reached the maximum
        xorwf   vUARTIntRxBufRdPtr,W    ;value then reset it for roll-over
        btfsc   STATUS,Z
        clrf    vUARTIntRxBufRdPtr      

        movf    INDF,W                  ;Read the FIFO buffer data

        decf    vUARTIntRxBufDataCnt,F  ;Decrement vUARTIntRxBuffer data size
        bcf     vUARTIntStatus,UARTIntRxBufFul    ;is read so Buffer has space
        bcf     vUARTIntStatus,UARTIntRxBufOF  ;for the new data

        return
#endif








#ifdef  UARTINT_TXON
;****************************************************************************
; Function: UARTIntPutCh
;
; PreCondition: None
;
; Overview:
;       It writes Content of W reg. in Transmit Buffer. If vUARTIntTxBuffer
;       is already empty then it immediately transmits the data. If
;       Buffer is already full then it returns without any job.
;       Otherwise it puts the data to be transmitted in vUARTIntTxBuffer and
;       accordingly adjusts vUARTIntTxBufDataCnt.
;
; Input:    Data to be transmitted in W reg.
;
; Output:   If vUARTIntTxBuffer becomes full it sets UARTIntTxBufFul flag bit in
;       USARTErros
;
; Side Effects: Databank , FSR, W , STATUS value changed
;
; Stack requirement: 1 level deep
;
;****************************************************************************

UARTIntPutCh:

        GLOBAL  UARTIntPutCh

        banksel temp3
        movwf   temp3           ;Store Value in Temp. location

        btfsc   vUARTIntStatus,UARTIntTxBufFul    ;Check for UARTIntTxBufFul bit
        return                  ;If buffer Full then return

        movf    vUARTIntTxBufDataCnt,W ;If vUARTIntTxBuffer is empty then transfer
        btfss   STATUS,Z        ;data for immediate transmission
        goto    AppendTxBuffer

        movf    temp3,W         ;from temporary location
        banksel TXREG
        movwf   TXREG           ;Immediately Transmit Data

        banksel vUARTIntTxBufDataCnt
        incf    vUARTIntTxBufRdPtr,F    ;Increment Read pointer
        movlw   UARTINT_TX_BUFFER_SIZE  ;If read pointer has reached the maximum
        xorwf   vUARTIntTxBufRdPtr,W    ;value then reset it for roll-over
        btfsc   STATUS,Z
        clrf    vUARTIntTxBufRdPtr      

;copy the data in buffer for Ptr Updates


AppendTxBuffer
    
        bankisel vUARTIntTxBuffer       ;The UDATA segments ensures that whole
        movf    vUARTIntTxBufWrPtr,W    ;buffer will be in one bank. Therefore
        addlw   low(vUARTIntTxBuffer)   ;setting IRP bit based on buffer
                                        ;start address is OK
        movwf   FSR                     ;Point FSR to Wrtie location
        incf    vUARTIntTxBufWrPtr,F    ;Increment Write pointer
        movlw   UARTINT_TX_BUFFER_SIZE  ;If Write pointer has reached the maximum
        xorwf   vUARTIntTxBufWrPtr,W    ;value then reset it for roll-over
        btfsc   STATUS,Z
        clrf    vUARTIntTxBufWrPtr      

        incf    vUARTIntTxBufDataCnt,F  ;Decrement vUARTIntRxBuffer data size
        movlw   UARTINT_TX_BUFFER_SIZE  ;If Write pointer has reached the maximum
        xorwf   vUARTIntTxBufDataCnt,W  ;value then reset it for roll-over
        btfss   STATUS,Z
        goto    TxBufNotFull

        bsf     vUARTIntStatus,UARTIntTxBufFul    ;is Buffer s full


TxBufNotFull
        movf    temp3,W
        movwf   INDF                    ;Copy the data into FIFO buffer

        banksel PIE1
        bsf     PIE1,TXIE       ;Enable TX interrupt

        return
#endif









;****************************************************************************
; Function: UARTIntISR
;
; PreCondition: None
;
; Overview:
;       This is a Interrupt service routine for Serial Interrupt.
;       It handles Reception and Transmission of data on interrupt.
;       Call it from Interrupt service routine at proper Interrupt
;       Vector (High or Low priority Vector)
;
; Input:
;
;
; Output:   If data is received it puts it in vUARTIntRxBuffer and accordingly
;       adjusts the RxBufferRdPtr and clears UARTIntRxBufEmpty flag.
;       If Receive Buffer becomes full then it will set UARTIntRxBufFul
;       bit. If data is received when Receive buffer was full it will
;       set UARTIntRxBufOF flag to indicate that transmitted data has
;       been missed because of full vUARTIntRxBuffer. If any error is
;       generated in reception it will set UARTIntRxError flag bit.
;
;       If last data is transmitted then it will transmit next pending
;       data if any. It will accordingly adjust the vUARTIntTxBufDataCnt. It
;       will clear the UARTIntTxBufFul bit to indicate space for data in
;       vUARTIntTxBuffer.
;
; Side Effects: Databank, W, STATUS changed
;
; Stack requirement: 1 level deep
;
;****************************************************************************

UARTIntISR:
        GLOBAL  UARTIntISR


        movf    FSR,W
        banksel ISR_FSR0L       ;Save FSR
        movwf   ISR_FSR0L

        banksel PIR1
        btfss   PIR1,TXIF       ;Check for Tx Interrupt
        goto    ChkReceiver

#ifdef  UARTINT_TXON
        banksel PIE1
        btfss   PIE1,TXIE       ;Check if TX int. enabled
        goto    ChkReceiver

        banksel vUARTIntStatus     ;Clear UARTIntTxBufFul Error bit as Data
        bcf     vUARTIntStatus,UARTIntTxBufFul    ;is transmitted so Buffer has
                                ;space for new data

        decfsz  vUARTIntTxBufDataCnt,F     ;is already transmitted.
        goto    TransmitData

;Place for possible Tx Buffer Empty flag

        banksel PIE1
        bcf     PIE1,TXIE       ;Disable TX interrupt until we have
                                ;more data to transmit
        goto    ChkReceiver     ;Chk if any data is received


TransmitData
        bankisel vUARTIntTxBuffer       ;The UDATA segments ensures that whole
        movf    vUARTIntTxBufRdPtr,W    ;buffer will be in one bank. Therefore
        addlw   low(vUARTIntTxBuffer)   ;setting IRP bit based on buffer
                                        ;start address is OK     
        movwf   FSR                     ;Point FSR to Wrtie location
        incf    vUARTIntTxBufRdPtr,F    ;Increment Read pointer
        movlw   UARTINT_TX_BUFFER_SIZE  ;If read pointer has reached the maximum
        xorwf   vUARTIntTxBufRdPtr,W    ;value then reset it for roll-over
        btfsc   STATUS,Z
        clrf    vUARTIntTxBufRdPtr      

        movf    INDF,W
        banksel TXREG           ;Transmit Next Data
        movwf   TXREG

#endif


ChkReceiver
 #ifdef  UARTINT_RXON
        banksel PIE1
        btfss   PIE1,RCIE       ;Check for Receive Interrupt
        goto    EndISR          ;Some other interrupt, exit
        
        banksel PIR1
        btfss   PIR1,RCIF       ;Check for Receive Interrupt
        goto    EndISR          ;Some other interrupt, exit

;Not required as read from RCREG will clear it
;        bcf     PIR1,RCIF       ;Clear Receive Interrupt

        banksel RCREG
        movf    RCREG,W         ;Read RCREG data to clear the interrupt
        banksel temp1   
        movwf   temp1

        banksel RCSTA
        movlw   06h             ;Mask out unwanted bits
        andwf   RCSTA,W         ;Check for errors
        btfss   STATUS,Z
        goto    RcvError        ;Found error, flag it

        banksel vUARTIntStatus     ;
        btfsc   vUARTIntStatus,UARTIntRxBufOF  ;Check for UARTIntRxBufOF bit
        goto    EndISR          ;If buffer Full then return

        btfss   vUARTIntStatus,UARTIntRxBufFul    ;Check for UARTIntRxBufFul bit
        goto    AppendRxBuffer   ;if Buffer Full then set Rx Buffer
        bsf     vUARTIntStatus,UARTIntRxBufOF  ;Over Flow flag to indicate that
        goto    EndISR           ;data is missed.


AppendRxBuffer
        banksel vUARTIntRxBufDataCnt
        bankisel vUARTIntRxBuffer       ;The UDATA segments ensures that whole
        movf    vUARTIntRxBufWrPtr,W    ;buffer will be in one bank. Therefore
        addlw   low(vUARTIntRxBuffer)   ;setting IRP bit based on buffer
                                        ;start address is OK     
        movwf   FSR                     ;Point FSR to Wrtie location
        incf    vUARTIntRxBufWrPtr,F    ;Increment Write pointer
        movlw   UARTINT_RX_BUFFER_SIZE  ;If Write pointer has reached the maximum
        xorwf   vUARTIntRxBufWrPtr,W    ;value then reset it for roll-over
        btfsc   STATUS,Z
        clrf    vUARTIntRxBufWrPtr      

        bcf     vUARTIntStatus, UARTIntRxBufEmpty
        incf    vUARTIntRxBufDataCnt,F  ;Decrement vUARTIntRxBuffer data size
        movlw   UARTINT_RX_BUFFER_SIZE  ;If Buffer has reached the maximum
        xorwf   vUARTIntRxBufDataCnt,W  ;value then set the falg for full
        btfsc   STATUS,Z
		bsf     vUARTIntStatus,UARTIntRxBufFul    ;in vUARTIntStatus
		;goto    RxBufFull

        movf    temp1,W
        movwf   INDF                    ;Copy the data into FIFO buffer
        goto    EndISR

; Corrigido por Hernandi em 21-01-2010
; se CARACTERE recebido for o ultimo q cabe, o buffer nao pode ficar FULL
; e sair, precisa armazenar este ultimo dado
RxBufFull
        ;bsf     vUARTIntStatus,UARTIntRxBufFul    ;in vUARTIntStatus
        ;goto    EndISR
		;goto 


RcvError
        banksel RCSTA          
        bcf     RCSTA,CREN      ;Clear reciever status
        bsf     RCSTA,CREN
        banksel vUARTIntStatus
        bsf     vUARTIntStatus,UARTIntRxError ;Set Data Error flag
#endif


EndISR
        banksel ISR_FSR0L
        movf    ISR_FSR0L,W
        movwf   FSR             ;Restore FSR

        return



