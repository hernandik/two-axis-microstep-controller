;*********************************************************************
;*
;*  The generic include file for PIC16 family
;*
;*********************************************************************
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
;* Author               Date            Comment
;*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;* Gaurang Kavaiya      Nov 17, 2000    Initial Release (V1.0)
;* Gaurang Kavaiya      Feb 4,  2003    Modified for CLM (V1.1)
;* Gaurang Kavaiya      Feb 10, 2003    Modified for Backward compatible
;*                                      module support (V1.2).
;* Vidyadhar            Feb 27, 2003    Added BSSP, SSP, MSSP Modules (V1.3)
;* Gaurang Kavaiya      Mar 7, 2003     AUSART issues clean-up (V1.4)
;* Gaurang Kavaiya      Mar 11, 2003    Added Timer 1 support (V1.5)
;* Vidyadhar            Mar 17, 2003    BSSP, SSP, MSSP issues clean-up (V1.6)
;********************************************************************/

;#ifndef __P16xxx_INC           ;The file inclusion check is not added to support
                                ;recursive inclusion. The individual definitions
                                ;must be checked.

;        #define __P16xxx_INC



;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C554
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C554

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C554.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C554 does not have USART"
                        #endif
                #endif




;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C554 does not have AUSART"
                        #endif
                #endif



;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16c554 does not have EUSART"
                        #endif
                #endif


;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16c554 does not have A/D"
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C554 does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16c554 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C554 does not have 12-bit A/D"
                        #endif
                #endif



;BSSP module
                #ifdef  BSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16c554 does not have BSSP"
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16c554 does not have SSP"
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16c554 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16c554 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C554 does not have Timer 1"
                        #endif
                #endif

        #endif                  ;End of device definition










;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C558
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C558

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C558.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C558 does not have USART"
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C558 does not have AUSART"
                        #endif
                #endif



;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16c558 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C558 does not have A/D"
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C558 does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C558 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C558 does not have 12-bit A/D"
                        #endif
                #endif




;BSSP module
                #ifdef  BSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16c558 does not have BSSP"
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16c558 does not have SSP"
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16c558 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16c558 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C558 does not have Timer 1"
                        #endif
                #endif



        #endif                  ;End of device definition








;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C62B
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C62B

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C62B.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C62B does not have USART"
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C62B does not have AUSART"
                        #endif
                #endif


;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C62B does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C62B does not have A/D"
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                 error "PIC 16C62B does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C62B does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C62B does not have 12-bit A/D"
                        #endif
                #endif



;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C62B does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C62B does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif



        #endif                  ;End of device definition






;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C63
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C63

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C63.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C63 does not have AUSART"
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C63 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C63 does not have A/D"
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C63 does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C63 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C63 does not have 12-bit A/D"
                        #endif
                #endif




;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C63 does not have SSP"
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C63 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C63 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif


;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif


        #endif                  ;End of device definition








;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C63A
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C63A

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C63A.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C63A does not have AUSART"
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C63A does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C63A does not have A/D"
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C63A does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C63A does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif

;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C63A does not have 12-bit A/D"
                        #endif
                #endif




;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C63A does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C63A does not have MSSP with Clock Stretching facility"
                        #endif
                #endif


;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif



        #endif                  ;End of device definition







;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16CR63
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16CR63

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16CR63.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR63 does not have AUSART"
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR63 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR63 does not have A/D"
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR63 does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR63 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR63 does not have 12-bit A/D"
                        #endif
                #endif




;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR63 does not have SSP"
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR63 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR63 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif


        #endif                  ;End of device definition







;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C65B
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C65B

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C65B.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                       #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C65B does not have AUSART"
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C65B does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C65B does not have A/D"
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C65B does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C65B does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C65B does not have 12-bit A/D"
                        #endif
                #endif




;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C65B does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C65B does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif


        #endif                  ;End of device definition








;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16CR65
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16CR65

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16CR65.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR65 does not have AUSART"
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR65 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR65 does not have A/D"
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR65 does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR65 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif

;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR65 does not have 12-bit A/D"
                        #endif
                #endif





;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR65 does not have SSP"
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR65 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR65 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif


        #endif                  ;End of device definition









;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C66
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C66

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C66.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C66 does not have AUSART"
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C66 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C66 does not have A/D"
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C66 does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C66 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C66 does not have 12-bit A/D"
                        #endif
                #endif





;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C66 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C66 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition







;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C67
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C67

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C67.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C67 does not have AUSART"
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C67 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C67 does not have A/D"
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C67 does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C67 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C67 does not have 12-bit A/D"
                        #endif
                #endif




;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C67 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C67 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif



        #endif                  ;End of device definition







;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C432
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C432

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C432.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C432 does not have USART"
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C432 does not have AUSART"
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C432 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C432 does not have A/D"
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C432 does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C432 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C432 does not have 12-bit A/D"
                        #endif
                #endif



;BSSP module
                #ifdef  BSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C432 does not have BSSP"
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C432 does not have SSP"
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C432 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C432 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C432 does not have Timer 1"
                        #endif
                #endif

        #endif                  ;End of device definition








;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C433
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C433

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C433.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C433 does not have USART"
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C433 does not have AUSART"
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C433 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C433 does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C433 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C433 does not have 12-bit A/D"
                        #endif
                #endif




;BSSP module
                #ifdef  BSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C433 does not have BSSP"
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C433 does not have SSP"
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C433 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C433 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C433 does not have Timer 1"
                        #endif
                #endif



        #endif                  ;End of device definition








;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C620A
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C620A

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C620A.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C620A does not have USART"
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                       #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C620A does not have AUSART"
                        #endif
                #endif


;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C620A does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C620A does not have A/D"
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                       #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C620A does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                       #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C620A does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif

;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C620A does not have 12-bit A/D"
                        #endif
                #endif




;BSSP module
                #ifdef  BSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C620A does not have BSSP"
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C620A does not have SSP"
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C620A does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C620A does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C620A does not have Timer 1"
                        #endif
                #endif

        #endif                  ;End of device definition





;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16CR620A
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16CR620A

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16CR620A.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR620A does not have USART"
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                       #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR620A does not have AUSART"
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR620A does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR620A does not have A/D"
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR620A does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR620A does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif

;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                         #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR620A does not have 12-bit A/D"
                        #endif
                #endif



;BSSP module
                #ifdef  BSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR620A does not have BSSP"
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR620A does not have SSP"
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR620A does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR620A does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR620A does not have Timer 1"
                        #endif
                #endif


        #endif                  ;End of device definition









;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C621A
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C621A

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C621A.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C621A does not have USART"
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C621A does not have AUSART"
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C621A does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR621A does not have A/D"
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                       #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR621A does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR621A does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif

;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C621A does not have 12-bit A/D"
                        #endif
                #endif



;BSSP module
                #ifdef  BSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C621A does not have BSSP"
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C621A does not have SSP"
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C621A does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C621A does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C621A does not have Timer 1"
                        #endif
                #endif

        #endif                  ;End of device definition









;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C622A
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C622A

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C622A.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                       #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C622A does not have USART"
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C622A does not have AUSART"
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                       #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C622A does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                       #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR622A does not have A/D"
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR622A does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR622A does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif

;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR622A does not have 12-bit A/D"
                        #endif
                #endif



;BSSP module
                #ifdef  BSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C622A does not have BSSP"
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C622A does not have SSP"
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C622A does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C622A does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C622A does not have Timer 1"
                        #endif
                #endif

        #endif                  ;End of device definition









;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16CE623
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16CE623

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16CE623.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CE623 does not have USART"
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CE623 does not have AUSART"
                        #endif
                #endif


;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CE623 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                       #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CE623 does not have A/D"
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CE623 does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CE623 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif

;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CE623 does not have 12-bit A/D"
                        #endif
                #endif



;BSSP module
                #ifdef  BSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CE623 does not have BSSP"
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CE623 does not have SSP"
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CE623 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CE623 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CE623 does not have Timer 1"
                        #endif
                #endif

        #endif                  ;End of device definition











;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16CE624
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16CE624

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16CE624.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CE624 does not have USART"
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CE624 does not have AUSART"
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CE624 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CE624 does not have A/D"
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CE624 does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CE624 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CE624 does not have 12-bit A/D"
                        #endif
                #endif




;BSSP module
                #ifdef  BSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CE624 does not have BSSP"
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CE624 does not have SSP"
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CE624 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CE624 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CE624 does not have Timer 1"
                        #endif
                #endif

        #endif                  ;End of device definition










;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16CE625
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16CE625

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16CE625.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CE625 does not have USART"
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CE625 does not have AUSART"
                        #endif
                #endif


;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CE625 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                         #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CE625 does not have A/D"
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CE625 does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CE625 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                         #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CE625 does not have 12-bit A/D"
                        #endif
                #endif








;BSSP module
                #ifdef  BSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CE625 does not have BSSP"
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CE625 does not have SSP"
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CE625 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CE625 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CE625 does not have Timer 1"
                        #endif
                #endif

        #endif                  ;End of device definition








;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C642
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C642

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C642.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C642 does not have USART"
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C642 does not have AUSART"
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C642 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C642 does not have A/D"
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C642 does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C642 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C642 does not have 12-bit A/D"
                        #endif
                #endif


;BSSP module
                #ifdef  BSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C642 does not have BSSP"
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C642 does not have SSP"
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C642 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C642 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C642 does not have Timer 1"
                        #endif
                #endif

        #endif                  ;End of device definition









;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C662
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C662

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C662.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C662 does not have USART"
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C662 does not have AUSART"
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C662 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C662 does not have A/D"
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C662 does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C662 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C662 does not have 12-bit A/D"
                        #endif
                #endif




;BSSP module
                #ifdef  BSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C662 does not have BSSP"
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C662 does not have SSP"
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C662 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C662 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif
;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C662 does not have Timer 1"
                        #endif
                #endif


        #endif                  ;End of device definition









;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C710
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C710

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C710.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C710 does not have USART"
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C710 does not have AUSART"
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C710 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                       #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C710 does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C710 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif

;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C710 does not have 12-bit A/D"
                        #endif
                #endif


;BSSP module
                #ifdef  BSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C710 does not have BSSP"
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C710 does not have SSP"
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C710 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C710 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C710 does not have Timer 1"
                        #endif
                #endif



        #endif                  ;End of device definition








;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C711
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C711

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C711.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C711 does not have USART"
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C711 does not have AUSART"
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C711 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C711 does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C711 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C711 does not have 12-bit A/D"
                        #endif
                #endif



;BSSP module
                #ifdef  BSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C711 does not have BSSP"
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C711 does not have SSP"
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C711 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C711 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif
;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C711 does not have Timer 1"
                        #endif
                #endif

        #endif                  ;End of device definition










;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C712
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C712

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C712.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C712 does not have USART"
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C712 does not have AUSART"
                        #endif
                #endif


;EUSART module
                #ifdef  EUSART_MODULE
                         #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C712 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C712 does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C712 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif

;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C712 does not have 12-bit A/D"
                        #endif
                #endif


;BSSP module
                #ifdef  BSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C712 does not have BSSP"
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C712 does not have SSP"
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C712 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C712 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition





;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C715
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C715

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C715.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C715 does not have USART"
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C715 does not have AUSART"
                        #endif
                #endif


;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C715 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C715 does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C715 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C715 does not have 12-bit A/D"
                        #endif
                #endif


;BSSP module
                #ifdef  BSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C715 does not have BSSP"
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C715 does not have SSP"
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C715 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C715 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C715 does not have Timer 1"
                        #endif
                #endif


        #endif                  ;End of device definition







;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C716
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C716

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C716.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C716 does not have USART"
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C716 does not have AUSART"
                        #endif
                #endif


;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C716 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C716 does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C716 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C716 does not have 12-bit A/D"
                        #endif
                #endif





;BSSP module
                #ifdef  BSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C716 does not have BSSP"
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C716 does not have SSP"
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C716 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C716 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition





;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C717
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C717

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C717.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C717 does not have USART"
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C717 does not have AUSART"
                        #endif
                #endif


;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C717 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C717 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C717 does not have 12-bit A/D"
                        #endif
                #endif



;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif

                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif

                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C717 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition












;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C72A
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C72A

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C72A.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C72A does not have USART"
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C72A does not have AUSART"
                        #endif
                #endif


;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C72A does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C72A does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C72A does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif

;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C72A does not have 12-bit A/D"
                        #endif
                #endif





;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C72A does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C72A does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif


        #endif                  ;End of device definition







;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16CR72
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16CR72

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16CR72.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR72 does not have USART"
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR72 does not have AUSART"
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR72 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR72 does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR72 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR72 does not have 12-bit A/D"
                        #endif
                #endif




;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR72 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16CR72 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif


        #endif                  ;End of device definition










;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C73A
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C73A

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C73A.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C73A does not have AUSART"
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C73A does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C73A does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C73A does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C73A does not have 12-bit A/D"
                        #endif
                #endif



;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C73A does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C73A does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif


        #endif                  ;End of device definition








;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C73B
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C73B

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C73B.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C73B does not have AUSART"
                        #endif
                #endif


;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C73B does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C73B does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C73B does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif

;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C73B does not have 12-bit A/D"
                        #endif
                #endif




;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C73B does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C73B does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition









;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C74A
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C74A

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C74A.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C74A does not have AUSART"
                        #endif
                #endif



;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C74A does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C74A does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C74A does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif

;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C74A does not have 12-bit A/D"
                        #endif
                #endif




;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C74A does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C74A does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition











;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C74B
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C74B

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C74B.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C74B does not have AUSART"
                        #endif
                #endif


;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C74B does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C74B does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C74B does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif

;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C74B does not have 12-bit A/D"
                        #endif
                #endif



;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C74B does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C74B does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition








;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C76
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C76

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C76.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C76 does not have AUSART"
                        #endif
                #endif


;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C76 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C76 does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C76 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif

;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C76 does not have 12-bit A/D"
                        #endif
                #endif





;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C76 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C76 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif
;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition










;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C77
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C77

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C77.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C77 does not have AUSART"
                        #endif
                #endif



;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C77 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C77 does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C77 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C77 does not have 12-bit A/D"
                        #endif
                #endif



;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C77 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C77 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition








;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C770
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C770

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C770.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C770 does not have USART"
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C770 does not have AUSART"
                        #endif
                #endif


;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C770 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif

                        #ifndef _A2D_12BIT_MODULE
                                #define _A2D_12BIT_MODULE      ;Backward compatible
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif

                        #ifndef _A2D_12BIT_MODULE
                                #define _A2D_12BIT_MODULE      ;Backward compatible
                        #endif

                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C770 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifndef _A2D_12BIT_MODULE
                                #define _A2D_12BIT_MODULE      ;Backward compatible
                        #endif

                        #ifdef  _GEN_MODULE_ERROR
;                                error "PIC 16C770 does not have 12-bit A/D"
                        #endif
                #endif



;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif

                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif

                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C770 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition





;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C771
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C771

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C771.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C771 does not have USART"
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C771 does not have AUSART"
                        #endif
                #endif


;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C771 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif

                        #ifndef _A2D_12BIT_MODULE
                                #define _A2D_12BIT_MODULE      ;Backward compatible
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif

                        #ifndef _A2D_12BIT_MODULE
                                #define _A2D_12BIT_MODULE      ;Backward compatible
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C771 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifndef _A2D_12BIT_MODULE
                                #define _A2D_12BIT_MODULE      ;Backward compatible
                        #endif
                #endif




;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif

                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif

                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C771 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition










;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C773
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C773

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C773.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif

                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif

                        #ifndef _EUSART_MODULE
;                                #define _EUSART_MODULE
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C773 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif

                        #ifndef _A2D_12BIT_MODULE
                                #define _A2D_12BIT_MODULE      ;Backward compatible
                        #endif

                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif

                        #ifndef _A2D_12BIT_MODULE
                                #define _A2D_12BIT_MODULE      ;Backward compatible
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C773 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifndef _A2D_12BIT_MODULE
                                #define _A2D_12BIT_MODULE      ;Backward compatible
                        #endif
                #endif






;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif

                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif

                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C773 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition










;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C774
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C774

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C774.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif

                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif


;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C774 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif

                        #ifndef _A2D_12BIT_MODULE
                                #define _A2D_12BIT_MODULE      ;Backward compatible
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif

                        #ifndef _A2D_12BIT_MODULE
                                #define _A2D_12BIT_MODULE      ;Backward compatible
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C774 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifndef _A2D_12BIT_MODULE
                                #define _A2D_12BIT_MODULE      ;Backward compatible
                        #endif
                #endif




;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif

                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif

                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C774 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition








;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C745
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C745

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C745.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C745 does not have AUSART"
                        #endif
                #endif


;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C745 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C745 does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C745 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C745 does not have 12-bit A/D"
                        #endif
                #endif





;BSSP module
                #ifdef  BSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C745 does not have BSSP"
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C745 does not have SSP"
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C745 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C745 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition









;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C765
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C765

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C765.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C765 does not have AUSART"
                        #endif
                #endif


;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C765 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif
               #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C765 does not have 10-bit A/D"
                        #endif


                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C765 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C765 does not have 12-bit A/D"
                        #endif
                #endif


;BSSP module
                #ifdef  BSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C765 does not have BSSP"
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C765 does not have SSP"
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C765 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C765 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition









;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C781
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C781

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C781.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C781 does not have USART"
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C781 does not have AUSART"
                        #endif
                #endif


;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C781 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C781 does not have 10-bit A/D"
                        #endif


                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C781 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C781 does not have 12-bit A/D"
                        #endif
                #endif



;BSSP module
                #ifdef  BSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C781 does not have BSSP"
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C781 does not have SSP"
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C781 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C781 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition











;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C782
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C782

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C782.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C782 does not have USART"
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C782 does not have AUSART"
                        #endif
                #endif


;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C782 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C782 does not have 10-bit A/D"
                        #endif


                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C782 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C782 does not have 12-bit A/D"
                        #endif
                #endif




;BSSP module
                #ifdef  BSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C782 does not have BSSP"
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C782 does not have SSP"
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C782 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C782 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition







;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C923
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C923

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C923.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C923 does not have USART"
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C923 does not have AUSART"
                        #endif
                #endif



;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C923 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C923 does not have A/D"
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C923 does not have 10-bit A/D"
                        #endif


                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C923 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C923 does not have 12-bit A/D"
                        #endif
                #endif



;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C923 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C923 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition






;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C924
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C924

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C924.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C924 does not have USART"
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C924 does not have AUSART"
                        #endif
                #endif



;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C924 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif
               #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C924 does not have 10-bit A/D"
                        #endif


                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C924 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C924 does not have 12-bit A/D"
                        #endif
                #endif




;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C924 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C924 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition








;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C925
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C925

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C925.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C925 does not have USART"
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C925 does not have AUSART"
                        #endif
                #endif



;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C925 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C925 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C925 does not have 12-bit A/D"
                        #endif
                #endif



;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C925 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C925 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

 ;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

       #endif                  ;End of device definition









;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16C926
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16C926

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16C926.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C926 does not have USART"
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C926 does not have AUSART"
                        #endif
                #endif


;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C926 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif

                        ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C926 does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C926 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C926 does not have 12-bit A/D"
                        #endif
                #endif





;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C926 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16C926 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition





;*****************************************************************************
; FLASH devices
;*****************************************************************************




;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16F87
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16F87

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16F87.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif

                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif

                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif

                #endif


;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F87 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F87 does not have A/D"
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F87 does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F87 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F87 does not have 12-bit A/D"
                        #endif
                #endif



;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F87 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F87 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition










;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16F88
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16F88

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16F88.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif

                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F88 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif

                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif

                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F88 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F88 does not have 12-bit A/D"
                        #endif
                #endif








;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F88 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F88 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition










;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16F627
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16F627

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16F627.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F627 does not have AUSART"
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F627 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F627 does not have A/D"
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F627 does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F627 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F627 does not have 12-bit A/D"
                        #endif
                #endif




;BSSP module
                #ifdef  BSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F627 does not have BSSP"
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F627 does not have SSP"
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F627 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F627 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition






;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16F627A
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16F627A

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16F627A.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F627A does not have AUSART"
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F627A does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F627A does not have A/D"
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F627A does not have 10-bit A/D"
                        #endif


                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F627A does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F627A does not have 12-bit A/D"
                        #endif
                #endif


;BSSP module
                #ifdef  BSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F627A does not have BSSP"
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F627A does not have SSP"
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F627A does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F627A does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

 ;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

       #endif                  ;End of device definition








;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16F628
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16F628

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16F628.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F628 does not have AUSART"
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F628 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F628 does not have A/D"
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F628 does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F628 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F628 does not have 12-bit A/D"
                        #endif
                #endif



;BSSP module
                #ifdef  BSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F628 does not have BSSP"
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F628 does not have SSP"
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F628 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F628 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition











;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16F628A
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16F628A

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16F628A.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F628A does not have AUSART"
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F628A does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F628A does not have A/D"
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F628A does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F628A does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F628A does not have 12-bit A/D"
                        #endif
                #endif



;BSSP module
                #ifdef  BSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F628A does not have BSSP"
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F628A does not have SSP"
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F628A does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F628A does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition






;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16F630
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16F630

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16F630.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F630 does not have USART"
                        #endif
                #endif

;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F630 does not have AUSART"
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F630 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F630 does not have A/D"
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F630 does not have 10-bit A/D"
                        #endif


                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F630 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F630 does not have 12-bit A/D"
                        #endif
                #endif



;BSSP module
                #ifdef  BSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F630 does not have BSSP"
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F630 does not have SSP"
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F630 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F630 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition










;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16F648A
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16F648A

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16F648A.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif

                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F648A does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F648A does not have A/D"
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F648A does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F648A does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F648A does not have 12-bit A/D"
                        #endif
                #endif



;BSSP module
                #ifdef  BSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F648A does not have BSSP"
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F648A does not have SSP"
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F648A does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F648A does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition









;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16F676
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16F676

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16F676.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F676 does not have USART"
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F676 does not have AUSART"
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F676 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F676 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F676 does not have 12-bit A/D"
                        #endif
                #endif



;BSSP module
                #ifdef  BSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F676 does not have BSSP"
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F676 does not have SSP"
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F676 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F676 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif
        #endif                  ;End of device definition








;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16F72
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16F72

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16F72.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F72 does not have USART"
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F72 does not have AUSART"
                        #endif
                #endif


;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F72 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F72 does not have 10-bit A/D"
                        #endif
                 #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F72 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F72 does not have 12-bit A/D"
                        #endif
                #endif


;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F72 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F72 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif
        #endif                  ;End of device definition









;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16F73
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16F73

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16F73.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F73 does not have AUSART"
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F73 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F73 does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F73 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F73 does not have 12-bit A/D"
                        #endif
                #endif


;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F73 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F73 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition








;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16F74
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16F74

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16F74.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F74 does not have AUSART"
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F74 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif
               #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                       #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F74 does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F74 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F74 does not have 12-bit A/D"
                        #endif
                #endif



;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F74 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F74 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition








;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16F76
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16F76

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16F76.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F76 does not have AUSART"
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F76 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F76 does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F76 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F76 does not have 12-bit A/D"
                        #endif
                #endif



;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F76 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F76 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition









;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16F77
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16F77

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16F77.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F77 does not have AUSART"
                        #endif
                #endif
;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F77 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F77 does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F77 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F77 does not have 12-bit A/D"
                        #endif
                #endif



;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F77 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F77 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition









;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16F84
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16F84

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16F84.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F84 does not have USART"
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F84 does not have AUSART"
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F84 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F84 does not have A/D"
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F84 does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F84 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F84 does not have 12-bit A/D"
                        #endif
                #endif



;BSSP module
                #ifdef  BSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F84 does not have BSSP"
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                         #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F84 does not have SSP"
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F84 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F84 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F84 does not have Timer 1"
                        #endif
                #endif

        #endif                  ;End of device definition








;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16F84A
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16F84A

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16F84A.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################
;USART module
                #ifdef  USART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F84A does not have USART"
                        #endif
                #endif

;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F84A does not have AUSART"
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F84A does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F84A does not have A/D"
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F84A does not have 10-bit A/D"
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F84A does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F84A does not have 12-bit A/D"
                        #endif
                #endif



;BSSP module
                #ifdef  BSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F84A does not have BSSP"
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F84A does not have SSP"
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F84A does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F84A does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F84A does not have Timer 1"
                        #endif
                #endif
        #endif                  ;End of device definition







;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16F818
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16F818

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16F818.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F818 does not have USART"
                        #endif
                #endif

;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F818 does not have AUSART"
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F818 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F818 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F818 does not have 12-bit A/D"
                        #endif
                #endif



;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F818 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F818 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition








;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16F819
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16F819

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16F819.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F819 does not have USART"
                        #endif
                #endif

;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F819 does not have AUSART"
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F819 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F819 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F819 does not have 12-bit A/D"
                        #endif
                #endif




;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F819 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F819 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

         #endif                  ;End of device definition









;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16F870
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16F870

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16F870.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif

                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F870 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F870 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F870 does not have 12-bit A/D"
                        #endif
                #endif




;BSSP module
                #ifdef  BSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F870 does not have BSSP"
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F870 does not have SSP"
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F870 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F870 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition







;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16F871
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16F871

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16F871.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif

                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F871 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F871 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif

;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F871 does not have 12-bit A/D"
                        #endif
                #endif



;BSSP module
                #ifdef  BSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F871 does not have BSSP"
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F871 does not have SSP"
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F871 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F871 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition








;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16F872
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16F872

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16F872.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error   "PIC16F872 does not have USART"
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F872 does not have AUSART"
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F872 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F872 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F872 does not have 12-bit A/D"
                        #endif
                #endif




;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif

                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif

                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F872 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition








;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16F873
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16F873

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16F873.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif

                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F873 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F873 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif

;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F873 does not have 12-bit A/D"
                        #endif
                #endif



;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif

                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif

                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F873 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition









;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16F873A
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16F873A

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16F873A.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif

                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F873A does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F873A does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F873A does not have 12-bit A/D"
                        #endif
                #endif



;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif

                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif

                        #ifndef _MSSP_CLOCK_STRETCH_MODULE
                                #define _MSSP_CLOCK_STRETCH_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif

                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif

                        #ifndef _MSSP_CLOCK_STRETCH_MODULE
                                #define _MSSP_CLOCK_STRETCH_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif

                        #ifndef _MSSP_CLOCK_STRETCH_MODULE
                                #define _MSSP_CLOCK_STRETCH_MODULE
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifndef _MSSP_CLOCK_STRETCH_MODULE
                                #define _MSSP_CLOCK_STRETCH_MODULE
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

         #endif                  ;End of device definition









;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16F874
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16F874

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16F874.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif

                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F874 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif

                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F874 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F874 does not have 12-bit A/D"
                        #endif
                #endif


;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif

                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif

                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F874 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition








;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16F874A
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16F874A

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16F874A.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif

                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F874A does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif

                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F874A does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F874A does not have 12-bit A/D"
                        #endif
                #endif










;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif

                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif

                        #ifndef _MSSP_CLOCK_STRETCH_MODULE
                                #define _MSSP_CLOCK_STRETCH_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif

                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif

                        #ifndef _MSSP_CLOCK_STRETCH_MODULE
                                #define _MSSP_CLOCK_STRETCH_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif

                        #ifndef _MSSP_CLOCK_STRETCH_MODULE
                                #define _MSSP_CLOCK_STRETCH_MODULE
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifndef _MSSP_CLOCK_STRETCH_MODULE
                                #define _MSSP_CLOCK_STRETCH_MODULE
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition







;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16F876
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16F876

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16F876.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif

                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F876 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F876 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F876 does not have 12-bit A/D"
                        #endif
                #endif




;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif

                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif
                 #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif

                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F876 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition












;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16F876A
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16F876A

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16F876A.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif

                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F876A does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F876A does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F876A does not have 12-bit A/D"
                        #endif
                #endif



;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif

                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif

                        #ifndef _MSSP_CLOCK_STRETCH_MODULE
                                #define _MSSP_CLOCK_STRETCH_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif

                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif

                        #ifndef _MSSP_CLOCK_STRETCH_MODULE
                                #define _MSSP_CLOCK_STRETCH_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif

                        #ifndef _MSSP_CLOCK_STRETCH_MODULE
                                #define _MSSP_CLOCK_STRETCH_MODULE
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifndef _MSSP_CLOCK_STRETCH_MODULE
                                #define _MSSP_CLOCK_STRETCH_MODULE
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition







;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16F877
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16F877

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16F877.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif

                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F877 does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F877 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif

;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F877 does not have 12-bit A/D"
                        #endif
                #endif


;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif

                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif

                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F877 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif

        #endif                  ;End of device definition






;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      16F877A
;+ Date:        2/10/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __16F877A

;PIC family derivation
                #ifndef _PIC16xxx
                        #define _PIC16xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P16F877A.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################

;USART module
                #ifdef  USART_MODULE
                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif

                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif


;AUSART module
                #ifdef  AUSART_MODULE
                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif

;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F877A does not have EUSART"
                        #endif
                #endif

;8-bit A/D module
                #ifdef  A2D_8BIT_MODULE
                        #ifndef _A2D_8BIT_MODULE
                                #define _A2D_8BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif
                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      ;Backward compatible
                        #endif
                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F877A does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif

;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 16F877A does not have 12-bit A/D"
                        #endif
                #endif



;BSSP module
                #ifdef  BSSP_MODULE
                        #ifndef _BSSP_MODULE
                                #define _BSSP_MODULE
                        #endif

                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif

                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif

                        #ifndef _MSSP_CLOCK_STRETCH_MODULE
                                #define _MSSP_CLOCK_STRETCH_MODULE
                        #endif
                #endif

;SSP module
                #ifdef  SSP_MODULE
                        #ifndef _SSP_MODULE
                                #define _SSP_MODULE
                        #endif

                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif

                        #ifndef _MSSP_CLOCK_STRETCH_MODULE
                                #define _MSSP_CLOCK_STRETCH_MODULE
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifndef _MSSP_MODULE
                                #define _MSSP_MODULE
                        #endif

                        #ifndef _MSSP_CLOCK_STRETCH_MODULE
                                #define _MSSP_CLOCK_STRETCH_MODULE
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifndef _MSSP_CLOCK_STRETCH_MODULE
                                #define _MSSP_CLOCK_STRETCH_MODULE
                        #endif
                #endif

;Timer1 module
                #ifdef  TIMER1_MODULE
                        #ifndef _TIMER1_MODULE
                                #define _TIMER1_MODULE
                        #endif
                #endif
        #endif                  ;End of device definition








;#endif                 ;File inclusion check
