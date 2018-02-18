;*********************************************************************
;*
;*  The generic include file for PIC18 family
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
;* Gaurang Kavaiya      Feb 18, 2003    Modified for CLM and backward
;*                                      compatible modules (V1.1)
;* Gaurang Kavaiya      Mar 11, 2003    Minor Clean-up
;* Anantha Ramu         Mar 17, 2003    Commented, non-available
;                                       features of ADC modules deleted
;* Vidyadhar            Mar 17, 2003    MSSP issues clean-up (V1.2)

;********************************************************************/





;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      18C242
;+ Date:        2/18/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __18C242

;PIC family derivation
                #ifndef _PIC18xxx
                        #define _PIC18xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P18C242.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################
                #ifndef _DONT_USE_LFSR
                        #define _DONT_USE_LFSR
                #endif


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
                                error "PIC 18C242 does not have EUSART"
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

                        #ifndef _A2D_10BIT_5CHANNEL_MODULE
                                #define _A2D_10BIT_5CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_4REGISTER_MODULE
                                #define _A2D_10BIT_4REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_5CHANNEL_MODULE
                                #define _A2D_10BIT_5CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_4REGISTER_MODULE
                                #define _A2D_10BIT_4REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18C242 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18C242 does not have 12-bit A/D"
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
                                error "PIC 18C242 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif


;CAN module
                #ifdef  CAN_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18C242 does not have CAN"
                        #endif
                #endif

        #endif                  ;End of device definition












;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      18C252
;+ Date:        2/18/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __18C252

;PIC family derivation
                #ifndef _PIC18xxx
                        #define _PIC18xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P18C252.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################
                #ifndef _DONT_USE_LFSR
                        #define _DONT_USE_LFSR
                #endif


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
                                error "PIC 18C252 does not have EUSART"
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

                        #ifndef _A2D_10BIT_5CHANNEL_MODULE
                                #define _A2D_10BIT_5CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_4REGISTER_MODULE
                                #define _A2D_10BIT_4REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_5CHANNEL_MODULE
                                #define _A2D_10BIT_5CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_4REGISTER_MODULE
                                #define _A2D_10BIT_4REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18C252 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18C252 does not have 12-bit A/D"
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
                                error "PIC 18C252 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif


;CAN module
                #ifdef  CAN_MODULE
                        #ifndef _CAN_MODULE
;                                #define _CAN_MODULE
                        #endif

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18C252 does not have CAN"
                        #endif
                #endif

        #endif                  ;End of device definition











;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      18C442
;+ Date:        2/18/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __18C442

;PIC family derivation
                #ifndef _PIC18xxx
                        #define _PIC18xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P18C442.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################
                #ifndef _DONT_USE_LFSR
                        #define _DONT_USE_LFSR
                #endif


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
                                error "PIC 18C442 does not have EUSART"
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

                        #ifndef _A2D_10BIT_8CHANNEL_MODULE
                                #define _A2D_10BIT_8CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_4REGISTER_MODULE
                                #define _A2D_10BIT_4REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_8CHANNEL_MODULE
                                #define _A2D_10BIT_8CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_4REGISTER_MODULE
                                #define _A2D_10BIT_4REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18C442 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18C442 does not have 12-bit A/D"
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
                                error "PIC 18C442 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif


;CAN module
                #ifdef  CAN_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18C442 does not have CAN"
                        #endif
                #endif


        #endif                  ;End of device definition










;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      18C452
;+ Date:        2/18/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __18C452

;PIC family derivation
                #ifndef _PIC18xxx
                        #define _PIC18xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P18C452.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################
                #ifndef _DONT_USE_LFSR
                        #define _DONT_USE_LFSR
                #endif


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
                                error "PIC 18C452 does not have EUSART"
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

                        #ifndef _A2D_10BIT_8CHANNEL_MODULE
                                #define _A2D_10BIT_8CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_4REGISTER_MODULE
                                #define _A2D_10BIT_4REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_8CHANNEL_MODULE
                                #define _A2D_10BIT_8CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_4REGISTER_MODULE
                                #define _A2D_10BIT_4REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18C452 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18C452 does not have 12-bit A/D"
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
                                error "PIC 18C452 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif


;CAN module
                #ifdef  CAN_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18C452 does not have CAN"
                        #endif
                #endif

        #endif                  ;End of device definition









;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      18C601
;+ Date:        2/18/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __18C601

;PIC family derivation
                #ifndef _PIC18xxx
                        #define _PIC18xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P18C601.INC>
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
                                error "PIC 18C601 does not have EUSART"
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

                        #ifndef _A2D_10BIT_8CHANNEL_MODULE
                                #define _A2D_10BIT_8CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_5REGISTER_MODULE
                                #define _A2D_10BIT_5REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_8CHANNEL_MODULE
                                #define _A2D_10BIT_8CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_5REGISTER_MODULE
                                #define _A2D_10BIT_5REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18C601 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18C601 does not have 12-bit A/D"
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
                                error "PIC 18C601 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif


;CAN module
                #ifdef  CAN_MODULE
                          #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18C601 does not have CAN"
                        #endif
                #endif


        #endif                  ;End of device definition









;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      18C801
;+ Date:        2/18/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __18C801

;PIC family derivation
                #ifndef _PIC18xxx
                        #define _PIC18xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P18C801.INC>
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
                                error "PIC 18C801 does not have EUSART"
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

                        #ifndef _A2D_10BIT_12CHANNEL_MODULE
                                #define _A2D_10BIT_12CHANNEL_MODULE
                        #endif

                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_12CHANNEL_MODULE
                                #define _A2D_10BIT_12CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_5REGISTER_MODULE
                                #define _A2D_10BIT_5REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18C801 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18C801 does not have 12-bit A/D"
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
                                error "PIC 18C801 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif


;CAN module
                #ifdef  CAN_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18C801 does not have CAN"
                        #endif
                #endif


        #endif                  ;End of device definition









;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      18C658
;+ Date:        2/18/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __18C658

;PIC family derivation
                #ifndef _PIC18xxx
                        #define _PIC18xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P18C658.INC>
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
                                error "PIC 18C658 does not have EUSART"
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

                        #ifndef _A2D_10BIT_12CHANNEL_MODULE
                                #define _A2D_10BIT_12CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_5REGISTER_MODULE
                                #define _A2D_10BIT_5REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_12CHANNEL_MODULE
                                #define _A2D_10BIT_12CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_5REGISTER_MODULE
                                #define _A2D_10BIT_5REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18C658 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18C658 does not have 12-bit A/D"
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
                                error "PIC 18C658 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif


;CAN module
                #ifdef  CAN_MODULE
                        #ifndef _CAN_MODULE
                                #define _CAN_MODULE
                        #endif
                #endif



        #endif                  ;End of device definition










;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      18C858
;+ Date:        2/18/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __18C858

;PIC family derivation
                #ifndef _PIC18xxx
                        #define _PIC18xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P18C858.INC>
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
                                error "PIC 18C858 does not have EUSART"
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

                        #ifndef _A2D_10BIT_16CHANNEL_MODULE
                                #define _A2D_10BIT_16CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_5REGISTER_MODULE
                                #define _A2D_10BIT_5REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_16CHANNEL_MODULE
                                #define _A2D_10BIT_16CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_5REGISTER_MODULE
                                #define _A2D_10BIT_5REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18C858 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18C858 does not have 12-bit A/D"
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
                                error "PIC 18C858 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif


;CAN module
                #ifdef  CAN_MODULE
                        #ifndef _CAN_MODULE
                                #define _CAN_MODULE
                        #endif
                #endif



        #endif                  ;End of device definition










;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      18F242
;+ Date:        2/18/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __18F242

;PIC family derivation
                #ifndef _PIC18xxx
                        #define _PIC18xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P18F242.INC>
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
                                error "PIC 18F242 does not have EUSART"
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

                        #ifndef _A2D_10BIT_5CHANNEL_MODULE
                                #define _A2D_10BIT_5CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_4REGISTER_MODULE
                                #define _A2D_10BIT_4REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      
                        #endif

                        #ifndef _A2D_10BIT_5CHANNEL_MODULE
                                #define _A2D_10BIT_5CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_4REGISTER_MODULE
                                #define _A2D_10BIT_4REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F242 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F242 does not have 12-bit A/D"
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


;CAN module
                #ifdef  CAN_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F242 does not have CAN"
                        #endif
                #endif



        #endif                  ;End of device definition














;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      18F248
;+ Date:        2/18/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __18F248

;PIC family derivation
                #ifndef _PIC18xxx
                        #define _PIC18xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P18F248.INC>
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
                                error "PIC 18F248 does not have EUSART"
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

                        #ifndef _A2D_10BIT_5CHANNEL_MODULE
                                #define _A2D_10BIT_5CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_4REGISTER_MODULE
                                #define _A2D_10BIT_4REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_5CHANNEL_MODULE
                                #define _A2D_10BIT_5CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_4REGISTER_MODULE
                                #define _A2D_10BIT_4REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F248 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F248 does not have 12-bit A/D"
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


;CAN module
                #ifdef  CAN_MODULE
                        #ifndef _CAN_MODULE
                                #define _CAN_MODULE
                        #endif
                #endif



        #endif                  ;End of device definition

















;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      18F252
;+ Date:        2/18/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __18F252

;PIC family derivation
                #ifndef _PIC18xxx
                        #define _PIC18xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P18F252.INC>
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
                                error "PIC 18F252 does not have EUSART"
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

                        #ifndef _A2D_10BIT_5CHANNEL_MODULE
                                #define _A2D_10BIT_5CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_4REGISTER_MODULE
                                #define _A2D_10BIT_4REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      
                        #endif

                        #ifndef _A2D_10BIT_5CHANNEL_MODULE
                                #define _A2D_10BIT_5CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_4REGISTER_MODULE
                                #define _A2D_10BIT_4REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F252 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F252 does not have 12-bit A/D"
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


;CAN module
                #ifdef  CAN_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F252 does not have CAN"
                        #endif
                #endif



        #endif                  ;End of device definition








;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      18F258
;+ Date:        2/18/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __18F258

;PIC family derivation
                #ifndef _PIC18xxx
                        #define _PIC18xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P18F258.INC>
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
                                error "PIC 18F258 does not have EUSART"
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

                        #ifndef _A2D_10BIT_5CHANNEL_MODULE
                                #define _A2D_10BIT_5CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_4REGISTER_MODULE
                                #define _A2D_10BIT_4REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_5CHANNEL_MODULE
                                #define _A2D_10BIT_5CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_4REGISTER_MODULE
                                #define _A2D_10BIT_4REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F258 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F258 does not have 12-bit A/D"
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


;CAN module
                #ifdef  CAN_MODULE
                        #ifndef _CAN_MODULE
                                #define _CAN_MODULE
                        #endif
                #endif



        #endif                  ;End of device definition










;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      18F442
;+ Date:        2/18/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __18F442

;PIC family derivation
                #ifndef _PIC18xxx
                        #define _PIC18xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P18F442.INC>
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
                                error "PIC 18F442 does not have EUSART"
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

                        #ifndef _A2D_10BIT_8CHANNEL_MODULE
                                #define _A2D_10BIT_8CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_4REGISTER_MODULE
                                #define _A2D_10BIT_4REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      
                        #endif

                        #ifndef _A2D_10BIT_8CHANNEL_MODULE
                                #define _A2D_10BIT_8CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_4REGISTER_MODULE
                                #define _A2D_10BIT_4REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F442 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F442 does not have 12-bit A/D"
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


;CAN module
                #ifdef  CAN_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F442 does not have CAN"
                        #endif
                #endif



        #endif                  ;End of device definition









;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      18F448
;+ Date:        2/18/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __18F448

;PIC family derivation
                #ifndef _PIC18xxx
                        #define _PIC18xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P18F448.INC>
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
                                error "PIC 18F448 does not have EUSART"
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

                        #ifndef _A2D_10BIT_8CHANNEL_MODULE
                                #define _A2D_10BIT_8CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_4REGISTER_MODULE
                                #define _A2D_10BIT_4REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_8CHANNEL_MODULE
                                #define _A2D_10BIT_8CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_4REGISTER_MODULE
                                #define _A2D_10BIT_4REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F448 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F448 does not have 12-bit A/D"
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


;CAN module
                #ifdef  CAN_MODULE
                        #ifndef _CAN_MODULE
                                #define _CAN_MODULE
                        #endif
                #endif



        #endif                  ;End of device definition








;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      18F452
;+ Date:        2/18/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __18F452

;PIC family derivation
                #ifndef _PIC18xxx
                        #define _PIC18xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P18F452.INC>
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
                                error "PIC 18F452 does not have EUSART"
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

                        #ifndef _A2D_10BIT_8CHANNEL_MODULE
                                #define _A2D_10BIT_8CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_4REGISTER_MODULE
                                #define _A2D_10BIT_4REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      
                        #endif

                        #ifndef _A2D_10BIT_8CHANNEL_MODULE
                                #define _A2D_10BIT_8CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_4REGISTER_MODULE
                                #define _A2D_10BIT_4REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F452 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F452 does not have 12-bit A/D"
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


;CAN module
                #ifdef  CAN_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F452 does not have CAN"
                        #endif
                #endif



        #endif                  ;End of device definition









;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      18F458
;+ Date:        2/18/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __18F458

;PIC family derivation
                #ifndef _PIC18xxx
                        #define _PIC18xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P18F458.INC>
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
                                error "PIC 18F458 does not have EUSART"
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

                        #ifndef _A2D_10BIT_8CHANNEL_MODULE
                                #define _A2D_10BIT_8CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_4REGISTER_MODULE
                                #define _A2D_10BIT_4REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_8CHANNEL_MODULE
                                #define _A2D_10BIT_8CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_4REGISTER_MODULE
                                #define _A2D_10BIT_4REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F458 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F458 does not have 12-bit A/D"
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


;CAN module
                #ifdef  CAN_MODULE
                        #ifndef _CAN_MODULE
                                #define _CAN_MODULE
                        #endif
                #endif



        #endif                  ;End of device definition












;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      18F1220
;+ Date:        2/18/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __18F1220

;PIC family derivation
                #ifndef _PIC18xxx
                        #define _PIC18xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P18F1220.INC>
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

                        #ifndef _EUSART_MODULE
                                #define _EUSART_MODULE
                        #endif
                #endif




;AUSART module
                #ifdef  AUSART_MODULE
                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif

                        #ifndef _EUSART_MODULE
                                #define _EUSART_MODULE
                        #endif
                #endif



;EUSART module
                #ifdef  EUSART_MODULE
                        #ifndef _EUSART_MODULE
                                #define _EUSART_MODULE
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

                        #ifndef _A2D_10BIT_7CHANNEL_MODULE
                                #define _A2D_10BIT_7CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_5REGISTER_MODULE
                                #define _A2D_10BIT_5REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_7CHANNEL_MODULE
                                #define _A2D_10BIT_7CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_5REGISTER_MODULE
                                #define _A2D_10BIT_5REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifndef _A2D_10BIT_MODULE_AUTO_ACQ_TIME
                                #define _A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #endif

                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F1220 does not have 12-bit A/D"
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F1220 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F1220 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif


;CAN module
                #ifdef  CAN_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F1220 does not have CAN"
                        #endif
                #endif



        #endif                  ;End of device definition











;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      18F1320
;+ Date:        2/18/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __18F1320

;PIC family derivation
                #ifndef _PIC18xxx
                        #define _PIC18xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P18F1320.INC>
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

                        #ifndef _EUSART_MODULE
                                #define _EUSART_MODULE
                        #endif
                #endif




;AUSART module
                #ifdef  AUSART_MODULE
                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif

                        #ifndef _EUSART_MODULE
                                #define _EUSART_MODULE
                        #endif

                        #ifdef  _GEN_MODULE_ERROR
;                                error "PIC 18F1320 does not have AUSART"
                        #endif
                #endif



;EUSART module
                #ifdef  EUSART_MODULE
                        #ifndef _EUSART_MODULE
                                #define _EUSART_MODULE
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

                        #ifndef _A2D_10BIT_7CHANNEL_MODULE
                                #define _A2D_10BIT_7CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_5REGISTER_MODULE
                                #define _A2D_10BIT_5REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_7CHANNEL_MODULE
                                #define _A2D_10BIT_7CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_5REGISTER_MODULE
                                #define _A2D_10BIT_5REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifndef _A2D_10BIT_MODULE_AUTO_ACQ_TIME
                                #define _A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #endif

                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F1320 does not have 12-bit A/D"
                        #endif
                #endif

;MSSP module
                #ifdef  MSSP_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F1320 does not have MSSP"
                        #endif
                #endif

;MSSP Clock Stretch module
                #ifdef  MSSP_CLOCK_STRETCH_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F1320 does not have MSSP with Clock Stretching facility"
                        #endif
                #endif


;CAN module
                #ifdef  CAN_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F1320 does not have CAN"
                        #endif
                #endif



        #endif                  ;End of device definition










;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      18F2220
;+ Date:        2/18/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __18F2220

;PIC family derivation
                #ifndef _PIC18xxx
                        #define _PIC18xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P18F2220.INC>
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
                                error "PIC 18F2220 does not have EUSART"
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

                        #ifndef _A2D_10BIT_10CHANNEL_MODULE
                                #define _A2D_10BIT_10CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_5REGISTER_MODULE
                                #define _A2D_10BIT_5REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_10CHANNEL_MODULE
                                #define _A2D_10BIT_10CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_5REGISTER_MODULE
                                #define _A2D_10BIT_5REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifndef _A2D_10BIT_MODULE_AUTO_ACQ_TIME
                                #define _A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #endif

                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F2220 does not have 12-bit A/D"
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


;CAN module
                #ifdef  CAN_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F2220 does not have CAN"
                        #endif
                #endif



        #endif                  ;End of device definition







;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      18F2320
;+ Date:        2/18/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __18F2320

;PIC family derivation
                #ifndef _PIC18xxx
                        #define _PIC18xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P18F2320.INC>
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
                                error "PIC 18F2320 does not have EUSART"
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

                        #ifndef _A2D_10BIT_10CHANNEL_MODULE
                                #define _A2D_10BIT_10CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_5REGISTER_MODULE
                                #define _A2D_10BIT_5REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_10CHANNEL_MODULE
                                #define _A2D_10BIT_10CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_5REGISTER_MODULE
                                #define _A2D_10BIT_5REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifndef _A2D_10BIT_MODULE_AUTO_ACQ_TIME
                                #define _A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #endif


                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F2320 does not have 12-bit A/D"
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


;CAN module
                #ifdef  CAN_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F2320 does not have CAN"
                        #endif
                #endif



        #endif                  ;End of device definition







;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      18F2439
;+ Date:        2/18/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __18F2439

;PIC family derivation
                #ifndef _PIC18xxx
                        #define _PIC18xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P18F2439.INC>
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
                                error "PIC 18F2439 does not have EUSART"
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

                        #ifndef _A2D_10BIT_5CHANNEL_MODULE
                                #define _A2D_10BIT_5CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_4REGISTER_MODULE
                                #define _A2D_10BIT_4REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_5CHANNEL_MODULE
                                #define _A2D_10BIT_5CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_4REGISTER_MODULE
                                #define _A2D_10BIT_4REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F2439 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F2439 does not have 12-bit A/D"
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


;CAN module
                #ifdef  CAN_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F2439 does not have CAN"
                        #endif
                #endif



        #endif                  ;End of device definition







;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      18F2539
;+ Date:        2/18/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __18F2539

;PIC family derivation
                #ifndef _PIC18xxx
                        #define _PIC18xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P18F2539.INC>
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
                                error "PIC 18F2539 does not have EUSART"
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

                        #ifndef _A2D_10BIT_5CHANNEL_MODULE
                                #define _A2D_10BIT_5CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_4REGISTER_MODULE
                                #define _A2D_10BIT_4REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_5CHANNEL_MODULE
                                #define _A2D_10BIT_5CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_4REGISTER_MODULE
                                #define _A2D_10BIT_4REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F2539 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F2539 does not have 12-bit A/D"
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


;CAN module
                #ifdef  CAN_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F2539 does not have CAN"
                        #endif
                #endif



        #endif                  ;End of device definition









;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      18F4220
;+ Date:        2/18/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __18F4220

;PIC family derivation
                #ifndef _PIC18xxx
                        #define _PIC18xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P18F4220.INC>
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
                                error "PIC 18F4220 does not have EUSART"
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

                        #ifndef _A2D_10BIT_13CHANNEL_MODULE
                                #define _A2D_10BIT_13CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_5REGISTER_MODULE
                                #define _A2D_10BIT_5REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_13CHANNEL_MODULE
                                #define _A2D_10BIT_13CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_5REGISTER_MODULE
                                #define _A2D_10BIT_5REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifndef _A2D_10BIT_MODULE_AUTO_ACQ_TIME
                                #define _A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #endif

                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F4220 does not have 12-bit A/D"
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


;CAN module
                #ifdef  CAN_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F4220 does not have CAN"
                        #endif
                #endif



        #endif                  ;End of device definition












;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      18F4320
;+ Date:        2/18/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __18F4320

;PIC family derivation
                #ifndef _PIC18xxx
                        #define _PIC18xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P18F4320.INC>
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
                                error "PIC 18F4320 does not have EUSART"
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

                        #ifndef _A2D_10BIT_13CHANNEL_MODULE
                                #define _A2D_10BIT_13CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_5REGISTER_MODULE
                                #define _A2D_10BIT_5REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_13CHANNEL_MODULE
                                #define _A2D_10BIT_13CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_5REGISTER_MODULE
                                #define _A2D_10BIT_5REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #ifndef _A2D_10BIT_MODULE_AUTO_ACQ_TIME
                                #define _A2D_10BIT_MODULE_AUTO_ACQ_TIME
                        #endif

                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F4320 does not have 12-bit A/D"
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


;CAN module
                #ifdef  CAN_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F4320 does not have CAN"
                        #endif
                #endif



        #endif                  ;End of device definition







;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      18F4439
;+ Date:        2/18/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __18F4439

;PIC family derivation
                #ifndef _PIC18xxx
                        #define _PIC18xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P18F4439.INC>
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
                                error "PIC 18F4439 does not have EUSART"
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

                        #ifndef _A2D_10BIT_8CHANNEL_MODULE
                                #define _A2D_10BIT_8CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_4REGISTER_MODULE
                                #define _A2D_10BIT_4REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_8CHANNEL_MODULE
                                #define _A2D_10BIT_8CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_4REGISTER_MODULE
                                #define _A2D_10BIT_4REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F4439 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F4439 does not have 12-bit A/D"
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


;CAN module
                #ifdef  CAN_MODULE
                       #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F4439 does not have CAN"
                        #endif
                #endif



        #endif                  ;End of device definition








;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      18F4539
;+ Date:        2/18/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __18F4539

;PIC family derivation
                #ifndef _PIC18xxx
                        #define _PIC18xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P18F4539.INC>
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
                                error "PIC 18F4539 does not have EUSART"
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

                        #ifndef _A2D_10BIT_8CHANNEL_MODULE
                                #define _A2D_10BIT_8CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_4REGISTER_MODULE
                                #define _A2D_10BIT_4REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_8CHANNEL_MODULE
                                #define _A2D_10BIT_8CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_4REGISTER_MODULE
                                #define _A2D_10BIT_4REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F4539 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F4539 does not have 12-bit A/D"
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


;CAN module
                #ifdef  CAN_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F4539 does not have CAN"
                        #endif
                #endif



        #endif                  ;End of device definition







;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      18F6520
;+ Date:        2/18/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __18F6520

;PIC family derivation
                #ifndef _PIC18xxx
                        #define _PIC18xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P18F6520.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################


;USART module
                #ifdef  USART_MODULE

                        #ifndef _AUSART_2_MODULE
                                #define _AUSART_2_MODULE
                        #endif

                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif

                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif




;AUSART module
                #ifdef  AUSART_MODULE
                        #ifndef _AUSART_2_MODULE
                                #define _AUSART_2_MODULE
                        #endif

                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif



;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F6520 does not have EUSART"
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

                        #ifndef _A2D_10BIT_12CHANNEL_MODULE
                                #define _A2D_10BIT_12CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_5REGISTER_MODULE
                                #define _A2D_10BIT_5REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_12CHANNEL_MODULE
                                #define _A2D_10BIT_12CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_5REGISTER_MODULE
                                #define _A2D_10BIT_5REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F6520 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F6520 does not have 12-bit A/D"
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


;CAN module
                #ifdef  CAN_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F6520 does not have CAN"
                        #endif
                #endif



        #endif                  ;End of device definition








;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      18F6620
;+ Date:        2/18/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __18F6620

;PIC family derivation
                #ifndef _PIC18xxx
                        #define _PIC18xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P18F6620.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################


;USART module
                #ifdef  USART_MODULE

                        #ifndef _AUSART_2_MODULE
                                #define _AUSART_2_MODULE
                        #endif

                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif

                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif




;AUSART module
                #ifdef  AUSART_MODULE
                        #ifndef _AUSART_2_MODULE
                                #define _AUSART_2_MODULE
                        #endif

                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif



;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F6620 does not have EUSART"
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

                        #ifndef _A2D_10BIT_12CHANNEL_MODULE
                                #define _A2D_10BIT_12CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_5REGISTER_MODULE
                                #define _A2D_10BIT_5REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      
                        #endif

                        #ifndef _A2D_10BIT_12CHANNEL_MODULE
                                #define _A2D_10BIT_12CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_5REGISTER_MODULE
                                #define _A2D_10BIT_5REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F6620 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F6620 does not have 12-bit A/D"
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


;CAN module
                #ifdef  CAN_MODULE
                       #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F6620 does not have CAN"
                        #endif
                #endif



        #endif                  ;End of device definition







;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      18F6720
;+ Date:        2/18/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __18F6720

;PIC family derivation
                #ifndef _PIC18xxx
                        #define _PIC18xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P18F6720.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################


;USART module
                #ifdef  USART_MODULE

                        #ifndef _AUSART_2_MODULE
                                #define _AUSART_2_MODULE
                        #endif

                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif

                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif




;AUSART module
                #ifdef  AUSART_MODULE
                        #ifndef _AUSART_2_MODULE
                                #define _AUSART_2_MODULE
                        #endif

                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif



;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F6720 does not have EUSART"
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

                        #ifndef _A2D_10BIT_12CHANNEL_MODULE
                                #define _A2D_10BIT_12CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_5REGISTER_MODULE
                                #define _A2D_10BIT_5REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      
                        #endif

                        #ifndef _A2D_10BIT_12CHANNEL_MODULE
                                #define _A2D_10BIT_12CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_5REGISTER_MODULE
                                #define _A2D_10BIT_5REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F6720 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F6720 does not have 12-bit A/D"
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


;CAN module
                #ifdef  CAN_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F6720 does not have CAN"
                        #endif
                #endif



        #endif                  ;End of device definition






;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      18F8520
;+ Date:        2/18/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __18F8520

;PIC family derivation
                #ifndef _PIC18xxx
                        #define _PIC18xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P18F8520.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################


;USART module
                #ifdef  USART_MODULE

                        #ifndef _AUSART_2_MODULE
                                #define _AUSART_2_MODULE
                        #endif

                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif

                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif




;AUSART module
                #ifdef  AUSART_MODULE
                        #ifndef _AUSART_2_MODULE
                                #define _AUSART_2_MODULE
                        #endif

                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif



;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F8520 does not have EUSART"
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

                        #ifndef _A2D_10BIT_16CHANNEL_MODULE
                                #define _A2D_10BIT_16CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_5REGISTER_MODULE
                                #define _A2D_10BIT_5REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE
                        #endif

                        #ifndef _A2D_10BIT_16CHANNEL_MODULE
                                #define _A2D_10BIT_16CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_5REGISTER_MODULE
                                #define _A2D_10BIT_5REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F8520 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F8520 does not have 12-bit A/D"
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


;CAN module
                #ifdef  CAN_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F8520 does not have CAN"
                        #endif
                #endif



        #endif                  ;End of device definition






;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      18F8620
;+ Date:        2/18/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __18F8620

;PIC family derivation
                #ifndef _PIC18xxx
                        #define _PIC18xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P18F8620.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################


;USART module
                #ifdef  USART_MODULE

                        #ifndef _AUSART_2_MODULE
                                #define _AUSART_2_MODULE
                        #endif

                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif

                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif




;AUSART module
                #ifdef  AUSART_MODULE
                        #ifndef _AUSART_2_MODULE
                                #define _AUSART_2_MODULE
                        #endif

                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif



;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F8620 does not have EUSART"
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

                        #ifndef _A2D_10BIT_16CHANNEL_MODULE
                                #define _A2D_10BIT_16CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_5REGISTER_MODULE
                                #define _A2D_10BIT_5REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      
                        #endif

                        #ifndef _A2D_10BIT_16CHANNEL_MODULE
                                #define _A2D_10BIT_16CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_5REGISTER_MODULE
                                #define _A2D_10BIT_5REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F8620 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F8620 does not have 12-bit A/D"
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


;CAN module
                #ifdef  CAN_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F8620 does not have CAN"
                        #endif
                #endif



        #endif                  ;End of device definition





;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;+ Device:      18F8720
;+ Date:        2/18/03
;+ By:          GK
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #ifdef __18F8720

;PIC family derivation
                #ifndef _PIC18xxx
                        #define _PIC18xxx
                #endif

;Processor specific include file inclusion
                #ifdef  _ADD_PROC_INC_FILE
                        #ifndef PROC_INC_ADDED
                                #define PROC_INC_ADDED
                                include <P18F8720.INC>
                        #endif
                #endif

;##########################################
;Resource definition
;##########################################


;USART module
                #ifdef  USART_MODULE

                        #ifndef _AUSART_2_MODULE
                                #define _AUSART_2_MODULE
                        #endif

                        #ifndef _USART_MODULE
                                #define _USART_MODULE
                        #endif

                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif




;AUSART module
                #ifdef  AUSART_MODULE
                        #ifndef _AUSART_2_MODULE
                                #define _AUSART_2_MODULE
                        #endif

                        #ifndef _AUSART_MODULE
                                #define _AUSART_MODULE
                        #endif
                #endif



;EUSART module
                #ifdef  EUSART_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F8720 does not have EUSART"
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

                        #ifndef _A2D_10BIT_16CHANNEL_MODULE
                                #define _A2D_10BIT_16CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_5REGISTER_MODULE
                                #define _A2D_10BIT_5REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module
                #ifdef  A2D_10BIT_MODULE
                        #ifndef _A2D_10BIT_MODULE
                                #define _A2D_10BIT_MODULE      
                        #endif

                        #ifndef _A2D_10BIT_16CHANNEL_MODULE
                                #define _A2D_10BIT_16CHANNEL_MODULE
                        #endif

                        #ifndef _A2D_10BIT_5REGISTER_MODULE
                                #define _A2D_10BIT_5REGISTER_MODULE
                        #endif

                #endif

;10-bit A/D module with Acquisition time
                #ifdef  A2D_10BIT_MODULE_AUTO_ACQ_TIME

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F8720 does not have 10-bit A/D with auto acquisition time facility"
                        #endif
                #endif
;12-bit A/D module
                #ifdef  A2D_12BIT_MODULE

                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F8720 does not have 12-bit A/D"
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


;CAN module
                #ifdef  CAN_MODULE
                        #ifdef  _GEN_MODULE_ERROR
                                error "PIC 18F8720 does not have CAN"
                        #endif
                #endif



        #endif                  ;End of device definition


