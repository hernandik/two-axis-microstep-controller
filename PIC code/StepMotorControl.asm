; Step motor controller
;   This is the 0.1 version. Lacks lots of things.
	; Not support input buffer
	; sending commands to this, look at main loop
	; Both the below drivers are controlled with microstep
	; * LM18245 never tested, but he code is available
	; * LB1946 (Eposn C60 driver from aleggro) work nice
	;	you should set outside via hardware a bias voltage to this chip
	;	to allow low current mode. Futher we will implement control via
	;	PWM pin to allow fine tunning of current for motor
	; no keys aquisition support
	; Due the tight timming, the top step speed is of 7KHz steps
	;   * Sum routines take a lot of processing, due the lack of adc (add with
	;	carry)
	;   * Serialization routine for LB1946 - it use 11 bit data. This caused
	;	the need of implement a software routine for this task, as SPI
	;	or i2c dont use this weird data length
	;   With LM chip, this could be better
; Hernandi Krammes - hernandik at gmail dot com
; MIT licence apply to this code
; This code was written in 2009

list p=16f877A

	#include <p16f877a.inc>
	#include "UARTInt.inc"

#define XTAL .20 ; clock frequency (MHz)
#define BRATE .150; Te = 1000 (us)
TIMER_VAL equ (XTAL * BRATE) / (4*4)
	IF (TIMER_VAL > .256) || (TIMER_VAL < .60)
		error "ERROR: Timer period exceeds limits" TIMER_VAL
	ENDIF

PORTAMOTORX equ PORTD ; ok
;PORTAMOTORY equ PORTD ; ok
;PORTAMOTORZ equ PORTC ; alinhar porta
PORTADIRECAO equ PORTE ; alinhar porta

UM	EQU 1
DOIS EQU 2
TRES EQU 3
QUATRO EQU 4
CINCO EQU 5
SEIS EQU 6
SETE EQU 7

; PORTAS UTILIZADAS NESTE SISTEMA
; PORTA	BIT0 - CLOCK SISTEMA SERIALIZAÇÃO
; 		BIT1 - DADOS SISTEMA SERIALIZAÇÃO
; 		BIT2 - BIT CONTROLE ENVIO DADOS SERIAIS BOBINA A
;		BIT3 - BIT CONTROLE ENVIO DADOS SERIAIS BOBINA B
; PORTC	BIT0 - INT/2 - ALTERNADO A CADA INTERRUPCAO
; 		BIT1 - ALTERNADO CADA CHAMADA MOVIMENTO MOTOR Y
;		BIT6 - tx SERIAL
;		BIT7 - RX SERIAL
; PORTE	BIT2 - ALTERNADO CADA CHAMADA MOVIMENTO MOTOR X
; 

memPrincipal UDATA 0x20
	; not the first to allocate RAM
STATUS_TEMP		RES 01; “ “ STATUS reg.
PCLATH_TEMP		RES 01; “ “ PCLATH reg.
FSR_TEMP		RES 01; “ “ FSR reg.
VARIAVEL_TIMER_VAL res 01

ctrlVelIni res 01

;***************************************
; Funcao processa passo; dados entrada da funcao
; pDirecao res 01
; w contem direcao a moverse
ppMotor res 01	; posicao do ultimo passo
; retornados ; devem ser enviados pela rotina local os dados!!!
Ibobina res 01 ; corrente da bobina???
direcao res 01	; direção a andar

motorWTmp res 01;

; para cada motor controlado evem existir um flag que em RAM que determine o proximo 
; ponto onde capturar informações a respeito do proximo passo na tabela de motores
; este ponto deve capturar todos os dados nas 4 tabelas de controle do motor de passo
; e setar propriamente os respectivos pinos de controle externos do PIC
; para o controle do CI externo de potencia que alimenta o motor de passo

MOTORX res 01
MOTORY res 01
MOTORZ res 01

; status do buffer de linha
STATUSLINHA res 01;

; cANG possui o coeficiente angular a ser incrementado a cada eixo em cada passagem
; CONTADOR cada eixo  para registar o movimento dos eixos, fixed math
	; (6 bytes) em HEX 0xx 1xx 2xx:3xx 4xx 5xx ; o primeiro bit MSB é o sinal q indica a direção
; fracaoX guarda o segundo byte dos contadores para verificar se houve um incremento significativo
	; desde a ultima passagem pela rotina
; estes dados devem ser mantidos juntos devido a rotina de carregamento deles
; admitir q eles estao em sequencia
cANGX 		res 03
dirMotorX 	res 01 ; direção do contador para o movimento do ponteiro na tabela
cANGY 		res 03
dirMotorY 	res 01
cANGZ 		res 03
dirMotorZ 	res 01
; numero de TICKs a processar e coef ang de cada eixo	
nPassos 	res 03; 24 bits utilizados como contador de passos importantissimo

CONTADORX 	res 06
carryX 		res 01

CONTADORY 	res 06
carryY 		res 01

CONTADORZ 	res 06
carryZ 		res 01

; Variaveis da rotina de soma
; sao ponteiros que serao colocados em FSR
; para endereçamento indireto e uma variavel temporaria
addA 		res 01	; ponteiro
addB 		res 01 ; ponteiro
addtmp 		res 06 ; variavel temporaria
somaCarry 	res 01 ; utilizada para sinalizar carry decimal durante a soma; 0x0 sem carry em 0.0000 e 0x1 indica carry em 1.000;
;somaFSR		res 01
;somaCarryCTRL res 01
carry 		res 01

ledTeste01 	res 01
timerL	res 01
timerH res 01

; rotina recepção da porta serial
RXByte		RES 01
tFSR 		RES 01
tW 			res 01

; rotinas de exibição
tempChar 	res 01 
tmpByte 	res 01
ZERO 		res 01


; Rotina de controle buffer                                          ;Indicator Pointer
;vUARTIntRxBuffer        RES     UARTINT_RX_BUFFER_SIZE   ;Receive Data FIFO buffer
;vUARTIntRxBufDataCnt    RES     01          ;Transmit Buffer Write Position
;vUARTIntRxBufWrPtr      RES     01
;vUARTIntRxBufRdPtr      RES     01

;***********************************
; rotina envio bits para motor passo
sHIGH res 01
sLOW res 02 ; bytes a serializar somente os primeiros 11 bits
serCount res 01


;*********************************************************************************
UARTTstShr UDATA_SHR
	;ISR_W	    RES 01	               ;For Saving W reg. value
W_TEMP			RES 01 ; salva de contexto, para ser acessado em qualquer rotina, USART maestro

;*********************************************************************************
; rotina de interrupção
STARThere       CODE    0x00
		goto main

	;org 0x4
INTserv CODE	0x04
	nop

IntVectorInterupt CODE    0x05
	movwf W_TEMP ; context saving, save W first
	swapf STATUS,W
	bcf STATUS,RP0 ; assuming use of variables in bank0
	movwf STATUS_TEMP ; save status
	movf PCLATH,W
	movwf PCLATH_TEMP ; save PCLATH
	clrf PCLATH ; assuming Async_ISR located in page0
	movf FSR,W
	movwf FSR_TEMP

	; ativa TIMER1 para controlar quantos ciclos
	; o sistema vai gastar para executar a INT
	bsf T1CON,0x0 ; ativa contador timer

	pagesel     UARTIntISR ;9 inst
	call	    UARTIntISR
	Banksel		STATUS_TEMP
	pagesel		Async_ISR

	btfss INTCON, T0IF	; verifica se ocorreu tambem/ou uma interrupção do timer
	goto ExitInt

Async_ISR
	;movlw TIMER_VAL; non disruptive timer reload
	movf VARIAVEL_TIMER_VAL, W
	;addlw 0x1
	subwf TMR0,F
	bcf INTCON,T0IF ; interrupt served

	; alterna bit 1 PORTC para indicar ocorrencia de INT a taxa escolhida
	banksel ledTeste01
	btfsc ledTeste01,0x2
	goto apagaLed3
	bsf ledTeste01,0x2
	bsf PORTC,0x0
	goto fimLed3
apagaLed3
	bcf ledTeste01,0x2
	bcf PORTC,0x0
fimLed3

	; processamento já foi comcluido?
	; INICIO DA ROTINA DE INTERRRUPÇÃO APOS SALVAR REGISTRADORES
	; verifica se tem linha a processar
	; processa a linha
	; somar a cada vertice de cada motor o incremento de 48 bits pre definido
	;CONTADORX = cANGX + CONTADORX
	;CONTADORY = cANGY + CONTADORY
	;CONTADORZ = cANGZ + CONTADORZ
	
	btfss STATUSLINHA, 0
	goto ExitInt

	; No começo de cada linha começa nas primeiras INTERRUPCOES com uma velocidade menor
	; para nao ter problemas de travamento do motor de passo
	;banksel ctrlVelIni
	;btfss ctrlVelIni,0x7 ; primeiros 128 passos já passaram?
	;goto crtlVel1; não
	;goto continuaPrograma1
;crtlVel1
	;incf ctrlVelIni
	;btfss ctrlVelIni,0x0
	;goto ExitInt
	
continuaPrograma1
	; addA contem endereço numero A
	; addB contem endereço numero B
	; resulatado é addA=addA+addB
	; carry na unidade é registrado em carrySoma
	movlw 0x0
	banksel somaCarry
	movwf somaCarry
	movlw CONTADORX
	movwf addA
	movlw cANGX
	movwf addB  ;14
	pagesel SomaAux
	call SomaAux
	pagesel Async_ISR
	banksel somaCarry
	movf somaCarry,W
	banksel carryX
	movwf carryX ;17

	movlw 0x0
	banksel somaCarry
	movwf somaCarry
	movlw CONTADORY
	movwf addA
	movlw cANGY
	movwf addB
	pagesel SomaAux	
	call SomaAux
	pagesel Async_ISR
	banksel somaCarry
	movf somaCarry,W
	movwf carryY ; 28

;	movlw 0x0
;	movwf somaCarry
;	movlw CONTADORZ
;	movwf addA
;	movlw cANGZ
;	movwf addB
;	pagesel     soma
;	call soma
;	pagesel Async_ISR
;	movf somaCarry,W
;	movwf carryZ ; 39


	
	
;*************************************	
; verifica se é necessario passo em X
;*************************************
	btfss carryX, 0x0
	goto proxY
		;efetua o passo
		;*****************
		bcf carryX,0x0 ; reseta bit carry

		movf MOTORX,W  ; variavel MOTORX
		movwf ppMotor	; ultimo passo

		;movlw 0x0
		;btfsc dirMotorX, 0x0 ; verifica para que direção mover-se 0 normal, 1 reversa
		;movlw 0x1
		banksel dirMotorX
		movf dirMotorX,W
		banksel direcao
		movwf direcao

		pagesel procStepLB1946
		call procStepLB1946
		pagesel Async_ISR
		movf ppMotor,W ; salva nova posicao do motor
		movwf MOTORX

		banksel PORTA
		bcf PORTA,0x2 ; bit controle
		movlw PORTA
		pagesel serializar
		call serializar
		pagesel Async_ISR
		
		banksel PORTA
		bsf PORTA,0x2 ; 
		; envia dados para as portas do motorX
		;banksel Ibobina
		;movf Ibobina,W
		;movwf PORTAMOTORX
		;banksel direcao
		;movf direcao,W
		;movwf PORTADIRECAO ; 52

		; ainda alterana BIT 2 PORTE para indicar q um byte foi enviado - para debug
		;banksel ledTeste01
		;btfsc ledTeste01,0x0
		;goto apagaLed1
		;bsf ledTeste01,0x0
		;bsf PORTE,0x2
		;goto fimLed1
apagaLed1
		;bcf ledTeste01,0x0
		;bcf PORTE,0x2
fimLed1

proxY
;*************************************
; verifica se é necessario passo em Y
;*************************************
	btfss carryY, 0	; testa se contador é maior que 0.5
	goto proxZ
		;efetua o passo
		;*****************
		bcf carryY,0x0 ; reseta bit carry

		movf MOTORY,W  ; variavel MOTORX
		movwf ppMotor	; ultimo passo

		banksel dirMotorY
		movf dirMotorY,W
		banksel direcao
		movwf direcao

		pagesel procStepLB1946
		call procStepLB1946
		pagesel Async_ISR

		movf ppMotor,W ; salva nova posicao do motor
		movwf MOTORY

		banksel PORTA
		bcf PORTA,0x3 ; bit controle
		movlw PORTA
		pagesel serializar
		call serializar
		pagesel Async_ISR
		
		banksel PORTA
		bsf PORTA,0x3 ; 

		; led teste
		;banksel ledTeste01
		;btfsc ledTeste01,0x1
		;goto apagaLed2
		;bsf ledTeste01,0x1
		;bsf PORTC,0x1
		;goto fimLed2
apagaLed2
		;bcf ledTeste01,0x1
		;bcf PORTC,0x1
fimLed2

proxZ
;*************************************
; verifica se é necessario passo em Z
;*************************************
;	btfss carryZ, 0	; testa se contador é maior que 0.5
;	goto naoIncrementa
		;efetua o passo
		;*****************
;		movlw MOTORZ	; ponteiro da variavel MOTORX
;		movwf ppMotor	;ultimo passo

;		movlw 0x0
;		btfsc dirMotorZ, 0 ; verifica para que direção mover-se 0 normal, 1 reversa
;		movlw 0x1
		
;		pagesel     processaPasso
;		call processaPasso
;		pagesel Async_ISR
		; envia dados para as portas do motorX
;		movf Ibobina,W
;		movwf PORTAMOTORZ
;		movf direcao,W
;		movwf PORTADIRECAO ; 77

;naoIncrementa

	; decrementa contador de ticks
	movlw 0x1
	subwf nPassos+2, F
	btfss STATUS,C ; ocorreu borrow
	subwf nPassos+1, F
	btfss STATUS,C ; ocorreu borrow
	subwf nPassos, F
	btfss STATUS,C ; ocorreu borrow
	bcf STATUSLINHA,0 ; caso termine a linha, zera contador    ;85

;nada a processar

ExitInt
	; pega dados TIMER
	bcf T1CON,0x0 ; desativa timer
	banksel TMR1L
	movf TMR1L,W
	movwf timerL
	movf TMR1H,W
	movwf timerH
	clrf TMR1L
	clrf TMR1H

	movf FSR_TEMP,W
	movwf FSR
	movf PCLATH_TEMP,W ; restore PCLATH (page)
	movwf PCLATH
	swapf STATUS_TEMP,W ; restore status and bank
	movwf STATUS
	swapf W_TEMP,F ; restore W reg.
	swapf W_TEMP,W
	retfie ; exit re-enabling interrupts ;
; total instruções aproximado: 332 instruções = 
;	89 isntruções basicas da INT + instrucoes da porta serial + 108(3 somas) + 135(3 analises motores)
; 	10KHz restam 165 instruções - 5KHz restam 665 instruções


;*******************************************************
; MAIN - entrada
main

	clrf INTCON ; disable interrupts

; limpa memoria
	clrf PORTA
	clrf ZERO
	clrf STATUSLINHA
	clrf ledTeste01

	movlw STATUS_TEMP
	movwf FSR
NEXTBYTE	clrf INDF
	incf FSR
	movlw tmpByte
	xorwf FSR,W
	btfss STATUS,Z
	goto NEXTBYTE

	bsf STATUS,RP0 ; bank 1

	MESSG "877 PORTA como saidas"
	movlw 0x6
	movwf ADCON1 ; desabilita conversor AD para porta A e E
	clrf TRISA
	; porta B como saidas
	movlw 0x0
	movwf PORTB
	movlw b'00000000'
	movwf TRISB		
	
	; porta C como saidas
	movlw b'10000000'
	movwf TRISC
	
	; porta D como saidas
	movlw b'00000000'
	movwf TRISD

	; porta E como saidas
	movlw b'00000000'
	movwf TRISE
	
; iniciar vetor de interrupção
	movlw b'10000001'; prescaler a 4
	movwf OPTION_REG ; prescaler assigned to WDT
	bcf STATUS,RP0 ; bank 0
	movlw -TIMER_VAL ; init timer
	movwf TMR0
	movlw TIMER_VAL
	banksel VARIAVEL_TIMER_VAL
	movwf VARIAVEL_TIMER_VAL
	
	; configura e deixa em HOLD o TIMER1 para controlar
	banksel T1CON
	movlw b'00000000'
	movwf T1CON
	banksel PIE1
	bcf PIE1, TMR1IE ; desabilita interrupcao timer 1; não quero esse tipo de interrupcao
	banksel TMR1L
	clrf TMR1L
	clrf TMR1H

	pagesel UARTIntInit
    call    UARTIntInit

	pagesel loop

	bcf INTCON, INTE
	bcf INTCON, RBIE
	bsf INTCON,T0IE ; habilita interrupts on TMR0 overflow
	bsf INTCON,GIE ; habilita global interrupt

	call PutOK
loop	; loop principal do programa
	call WaitRxData
	banksel RXByte
	movwf RXByte

STATUS_SIMPLES
	xorlw 's'
	btfss STATUS, Z
		goto LINHA
	btfsc STATUSLINHA, 0x0
	goto bufferFull
	; buffer disponivel
	call PutOK
	movlw 0x0d ; nova linha
	call PutChar
	movlw 0x0a
	call PutChar
	goto loop

LINHA movf RXByte,W
	xorlw 'L'
	btfss STATUS, Z
		goto STATUS_SISTEMA
	; verifica se já nao esta trabalhando em uma linha
	;btfsc STATUSLINHA, 0x0
	;goto bufferFull
	; buffer disponivel
	;call PutOK

	; proximos 15 bytes defines comprimento do vetor X(6)+dir Y(6)+dir Z(6)+dir e nPassos(3)
	bcf STATUS,IRP ; seta bit IRP+FSR end indireto

	movlw cANGX ; inicializa FSR
	movwf FSR
	call WaitRxData ; w vai conter 1 caractere recebido
	movwf INDF ; cANGX MSB ; move caractere para endereço indicado por FSR

	call WaitRxData ; w vai conter caractere
	incf FSR,F
	movwf INDF ; cANGX

	call WaitRxData ; w vai conter caractere
	incf FSR,F
	movwf INDF ; cANGX

	call WaitRxData ; w vai conter caractere
	incf FSR,F
	movwf INDF ; DIRX

	call WaitRxData ; w vai conter caractere
	incf FSR,F
	movwf INDF ; cANGY MSB

	call WaitRxData ; w vai conter caractere
	incf FSR,F
	movwf INDF ; cANGY

	call WaitRxData ; w vai conter caractere
	incf FSR,F
	movwf INDF ; cANGY

	call WaitRxData ; w vai conter caractere
	incf FSR,F
	movwf INDF ; DIRY

	call WaitRxData ; w vai conter caractere
	incf FSR,F
	movwf INDF ; cANGZ MSB

	call WaitRxData ; w vai conter caractere
	incf FSR,F
	movwf INDF ; cANGZ

	call WaitRxData ; w vai conter caractere
	incf FSR,F
	movwf INDF ; cANGZ

	call WaitRxData ; w vai conter caractere
	incf FSR,F
	movwf INDF ; DIRZ

	call WaitRxData ; w vai conter caractere
	incf FSR,F
	movwf INDF ; nPassos MSB

	call WaitRxData ; w vai conter caractere
	incf FSR,F
	movwf INDF ; nPassos

	call WaitRxData ; w vai conter caractere
	incf FSR,F
	movwf INDF ; nPassos LSB

	; ao terminar de receber o byte, ativa desenho da linha
	bsf STATUSLINHA, 0x0	
	call PutOK
	movlw 0x0d ; nova linha
	call PutChar
	movlw 0x0a
	call PutChar
	
	; LIMPA area da memoria que contem os contadores de cada motor
	; e seus respectivos flags de direção
	;clrf ctrlVelIni
	movlw CONTADORX
	movwf FSR
NEXTBYTE2	clrf INDF
	incf FSR
	movlw carryZ ; até carryZ que é o ultimo
	xorwf FSR,W
	btfss STATUS,Z
	goto NEXTBYTE2
	goto loop

bufferFull
	call PutFull
	movlw 0x0d ; nova linha
	call PutChar
	movlw 0x0a
	call PutChar
	goto loop

; envia STATUS
STATUS_SISTEMA	nop
	;bcf INTCON,T0IE ; desabilita interrupções on TMR0 overflow
	banksel RXByte
	movf RXByte,W
	xorlw 'S'
	btfss STATUS, Z
		goto ALTERA_TIMER_VAL
	btfsc STATUSLINHA,0x0
	call PutFull
	
	movlw 'X'
	call PutChar
	movlw ' '
	call PutChar
	banksel CONTADORX
	movf CONTADORX,W
	call PutBIN2HEX
	movlw ' '
	call PutChar
	banksel CONTADORX+1
	movf CONTADORX+1,W
	call PutBIN2HEX
	movlw ' '
	call PutChar
	banksel CONTADORX+2
	movf CONTADORX+2,W
	call PutBIN2HEX
	movlw ' '
	call PutChar
	banksel CONTADORX+3
	movf CONTADORX+3,W
	call PutBIN2HEX
	movlw ' '
	call PutChar
	banksel CONTADORX+4
	movf CONTADORX+4,W
	call PutBIN2HEX
	movlw ' '
	call PutChar
	banksel CONTADORX+5
	movf CONTADORX+5,W
	call PutBIN2HEX
	movlw 0x0d		;nova linha
	call PutChar
	movlw 0x0a
	call PutChar

	movlw 'Y'
	call PutChar
	movlw ' '
	call PutChar
	banksel CONTADORY
	movf CONTADORY,W
	call PutBIN2HEX
	movlw ' '
	call PutChar
	banksel CONTADORY+1
	movf CONTADORY+1,W
	call PutBIN2HEX
	movlw ' '
	call PutChar
	banksel CONTADORY+2
	movf CONTADORY+2,W
	call PutBIN2HEX
	movlw ' '
	call PutChar
	banksel CONTADORY+3
	movf CONTADORY+3,W
	call PutBIN2HEX
	movlw ' '
	call PutChar
	banksel CONTADORY+4
	movf CONTADORY+4,W
	call PutBIN2HEX
	movlw ' '
	call PutChar
	banksel CONTADORY+5
	movf CONTADORY+5,W
	call PutBIN2HEX
	movlw 0x0d		;nova linha
	call PutChar
	movlw 0x0a
	call PutChar

	movlw 'P'
	call PutChar
	movlw ' '
	call PutChar
	banksel nPassos
	movf nPassos,W
	call PutBIN2HEX
	movlw ' '
	call PutChar
	banksel nPassos
	movf nPassos+1,W
	call PutBIN2HEX
	movlw ' '
	call PutChar
	banksel nPassos
	movf nPassos+2,W
	call PutBIN2HEX
	movlw '-'
	call PutChar
	movlw 'T'
	call PutChar
	banksel VARIAVEL_TIMER_VAL
	movf VARIAVEL_TIMER_VAL,W
	call PutBIN2HEX
	movlw '-'
	call PutChar
	movlw 't'
	call PutChar
	banksel timerH
	movf timerH,W
	call PutBIN2HEX
	banksel timerL
	movf timerL,W
	call PutBIN2HEX

	movlw 0x0d ; nova linha
	call PutChar
	movlw 0x0a
	call PutChar
	goto loop

ALTERA_TIMER_VAL
	banksel RXByte
	movf RXByte,W
	xorlw 'T'
	btfss STATUS, Z
		goto MErr
	call WaitRxData
	banksel VARIAVEL_TIMER_VAL
	movwf VARIAVEL_TIMER_VAL
	call PutOK
	movlw 0x0d ; nova linha
	call PutChar
	movlw 0x0a
	call PutChar
	goto loop

	;movlw cANGX ; inicializa FSR
	;movwf FSR
	;call WaitRxData ; w vai conter 1 caractere recebido
	;movwf INDF ; cANGX MSB ; move caractere para endereço indicado por FSR

MErr
	call PutErro
	movlw 0x0d ; nova linha
	call PutChar
	movlw 0x0a
	call PutChar
	goto loop

;******************************************
; fica aguardando dados da porta serial
WaitRxData
;Check if Receive buffer is full
	;banksel vUARTIntStatus
	;btfss   vUARTIntStatus,UARTIntRxBufFul
	;btfsc   vUARTIntStatus,UARTIntRxBufEmpty
	banksel tFSR ; salva FSR
	movf FSR,W
	movwf tFSR
WaitRxData1
	banksel vUARTIntRxBufDataCnt
	movf vUARTIntRxBufDataCnt,F
	btfsc STATUS,Z
	goto    WaitRxData1

;If receive buffer is full then read the data
ReadAgain
	bsf vUARTIntStatus,UARTIntRxBufEmpty

	bcf INTCON,T0IE ; desabilita interrupções on TMR0 overflow
	Pagesel UARTIntGetCh
	call    UARTIntGetCh
	banksel tW
	bsf INTCON,T0IE ; habilita interrupções on TMR0 overflow
	movwf tW
	movf tFSR,W; restaura FSR
	movwf FSR
	movf tW,W
	return

; *******************************************
; Mensagens para retorno
; Envia OK 'K' pela serial
PutOK
	movlw   'K'
	banksel vUARTIntStatus
Wait1	btfsc vUARTIntStatus, UARTIntTxBufFul
	goto Wait1
	Pagesel UARTIntPutCh
	call UARTIntPutCh
	return	;9+2 CALL PUT CHAR

; Envia Mensagem de FULL 'F' pela serial
PutFull
	movlw   'F'
		banksel vUARTIntStatus
Wait2	btfsc vUARTIntStatus, UARTIntTxBufFul
	goto Wait2
	Pagesel UARTIntPutCh
	call UARTIntPutCh
	return	;9+2 CALL PUT CHAR

; Envia Mensagem de ERRO 'E' pela serial
PutErro
	movlw   'E'
		banksel vUARTIntStatus
Wait3	btfsc vUARTIntStatus, UARTIntTxBufFul
	goto Wait3
	Pagesel UARTIntPutCh
	call UARTIntPutCh
	return	;9+2 CALL PUT CHAR

; Envia caractere em W pela serial
PutChar
	banksel vUARTIntStatus
Wait4	btfsc vUARTIntStatus, UARTIntTxBufFul
	goto Wait4
	Pagesel UARTIntPutCh
	call UARTIntPutCh
	return

;**********************************************
; Rotina para enviar pela porta serial um byte
;	em formato HEXADECIMAL
; recebe W o byte a ser enviado
; envia o byte pela porta serial

PutBIN2HEX
	banksel tmpByte
	movwf tmpByte

	swapf tmpByte,w
	andlw 0xF
	call bin2hex
	call WaitTX
	Pagesel UARTIntPutCh
	call	UARTIntPutCh

	banksel tmpByte
	movf tmpByte,w
	andlw 0xF
	call bin2hex
	call WaitTX
	Pagesel UARTIntPutCh
	call	UARTIntPutCh

	return

;*****************************
; AGUARDA SE O BUFFER DE ENVIO ESTIVER CHEIO
WaitTX
	banksel vUARTIntStatus
	btfsc vUARTIntStatus, UARTIntTxBufFul
	goto WaitTX
	return

;*****************************
; CONVERSAO de Caractere BIN to HEX
; recebe bytes com 4 bit LSB contendo o valor a ser convertido
; retorna em W o valor ascii do caractere hexadecimal
bin2hex
	banksel tempChar
	addlw  TABELA1
	movwf  tempChar
	rlf    ZERO,w ; PARA LIMPAR BIT CARRY
	addlw  high(TABELA1)
	movwf  PCLATH
	movf   tempChar,w
	movwf  PCL
TABELA1
	dt '0','1','2','3','4','5','6','7','8','9','a','b','c','d','e','f';
	return

P1 CODE 0x0400
;*****************************************************
; Rotina de SOMA AUXILIAR Funcional - 6 bytes - OTIMIZADA
; copia variavel apontada por addB para area temporaria ADDTMP
SomaAux
	banksel addB
	bcf STATUS, IRP
	movf addB,W	; addB é um endereço apenas  ; 3 4 e 5 do byte em ADDB (cAng(n) neste caso q contem apenas 3 bytes, os demais sao 0)
	movwf FSR	; endereçamento indireto
;	clrf addtmp
;	clrf addtmp+1
;	clrf addtmp+2

	movf INDF,W
	movwf addtmp+3	; quarto byte
	incf FSR
	movf INDF,W
	movwf addtmp+4	; quinto byte
	incf FSR
	movf INDF,W
	movwf addtmp+5	; sexto byte

	bcf STATUS,IRP
	banksel addtmp
	; inicia soma !!!!!!!REVER ESTES DADOS
	movf addA,W		; indireto, addA é um endereço inicial apenas
	addlw 0x5		; começo pelo ultimo byte
	movwf FSR		; INDF esta apontando para ultimo byte dito por addA
	movlw 0x0
	movwf carry

;*******
; BYTE 5
		movf addtmp+5,W
		addwf INDF,F	; soma byte 5, (LSB) resultado fica no registrador F que neste caso é addA
		;calculaCarry 0x6 ; calcula o carry em cadeia
		btfss STATUS,C ; houve carry?
		goto fim5
		movlw 0x1
		movwf carry
fim5
		decf FSR,F
;*******
; BYTE 4
		; adiciona carry
		movf carry,W
		addwf INDF,F
		btfss STATUS,C
		goto fim41
		movlw 0x1	; com carry
		movwf carry
		goto fim42
fim41
		clrf carry ; sem carry ; limpa carry para proximo nivel
fim42	; soma normal
		movf addtmp+4,W		; pega proximo byte para soma em addTmp
		addwf INDF,F	; soma byte 4
		btfss STATUS,C ; carry soma?
		goto fim43
		;movlw 0x1 ; incrementa carry
		;addwf carry,F
		incf carry,F
fim43	
		decf FSR,F	; nao incrementa carry
	
;*******
; BYTE 3
		; adiciona carry
		movf carry,W
		addwf INDF,F
		btfss STATUS,C
		goto fim31
		movlw 0x1
		movwf carry
		BSF somaCarry,0x0; ajusta flag apenas de controle do carry nessa casa
		goto fim32
fim31
		clrf carry ; limpa carry para proximo nivel
fim32
		movf addtmp+3,W
		addwf INDF,F	; soma byte 3
		BTFSs STATUS,C        ; houve carry?
		goto fim33
			; houve carry na soma
		incf carry,F ;  o carry nunca vai ter overflow, desde que seja abaixo de 128 numeros!
		BSF somaCarry,0x0; ajusta flag apenas
fim33
		decf FSR,F
	
;*******
; BYTE 2
		; adiciona carry
		movf carry,W
		addwf INDF,F
		btfss STATUS,C
		goto fim21
		movlw 0x1
		movwf carry
		goto fim22
fim21
		clrf carry ; limpa carry para proximo nivel
fim22
;		movf addtmp+2,W		; essa rotina nao utilizará essa parte da soma; disponivel para outros casos
;		addwf INDF,F	; soma byte 2
;		btfss STATUS,C
;		goto fim23
;		incf carry,F
fim23
		decf FSR,F

;*******
; BYTE 1
		; adiciona carry
		movf carry,W
		addwf INDF,F
		btfss STATUS,C
		goto fim11
		movlw 0x1
		movwf carry
		goto fim12
fim11
		clrf carry ; limpa carry para proximo nivel
fim12
;		movf addtmp+1,W
;		addwf INDF,F	; soma byte 1
;		btfss STATUS,C
;		goto fim13
;		incf carry,F
fim13
		decf FSR,F

;*******
; BYTE 0
		movf carry,W
		addwf INDF,F
;		movf addtmp,W
;		addwf INDF,F	; soma byte 0, (MSB)
		return

;*********************************************
P2 CODE 0x0800
;*********************************************
; Rotina Processamento passos para chip EPSON C60 - LB1946 - work nice
; Tabela configurada para 4 micropassos entre passos
; Recebe:
;	ppMotor - com a posicao onde estava o ultimo passo
;	direcao - para onde mover para o proximo passo
; Retorna:
;	ppMotor - atualizado com nova posicao
;	grava as variaveis sLOW e sHIGH com valores a serem serializados

procStepLB1946
	banksel direcao
	;movf direcao,W
	btfsc direcao, 0x0 ; verifica para que direção mover-se
; diminuir 1
	goto diminuir2
; adicionar 1
	incf ppMotor,W
	andlw 0x0f ; CORRIGIR CONFORME TABELA PASSOS
	goto continua2
diminuir2
	decf ppMotor,W
	andlw 0x0f ; mantem dentro dos limites

continua2
	banksel ppMotor
	movwf ppMotor
	pagesel tabLB1946a
	call tabLB1946a ; W deve conter contador e W vai retornar valor
	banksel sLOW
	movwf sLOW
	movf ppMotor,W
	pagesel tabLB1946b
	call tabLB1946b ; W deve conter contador e W vai retornar valor
	banksel sHIGH
	movwf sHIGH
	return

tabLB1946a ; W contem o ponteiro da tabela ; vairaveis compatilhadas com outra rotina de passos
	banksel motorWTmp
	movwf motorWTmp
	pageselw TAB1946A
	movf motorWTmp,W
	andlw 0x0f
	addlw LOW TAB1946A
	btfsc STATUS,C                ;then increment PCLATH. Then load the
	incf PCLATH,f                 ;program counter with computed goto.
	movwf PCL
TAB1946A	retlw 0xf0		; 00EP 3210 bits a enviar para sistema sa invertidos no segundo nible
	RETLW 0xd0				;0.924	; dados para 4 micosteps adicionais
	retlw 0x80
	RETLW 0x40 ;0.383
	retlw 0x04
	RETLW 0x44 ;0.383
	retlw 0x84
	RETLW 0xd4 ;0.924
	retlw 0xf4
	RETLW 0xd4 ;0.924
	retlw 0x84
	RETLW 0x44 ;0.383
	retlw 0x00
	RETLW 0x40;0.383
	retlw 0x80
	RETLW 0xd0;0.924
	return

tabLB1946b
	banksel motorWTmp
	movwf motorWTmp
	pageselw TAB1946B
	movf motorWTmp,W
	andlw 0x0f
	addlw LOW TAB1946B
	btfsc STATUS,C                ;then increment PCLATH. Then load the
	incf PCLATH,f                 ;program counter with computed goto.
	movwf PCL
TAB1946B retlw 0x00
	RETLW 0x40;0.383
	retlw 0x80
	RETLW 0xd0;0.924
	retlw 0xf0
	RETLW 0xd0;0.924
	retlw 0x80
	RETLW 0x40;0.383;
	retlw 0x04
	RETLW 0x44;0.383;
	retlw 0x84
	RETLW 0xd4;0.924
	retlw 0xf4
	RETLW 0xd4;0.924
	retlw 0x84
	RETLW 0x44;0.383
	return

;*************************************
; Rotina processamento de passos para - never tested with the real chip
; LM18245 National - baseada na tabela fornecida pelo fabricante
; Recebe:
;	ppMotor - com a posicao onde estava o ultimo passo
;	direcao - para onde mover para o proximo passo
; Retorna:
;	IBOBINA - com os valores a serem enviados a bobina
;	ppMotor - com valor atualizado

processaPasso
	banksel direcao
	clrf direcao
	btfsc W, 0x0 ; verifica para que direção mover-se
; diminuir 1
	goto diminuir1
; adicionar 1
	incf ppMotor,W
	andlw 0x0f ; caso tenha estourado o valor da variavel, W AND 0xF mantem ela como 0x0!!!!
	goto continua1
diminuir1
	decf ppMotor,W
	andlw 0x0f ; mantem dentro dos limites

continua1
	; W DEVE conter o numero que representa o ponto da tabela
	; IBOBINA
	movwf ppMotor
	pagesel dacB
	call dacB
	andlw 0x0f
	movwf Ibobina ; salva valor corrente bobinaA na primeira metade da variavel
	swapf Ibobina,F
	movf ppMotor,W
	pagesel dacA
	call dacA
	andlw 0x0f
	iorwf Ibobina,F ; IbobinaA agora contem dados da corrente a ser aplicada na bobina a do motor 4MSB = dacB e 4LSB = dacA
	; DIRECAO
	movf ppMotor,W
	pagesel direB
	call direB
	banksel direcao
	movwf direcao
	rlf direcao,F
	andlw 0x2 ; asegura que o primeiro BIT é 0
	movf ppMotor,W
	pagesel direA
	call direA
	banksel direcao
	iorwf direcao,F ; bit 0=direcao A e bit 1=direcao B
	return ; 29 + 16 = 45


;******************************************
; As tabelas abaixo representam os dados para o correto funcionamento dos motores de passo
; dacA e dacB representam os valores da corrente de corte que o DAC 
; exigirá, valores de 16 bits para isso
; direA e direB sao os valores de tensão exigidos nos pinos do CI para o correto movimento dos motores
; dados extraidos das tabelas do datasheet dos LM16xxx da national ; micropasso 4 bits
dacA
	banksel motorWTmp
	movwf motorWTmp
	pageselw TAB1
	movf motorWTmp,W
	andlw 0x0f
	addlw LOW TAB1
	btfsc STATUS,C                ;then increment PCLATH. Then load the
	incf PCLATH,f                 ;program counter with computed goto.
	movwf PCL
TAB1	retlw  .0
	retlw  .6
	retlw  .11
	retlw  .14
	retlw  .15
	retlw  .14
	retlw  .11
	retlw  .6
	retlw  .0
	retlw  .6
	retlw  .11
	retlw  .14
	retlw  .15
	retlw  .14
	retlw  .11
	retlw  .6
	return

dacB
	banksel motorWTmp
	movwf motorWTmp
	pageselw TAB2
	movf motorWTmp,W
	andlw 0x0f
	addlw LOW TAB2
	btfsc STATUS,C                ;then increment PCLATH. Then load the
	incf PCLATH,f                 ;program counter with computed goto.
	movwf PCL
TAB2	retlw  .15
	retlw  .14
	retlw  .11
	retlw  .6
	retlw  .0
	retlw  .6
	retlw  .11
	retlw  .14
	retlw  .15
	retlw  .14
	retlw  .11
	retlw  .6
	retlw  .0
	retlw  .6
	retlw  .11
	retlw  .14
	return

direA
	banksel motorWTmp
	movwf motorWTmp
	pageselw TAB3
	movf motorWTmp,W
	andlw 0x0f
	addlw LOW TAB3
	btfsc STATUS,C                ;then increment PCLATH. Then load the
	incf PCLATH,f                 ;program counter with computed goto.
	movwf PCL
TAB3	retlw 0x1
	retlw 0x1
	retlw 0x1
	retlw 0x1
	retlw 0x0
	retlw 0x0
	retlw 0x0
	retlw 0x0
	retlw 0x0
	retlw 0x0
	retlw 0x0
	retlw 0x0
	retlw 0x1
	retlw 0x1
	retlw 0x1
	retlw 0x1
	return

direB
	banksel motorWTmp
	movwf motorWTmp
	pageselw TAB4
	movf motorWTmp,W
	andlw 0x0f
	addlw LOW TAB4
	btfsc STATUS,C                ;then increment PCLATH. Then load the
	incf PCLATH,f                 ;program counter with computed goto.
	movwf PCL
TAB4	retlw 0x1
	retlw 0x1
	retlw 0x1
	retlw 0x1
	retlw 0x1
	retlw 0x1
	retlw 0x1
	retlw 0x1
	retlw 0x1
	retlw 0x0
	retlw 0x0
	retlw 0x0
	retlw 0x0
	retlw 0x0
	retlw 0x0
	retlw 0x0
	retlw 0x0
	return

;**************************************
; rotinas de envia de mensagens
; K = OK
; F = FULL; buffer cheio
; E = ERRO
	dt "Controle Motor de Passo 0.2 - 6-01-2010",0

;*********************************
; Rotina serialização de dados
; W contem porta a enviar dados
; PORTA W - BIT 0 CLOCK e BIT 1 DADOS
; sLOW e sHIGH contem dados a enviar - primeiros 6 bits para cada motor
; 
serializar
;*************
; 6 bits iniciais
	movwf FSR
	movlw .6
	movwf serCount

serProximoBit1
	bcf INDF,0x0 ;clock LOW
	rlf sLOW,F
	btfss STATUS, C
	goto ser1
	bsf INDF,0x1
	goto serE
ser1
	bcf INDF,0x1
serE
	bsf INDF,0x0 ; transicao clock HIGH

	; verifica se já é o bit 6
	decf serCount
	btfss STATUS,Z
	goto serProximoBit1
	; termina rotina serialização
	bcf INDF,0x0

serNextByte
;*************
; 6 bits finais
	movlw .6
	movwf serCount

serProximoBit2
	bcf INDF,0x0 ;clock LOW
	rlf sHIGH,F
	btfss STATUS, C
	goto ser2
	bsf INDF,0x1
	goto serE2
ser2
	bcf INDF,0x1
serE2
	bsf INDF,0x0 ; transicao clock HIGH

	decf serCount ; proximo bit
	; verifica se já é o bit 6
	btfss STATUS,Z
	goto serProximoBit2
	; termina rotina serialização
	bcf INDF,0x0
	return ; TERMINA

	end
