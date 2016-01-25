; ****************************************************************************************
; Step motor controller for PIC 16f877(a)
; Hernandi Francisco Krammes Filho - hernandik at gmail dot com
;	MIT licence apply to this code
;	This code was written in 2009/2010
;
;
; ABOUT the software
;
; Receive commands as simple binary data by serial port
;	This program take care of axis interpolation
; The data input is based
;	* in axis deltas
;	* plus lenght to run
; axis deltas are given in fixed integer math, in format:
;	xx:xx:xx:ff:ff:ff
;		where
;			xx is the integer part (not present in data in, only inside the counters of program)
;			ff the fractional part
;
; When sending commands, check if the buffer isn't full.
;	There are a command for this
; To better learn how to use this program, and send commands
;	look at main loop
; 
; Both the below drivers are controlled with microstep
; * LM18245 never tested, but he code is available
; * LB1946 (Eposn C60 driver from aleggro) work nice
;		you should set outside via hardware a bias voltage to this chip
;		to allow low current mode.
;	Futher we will implement control via PWM pin to allow fine tunning
;		of current for motor
;
; Due the tight timming, the top step speed is of 7KHz steps
;   * Sum routines take a lot of processing, due the lack of adc (add with
;	input/output(I can remember) carry) inside the 16f877
;   * Serialization routine for LB1946 - it use 11 bit data. This caused
;	the need of implement a software routine for this task, as SPI
;	or i2c dont use this weird data length
;   I do not test with the LM chip, but it should allow higher frequencies and
;		this could be better
;	Enjoy the software!
; ****************************************************************************************

; Versao 0.5 - 1-09-2010
; Incluido suporte a controle de Servomotor PWM - 153uS entre passos aproximadamente
; Criadas rotinas para
;	Desligar motor
;	Receber byte de comportamento - beh0 a beh3
;
; VERSAO 0.4 - 10-02-2010
; Rotina de recepção USART teve alguma correções
;
; ****************************************************************************************
; Com FIFO circular de FIFO_N_REG regioes para dados de entrada(atualmente 4 areas)
; Suporte a rotina antiga que envia dados e sobreescreve o FIFO

; Recebe comandos através da PORTA SERIAL a 9600,n,8,1
;	Comandos implementados
;		são todos baseados em simples caracteres seguidos de alguma sequencia auxiliar:
;		'S' - STATUS geral dos contadores XYZ, CONTADOR, TIMERINT 
;			Retorna uma string contendo o status processado eo caractere final 'K' finalizando transmissao
;		's' - status do buffer de entrada retorna 2 bytes com o status do PLOTTER e do FIFO
;			Retorna 3 bytes, sendo 2 STATUS e 1 de finalização de envio dados 'K'
;		'lxxxcyyyczzzcDDD' - executa desenho linha com coef angulares xxx/dD yyy/dD e zzz/dD
;		'LxxxcyyyczzzcDDD' - executa desenho linha com coef angulares xxx/dD yyy/dD e zzz/dD; guarda dados no FIFO Circular
;			e comprimento DDD - 24 bits para cada conj
;			AMBAS as rotinas esperam receber 15 bytes de dados; ao termino enviam um K confirmando o recebimento
;		'Tt' - onde t é um valor de 1 a 255 que controle a INT do sistema para controle da frequencia
;			Retorna um K quando transmissao termina

; CONTADORES INTERNOS DO MICRONTROLADOR - USO
; TIMER0 controla INT geral de 7Khz
; TIMER1 para contar quantas instruções sao executadas
;	dentro da rotina de INTERRUPCAO
;
;***************************************************************************************
;
; Necessidades extras:
; 	Incluir função para botoes externos para mover eixos - EM TESTES JÁ
; 	Suporte a linha de OnLine (botao) - EM TESTES JÁ
;	Incluir funcao para andar numero pre definido de passos para verificar eixos - verificar implementacao no programa controlador
;	Incluir função para chave de final de curso dos eixos - e parar eixo neste caso
;	Colocar funcao para setar linha controlador motores em STANDBY
;	Setar funcao para setar modo de baixa amperagem nos motores; para nao esquentarem
;
;***************************************************************************************
	list p=16f877A

	#include <p16f877a.inc>
	#include "UARTInt.inc"

#define XTAL .20 ; clock frequency (MHz)
#define BRATE .150; Te = 1000 (us)
TIMER_VAL equ (XTAL * BRATE) / (4*4)
	IF (TIMER_VAL > .256) || (TIMER_VAL < .60)
		error "ERROR: Timer period exceeds limits" TIMER_VAL
	ENDIF

ENDLINE equ 0x0d

;PORTAMOTORX equ PORTD ; ok
;PORTAMOTORY equ PORTD ; ok
;PORTAMOTORZ equ PORTC ; alinhar porta
PORTADIRECAO equ PORTE ; alinhar porta

;**********************************************************
; PORTAS UTILIZADAS NESTE SISTEMA
; PORTA	BIT0 - CLOCK SISTEMA SERIALIZAÇÃO
; 		BIT1 - DADOS SISTEMA SERIALIZAÇÃO
; 		BIT2 - BIT CONTROLE ENVIO DADOS SERIAIS MOTORA
;		BIT3 - BIT CONTROLE ENVIO DADOS SERIAIS MOTORB
;		BIT4 - CONTROLE SERVOMOTOR1
; 		BIT5 - CONTROLE SERVOMOTOR2

; PORTB
;		BIT0 - Botao externo para seleção de OnLine
;		BIT1 - CONTROLE SERVO MOTOR3
;		BIT2 - CONTROLE SERVO MOTOR4
;		BIT3 - PGM - não possivel ser utilizado
;		BIT4 - Botao para avanço de X
;		BIT5 - Botao para retrocesso de X
;		BIT6 - Botao para avanço de Y
;		BIT7 - Botao para retrocesso de Y

; PORTC	BIT0 - INT/2 - ALTERNADO A CADA INTERRUPCAO
; 		BIT1 - ALTERNADO CADA CHAMADA MOVIMENTO MOTOR Y
;		BIT2 - ALTERNADO CADA CHAMADA MOVIMENTO MOTOR X
;		BIT3 - SCK - clock SSP
;		BIT4 - CONTROLE SERVO4 (SDI input data)
;		BIT5 - SDO	(SDO out data)
;		BIT6 - Tx SERIAL
;		BIT7 - Rx SERIAL

; PORTD BIT0 - Todas configuradas como entradas
;		BIT1 - 
;		BIT2 - 
;		BIT3 - 
;		BIT4 - 
;		BIT5 - 
;		BIT6 - 
;		BIT7 - 

; PORTE	BIT0 - 
;		BIT1 - 
;		BIT2 - 


memPrincipal UDATA 0x20

VARIAVEL_TIMER_VAL res 01

;*****************************************
; Funcao Processa os Passo
; Dados Entrada
; w deve conter a direcao a moverse
ppMotor res 01	; posicao do ultimo passo
; Retornados ; devem ser enviados pela rotina local os dados!!!
; para LM18xxx
Ibobina res 01 ; corrente da bobina???
direcao res 01	; direção a andar

;*****************************************
; Utilizado nas rotinas das tabelas na busca dos dados
motorWTmp res 01
; Ponteiro para captura de passo das tabelas para cada motor
MOTORX res 01
MOTORY res 01
MOTORZ res 01

;*****************************************
; status do buffer de linha
vPLOTStatus res 01;

PLOTDrawing equ 0x0
PLOTOnLine equ 0x1
; cANG possui o coeficiente angular a ser incrementado a cada eixo em cada passagem
; CONTADOR cada eixo  para registar o movimento dos eixos, fixed math
	; (6 bytes) em HEX 0xx 1xx 2xx:3xx 4xx 5xx ; o primeiro bit MSB é o sinal q indica a direção
; fracaoX guarda o segundo byte dos contadores para verificar se houve um incremento significativo
	; desde a ultima passagem pela rotina
; estes dados devem ser mantidos juntos devido a rotina de carregamento deles
; admitir q eles estao em sequencia

cANGX 		res .03
dirMotorX 	res .01 ; direção do contador para o movimento do ponteiro na tabela
cANGY 		res .03
dirMotorY 	res .01
cANGZ 		res .03
dirMotorZ 	res .01
; numero de TICKs a processar e coef ang de cada eixo	
nPassos 	res .03; 24 bits utilizados como contador de passos importantissimo

CONTADORX 	res .06
carryX 		res .01
CONTADORY 	res .06
carryY 		res .01
CONTADORZ 	res .06
carryZ 		res .01

;*****************************************
; Variaveis da rotina de soma
; sao ponteiros que serao colocados em FSR
; para endereçamento indireto e uma variavel temporaria
addA 		res .01	; ponteiro
addB 		res .01 ; ponteiro
addtmp 		res .06 ; variavel temporaria
somaCarry 	res .01 ; utilizada para sinalizar carry decimal durante a soma; 0x0 sem carry em 0.0000 e 0x1 indica carry em 1.000;
carry 		res .01

;*****************************************
; Rotinas de debug
ledTeste01 	res .01
timerL	res .01
timerH res .01

;***********************************
; Rotina envio bits para motor passo
; 		para controladores EPSON
sHIGH res .01
sLOW res .01 ; bytes a serializar somente os primeiros 11 bits
serCount res .01

;************************************
; Rotina KeyScan
debOnLine res 01

;**********************************
; Variaveis da rotina para enviar 3 bytes para a saida em hexdecimal
; Put3BYTE2HEX
; endereço da variavel de origem H:L que contem os bytes a serem exibidos
tmpLadr 	res 01
tmpHadr 	res 01
putCount 	res 01
nBytes2Send 	res 01

; Rotinas de exibição
tmpChar 	res 01 
tmpByte 	res 01
ZERO 		res 01

;*****************************************
; Segundo banco de memoria
; Area de memoria com 80 bytes disponiveis
memFIFO		UDATA 0xA0
;*****************************************
; Rotina de controle da memoria circular - FIFO

FIFO_N_REG equ .4
BYTES_PER_FIFO equ .15
FIFO_RX_BUF_SIZE equ BYTES_PER_FIFO * FIFO_N_REG

FIFOBufEmpty equ 0x0
FIFOBufFul equ 0x1
FIFOBufOF equ 0x2

vFIFOBuffer res FIFO_RX_BUF_SIZE
vFIFOStatus res .01
vFIFODataCnt res .01
vFIFOWrPtr res .01
vFIFORdPtr res .01



;************************************
; Váriaveis de controle dos servos
vServoClockCnt res 01	; contador continuo - reiniciando ao alcançar SERVO_MAX_CNT
vServoLmt res .4 ; posição de cada servo

;************************************
; Variaveis da rotinha de comportamentos - BEHAVIOR
; Reflexos do sistema a eventos da porta D
beh0 res 01	; UP
beh1 res 01 ; DOWN
beh2 res 01 ; LEFT
beh3 res 01 ; RGHT

;************************************
; Area de memoria com dados do ADC
memADC	UDATA 0x110

vADCPort res .01
vADCWrPtr res .01
vADCRdPtr res .01
vADCDataCnt res .01
vADCStatus res .01

adcBuffer res .80


;*************************************
; AREA MEMORIA COMPARTILHADA
UARTTstShr UDATA_SHR

W_TEMP			RES 01 ; salva de contexto, para ser acessado em qualquer rotina, USART maestro
STATUS_TEMP		RES 01; “ “ STATUS reg.
PCLATH_TEMP		RES 01; “ “ PCLATH reg.
FSR_TEMP		RES 01; “ “ FSR reg.

; rotina recepção da porta serial
RXByte		RES 01
tFSR 		RES 01
tSTATUS		res 01
tW 			res 01

;****************************************************
; MACROS
; Jump Equal - Se var1 = var2 entao salta
; 	var1 LITERAL
; 	var2 MEMORIA
; 	adr - address mem
; 	altera W e STATUS Z
Je MACRO var1, var2, adr
	local  JeEnd

	movlw var1
	xorwf var2,W
	btfss STATUS, Z
	goto JeEnd
	pagesel adr
	goto adr
JeEnd
	endm

SendTAB MACRO
	movlw 0x9 ; TAB
	call PutChar
	endm

SendCRLF MACRO
	movlw ENDLINE ;0x0d ; nova linha
	call PutChar
	movlw 0x0a
	call PutChar
	endm
	
;**********************************************
STARThere       CODE    0x00
		goto main

;**********************************************
;**********************************************
; Rotina de interrupção
INTserv CODE	0x04	; codigo realocavel
	nop					; para evitar problemas de pagina
IntVectorInterupt CODE    0x05
	movwf W_TEMP ; context saving, save W first
	swapf STATUS,W
	;bcf STATUS,RP0 ; assuming use of variables in bank0
	movwf STATUS_TEMP ; save status
	movf PCLATH,W
	movwf PCLATH_TEMP ; save PCLATH
	clrf PCLATH ; assuming Async_ISR located in page0
	movf FSR,W
	movwf FSR_TEMP

	banksel T1CON	; ativa TIMER1 para controlar quantos ciclos
	bsf T1CON,0x0	; o sistema vai gastar para executar a INT
					
	pagesel     UARTIntISR ;9 inst
	call	    UARTIntISR

	banksel ppMotor
	pagesel		Async_ISR

	btfss INTCON, T0IF	; verifica se ocorreu tambem/ou uma interrupção do timer
	goto ExitInt

Async_ISR
	;movlw TIMER_VAL; non disruptive timer reload
	movf VARIAVEL_TIMER_VAL, W
	;addlw 0x1
	subwf TMR0,F
	bcf INTCON,T0IF ; interrupt served

	;**********************************************
	; PROCESSA SAIDAS PWM
	pagesel 	processaPWMFunc
	call		processaPWMFunc
	pagesel		Async_ISR

	;**********************************************
	; ROTINA DO FIFO CIRCULAR
	; 	Variaveis utilizadas para controlar o buffer
	;		vFIFOBufDataCnt res 01
	;		vFIFOWrPtr res 01
	;		vFIFORdPtr res 01
	banksel vPLOTStatus

	; Verifica se tem alguma linha sendo desenhada no momento
	btfsc vPLOTStatus, PLOTDrawing
	goto continuaPrograma1	; existe uma linha em andamento, nao faz nada na rotina
	; Nao exite nenhuma linha em andamento

	; Verificar status do flag PLOTOnLine
		; 1 online
		; 0 offline
		; Depois do PLOTDrawing para permitir que uma possivel
		; 	linha termine seu desenho antes de interromper futuros
		;	carregamentos de dados para proxima linha nos regs
	btfss vPLOTStatus, PLOTOnLine
	goto ExitInt

; UTILIZADO PARA DEBUG
; alterna bit 1 PORTC para indicar ocorrencia de INT a taxa escolhida
; utilizado como indicador de OnLine!
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

; ROTINA DE VERIFICAÇÂO E MOVIMENTAÇÂO DE DADOS DO BUFFER
; 	PARA EXECUÇÃO
	banksel vFIFODataCnt 	
	movf    vFIFODataCnt,W	; Verifica se existe alguma linha pendente no buffer
	btfss   STATUS,Z
	goto 	TransferRecdDataToExec		; Exitem dados pendentes
	
	bsf     vFIFOStatus,FIFOBufEmpty   	; Nao existem dados pendentes
	goto ExitInt						; encerra INT

TransferRecdDataToExec	; transfere os 15 bytes de dados a partir do endereço informado por
	bankisel vFIFOBuffer	; vFIFORdPtr
	movf vFIFORdPtr,W
	addlw low(vFIFOBuffer)
	movwf FSR

	banksel cANGX		; move os proximos 15 bytes 
	movf INDF,W			; 	do FIFO para a area de desenho 
	movwf cANGX			; 	executado apenas quando vai
	incf FSR,F			; 	carregar linha
	movf INDF,W
	movwf cANGX+.1
	incf FSR,F
	movf INDF,W
	movwf cANGX+.2
	incf FSR,F
	movf INDF,W
	movwf cANGX+.3
	incf FSR,F
	movf INDF,W
	movwf cANGX+.4
	incf FSR,F
	movf INDF,W
	movwf cANGX+.5
	incf FSR,F
	movf INDF,W
	movwf cANGX+.6
	incf FSR,F
	movf INDF,W
	movwf cANGX+.7
	incf FSR,F
	movf INDF,W
	movwf cANGX+.8
	incf FSR,F
	movf INDF,W
	movwf cANGX+.9
	incf FSR,F
	movf INDF,W
	movwf cANGX+.10
	incf FSR,F
	movf INDF,W
	movwf cANGX+.11
	incf FSR,F
	movf INDF,W
	movwf cANGX+.12
	incf FSR,F
	movf INDF,W
	movwf cANGX+.13
	incf FSR,F
	movf INDF,W
	movwf cANGX+.14
	;incf FSR	; byte 15

	; move ponteiro de leitura para proximo conjunto de dados
	banksel vFIFORdPtr
	movf vFIFORdPtr,W
	addlw BYTES_PER_FIFO
	movwf vFIFORdPtr
	; verifica se nao extrapolou valor maximo do vFIFOPtr - caso tenha, reseta contador
	movlw FIFO_RX_BUF_SIZE
	xorwf vFIFORdPtr,W
	btfsc STATUS,Z
	clrf vFIFORdPtr	; zera ponteiro, caso ultrapassou limite

	; decrementa contador de BYTES no buffer, caso seja zero, limpa status de disponibilidade de novos bytes
	decf vFIFODataCnt,F
	movf vFIFODataCnt,W
	btfsc STATUS,Z
	bsf vFIFOStatus,FIFOBufEmpty	; empty = 1 informando que buffer esta vazio

	bcf vFIFOStatus, FIFOBufFul ; tem espaço pra mais dados agora

	; Ajusta PLOTSTATUS para começar a desenhar
	banksel vPLOTStatus
	bankisel vPLOTStatus
	bsf vPLOTStatus, PLOTDrawing ; agora tem uma linha em andamento

; Limpa area da memoria que contem os contadores de cada motor
; 	AVALIAR SE ESTA ROTINA É REALMENTE NECESSARIA, SE MANTER OS DADOS NÃO É MAIS EFICIENTE.
; 	Rotina é muito lenta, consome muitos ciclos para limpar os 20 bytes de memoria
;	Mas se executa apenas 1 vez a cada nova linha, portanto deve ocupar poucos Hz
	bankisel CONTADORX
	banksel CONTADORX
	movlw CONTADORX
	movwf FSR
NEXTBYTE2	clrf INDF
	incf FSR,F
	movlw carryZ+.1 ; até carryZ que é o ultimo
	xorwf FSR,W
	btfss STATUS,Z	; verifica se é ultimo byte
	goto NEXTBYTE2

; Caso positivo, continue desenhando a linha ou Começe a desenhar

; No processamento da linha
; 	somar a cada vertice de cada motor o incremento
;	correspondente de 24 bits pre definido
;	CONTADORX = cANGX + CONTADORX
;	CONTADORY = cANGY + CONTADORY
;	CONTADORZ = cANGZ + CONTADORZ

continuaPrograma1
	; addA contem endereço numero A (6 bytes (3)MSB.(3)LSB )
	; addB contem endereço numero B (3 bytes .(3)LSB )
	; resulatado é addA=addA+addB
	; carry passando pela unidade é registrado em carrySoma
	;	- necessario para estabelecer o passo
	movlw 0x0
	banksel somaCarry
	movwf somaCarry
	movlw CONTADORX
	movwf addA
	movlw cANGX
	movwf addB
	pagesel SomaAux
	call SomaAux
	pagesel Async_ISR
	banksel somaCarry
	movf somaCarry,W
	banksel carryX
	movwf carryX

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
	movwf carryY


;*************************************	
; verifica se é necessario passo em X
;*************************************
	btfss carryX, 0x0
	goto proxY
		;efetua o passo
		;*****************
		bcf carryX,0x0 ; reseta bit indicando necessidade de passo
					;	ocorre apenas quando o contador overflow

		; preenche variaveis necessarias para chamar a função que executa o passo
		movf MOTORX,W  ; variavel MOTORX
		movwf ppMotor	; ultimo passo
		banksel dirMotorX
		movf dirMotorX,W
		banksel direcao
		movwf direcao
		pagesel procStepLB1946
		call procStepLB1946

		pagesel Async_ISR
		movf ppMotor,W ; salva nova posicao do motor
		movwf MOTORX

		; Rotina especifica para o LB da EPSON
		banksel PORTA
		bcf PORTA,0x2 ; bit controle linha do motor em questao
		movlw PORTA
		pagesel serializar
		call serializar
		;pagesel SerializarViaSSP
		;call SerializarViaSSP
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
fimLed1

proxY
;*************************************
; verifica se é necessario passo em Y
;*************************************
	btfss carryY, 0x0	; testa se contador é maior que 0.5
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

	proxZ


; decrementa contador de ticks
	banksel nPassos
	movlw 0x1
	subwf nPassos+2, F
	btfss STATUS,C ; ocorreu borrow
	subwf nPassos+1, F
	btfss STATUS,C ; ocorreu borrow
	subwf nPassos, F
	btfsc STATUS,C ; ocorreu borrow
	goto linhaAindaEmAndamento
	bcf vPLOTStatus,PLOTDrawing ; caso termine a linha, zera contador    ;85


linhaAindaEmAndamento


ExitInt
	; pega dados TIMER
	banksel T1CON
	bcf T1CON,0x0 ; desativa timer
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
	swapf W_TEMP,F 	; restore W reg.
	swapf W_TEMP,W	; tem que ser 2 swapf pois 
	;movf W_TEMP,W 	; movf pode alterar o status
					; e causar problemas
	retfie ; exit re-enabling interrupts ;


;*******************************************************
; MAIN - entrada
main
	clrf INTCON ; disable interrupts

; limpa memoria
	clrf PORTA
	clrf ZERO
	clrf vPLOTStatus
	clrf ledTeste01

	; Efetua uma limpeza da memoria
	; de STATUS_TEMP até tmpByte
	bankisel VARIAVEL_TIMER_VAL
	movlw VARIAVEL_TIMER_VAL
	movwf FSR
NEXTBYTE_BANK1	clrf INDF
	incf FSR,F
	movlw ZERO
	xorwf FSR,W
	btfss STATUS,Z
	goto NEXTBYTE_BANK1

	bankisel 0x00A0
	movlw 0xA0
	movwf FSR
NEXTBYTE_BANK2	clrf INDF
	incf FSR,F
	movlw beh3		; ultima posicao utilizada da memoria
	xorwf FSR,W
	btfss STATUS,Z
	goto NEXTBYTE_BANK2

	; Inicializa portas
	banksel TRISA ;bsf STATUS,RP0 ; bank 1
	MESSG "877 PORTA como saidas"
	movlw 0x6
	movwf ADCON1 ; desabilita conversor AD para porta A e E

	clrf TRISA				; PORTA como saidas

	movlw b'11110001'		; porta B como saidas
	movwf TRISB				; BIT 04567 entrada botões externos
	
	movlw b'10000000' 		; porta C como saidas
	movwf TRISC

	movlw b'11111111' 		; porta D como entradas
	movwf TRISD

	movlw b'00000000'		; porta E como saidas
	movwf TRISE
	
; TIMER 0 como INTERRUPÇÂO PRINCIPAL
; Iniciar vetor de interrupção
	movlw b'10000001'; prescaler a 4
	movwf OPTION_REG ; prescaler assigned to WDT
	bcf STATUS,RP0 ; bank 0
	movlw -TIMER_VAL ; init timer
	movwf TMR0
	movlw TIMER_VAL
	banksel VARIAVEL_TIMER_VAL
	movwf VARIAVEL_TIMER_VAL
	
; Configura e deixa em HOLD o TIMER1 para controlar
; a contagem de instruções da rotina de INT 1:1
	banksel T1CON
	movlw b'00000000'
	movwf T1CON
	banksel PIE1
	bcf PIE1, TMR1IE ; desabilita interrupcao timer 1; não quero esse tipo de interrupcao
	banksel TMR1L
	clrf TMR1L
	clrf TMR1H

; Configurar TIMER2 para interrupção do ADC

; Inicializa porta serial 9600,n,8,1
	pagesel UARTIntInit
    call    UARTIntInit

; Inicializa variaveis do FIFO CIRCULAR
	banksel vFIFOStatus
	clrf vFIFOStatus
	clrf vFIFODataCnt
	clrf vFIFOWrPtr
	clrf vFIFORdPtr

	; Inicializa variaveis plotter
	banksel vPLOTStatus
	clrf vPLOTStatus
	bsf  vPLOTStatus, PLOTOnLine ; deixa em ONLINE

	pagesel Loop
	
	banksel INTCON
	bcf INTCON, INTE
	bcf INTCON, RBIE
	bsf INTCON,T0IE ; habilita interrupts on TMR0 overflow
	bsf INTCON,GIE ; habilita global interrupt
	call PutOK


;***********************************
;***********************************
;***********************************
Loop	; Loop principal do programa
	; verifica estado das chaves externas
	pagesel KeyScan
	call KeyScan

	pagesel ExecBehavior
	call ExecBehavior

	pagesel Loop
	banksel vUARTIntRxBufDataCnt	; verifica se tem dados a serem processados
	movf vUARTIntRxBufDataCnt,W		;	na porta serial
	btfsc STATUS,Z
	goto Loop

	; Processa Mensagem da Serial
	call WaitRxData
	banksel RXByte
	movwf RXByte

	Je 's', RXByte, STATUS_SIMPLES
	Je 'l', RXByte, LINHA_SIMPLES
	Je 'L', RXByte, LINHA
	Je 'S', RXByte, STATUS_SISTEMA
	Je 'T', RXByte, ALTERA_TIMER_VAL
	Je 'A', RXByte, ALTERA_PWM
	Je 'R', RXByte, CAPTURA_PORTD
	Je 'O', RXByte, DESLIGAMOTORES
	Je 'B', RXByte, SETABEHAVIOR

	goto DADOINVALIDO

;********************************************
; Seta variaveis de comportamento
; comportamentos = movimento em cada uma das 4 direções
SETABEHAVIOR

	call WaitRxData	; W contem dado
	banksel beh0
	movwf beh0
	call WaitRxData	; W contem dado
	banksel beh1
	movwf beh1
	call WaitRxData	; W contem dado
	banksel beh2
	movwf beh2
	call WaitRxData	; W contem dado
	banksel beh3
	movwf beh3
	call PutOK

	goto Loop

;********************************************
DESLIGAMOTORES
	call desligaMotoresFunc
	call PutOK
	goto Loop

;********************************************
; 's' retorna 2 byte contendo os flags do estado atual do FIFO e do PLOTTER
; envia pela serial vFIFOStatus, vPLOTStatus,'K'
STATUS_SIMPLES
	banksel vFIFOStatus
	movf vFIFOStatus,W	; byte inteiro de status do FIFO
	call PutChar
	banksel vPLOTStatus
	movf vPLOTStatus, W
	call PutChar
	call PutOK ; manda apenas um K para informar que completou envio linha
	;movlw ENDLINE ;0x0d ; nova linha
	;call PutChar
	goto Loop

;********************************************
CAPTURA_PORTD
	banksel PORTD
	movf PORTD,W
	call PutChar
	call PutOK
	
	goto Loop

;********************************************
; Altera contador PWM de um pinoe specifico - 4 disponiveis
; Sobre 6.5Khz/60hz - 65hz - 150uS aprox entre passos
;vServoLmt res .4 ; posição de cada servo
ALTERA_PWM
	movlw vServoLmt
	movwf FSR
	bankisel vServoLmt

	call WaitRxData
	andlw 0x3 ; dentro de conjunto de 4 bytes no maximo
	addwf FSR,F
	call WaitRxData
	movwf INDF
	call PutOK ; manda apenas um K para informar que completou recebimento do sinal PWM

	goto Loop

;******************************************************
; Esta rotina quando utilizada sobreescreve o trabalho
; atual do PIC que estiver sendo executado; qualquer
; informação enviada pelo FIFO Circular
; As proximas instruções do FIFO ficam aguardando para serem executadas
; caso  ultrapasse o limite do FIFO, ele é sobreescrito
LINHA_SIMPLES

	banksel cANGX
	bankisel cANGX
	movlw cANGX ; inicializa FSR
	movwf FSR

	call WaitRxData ; w vai conter 1 caractere recebido
	movwf INDF ; cANGX MSB ; move caractere para endereço indicado por FSR

	call WaitRxData
	incf FSR,F
	movwf INDF ; cANGX

	call WaitRxData
	incf FSR,F
	movwf INDF ; cANGX

	call WaitRxData
	incf FSR,F
	movwf INDF ; DIRX

	call WaitRxData
	incf FSR,F
	movwf INDF ; cANGY MSB

	call WaitRxData
	incf FSR,F
	movwf INDF ; cANGY

	call WaitRxData
	incf FSR,F
	movwf INDF ; cANGY

	call WaitRxData
	incf FSR,F
	movwf INDF ; DIRY

	call WaitRxData
	incf FSR,F
	movwf INDF ; cANGZ MSB

	call WaitRxData
	incf FSR,F
	movwf INDF ; cANGZ

	call WaitRxData
	incf FSR,F
	movwf INDF ; cANGZ

	call WaitRxData
	incf FSR,F
	movwf INDF ; DIRZ

	call WaitRxData
	incf FSR,F
	movwf INDF ; nPassos MSB

	call WaitRxData
	incf FSR,F
	movwf INDF ; nPassos

	call WaitRxData
	incf FSR,F
	movwf INDF ; nPassos LSB

	bankisel CONTADORX
	banksel CONTADORX
	movlw CONTADORX
	movwf FSR
NEXTBYTE3	clrf INDF
	incf FSR,F
	movlw carryZ+1 ; até carryZ que é o ultimo
	xorwf FSR,W
	btfss STATUS,Z
	goto NEXTBYTE3

	banksel vPLOTStatus
	bsf vPLOTStatus, PLOTDrawing ; agora tem uma linha em andamento

	call PutOK ; manda apenas um K para informar que completou recebimento da linha

	goto Loop

; Rotina de recebimento de 15 bytes	
; 	definem comprimento do vetor X(3)+dir(1) Y(3)+dir(1) Z(3)+dir(1) e nPassos(3)
; Grava na região apontada por vFIFOWrPtr - implementa um FIFO Circular
;		A rotina da Interrupção retira dados deste FIFO CIRCULAR
; Caso o FIFO encha, ativa flag FIFOBufFul
; Caso chegue mais um dado alem do limite FIFO, ativa o flag FIFOBufOF,
;		e sobreescreve o proximo dado
LINHA	
	banksel vFIFOWrPtr		; copia valor para a memoria
	bankisel vFIFOBuffer		; aponta para região onde começar a gravar os dados
	movf vFIFOWrPtr,W		; registros estao todos no mesmo banco
	addlw low(vFIFOBuffer)
	movwf FSR

	; rotina buffer simples,nao utilizada mais
	;movlw cANGX ; inicializa FSR
	;movwf FSR
	call WaitRxData ; w vai conter 1 caractere recebido
	movwf INDF ; cANGX MSB ; move caractere para endereço indicado por FSR

	call WaitRxData
	incf FSR,F
	movwf INDF ; cANGX

	call WaitRxData
	incf FSR,F
	movwf INDF ; cANGX

	call WaitRxData
	incf FSR,F
	movwf INDF ; DIRX

	call WaitRxData
	incf FSR,F
	movwf INDF ; cANGY MSB

	call WaitRxData
	incf FSR,F
	movwf INDF ; cANGY

	call WaitRxData
	incf FSR,F
	movwf INDF ; cANGY

	call WaitRxData
	incf FSR,F
	movwf INDF ; DIRY

	call WaitRxData
	incf FSR,F
	movwf INDF ; cANGZ MSB

	call WaitRxData
	incf FSR,F
	movwf INDF ; cANGZ

	call WaitRxData
	incf FSR,F
	movwf INDF ; cANGZ

	call WaitRxData
	incf FSR,F
	movwf INDF ; DIRZ

	call WaitRxData
	incf FSR,F
	movwf INDF ; nPassos MSB

	call WaitRxData
	incf FSR,F
	movwf INDF ; nPassos

	call WaitRxData
	incf FSR,F
	movwf INDF ; nPassos LSB

	movf vFIFOWrPtr,W	; ao terminar incremeta ponteiro de escrita;
	addlw BYTES_PER_FIFO
	movwf vFIFOWrPtr
	movlw FIFO_RX_BUF_SIZE	; nao estourou o limite do ponteiro escrita?
	xorwf vFIFOWrPtr,W
	btfsc STATUS,Z
	clrf vFIFOWrPtr	; sim, zera ponteiro escrita

	btfsc vFIFOStatus,FIFOBufFul	; Já estava cheio?
	goto rFIFOBufFull	; buffer cheio, ativa OVERFLOW e
						; evita que Cnt ultrapasse limite contagem
	
	bcf vFIFOStatus, FIFOBufEmpty	; buffer nao está mais vazio
	incf vFIFODataCnt,F	; numero de bytes chegou ao limite? ativa flag de FIFOBufFul
	movlw FIFO_N_REG
	xorwf vFIFODataCnt,W
	btfsc STATUS,Z
	bsf vFIFOStatus,FIFOBufFul

	goto rFIFOContinue

rFIFOBufFull
	bsf vFIFOStatus,FIFOBufOF		; marca o Overflow do buffer!, dados sobre escritos
	; nao incrementa FIFO ;incf vFIFODataCnt	; numero de bytes chegou ao limite? ativa flag de FIFOBufFul
	bcf vFIFOStatus, FIFOBufEmpty

rFIFOContinue
	; depois de receber o byte, ativa desenho da linha
	;movlw ENDLINE
	call PutOK ; manda apenas um K para informar que completou recebimento da linha
	;movlw ENDLINE ;0x0d ; nova linha
	;call PutChar
	goto Loop

ALTERA_TIMER_VAL
	call WaitRxData
	banksel VARIAVEL_TIMER_VAL
	movwf VARIAVEL_TIMER_VAL
	call PutOK
	goto Loop

DADOINVALIDO
	call PutERRO
	movlw ENDLINE ;0x0d ; nova linha
	call PutChar
	goto Loop

;******************************************************
; Verifica se existem dados disponiveis na porta serial
LookForRxData ;Check if Receive buffer is full
	banksel vUARTIntRxBufDataCnt
	movf vUARTIntRxBufDataCnt,W
	;btfsc STATUS,Z
	;goto ReadAgain
	return
	

;**********************************************
; Aguarda a chegada de 1 BYTE na porta serial
; 		Rotina salva o FSR e bits de IPR , RP0 e RP1 do STATUS
; Olha o buffer serial por dados de entrada e retorna
; os mesmos caso existam, caso contrario sai da rotina
; Retorna dado em W

WaitRxData ;Check if Receive buffer is full
	movf FSR,W
	movwf tFSR
	movf STATUS,W
	;andlw b'11100000'
	movwf tSTATUS;
	
WaitRxData1
	banksel vUARTIntRxBufDataCnt
	movf vUARTIntRxBufDataCnt,F
	; INT entra aqui e destroi o valor de STATUS:Z verificar
	btfsc STATUS,Z
	goto    WaitRxData1

;If receive buffer have data then read it
ReadAgain
	;bsf vUARTIntStatus,UARTIntRxBufEmpty	; estranho essa linha
	bcf INTCON,T0IE ; desabilita interrupções on TMR0 overflow
	Pagesel UARTIntGetCh	; destroi FSR!!!
	call    UARTIntGetCh
	banksel tW
	bsf INTCON,T0IE ; habilita interrupções on TMR0 overflow
	movwf tW

	movf tSTATUS,W ; restaura registradores IPR e RP0 e RP1
	;iorwf STATUS,F
	movwf STATUS

	movf tFSR,W; restaura FSR
	movwf FSR
	movf tW,W
	return

;*********************************************************
; Aguarda a disponibilizacao de espaço no buffer de envio
WaitTX
	banksel vUARTIntStatus
	btfsc vUARTIntStatus, UARTIntTxBufFul
	goto WaitTX
	return

;**************************************
; Mensagens para retorno
; K = OK
; F = FULL; buffer cheio
; E = ERRO
; R = READY
; *******************************************

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
PutFULL
	movlw   'F'
		banksel vUARTIntStatus
Wait2	btfsc vUARTIntStatus, UARTIntTxBufFul
	goto Wait2
	Pagesel UARTIntPutCh
	call UARTIntPutCh
	return	;9+2 CALL PUT CHAR

; Envia Mensagem de READY 'R' pela serial
PutREADY
	movlw   'R'
		banksel vUARTIntStatus
	btfsc vUARTIntStatus, UARTIntTxBufFul
	goto $-1
	Pagesel UARTIntPutCh
	call UARTIntPutCh
	return	;9+2 CALL PUT CHAR

; Envia Mensagem de ERRO 'E' pela serial
PutERRO
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
	btfsc vUARTIntStatus, UARTIntTxBufFul
	goto $-1
	Pagesel UARTIntPutCh
	call UARTIntPutCh
	return

;**********************************************
; Rotina para enviar pela porta serial um byte
;	em formato HEXADECIMAL xx
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

; CONVERSAO de Caractere BIN to HEX
; Recebe bytes com 4 bit LSB contendo o valor a ser convertido
; Retorna em W o valor ascii do caractere hexadecimal
; Destroi conteudo do PCLATCH

bin2hex
	banksel tmpChar
	addlw  TABELA1
	movwf  tmpChar
	rlf    ZERO,w ; PARA LIMPAR BIT CARRY
	addlw  high(TABELA1)
	movwf  PCLATH
	movf   tmpChar,w
	movwf  PCL
TABELA1
	dt '0','1','2','3','4','5','6','7','8','9','a','b','c','d','e','f';
	return


P1 CODE 0x0400
;*****************************************************
; Rotina de SOMA AUXILIAR Funcional - 6 bytes - OTIMIZADA
; copia variavel apontada por addB para area temporaria ADDTMP
; efetua a soma dos 3 bytes LSB
; propaga carry
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
	incf FSR,F
	movf INDF,W
	movwf addtmp+4	; quinto byte
	incf FSR,F
	movf INDF,W
	movwf addtmp+5	; sexto byte

	bcf STATUS,IRP
	banksel addtmp
	; inicia soma !!!!!!!REVER ESTES DADOS
	movf addA,W		; indireto, addA é um endereço inicial apenas
	addlw 0x5		; começo pelo ultimo byte
	movwf FSR		; INDF esta apontando para ultimo byte dito por addA
	clrf carry

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
		incf carry,F ;  o carry nunca vai ter overflow, desde que seja abaixo de 128 numeros somados consecutivamente!
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

;**************************************************************
; A Rotina de status completo alocada em um segmento sem muitos
; dados para nao ficar comendo memoria inicial do sistema
; exibe varios dados formatados na saida serial
; caractere K utilizado como finalizador da string de dados
;**************************************************************
STATUS_SISTEMA	nop
	; envia FULL(F) ou READY(R) para status atual - desenhando ou livre
	banksel vPLOTStatus
	btfsc vPLOTStatus,PLOTDrawing
	call PutFULL
	btfss vPLOTStatus,PLOTDrawing
	call PutREADY

	banksel tmpLadr	; exibe str1
	movlw low str1
	movwf tmpLadr
	movlw high str1
	movwf tmpHadr
	call printS

	banksel tmpLadr
	movlw CONTADORX ; exibe X
	movwf tmpLadr
	movlw HIGH CONTADORX
	movwf tmpHadr
	movlw 0x6		; 6 bytes a exibir
	movwf nBytes2Send
	call PutBigHEX

	SendTAB

	banksel tmpLadr	
	movlw CONTADORY ; exibe Y
	movwf tmpLadr
	movlw HIGH CONTADORY
	movwf tmpHadr
	movlw 0x6		; 6 bytes a exibir
	movwf nBytes2Send
	call PutBigHEX

	SendTAB

	banksel tmpLadr
	movlw CONTADORZ ; exibe Z
	movwf tmpLadr
	movlw HIGH CONTADORZ
	movwf tmpHadr
	movlw 0x6		; 6 bytes a exibir
	movwf nBytes2Send
	call PutBigHEX

	SendCRLF

	movlw 'P'
	call PutChar

	SendTAB

	banksel tmpLadr
	movlw nPassos ; exibe contador de passos
	movwf tmpLadr
	movlw HIGH nPassos
	movwf tmpHadr
	movlw 0x3			; 3 bytes a exibir
	movwf nBytes2Send
	call PutBigHEX

	SendCRLF

	movlw 'T'	; valor do Timer
	call PutChar
	banksel VARIAVEL_TIMER_VAL
	movf VARIAVEL_TIMER_VAL,W
	call PutBIN2HEX
	
	SendTAB

	movlw 't'		; numero de iteracoes da ultima
	call PutChar	; 	entrada na Interrupcao
	banksel timerH
	movf timerH,W
	call PutBIN2HEX
	banksel timerL
	movf timerL,W
	call PutBIN2HEX

	call PutOK

	goto Loop


;*******************************************
; Exibição de string pela serial printString
; Recebe:
;	tmpHadr:tmpLadr ponteiro posicao inicial da string
;	Exibe a string até encontrar um NULL
;
;	Variaveis internas:
;			tmpChar
;	Destroi PCLATCH,tmpChar, tmpHadr:tmpLadr, e DATABANK
printS
	call getChar	; W conterá o caractere
	banksel tmpChar
	movwf tmpChar
	xorlw 0x0
	btfsc STATUS,Z
	goto printSFim
	
	movf tmpChar,W
	call PutChar	; exibe o caractere
	
	; incrementa posicões da memoria
	banksel tmpLadr
	incfsz tmpLadr,F
	decf tmpHadr,F
	incf tmpHadr,F
	goto printS
printSFim
	return

; Retorna um caractere da string
getChar
	banksel tmpLadr
	movf tmpHadr,W
	movwf PCLATH
	movf tmpLadr,W
	movwf PCL

;*******************************************************
; Envia nByte2Send de informação da RAM formatado em
;		HEXADECIMAL XX:XX:XX:....
; 	através da porta serial
; Entrada:
;		tmpLadr - endereço menor
;		tmpHadr - endereço maior
;		nBytes2Send - numero de bytes a transmitir
;
;		Os bytes nao devem cruzar a borda de memoria de 256 bytes
;	Destroi W, IRP, RP0, RP1, FSR
PutBigHEX
	banksel putCount
	clrf putCount	; sera contador
pBNextByte
	; banksel nBytes2Send	; tudo na mesma area de memoria
	movf nBytes2Send,W	; verifica se ainda tem algo a enviar
	xorwf putCount,W
	btfsc STATUS,Z
	goto pBEnd

	; ajusta DATABANK e FSR para capturar byte
	movf tmpLadr,W
	addwf putCount,W
	movwf FSR

	movf tmpHadr,W
	bcf STATUS,IRP
	btfsc tmpHadr,0x0
	bsf STATUS, IRP

	banksel tmpByte	; captura byte a ser enviado
	movf INDF,W		; e salva em tmpByte
	movwf tmpByte

	swapf tmpByte,w		; nible superior
	andlw 0xF
	call bin2hex
	call WaitTX
	Pagesel UARTIntPutCh
	call	UARTIntPutCh

	banksel tmpByte 	; nible inferior
	movf tmpByte,w
	andlw 0xF
	call bin2hex
	call WaitTX
	Pagesel UARTIntPutCh
	call	UARTIntPutCh

	movlw ' '
	call WaitTX
	Pagesel UARTIntPutCh
	call	UARTIntPutCh

	;next byte
	banksel putCount	; importante cuidar os bancos, principamente
	incf putCount,F		; em rotinas onde exite muitas chamadas de rotinas
	goto pBNextByte		; externas

pBEnd

	return

;*******************************************
Strings
str1	dt 0xd,0xa,"xx xx xx:xx xx xx	yy yy yy:yy yy yy	zz zz zz:zz zz zz",0xd, 0xa,.0
str2	dt "Controle Motor de Passo 0.5 - 09-09-2010", .0


;*********************************************
P2 CODE 0x0800		; rotinas alocadas em uma area completamente limpa, para evitar problemas de cruzamento de borda de memoria
;*********************************************
; Rotina Processamento passos para
; LB1946 - EPSON C60 - Tabela configurada para 4 micropassos entre passos
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

; Tabelas de passos
; Recebe: W contem o ponteiro com valor desejado na tabela ;
; vaiaveis compatilhadas com outra rotina de passos para outros modelos de motores
; retorna o valor em W

tabLB1946a
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


;*********************************************
; Rotina Processamento passos para
; A2919SLB - EPSON - Tabela configurada para 4 micropassos entre passos
; Recebe:
;	ppMotor - com a posicao onde estava o ultimo passo
;	direcao - para onde mover para o proximo passo
; Retorna:
;	ppMotor - atualizado com nova posicao
;	grava as variaveis sLOW e sHIGH com valores a serem enviados

procStepA2919
	banksel direcao
	;movf direcao,W
	btfsc direcao, 0x0 ; verifica para que direção mover-se
; diminuir 1
	goto diminuir2a
; adicionar 1
	incf ppMotor,W
	andlw 0x0f ; CORRIGIR CONFORME TABELA PASSOS
	goto continua2a
diminuir2a
	decf ppMotor,W
	andlw 0x0f ; mantem dentro dos limites

continua2a
	banksel ppMotor
	movwf ppMotor
	pagesel tabA2919
	call tabA2919 ; W deve conter contador e W vai retornar valor
	banksel sLOW
	movwf sLOW
	return

; Tabelas de passos tabA2919
; Recebe: W contem o ponteiro com valor desejado na tabela ;
; variaveis compatilhadas com outra rotina de passos para outros modelos de motores
; retorna o valor em W

tabA2919
	banksel motorWTmp
	movwf motorWTmp
	pageselw A2919
	movf motorWTmp,W
	andlw 0x0f
	addlw LOW A2919
	btfsc STATUS,C                ;then increment PCLATH. Then load the
	incf PCLATH,f                 ;program counter with computed goto.
	movwf PCL
A2919	retlw b'00000110'	; BITs MSB:LBS (012X:456X) || (PH1 I11 I01 X):(PH2 I12 I02 X)
	RETLW b'00000100'
	retlw b'00100010'
	RETLW b'01000000'
	retlw b'11100000'
	RETLW b'11000000'
	retlw b'10100010'
	RETLW b'10000100'
	retlw b'10001110'
	RETLW b'10001100'
	retlw b'10101010'
	RETLW b'11001000'
	retlw b'01101000'
	RETLW b'01001000'
	retlw b'00101010'
	RETLW b'00001100'
	return

;*************************************
; Rotina processamento de passos para
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

; As tabelas abaixo representam os dados para o correto funcionamento
; dos motores de passo LM18245 da National
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

;****************************************************
; Rotina serialização de dados LB1946 - EPSON C60 
; W contem porta a enviar dados
; PORTA W - BIT 0 CLOCK e BIT 1 DADOS
; 	o bit de seleção já deve ter previamente setado pela rotina que chamou
; sLOW e sHIGH contem dados a enviar - primeiros 6 bits são enviados para cada motor
; ROTINA LENTA - OTIMIZAR UTILIZANDO TIMER2 e MODULO SPI
serializar
	movwf FSR	
	movlw .6
	movwf serCount ; 6 bits iniciais

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
	decf serCount,F
	btfss STATUS,Z
	goto serProximoBit1
	; termina rotina serialização
	;bcf INDF,0x0

serNextByte	
	movlw .6	; 6 bits finais
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

	decf serCount,F ; proximo bit
	; verifica se já é o bit 6
	btfss STATUS,Z
	goto serProximoBit2
	; termina rotina serialização
	bcf INDF,0x1
	bcf INDF,0x0
	return ; TERMINA


;********************************************
; Rotina de serialização de dados de teste
; Utilizando o modulo MSSP
SerializarViaSSP	nop
	banksel TRISC
	bcf TRISC,0x5
	bcf TRISC,0x3
	movlw b'00000000'
	movwf SSPCON
	banksel SSPSTAT
	bsf SSPSTAT, 0x6
	banksel SSPCON
	bsf SSPCON, 0x5	; habilita modulo SSP
	banksel sLOW
	movf sLOW,W
	movwf SSPBUF	; dispara transmissao
	nop
	nop
	nop
	;nop
	;nop	; 6 bits
	bcf SSPCON,0x5	; tenta interromper transmissão no 6 bit(6 ciclo depois de setar o BUFFER)

	movf sHIGH,W
	bsf SSPCON, 0x5	; habilita modulo SSP
	movwf SSPBUF	; dispara transmissao
	nop
	nop
	nop
	;nop
	;nop	; 6 bits
	bcf SSPCON,0x5		; suposto ter enviado todos os dados neste momento

	return

;********************************************
; Rotina de processamento das teclas
; admite que nao precisa salvar nenhum registrador neste ponto
;
; Variaveis serão alocadas na 2 ou 3 area de memoria vaga

;		BIT4 - Botao para avanço de X
;		BIT5 - Botao para retrocesso de X
;		BIT6 - Botao para avanço de Y
;		BIT7 - Botao para retrocesso de Y

ONLINEPORT equ PORTB
ONLINEBIT equ 0x0

DIRECTIONPORT equ PORTB
UPBIT equ 0x4
DOBIT equ 0x5
LEBIT equ 0x6
RIBIT equ 0x7

VELTESTE equ 0x40FFFF ; meia velocidade por CLOCK
NPASSOS equ .5000	; 3000 passos
TIME2DEBOUNCE equ 0x60

KeyScan	; todas as portas estao no primeiro banco
	banksel PORTB
	btfsc ONLINEPORT, ONLINEBIT
	goto tglOnLineStatus

	; executa movimentação adicional
	; 	apenas se estiver no modo OFFLINE e
	btfsc vPLOTStatus, PLOTOnLine
	goto FimKeyScan
	; nao houver nenhuma linha ativa ocorrendo PLOTDrawing != 1
	btfsc vPLOTStatus, PLOTDrawing
	goto FimKeyScan

	btfsc DIRECTIONPORT, UPBIT
	call actUp
	
	btfsc DIRECTIONPORT, DOBIT
	call actDown

	btfsc DIRECTIONPORT, LEBIT
	call actLeft
	
	btfsc DIRECTIONPORT, RIBIT
	call actRight

FimKeyScan
	return;

tglOnLineStatus
	banksel debOnLine
	incf debOnLine,F
	movlw TIME2DEBOUNCE
	xorwf debOnLine,W
	btfss STATUS,Z
	goto FimKeyScan

	clrf debOnLine ; zera reg debounce
	banksel vPLOTStatus	; ENTAO alterna LED e status
	movlw .1 << PLOTOnLine
	xorwf vPLOTStatus, F
	goto FimKeyScan

;*******
; Cima
actUp
	banksel dirMotorX
	bcf dirMotorX,0x0	; seta flag de direção para 0
	goto actDownR

;*******
; Baixo
actDown
	banksel dirMotorX
	bsf dirMotorX,0x0	; seta flag de direção para 1
actDownR
	; seta valor do contador de movimento para 3000 passos
	banksel  nPassos
	movlw NPASSOS & 0xFF
	movwf  nPassos+.2
	movlw (NPASSOS & 0xFF00) >> .8
	movwf nPassos+.1
	clrf nPassos

	movlw VELTESTE & 0xFF
	movwf cANGX+.2
	movlw (VELTESTE & 0xFF00)>> .8
	movwf cANGX+.1
	movlw (VELTESTE & 0xFF0000) >> .16
	movwf cANGX

	clrf cANGY
	clrf cANGY+.1
	clrf cANGY+.2
;	clrf cANGZ
;	clrf cANGZ+.1
;	clrf cANGZ+.2

	; ativa desenho
	banksel vPLOTStatus
	bsf vPLOTStatus, PLOTDrawing

	return

;*******
; Esquerda
actLeft
	banksel dirMotorY
	bcf dirMotorY,0x0	; seta flag de direção para 0
	goto actRightR

;*******
; Direita
actRight
	banksel dirMotorY
	bsf dirMotorY,0x0	; seta flag de direção para 0
	; seta valor do contador de movimento para 3000 passos
actRightR
	banksel  nPassos
	movlw NPASSOS & 0xFF
	movwf  nPassos+.2
	movlw (NPASSOS & 0xFF00) >> .8
	movwf nPassos+.1
	clrf nPassos

	movlw VELTESTE & 0xFF
	movwf cANGY+.2
	movlw (VELTESTE & 0xFF00)>> .8
	movwf cANGY+.1
	movlw (VELTESTE & 0xFF0000) >> .16
	movwf cANGY

	clrf cANGX
	clrf cANGX+.1
	clrf cANGX+.2
;	clrf cANGZ
;	clrf cANGZ+.1
;	clrf cANGZ+.2

	; ativa desenho	
	banksel vPLOTStatus
	bsf vPLOTStatus, PLOTDrawing
	return

;************************************
; Resposta imediata a Colisoes
; Cada reposta dos motores (AB) tem
; um comportamento correspondente de resposta
; que é avaliado via (SENSOR AND RESPOSTA)
; caso seja positiva a respota, ativa o movimento correspondente
; actRight actLeft actUp actDown

ExecBehavior
	banksel beh0	; COMPORTAMENTO 0
	movf beh0,W
	banksel PORTD
	andwf PORTD,W
	btfss STATUS,Z
	; ocorreu algo
	call actUp
	; não ocorreu nada

	banksel beh1	; COMPORTAMENTO 1
	movf beh1,W
	banksel PORTD
	andwf PORTD,W
	btfss STATUS,Z
	; ocorreu algo
	call actDown
	; não ocorreu nada

	banksel beh2	; COMPORTAMENTO 2
	movf beh2,W
	banksel PORTD
	andwf PORTD,W
	btfss STATUS,Z
	; ocorreu algo
	call actLeft
	; não ocorreu nada

	banksel beh3	; COMPORTAMENTO 3
	movf beh3,W
	banksel PORTD
	andwf PORTD,W
	btfss STATUS,Z
	; ocorreu algo
	call actRight
	; não ocorreu nada

	return

;*****************************************
; ROTINA DE GERAÇÂO DE SINAIS PWM
;************************************
;vServoClockCnt res 01
;vServoLmt res .4 ; posição de cada servo

SERVO_MAX_CNT equ .100

#define BIT_SERVO1 PORTA,0x4
#define BIT_SERVO2 PORTA,0x5
#define BIT_SERVO3 PORTB,0x1
#define BIT_SERVO4 PORTB,0x2

processaPWMFunc
	; seleciona area de memoria onde estao gravados os registros de PWM
	banksel vServoClockCnt
	incf vServoClockCnt,f
	movlw SERVO_MAX_CNT
	xorwf vServoClockCnt,w
	btfss STATUS,Z ; ZERO -> 1 entao são iguais - significa que é hora de resetar o sinal PWM
	; diferentes
	goto procPWM_ANALISA
	; iguais
procPWM_SETA_ALL
	clrf vServoClockCnt	; zera contador
	banksel PORTA
	bsf BIT_SERVO1 
	bsf BIT_SERVO2
	bsf BIT_SERVO3
	bsf BIT_SERVO4

procPWM_ANALISA
	banksel vServoClockCnt
	movf vServoClockCnt,W
	xorwf vServoLmt,W
	btfss STATUS,Z	; se for igual tempo escolhido entao reseta pino
	goto NEXT_PWM_SIG1
	banksel PORTA
	bcf BIT_SERVO1
	banksel vServoClockCnt
	; senao o proximo teste

NEXT_PWM_SIG1
	movf vServoClockCnt,W
	xorwf vServoLmt+.1,W
	btfss STATUS,Z
	goto NEXT_PWM_SIG2
	banksel PORTA
	bcf BIT_SERVO2
	banksel vServoClockCnt
	; senao o proximo teste

NEXT_PWM_SIG2
	movf vServoClockCnt,W
	xorwf vServoLmt+.2,W
	btfss STATUS,Z
	goto NEXT_PWM_SIG3
	banksel PORTA
	bcf BIT_SERVO3
	banksel vServoClockCnt
	; senao o proximo teste

NEXT_PWM_SIG3
	movf vServoClockCnt,W
	xorwf vServoLmt+.3,W
	btfss STATUS,Z
	goto NEXT_PWM_SIG4
	banksel PORTA
	bcf BIT_SERVO4
	banksel vServoClockCnt

NEXT_PWM_SIG4
	return

;********************************************
; Desliga MOTORES para reduzir consumo
desligaMotoresFunc
	banksel sLOW
	clrf sLOW
	clrf sHIGH
	
	banksel PORTA
	bcf PORTA,0x3 ; bit controle
	movlw PORTA
	pagesel serializar
	call serializar

	banksel PORTA
	bsf PORTA,0x3 ; bit controle
	pagesel desligaMotoresFunc

	banksel PORTA
	bcf PORTA,0x2 ; bit controle
	movlw PORTA
	pagesel serializar
	call serializar
	banksel PORTA
	bsf PORTA,0x3 ; bit controle
	pagesel desligaMotoresFunc
	
	return

;********************************************
; Rotina de captura de dados do ADC
; Verifica se dado já está disponivel no ADC & se está em um ciclo de conversão?
; 	Caso não ativo ou esperando conversao
;		Já é momento de pegar novo dado então
;			programa ADC para pegar dado da ENTRADA vADCInput
;			retorna
;		Caso contrario retorna

; 	Caso dados prontos então
;		pega dado e salva na posição do buffer vADCWrPtr
;		
;
;
;
;
;


end