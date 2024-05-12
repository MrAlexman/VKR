
.syntax unified
.thumb
.cpu cortex-m4
.fpu fpv4-sp-d16

.macro	MOV32 regnum,number
	MOVW \regnum,:lower16:\number
	MOVT \regnum,:upper16:\number
.endm

.equ 	PERIPH_BASE			,0x40000000
.equ 	APB1PERIPH_BASE     ,0x00000000

.equ 	_PWR_BASE            ,(APB1PERIPH_BASE + 0x7000)
.equ 	_PWR_CR1             ,0x00000000
.equ 	mPWR_CR_VOS            ,0x4000


.equ FLASH_R_BASE			,0x40023C00
.equ	_ACR				,0x00000000
.equ		fICEN			,0x00000200
.equ		fDCEN			,0x00000400
.equ		mLATENCY		,0x00000005
.equ		fPRFTEN			,0x00000100



.equ 		GPIOA_BASE			,0x40020000 		//Базовый адрес
.equ		GPIOB_BASE			,0x40020400
.equ		GPIOC_BASE			,0x40020800

.equ		_MODER,		0x00
.equ		_IDR,		0x10
.equ		_ODR,		0x14
.equ		_OTYPER,	0x04
.equ		_PUPDR,		0x0C
.equ		_OSPEEDR,	0x08
.equ		_AFRL,		0x20
.equ		_AFRH,		0x24

.equ		mMODER5				,0x00000400		//Маска
.equ		mPUPDR5				,0x00000400
.equ		mPUPDRA				,0x001100400
.equ		mPUPDRB				,0x05
.equ 		fORD5				,0x00000020		//Флаг
.equ		fnORD5				,0xFFFFFFDF

.equ		fUsrBtn				,0x200

.equ		_A10nmb,			20
.equ		_A12nmb,			24

.equ		_B00nmb,			00
.equ		_B01nmb,			02

.equ		valAF,				02

.equ 		RCC_BASE 			,0x40023800
.equ	_CR						,0x00000000
.equ		fHSION				,0x00000001
.equ		fHSIRDY				,0x00000002
.equ		fPLLON				,0x01000000
.equ		fPLLRDY				,0x02000000

.equ		_PLLCFGR			,0x00000004
			/*		---		*/
.equ	_AHB1ENR			,0x00000030
.equ		fGPIOAEN		,0x01
.equ		fGPIOBEN		,0x02
.equ		fGPIOCEN		,0x04

.equ	_APB1ENR			,0x00000040
.equ		fUSART2EN		,0x20000
.equ		fPWREN			,0x10000000

.equ	_APB2ENR			,0x44
.equ		fUSART1EN		,0x10
.equ		fUSART6EN		,0x20
.equ		fSPI1EN			,0x001000
.equ		fSPI5EN			,0x100000

.equ	_CFGR				,0x00000008
.equ		mHPRE				,0x00000010
.equ		mPPRE2				,0x00008000
.equ		mPPRE1				,0x00001400

.equ		valPLLM			, 0x04
.equ		valPLLN			, 0x40
.equ		valPLLP			, 0x00
.equ		valPLLQ			, 0x04


.equ	USART1_BASE, 	0x40011000					@@ UART registers
.equ	USART2_BASE,	0x40004400
.equ	USART6_BASE,	0x40011400

.equ	_USART_CR1,	0x0C
.equ		fRE,		0b0100
.equ		fTE,		0b1000
.equ		fTXEIE,		0b10000000
.equ		fTCIE,		0b01000000
.equ		fRXNEIE,	0b00100000

.equ	USARTCR1val,	0b0010000000100000

.equ	_USART_BRR,	0x08
.equ	_USART_SR,	0x00

.equ	fTXE,		0b10000000
.equ	fTC,		0b01000000
.equ	fRXNE,		0b00100000

.equ	_USART_DR,	0x04

.equ	SPI1BASE, 	0x40013000
.equ	SPI5BASE,	0x40015000
.equ	_SPI_CR1,		0x00
.equ	_SPI_CR2,		0x04
.equ	_SPI_SR,		0x08
.equ	_SPI_DR,		0x0C


.equ	valSPI_CR1,		0b0000100001010111
.equ	RXNEIEMask,		0x40

.equ	ACL_BWRReg,	0x2C					//Internal registers of ADXL345
.equ	BWRval,		0x0F

.equ	ACL_DFReg,	0x31
.equ	DFval,		0x09

.equ	ACL_BWRateReg,	0x2C
.equ	ACL_POWCTLReg,	0x2D
.equ	ACL_INTSRCReg,	0x30
.equ	ACL_DZ0Reg,	0x36
.equ	ACL_DZ1Reg,	0x37

.equ	Rmask,		0x8000
.equ	Wmask,		0x7FFF



.equ	SysTickBASE,0xE000E010				@@ SysTick Timer registers
.equ	_STK_CTRL,	0x00
.equ	_STK_LOAD,	0x04
.equ	_STK_VAL,	0x08

.equ	MaxTimVal,	5000
.equ	SysIntrPeriod, 8000			// SysTick interrupt period: 80000 mean 1 ms, 8000 mean 0.1 ms etc.

.equ	NVIC_BASE,	0xE000E100
.equ	_ISER1,		4*1
.equ		fUSART1IRQ,	0x20
.equ		fUSART2IRQ,	0x40

.equ	_ISER2,		4*2
.equ	SPI5IRQMask, 0x200000
.equ	fUSART6IRQ,	0x80

.equ	_ISER3,		4*3

.equ	_ISPR1,		0x200 + 4*1
.equ	_ISPR2,		0x200 + 4*2

.equ	_ICPR1,		0x280 + 4*1
.equ	_ICPR2,		0x280 + 4*2


.equ	fRead00Byte,	0x02			// Flags of actions
.equ	fRead01Byte,	0x04
.equ	fRead10Byte,	0x08
.equ	fRead11Byte,	0x10
.equ	fTimeToRead,	0x20

.equ	fWriteCmd,		0x01

.equ	MaxBufSz,	0x10000

.equ	fSending,		0x01

.section	.bss

TimCount:	.word	0				// 0000
BtnState:	.word	0				// 0004
MesFlag:	.word	0				// 0008
SPIMode:	.word	0				// 000C
UARTMode:	.word	0				// 0010
SndPos:		.word	0
DataPos:	.word	0
DataBuf:	.word	0




.section	.text

.global main
.global MySysTick_Handler, MySPI5_Handler, MyUSART_Handler



main:


		bl 		SYSCLK_START

		ldr 	r0, =RCC_BASE
		ldr 	r1, [r0, _AHB1ENR]
		eor		r2,	r2
		orr 	r2, #fGPIOAEN				/* Enable GPIO A*/
		orr		r2,	#fGPIOBEN				/* Enable GPIO B*/
		orr		r2,	#fGPIOCEN				/* Enable GPIO C*/
		orr 	r1, r2
		str 	r1, [r0, _AHB1ENR]

		ldr		r1,	[r0, _APB2ENR]
		mov		r2,	#fSPI5EN				/* Enable SPI5  */
		orr		r1,	r2
		str		r1,	[r0, _APB2ENR]



		ldr 	r0, =GPIOA_BASE		//В r0 лег адрес равный GPIOA_BASE
		ldr 	r1, [r0, _MODER]	//В r1 легло значение по адресу r0+_MODER
		mov 	r2, #mMODER5		//В r2 легла маска значения для адреса r0+_MODER
											//На порт Α PA10, PA12
											//На порт B PB00, PB01           Για σου!
		orr 	r1, r2				//В r1 изменилось прежнее значение по маске в r2
		mov		r2,	#valAF
		bfi		r1,	r2,	#_A10nmb, #2
		bfi		r1, r2, #_A12nmb, #2

		str 	r1, [r0, _MODER]	//По адресу r0+_MODER легло значение из r1

		mov32	r1,	#0xFFFFFFFF
		str		r1,	[r0, #_OSPEEDR]

/*		ldr		r1,	[r0, #_OTYPER]
		mov		r2,	#0x1420
		orr		r1,	r2
		str		r1,	[r0, #_OTYPER]  */


		ldr		r1,	[r0, _AFRH]
		mov		r2,	#0b0110
		bfi		r1,	r2, #8, #4
		bfi		r1,	r2,	#16, #4
		str		r1,	[r0, _AFRH]

		ldr 	r1, [r0, _PUPDR]
		mov32	r2, #mPUPDRA
		orr 	r1, r2
		orr		r1,	#0x40000				//Pull-up for PA9
		str 	r1, [r0, _PUPDR]

		ldr 	r1, [r0, _ODR]
		mov	 	r2, #fORD5
		orr 	r1, r2
		str 	r1, [r0, _ODR]

		ldr r1, [r0, _ODR]
		mov		r2, #fnORD5
		and 	r1, r2
		str 	r1, [r0, _ODR]
///////////////
		ldr		r0,	=GPIOA_BASE				// User button settings
		ldr		r1,	[r0, #_MODER]
		eor		r2,	r2
		bfi		r1,	r2, #2*9, #2
		str		r1,	[r0, #_MODER]

		ldr		r0,	=BtnState
		mov		r1,	#1
		str		r1,	[r0]
/////////////////////////
								// Настройка работы и прерывания SysTick
		ldr		r0,	=SysTickBASE
		mov32	r1,	#SysIntrPeriod
		str		r1,	[r0, #_STK_LOAD]

		mov		r1,	#0b111
		str		r1,	[r0, #_STK_CTRL]

		ldr		r0,	=TimCount
		mov		r1,	#MaxTimVal
		str		r1,	[r0]


/* UART6 initialization ======================================= */
		ldr		r0, =RCC_BASE
		ldr		r1, [r0, #_APB2ENR]
		orr		r1, #fUSART6EN
		str		r1,	[r0, #_APB2ENR]

/*
		ldr		r1, [r0, #_APB2ENR]
		orr		r1, #fUSART1EN
		str		r1,	[r0, #_APB2ENR]
*/

		ldr		r0,	=GPIOC_BASE
		ldr		r1,	[r0, _MODER]
		movw	r2,	0x0A								//PC6, PC7 in the alternate function mode
		bfi		r1,	r2,	#12, #4

		str		r1,	[r0, _MODER]



		ldr		r1,	[r0, _PUPDR]
		mov		r2,	#0b0101						////!!!!!!!!!!!!!!!!!
		bfi		r1, r2, #12, #4
		str		r1,	[r0, _PUPDR]

		ldr		r0,	=GPIOC_BASE
		ldr		r1,	[r0, _AFRL]
		movw	r2,	#0x88
		bfi		r1, r2, #24, #8
		str		r1, [r0, _AFRL]


/*
		ldr		r1,	[r0, #_RCC_CIPR]
		movw	r2,	#0b00
		bfi		r1,	r2, #6, #2
		str		r1,	[r0, _RCC_CIPR]
*/

		ldr		r0,	=USART6_BASE
		mov32	r1, #USARTCR1val
		str		r1,	[r0, _USART_CR1]


		ldr		r0,	=USART6_BASE
		movw	r1,	#0x0C
		lsl		r1,	#4
		orr		r1, #0x0E
		str		r1, [r0, #_USART_BRR]

/*
		ldr		r0,	=USART1_BASE
		str		r1, [r0, #_USART_BRR]

		ldr		r1,	[r0, _USART_CR1]
		orr		r1,	#fTE
		orr		r1,	#fRE
		orr		r1,	#fTCIE
		str		r1,	[r0, _USART_CR1]  */

		ldr		r0,	=USART6_BASE
		ldr		r1,	[r0, _USART_CR1]
		orr		r1,	#fTE
		orr		r1,	#fRE
		orr		r1,	#fTCIE
		str		r1,	[r0, _USART_CR1]



		ldr		r0,	=USART6_BASE
		movw	r1,	#0xA5
		str		r1,	[r0, _USART_DR]

/*
		ldr		r0,	=USART1_BASE
		movw	r1,	#0xA5
		str		r1,	[r0, _USART_DR]
*/
		ldr		r0,	=NVIC_BASE
		ldr		r1,	[r0, #_ISER2]				/// !!!!!!!!!!!!!!!!
		orr		r1, #fUSART6IRQ					/// !!!!!!!!!!!!!!!!
		str		r1, [r0, #_ISER2]
											/* End of UART2 initialization */

/**/

///////////////

		ldr 	r0, =GPIOB_BASE				// Настройка работы SPI5, включая линии GPIO
		ldr		r1,	[r0, #_MODER]
		mov		r2,	#valAF
		bfi		r1,	r2,	#_B00nmb, #2
		bfi		r1, r2, #_B01nmb, #2
		str		r1,	[r0, #_MODER]

		mov32	r1,	#0xFFFFFFFF
		str		r1,	[r0, #_OSPEEDR]

/*		ldr		r1,	[r0, #_OTYPER]
		mov		r2,	#0x3
		orr		r1,	r2
		str		r1,	[r0, #_OTYPER]  /**/

		ldr 	r1, [r0, _PUPDR]
		mov32	r2, #mPUPDRB
		orr 	r1, r2
		str 	r1, [r0, _PUPDR]

		ldr		r1,	[r0, _AFRL]
		mov		r2,	#0b0110
		bfi		r1,	r2, #0, #4
		bfi		r1,	r2,	#4, #4
		str		r1,	[r0, _AFRL]

		ldr		r0,	=TimCount
		mov		r1,	#MaxTimVal
		str		r1,	[r0]
DelayForPUP:
		ldr		r1,	[r0]
		orrs	r1,	r1
		bne		DelayForPUP

		mov		r1,	#MaxTimVal
		str		r1,	[r0]

		ldr		r0,	=SPI5BASE
		mov		r1,	#valSPI_CR1
		str		r1,	[r0, #_SPI_CR1]

		ldr		r0,	=SPI5BASE
		mov		r1,	#0x04					// SSOE
		orr		r1,	#RXNEIEMask
		str		r1,	[r0, #_SPI_CR2]


		ldr		r0,	=TimCount
		mov		r1,	#MaxTimVal
		str		r1,	[r0]
DelayForSSOE:
		ldr		r1,	[r0]
		orrs	r1,	r1
		bne		DelayForSSOE

		mov		r1,	#MaxTimVal
		str		r1,	[r0]



		ldr		r0,	=NVIC_BASE
		ldr		r1,	[r0, #_ISER2]
		orr		r1,	#SPI5IRQMask
		str		r1,	[r0, #_ISER2]		/**/

		nop

/////////////////

		mov		r1,	#ACL_DFReg
		lsl		r1,	r1, #8
		orr		r1,	r1,	#DFval
		bl		SendSPICmd

		mov		r1,	#ACL_BWRateReg
		lsl		r1,	r1,	#8
		orr		r1,	r1,	#0x0F
		bl		SendSPICmd


		mov		r1,	#ACL_POWCTLReg
		lsl		r1,	r1, #8
		orr		r1,	r1,	#0b1000							//Turning on measurements
		bl		SendSPICmd

		ldr		r0,	=SPIMode
		mov		r1,	#fRead00Byte
		str		r1,	[r0]

/*
		mov		r1,	#ACL_DFReg
		lsl		r1,	r1, #8
		//orr		r1,	r1,	#DFval
		orr		r1,	r1,	#Rmask
		str		r1,	[r0, #_SPI_DR]
/**/

		ldr		r0,	=USART6_BASE
		movw	r1,	#0xA7
		str		r1,	[r0, _USART_DR]
/////////////////////////////////////////////////////////

Loop:

		ldr		r0,	=GPIOA_BASE
		ldr		r1,	[r0, #_IDR]						// reading the state of the user button
		ands	r1,	#fUsrBtn
		bne		btn1_found							//if the button is not pushed, goto...

		ldr		r0,	=BtnState
		ldr		r1,	[r0]
		orrs	r1,	r1
		bne		is_0was_1

is_0was_0:
		b		CheckMode


is_0was_1:
		eor		r1,	r1
		str		r1,	[r0]
		b		CheckMode


btn1_found:
		ldr		r0,	=BtnState
		ldr		r1,	[r0]
		orrs	r1,	r1								// what was the previous state?
		beq		is_1was_0							// if "was 0" then the button has been just released

is_1was_1:
		b		CheckMode


is_1was_0:

		mov		r1,	#1
		str		r1,	[r0]

		ldr		r0,	=MesFlag
		ldr		r1,	[r0]
		eors	r1,	#1
		str		r1,	[r0]

		bne		CheckMode

		ldr 	r0, =GPIOA_BASE						//if the measurments are turned off, turn off the light as well
		ldr 	r1, [r0, _ODR]
		mov		r2,	#fORD5
		eor		r2,	#0xFFFFFFFF
		and		r1,	r2
		str		r1,	[r0, _ODR]

		ldr		r0,	=DataPos
		eor		r1,	r1
		str		r1,	[r0]


CheckMode:

		ldr		r0,	=MesFlag						// is the measurements flag set?
		ldr		r1,	[r0]
		orrs	r1,	r1
		beq		L002								// if no, do nothing



		ldr		r0,	=TimCount
		ldr		r1,	[r0]
		orrs	r1,	r1
		bne		L001

		mov		r1,	#MaxTimVal
		str		r1,	[r0]

		ldr 	r0, =GPIOA_BASE
		ldr 	r1, [r0, _ODR]
		mov		r2,	#fORD5
		eor		r1,	r2
		str		r1,	[r0, _ODR]

		/*ldr		r0,	=USART6_BASE
		movw	r2,	#0xA5
		str		r2,	[r0, _USART_DR]  */

L001:


		ldr		r0,	=SPIMode
		ldr		r1,	[r0]
		mov		r2,	r1
		ands	r2,	#fTimeToRead					//Checking wether it's time to read
		beq		L002



read_low:
		mov		r1,	#ACL_DZ0Reg
		ldr		r0,	=SPI5BASE						// Read data from ACL register
		lsl		r1,	r1, #8
		orr		r1,	r1,	#Rmask
		str		r1,	[r0, #_SPI_DR]

		mov		r2,	#fRead01Byte
		orr		r2,	#fRead11Byte

wr_lr00:
		ldr		r0,	=SPIMode
		ldr		r1,	[r0]
		ands	r1,	r1,	r2
		beq		wr_lr00

read_hi:
		mov		r1,	#ACL_DZ1Reg
		ldr		r0,	=SPI5BASE						// Read data from ACL register
		lsl		r1,	r1, #8
		orr		r1,	r1,	#Rmask
		str		r1,	[r0, #_SPI_DR]

L002:

CheckToSend:

		ldr		r0,	=DataPos
		ldr		r1,	[r0]
		ldr		r2,	=SndPos
		ldr		r2,	[r2]

		eors	r1,	r2
		beq		L003							// Branch if SndPos==DataPos, i.e. the buffer is empty

		/*ldr		r0,	=USART6_BASE
		ldr		r0,	=NVIC_BASE
		ldr		r1,	[r0, #_ISPR2]
		orr		r1,	#fUSART6IRQ
		str		r1,	[r0, #_ISPR2]			/**/

		ldr		r0,	=UARTMode
		ldr		r1,	[r0]
		ands	r1,	#fSending
		bne		L003							// Branch if already in sending mode

		orrs	r1, #fSending
		str		r1,	[r0]						// Set the flag of sending

		bl		SendNext


L003:
		nop
		nop



		b 	Loop
/* ============================================================== */
SendSPICmd:
		push	{lr}
		ldr		r0,	=SPIMode
		mov		r2,	#fWriteCmd
		str		r2,	[r0]

		ldr		r0,	=SPI5BASE
		str		r1,	[r0, #_SPI_DR]

		ldr 	r0, =GPIOA_BASE
		ldr 	r1, [r0, _ODR]
		mov		r2,	#fORD5
		orr		r1,	r2
		str		r1,	[r0, _ODR]

		ldr		r0,	=SPIMode
wt_sc00:
		ldr		r1,	[r0]
		ands	r1,	#fWriteCmd
		bne		wt_sc00

		ldr 	r0, =GPIOA_BASE
		ldr 	r1, [r0, _ODR]
		bic		r1,	#fORD5
		str		r1,	[r0, _ODR]

		pop		{pc}

/* =============================================================== */
SendNext:
		push	{lr}

		ldr		r0,	=SndPos
		ldr		r2,	[r0]
		ldr		r0,	=DataBuf
		ldr		r1,	[r0, r2]

		ldr		r0,	=USART6_BASE
		str		r1,	[r0, #_USART_DR]

		ldr		r0,	=SndPos
		add		r2,	#1

		mov		r1,	#MaxBufSz			// If the max. value reached,
		eor		r1,	#0xFFFFFFFF
		ands	r2,	r1					//  set to 0
		str		r2,	[r0]


		pop		{pc}
/* =============================================================== */
MySysTick_Handler:											/* SysTick interrupt handler */

		push	{r0, r1, lr}

		ldr		r0,	=SPIMode
		ldr		r1,	[r0]
		orr		r1,	#fTimeToRead
		str		r1,	[r0]

		ldr		r0,	=TimCount
		ldr		r1, [r0]
		orrs	r1, r1
		beq		exit_systickintr

		sub		r1, #1

mst10:
		str		r1, [r0]

exit_systickintr:
		pop		{r0, r1, pc}
/* ==================================================================== */
MySPI5_Handler:

		push	{r0, r1, r2, r3, lr}

		ldr		r0,	=SPI5BASE
		ldr		r1,	[r0, #_SPI_SR]

		ldr		r1,	[r0, #_SPI_DR]

		ldr		r0,	=SPIMode
		ldr		r2,	[r0]

		mov		r0,	r2
		ands	r2,	#fWriteCmd
		bne		cmd_written

		mov		r2,	r0
		ands	r2,	#fRead01Byte
		bne		read_01byte

		mov		r2,	r0
		ands	r2,	#fRead11Byte
		bne		read_11byte

		mov		r2,	r0
		ands	r2,	#fRead10Byte
		bne		read_10byte

read_00byte:							//reading LSB of the 1st 2word
		ldr		r0,	=DataPos
		ldr		r2,	[r0]
		ldr		r0,	=DataBuf
		str		r1,	[r0, r2]

		ldr		r0,	=SPIMode
		mov		r1,	#fRead01Byte
		str		r1,	[r0]

		b		exit_intr_spi5

read_01byte:
		ldr		r0,	=DataPos
		ldr		r2,	[r0]
		ldr		r0,	=DataBuf
		ldr		r3,	[r0, r2]

		bfi		r3,	r1,	#8, #8
		str		r3,	[r0, r2]

		ldr		r0,	=SPIMode
		mov		r1,	#fRead10Byte
		str		r1,	[r0]


		b		exit_intr_spi5

read_10byte:
		ldr		r0,	=DataPos
		ldr		r2,	[r0]
		ldr		r0,	=DataBuf
		ldr		r3,	[r0, r2]

		bfi		r3,	r1,	#16, #8
		str		r3,	[r0, r2]

		ldr		r0,	=SPIMode
		mov		r1,	#fRead11Byte
		str		r1,	[r0]

		b		exit_intr_spi5


read_11byte:							//reading MSB of the 2nd 2word
		ldr		r0,	=DataPos
		ldr		r2,	[r0]
		ldr		r0,	=DataBuf
		ldr		r3,	[r0, r2]

		bfi		r3,	r1,	#24, #8
		str		r3,	[r0, r2]

		ldr		r0,	=DataPos			// Next buffer position
		add		r2,	#4
		mov		r1,	#MaxBufSz			// If the max. value reached,
		eor		r1,	#0xFFFFFFFF
		ands	r2,	r1					//  set to 0
		str		r2,	[r0]

		bne		u10

CheckTail:								// if the queue is wrapped to 0


		orr		r2,	r2
u10:
		ldr		r0,	=SPIMode
		mov		r1,	#fRead00Byte
		str		r1,	[r0]

		b		exit_intr_spi5


cmd_written:
		ldr		r0,	=SPIMode
		eor		r2,	r2
		str		r2,	[r0]				// SPIMode := 0;


exit_intr_spi5:


		pop		{r0, r1, r2, r3, pc}


/* ==================================================================== */
MyUSART_Handler:

		push	{r0, r1, r2, r3, r4, lr}

		ldr		r0,	=USART6_BASE				/////////////!!!!!!!!!!!!!!!!!
		ldr		r1,	[r0, #_USART_SR]

		ldr		r0,	=NVIC_BASE
		ldr		r2,	[r0, #_ICPR2]
		orr		r2,	#fUSART6IRQ
		str		r2,	[r0, #_ICPR2]

		ands	r1,	#fRXNE
		bne		recvd


		ldr		r0,	=USART6_BASE
		ldr		r1,	[r0, #_USART_SR]
		bic		r1,	#fTC
		str		r1,	[r0, #_USART_SR]



		ldr		r2,	=DataPos
		ldr		r2,	[r2]
		ldr		r3,	=SndPos
		ldr		r1,	[r3]

		subs	r2,	r1
		beq		noth_to_send

have_to_send:

		bl		SendNext
		b		exit_intr_usart2


noth_to_send:
		ldr		r0,	=UARTMode
		ldr		r1,	[r0]
		bic		r1,	#fSending
		str		r1,	[r0]

		b		exit_intr_usart2

recvd:
		orr		r1,	r1

exit_intr_usart2:

		pop		{r0, r1, r2, r3, r4, pc}
/* ==================================================================== */

SYSCLK_START:
		push    { lr }
        ldr     r7, =RCC_BASE

        @ Включаем HSE

		ldr  	r1, [r7, _CR]	//В r1 значение по адресу r7+_CR
		mov	 	r2, #fHSION		//В r2 флаг для _CR
		orr 	r1, r2			//Операция ИЛИ для значения по адресу r7+_CR и в r2
		str 	r1, [r7, _CR]	//Новое значение по адресу [r7, _CR]

        @ Ожидаем стабилизации частоты генератора
		mov     r0, 1                @ код ошибки при выходе по timeout
        add     r6,  r7, _CR      @ регистр для проверки
        mov     r2, #fHSIRDY   @ бит для проверки
        bl      TST_BIT

        @ Включаем POWER control
        ldr     r1, [r7, #_APB1ENR]
        orr     r1,  r1, #fPWREN
        str     r1, [r7, #_APB1ENR]


        @ Вн. регулятор в режим "нагрузкa" (выходим из энергосбережения)
		LDR     R1, =(PERIPH_BASE + _PWR_BASE + _PWR_CR1)
        LDR     R2, [R1]
        ORR     R2, R2, mPWR_CR_VOS
        STR     R2, [R1]					/**/

	@ Установим делители шин						@@@ Может потребоваться коррекция!!!
                ldr     r1, [r7, _CFGR]             @ делитель шины AHB
                orr     r1,  r1, #mHPRE			    @ HCLK=SYSCLK
                str     r1, [r7, _CFGR]

                ldr     r1, [r7, _CFGR]             @ делитель шины APB2
                orr     r1,  r1, #mPPRE2			@ PCLK2=HCLK / 2
                str     r1, [r7, _CFGR]

                ldr     r1, [r7, _CFGR]             @ делитель шины APB1
                orr     r1,  r1, #mPPRE1			@ PCLK1=HCLK / 4
                str     r1, [r7, _CFGR]

        @ Настройка PLL коэффициентами PLL_M, PLL_N, PLL_Q, PLL_P
               @@@ LDR     R1, =RCC_PLLCFGR_val           @ расчитанное значение

               eor		r1,	r1
               mov		r2,	#valPLLM
               bfi		r1, r2, #0, #6

               mov		r2,	#valPLLN
               bfi		r1, r2, #6, #9

               mov		r2,	#valPLLP
               bfi		r1, r2, #16, #2

               mov		r2, #valPLLQ
               bfi		r1, r2, #24, #4

               str     r1, [r7, _PLLCFGR]

        @ Включаем питание PLL
                ldr     r1, [r7, _CR]
                orr     r1, r1, #fPLLON
                str     r1, [r7, _CR]

        @ Ожидаем готовности PLL
                add     r0, r0, 1
                mov     r2, #fPLLRDY
                bl      TST_BIT

        @ Настройка Flash prefetch, instruction cache, data cache и wait state
                ldr     r2, =(FLASH_R_BASE + _ACR)			@@@ Может потребоваться коррекция!!!
                ldr     r1, [r2]
                ldr     r1, =(fICEN + fDCEN + mLATENCY + fPRFTEN)
                str     r1, [r2]

        @ Выбираем PLL источником такта
                ldr     r1, [r7, _CFGR]


                mov		r2,	#0b10				@@ PLL is set as the clock source
                bfi		r1, r2, #0, #2

                str     R1, [R7, _CFGR]

        @ Ожидаем переключения на PLL
                add     r0, r0, 1
                add     r6, r7, _CFGR
                @ LDR     R2, =RCC_CFGR_SWS_PLL
                mov		r2, #0b1000
				bl      TST_BIT

                mov     r0, 0	         @ признак успешности выполнения
                b       exit

@ Подпрограмма проверки готовности: ------------------------------------------
@     R0 - статус на выход
@     R1 - адрес для чтения
@     R2 - бит-карта для сравнения
@     R3 портится !
@     R4 портится !
TST_BIT:
                /* ADD     R3, R0, R0, lsl  timeout     @ значение timeout	*/

                mov32	r3,	#1000000
TST_ready:
        @ проверка на таймаут
                SUBS    R3, R3, 1
                BEQ     exit                         @ timeout истек, выходим !

        @ проверка готовности HSE
                LDR     R4, [R6, 0]
                TST     R4,  R2
                BEQ     TST_ready
                BX      LR

        @ выход из процедуры
 exit:
                POP     { PC }
/* ================================================================= */


