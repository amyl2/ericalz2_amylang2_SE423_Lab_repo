//#############################################################################
// FILE:   LABstarter_main.c
//
// TITLE:  Lab Starter
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "F28379dSerial.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

float x1 = 6.0;
float x2 = 2.3;
float x3 = 7.3;
float x4 = 7.1;

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
int32_t numTimer2calls = 0;
int16_t returnLED = 0;


// Worker Functions
void SetLEDsOnOff( int16_t LEDvalue );
int16_t ReadSwitches( void );



void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();
	
	// Blue LED on LaunchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

	// Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

	// LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
	
	// LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

	// LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

	// LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

	// LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

	// LS7366#1 CS
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

	// LS7366#2 CS	
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

	// LS7366#3 CS
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

	// LS7366#4 CS
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

	// WIZNET RST
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

	//PushButton 1
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_INPUT, GPIO_PULLUP);

	//SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //F28027 CS
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;
	
    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
	
	//WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;

    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIB_RX_INT = &RXBINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIB_TX_INT = &TXBINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 10000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 20000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 1000);

    // Enable CpuTimer Interrupt bit TIE
//    CpuTimer0Regs.TCR.all = 0x4000;
//    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA,115200);
    init_serialSCIB(&SerialB,19200);
//    init_serialSCIC(&SerialC,115200);
//    init_serialSCID(&SerialD,115200);

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
	// Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
	
    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    
    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
			serial_printf(&SerialA,"NumTimer2 Calls:%ld Num SerialRX: %ld LED binary value:%d\r\n",numTimer2calls,numRXA,returnLED);
            UART_printfLine(1,"Timer2 Calls %ld",CpuTimer2.InterruptCount);
            UART_printfLine(2,"Num SerialRX %ld",numRXA);
            UARTPrint = 0;
        }
    }
}

// Exercise 2: SetLEDsOnOff determines if the LEDs are on or off based on bit values. The function takes in a 16 bit parameter value (LEDvalue),
// which for this code was just iterated every time the Timer 2 interrupt occurred. This allowed it to go through a variety
// of binary numbers to turn the LEDs on and off. The function itself decides whether to turn the LED on or off based on the
// value of the bits in this parameter value, particularly the 5 least significant bits. This is done by using a collection
// of if statements to investigate the bits of LEDvalue. For example, to determine if the first LED should be on or off, the
// function investigates bit 0, the least significant bit. This is done by using an AND operator between LEDvalue and 0x1,
// 0001 in binary terms, which selects bit 0. If this results in the value being the same as 0x1 itself, bit 0 must be 1 and
// is therefore high, meaning that the LED should be on. This is completed similarly for LEDs 2, 3, 4, and 5. If the value is
// not equal to itself (!=) then the bit is 0, and the LED should be off.
void SetLEDsOnOff( int16_t LEDvalue ){
    // If bit 0 is high, turn LED 1 on
    if ((LEDvalue & 0x1) == 0x1){
        GpioDataRegs.GPASET.bit.GPIO22 = 1;
    }
    // If bit 1 is high, turn LED 2 on
    if ((LEDvalue & 0x2) == 0x2){
        GpioDataRegs.GPCSET.bit.GPIO94 = 1;
    }
    // If bit 2 is high, turn LED 3 on
    if ((LEDvalue & 0x4) == 0x4 ){
        GpioDataRegs.GPCSET.bit.GPIO95 = 1;
    }
    // If bit 3 is high, turn LED 4 on
    if ((LEDvalue & 0x8) == 0x8 ){
        GpioDataRegs.GPDSET.bit.GPIO97 = 1;
    }

    // If bit 4 is high, turn LED 5 on
    if ((LEDvalue & 0x10) == 0x10 ){
        GpioDataRegs.GPDSET.bit.GPIO111 = 1;
    }

    // If bit 0 is low, turn LED 1 off
    if ((LEDvalue & 0x1) != 0x1){
        GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
    }
    // If bit 1 is low, turn LED 2 off
    if ((LEDvalue & 0x2) != 0x2){
        GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;
    }
    // If bit 2 is low, turn LED 3 off
    if ((LEDvalue & 0x4) != 0x4 ){
        GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;
    }
    // If bit 3 is low, turn LED 4 off
    if ((LEDvalue & 0x8) != 0x8 ){
        GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;
    }

    // If bit 4 is low, turn LED 5 off
    if ((LEDvalue & 0x10) != 0x10 ){
        GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;
    }
}


// Exercise 2: ReadSwitches indicates the state of the four push buttons based on a returned 16 bit integer with the 4 least significant
// bits. This is done by checking the status of each of the four push buttons using an if statement. Note that, as each push
// button has a pull up resistor, GP?DAT is 0 when the button is pushed. So, if the button is pushed we want to turn on the
// LED. To do this, we made a local variable buttonReader that is originally set to 0. Every time a button is pressed, the
// function sets the corresponding bit to one using an OR operator. The function then returns buttonReader, which will be
// a binary that can be read later in the code to turn the LEDs on whenever the button is pressed.
int16_t ReadSwitches( void ) {
    int16_t buttonReader = 0;
    if ((GpioDataRegs.GPEDAT.bit.GPIO157 == 0) ) {
        buttonReader = buttonReader | 0x1;
    }
    if ((GpioDataRegs.GPEDAT.bit.GPIO158 == 0) ) {
        buttonReader = buttonReader | 0x2;
    }
    if ((GpioDataRegs.GPEDAT.bit.GPIO159 == 0) ) {
        buttonReader = buttonReader | 0x4;
    }
    if ((GpioDataRegs.GPFDAT.bit.GPIO160 == 0) ) {
        buttonReader = buttonReader | 0x8;
    }

    return buttonReader;
}

// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
	// making it lower priority than all other Hardware interrupts.  
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts
	
	
	
    // Insert SWI ISR Code here.......
	
	
    numSWIcalls++;
    
    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;

//    if ((numTimer0calls%50) == 0) {
//        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
//    }

    if ((numTimer0calls%5) == 0) {
		// Blink LaunchPad Red LED
		GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    }


    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
		
    CpuTimer1.InterruptCount++;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
    // Iterate numTimer 2 calls by 1 every time the function is entered
	numTimer2calls++;


// Exercise 2
//	SetLEDsOnOff(numTimer2calls); // Turn the LEDs on or off based on the binary number passed. See SetLEDsOnOff description for more detail

//	returnLED = ReadSwitches(); // With the previous line commented out, check whether the switches are pressed or not to get the binary value
//
//	SetLEDsOnOff(returnLED); // Use the previously assigned returnLED value to turn the LEDs on or off


	// Exercise 2: Every time the function is entered, set UARTPrint equal to 1
//	UARTPrint = 1;

	// Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;

    // Exercise 4: nonsense functions to test the use of breakpoints. F5 can be used to proceed step by step. This can
    // be seen by the fact that x1 will not be calculated if the code was set to a breakpoint at x3 until it is told to
    // proceed.
    x4 = x3 + 2.0;
    x3 = x4 +1.3;
    x1 = 9*x2;
    x2 = 34*x3;

//	Exercise 5: When Timer2 period is 1ms, only print every 100th time the timer interrupt occurs. 1ms is a very rapid
    // period, so it would be unwise to print or run certain pieces of code at such a rapid rate. As such, we can slow
    // it down using a mod 100 to make the code run every 100th time the interrupt occurs rather than every single time.
	if ((CpuTimer2.InterruptCount % 100) == 0) {
		UARTPrint = 1;
		// Want these to occur at a slower rate
	    returnLED = ReadSwitches();

	    SetLEDsOnOff(returnLED);

	}


}

