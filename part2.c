// Philip Chan and Thien Nguyen
// Lab 5
//      PA5 - SSI0Tx to SI
//      PA4 - SSI0Rx to 
//      PA3 - SSI0Fss to CS
//      PA2 - SSI0CLK to CL
//			PA6 - DC
//			PA7 - reset
//			PB2 - IR output



#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"
#include "driverlib/fpu.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "utils/uartstdio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/ssi.h"
#include "math.h"

// xxxa_bxcd   a = power down, b = update both DAC registers
//             c = load A register, d = load B register
#define LOAD_DACA_INPUT 0x0100
#define LOAD_DACB_INPUT 0x0200
#define LOAD_DAC_INPUTS 0x0300
#define UPDATE_DAC_REG  0x0800
#define UPDATE_LOAD_A   0x0900
#define UPDATE_LOAD_B   0x0A00
#define UPDATE_LOAD     0x0B00

#define pi	3.14159

//*****************************************************************************
//
//! \addtogroup adc_examples_list
//! <h1>Single Ended ADC (single_ended)</h1>
//!
//! This example shows how to setup ADC0 as a single ended input and take a
//! single sample on AIN0/PE3.
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - ADC0 peripheral
//! - GPIO Port E peripheral (for AIN0 pin)
//! - AIN0 - PE3
//!
//! The following UART signals are configured only for displaying console
//! messages for this example.  These are not required for operation of the
//! ADC.
//! - UART0 peripheral
//! - GPIO Port A peripheral (for UART0 pins)
//! - UART0RX - PA0
//! - UART0TX - PA1
//!
//! This example uses the following interrupt handlers.  To use this example
//! in your own application you must add these interrupt handlers to your
//! vector table.
//! - SS3IntHandler.
//
//*****************************************************************************

//
// This array is used for storing the data read from the ADC FIFO. It
// must be as large as the FIFO for the sequencer in use.  This example
// uses sequence 3 which has a FIFO depth of 1.  If another sequence
// was used with a deeper FIFO, then the array size must be changed.
//

uint32_t pui32ADC0Value[1];
int ADCValueArray[128];
int sampled = 0;
int buttonPushed = 0;
int ready = 0;	// collected 128 samples?
float sineBuffer[256];
static int sineIndex = 0;


void Switch_Handler (void);
void Timer0A_Int(void);
void writeData(uint16_t c);
void loadDACA(uint8_t c);
void loadDACB(uint8_t c);


void SS3IntHandler (void) {

	//
	// Clear the ADC interrupt flag.
	//
	ADCIntClear(ADC0_BASE, 3);
	uint16_t movingAverage = 0;
	uint16_t comp = 0xFFFF;
  //
  // Read ADC Value.
  //
  ADCSequenceDataGet(ADC0_BASE, 3, pui32ADC0Value);
	ADCValueArray[sampled] = pui32ADC0Value[0];
	if (ready)
	{
		switch(sampled)
		{
			case 0:
				movingAverage = (ADCValueArray[0] + ADCValueArray[127] + ADCValueArray[126] + ADCValueArray[125])/4;
				break;
			case 1:
				movingAverage = (ADCValueArray[1] + ADCValueArray[0] + ADCValueArray[127] + ADCValueArray[126])/4;
				break;
			case 2:
				movingAverage = (ADCValueArray[2] + ADCValueArray[1] + ADCValueArray[0] + ADCValueArray[127])/4;
				break;
			default:
				movingAverage = (ADCValueArray[sampled] + ADCValueArray[sampled-1] + 
						ADCValueArray[sampled-2] + ADCValueArray[sampled-3])/4;
		}
		comp = 0xFFFF - movingAverage;
	}
	
	// Add output stuff here
	loadDACA(movingAverage);
	loadDACB(comp);
	
	sampled++;
	if (sampled == 128)
	{
		sampled = 0;
		ready = 1;
	}
}

//*****************************************************************************
//
// This function sets up UART0 to be used for a console to display information
// as the example is running.
//
//*****************************************************************************

void ConfigureUART0(void) {
    //
    // Enable the GPIO Peripheral used by the UART0 and enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);  
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize UART0 for console I/O. (See uartstdio.c)
    //
		UARTStdioConfig(0, 9600, 16000000);
}

//*****************************************************************************
//
// Configure ADC0 for a single-ended input and a single sample.  Once the
// sample is ready, an interrupt flag will be set.  Using a polling method,
// the data will be read then displayed on the console via UART0.
//
//*****************************************************************************

// ============================================================================
// Main
// ============================================================================

int main(void)
{
		ConfigureUART0();
		ROM_FPULazyStackingEnable();
		int loopControl = 0;
		int printingArray[8] = {0,0,0,0,0,0,0,0};
		int printingCounter = 0;
		int printRow = 0;
		int val = 0;
		for (int i = 0; i < 256; i++)
		{
			sineBuffer[i] = sin((i*2*pi)/256)*128+127;

		}
		
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);

    //
    // Set up the serial console to use for displaying messages.  This is
    // just for this example program and is not needed for ADC operation.
    //
		
		

    //
    // Display the setup on the console.
    //
    UARTprintf("\r\nADC Example Program->\n");
    UARTprintf("  Type: Single Ended\n");
    UARTprintf("  Samples: One\n");
    UARTprintf("  Input Pin: AIN0/PE3\n\n");
	
	// Configure switches
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
		HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
		ROM_GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);
		ROM_GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);
		ROM_GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, GPIO_FALLING_EDGE);
		GPIOIntEnable(GPIO_PORTF_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_4);
		
		ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,GPIO_PIN_2);
		
		// Configure SSI0
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
		GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
		ROM_GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 |
                   GPIO_PIN_2);
		ROM_SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                       SSI_MODE_MASTER, 1000000, 8);
		ROM_SSIEnable(SSI0_BASE);
		
		// Configure Timer0	
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);	
		ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
		ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet()/10);
		
		ROM_IntEnable(INT_TIMER0A);
		TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
		
		ROM_IntEnable(INT_GPIOF);
		ROM_IntMasterEnable();
		ROM_TimerEnable(TIMER0_BASE, TIMER_A);

		UARTprintf("starting\n");
// ============================================================================
// while
// ============================================================================

    while(1)
    {
				ROM_SysCtlSleep();
        //SysCtlDelay(SysCtlClockGet() / 30);
							
        //
        // Display the AIN0 (PE3) digital value on the console.
        //

			if (buttonPushed)
			{
				if(ready)
				{
					loopControl = sampled;
					UARTprintf("New stream....\n");
					for (int i = loopControl ; i < 128; i++)
					{
						printingArray[printingCounter] = ADCValueArray[i];
						printingCounter++;
						if (printingCounter == 8)
						{
							printRow++;
							UARTprintf("Row %2d: %5d %5d %5d %5d %5d %5d %5d %5d\n",printRow, printingArray[0],
											printingArray[1], printingArray[2],printingArray[3],printingArray[4],
											printingArray[5], printingArray[6],printingArray[7]);
							printingCounter = 0;
						}
					}
					for (int i = 0; i < loopControl; i++)
					{
						printingArray[printingCounter] = ADCValueArray[i];
						printingCounter++;
						if (printingCounter == 8)
						{
							printRow++;
							UARTprintf("Row %2d: %5d %5d %5d %5d %5d %5d %5d %5d\n",printRow, printingArray[0],
											printingArray[1], printingArray[2],printingArray[3],printingArray[4],
											printingArray[5], printingArray[6],printingArray[7]);
							printingCounter = 0;
						}
					}
					printRow = 0;
				}
				else
					UARTprintf("Not enough samples yet.\n");
				buttonPushed = 0;
			}
			GPIOIntClear(GPIO_PORTF_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_4);
			GPIOIntEnable(GPIO_PORTF_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_4);
    }
}


// ================================================================================
// Functions
// ================================================================================

void Timer0A_Int(void)
{
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	uint8_t dac_out = sineBuffer[sineIndex];
	loadDACA(dac_out);
	sineIndex++;
	if (sineIndex >255)
		sineIndex = 0;
}

void Switch_Handler (void) {
	// Disable interrupt for awhile to avoid switch bounce
	GPIOIntDisable(GPIO_PORTF_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_4);

	// Clear interrupt request
	GPIOIntClear(GPIO_PORTF_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_4);
	
	// Toggle LEDs	
	buttonPushed = 1;
}	

void writeCommand(uint8_t c) {
	//wait until TxFIFO empty
	// set D/C low
	// send 8-bit command
	// wait until TxFIFO and transmit mode

	while(SSIBusy(SSI0_BASE))
    {
    }

	ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0);

	SSIDataPut(SSI0_BASE, c);

	while(SSIBusy(SSI0_BASE))
    {
    }
	
}


void writeData(uint16_t c) {
	
	//set D/C output high
	// sent 8-bit data using blocking put function
	while(SSIBusy(SSI0_BASE))
    {
    }

	ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);
	SSIDataPut(SSI0_BASE, c);

	while(SSIBusy(SSI0_BASE))
    {
    }


} 


void loadDACA(uint8_t c)
{
	uint16_t data;
	data = UPDATE_LOAD_A + c;
	writeData(data);
}
void loadDACB(uint8_t c)
{
	uint16_t data;
	data = UPDATE_LOAD_B + c;
	writeData(data);
}
