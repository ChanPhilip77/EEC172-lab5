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




void Switch_Handler (void);
void Timer0A_Int(void);
void SS3IntHandler (void) {

	//
	// Clear the ADC interrupt flag.
	//
	ADCIntClear(ADC0_BASE, 3);

  //
  // Read ADC Value.
  //
  ADCSequenceDataGet(ADC0_BASE, 3, pui32ADC0Value);
	ADCValueArray[sampled] = pui32ADC0Value[0];
	sampled++;
	if (sampled == 128)
		sampled = 0;
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
int
main(void)
{

    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);

    //
    // Set up the serial console to use for displaying messages.  This is
    // just for this example program and is not needed for ADC operation.
    //
		ConfigureUART0();
		ROM_FPULazyStackingEnable();

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
		

		// Configure Timer0	
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);	

    //
    // The ADC0 peripheral must be enabled for use.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    //
    // For this example ADC0 is used with AIN0 on port E3.
		// GPIO port E needs to be enabled so these pins can be used.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    //
    // Select the analog ADC function for these pins.
    // Consult the data sheet to see which functions are allocated per pin.
    //
    ROM_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

    //
    // Enable sample sequence 3 with a processor signal trigger.  Sequence 3
    // will do a single sample when the processor sends a signal to start the
    // conversion.  Each ADC module has 4 programmable sequences, sequence 0
    // to sequence 3.  This example is arbitrarily using sequence 3.
    //
    ROM_ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_TIMER, 0);

    //
    // Configure step 0 on sequence 3.  Sample channel 0 (ADC_CTL_CH0) in
    // single-ended mode (default) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
    // that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
    // 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
    // sequence 0 has 8 programmable steps.  Since we are only doing a single
    // conversion using sequence 3 we will only configure step 0.  For more
    // information on the ADC sequences and steps, reference the datasheet.
    //
    ROM_ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE |
                             ADC_CTL_END);

    //
    // Since sample sequence 3 is now configured, it must be enabled.
    //
    ROM_ADCSequenceEnable(ADC0_BASE, 3);

    //
    // Clear the interrupt status flag.  This is done to make sure the
    // interrupt flag is cleared before we sample.
    //
    ROM_ADCIntClear(ADC0_BASE, 3);

		ROM_IntEnable(INT_GPIOF);

		ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

		ROM_TimerControlTrigger(TIMER0_BASE, TIMER_A, true);
		ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet()/10);
		ROM_TimerEnable(TIMER0_BASE, TIMER_A);
		
		ROM_ADCIntEnable(ADC0_BASE, 3);
		ROM_IntEnable(INT_ADC0SS3);
		ROM_IntMasterEnable();
		UARTprintf("starting\n");
    while(1)
    {

        SysCtlDelay(SysCtlClockGet() / 30);
							
        //
        // Display the AIN0 (PE3) digital value on the console.
        //

			if (buttonPushed)
			{
				
					UARTprintf("New stream....\n");
					for (int i = 0; i < 128; i++)
						UARTprintf("AIN0 = %4d\n", ADCValueArray[i]);
				buttonPushed = 0;
			}
			GPIOIntClear(GPIO_PORTF_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_4);
			GPIOIntEnable(GPIO_PORTF_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_4);
    }
}

void Timer0A_Int(void)
{
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	
}

void Switch_Handler (void) {
	int leds;

	// Disable interrupt for awhile to avoid switch bounce
	GPIOIntDisable(GPIO_PORTF_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_4);

	// Clear interrupt request
	GPIOIntClear(GPIO_PORTF_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_4);
	
	// Toggle LEDs	
	buttonPushed = 1;
}	
