// multimod_uart.c
// Date Created: 2023-07-25
// Date Updated: 2023-07-27
// Defines for UART functions

/************************************Includes***************************************/

#include "../multimod_uart.h"


#include <stdbool.h>

#include <inc/tm4c123gh6pm.h>
#include <inc/hw_memmap.h>
#include <inc/hw_gpio.h>

#include <driverlib/uartstdio.h>
#include <driverlib/gpio.h>
#include <driverlib/uart.h>
#include <driverlib/sysctl.h>
#include <driverlib/pin_map.h>

/************************************Includes***************************************/

/********************************Public Functions***********************************/

// UART_Init
// Initializes UART serial communication with PC
// Return: void
void UART_Init() {
    /*
    // Enable port A
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    // Enable UART0 module
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Configure UART0 pins on port A

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTFIFODisable(UART0_BASE);

    // Set UART clock source
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_SYSTEM);
    // Configure UART baud rate

    UARTStdioConfig(0, 115200, SysCtlClockGet());
    */

    // This should have been done in lab 0, so it's just copy & paste.
        // Enable port A
        SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;

        // Enable UART0 module
        SYSCTL_RCGCUART_R |= 0x00000001;


        // Configure UART0 pins on port A
        GPIO_PORTA_DIR_R |= ~0x00000001; // Set PA0 as input
        GPIO_PORTA_DIR_R |= 0x00000002; // Set PA2 as output
        GPIO_PORTA_AMSEL_R = 0x0;
        GPIO_PORTA_AFSEL_R |= 0x00000003; // select alternate function for PA0 and PA1
        GPIO_PORTA_PCTL_R |= 0x00000011; // configure PA0 and PA1 for UART
        GPIO_PORTA_DEN_R |= 0x00000003; // digital enable PA0 AND PA1


        // Set UART clock source


        // Configure UART baud rate

        UARTStdioConfig(0, 115200, SysCtlClockGet());
}

/********************************Public Functions***********************************/

