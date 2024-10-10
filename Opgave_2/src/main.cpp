// Libaries
#include <Arduino.h>
#include <stdint.h>
#include "i2c.h"
#include "ssd1306.h"


void initTimer()
{
    // Set TimerA0 to use SMCLK, divide by 64, and count up mode
    TA0CTL = TASSEL_2 | ID_3 | MC_1;
    // Set the compare value for 1 second interval
    TA0CCR0 = 51089; // 1.048 MHz / 64 = 16384
    // Enable interrupt for compare match
    TA0CCTL0 = CCIE;
    // Enable global interrupts
    __bis_SR_register(GIE);
}

#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A(void)
{
    // Toggle LED on P1.0
    P1OUT ^= BIT0;
    // Set a flag or update time variables here if needed
}

int main(void) {
    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer


   

    // Initialize TimerA0
    initTimer();

    // Set P1.0 as output for LED
    P1DIR |= BIT0;
    P1OUT &= ~BIT0; // Ensure LED is off

    while (1)
    {
        // Main loop
        // Update display with time if needed
        // Example: ssd1306_printText(0, 0, "Time: 00:00:00");
    }

    return 0;
}