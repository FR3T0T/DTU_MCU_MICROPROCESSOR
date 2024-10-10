#include <Arduino.h>
#include <stdint.h>
#include "i2c.h"
#include "ssd1306.h"

// Time variables
volatile uint8_t seconds = 0;
volatile uint8_t minutes = 0;
volatile uint8_t hours = 0;

// Flag to indicate when to update the display
volatile bool update_display = false;

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
    
    // Update time
    seconds++;
    if (seconds >= 60) {
        seconds = 0;
        minutes++;
        if (minutes >= 60) {
            minutes = 0;
            hours++;
            if (hours >= 24) {
                hours = 0;
            }
        }
    }
    
    // Set flag to update display
    update_display = true;
}

// Function to update the display without clearing it entirely
void updateDisplay(const char* line1 = NULL, const char* line2 = NULL, const char* line3 = NULL)
{
    if (line1 != NULL)
    {
        ssd1306_printText(0, 0, "                    "); // Clear line 1
        ssd1306_printText(0, 0, line1);
    }
    if (line2 != NULL)
    {
        ssd1306_printText(0, 1, "                    "); // Clear line 2
        ssd1306_printText(0, 1, line2);
    }
    if (line3 != NULL)
    {
        ssd1306_printText(0, 2, "                    "); // Clear line 3
        ssd1306_printText(0, 2, line3);
    }
    //ssd1306_updateScreen(); // Update the entire screen
}

void updateClockDisplay()
{
    char timeStr[9];
    char dateStr[11] = "2024-10-10"; // Example date, you can update this as needed
    
    snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d", hours, minutes, seconds);
    
    updateDisplay("MSP430 Clock", timeStr, dateStr);
}

int main(void) {
    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer

    // Initialize I2C
    i2c_init();

    // Initialize OLED display
    ssd1306_init();
    ssd1306_clearDisplay();
    updateDisplay("MSP430 Clock", "Initializing...");
    
    // Short delay to show initial message
    __delay_cycles(1000000);

    // Initialize TimerA0
    initTimer();

    // Set P1.0 as output for LED
    P1DIR |= BIT0;
    P1OUT &= ~BIT0; // Ensure LED is off

    while (1)
    {
        // Main loop
        if (update_display) {
            updateClockDisplay();
            update_display = false;
        }
        
        // You can add power-saving code here if needed
        // __bis_SR_register(LPM0_bits + GIE);
    }

    return 0;
}