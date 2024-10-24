#include <msp430f5529.h>
#include "ssd1306.h"
#include "i2c.h"
#include <stdio.h>

#define DIP_SWITCH_VALUE (*((volatile unsigned char*) 0x0200)) // Example address for dip-switch input

void pwm_init(void);
void adc_init(void);
void rc_filter_init(void);
void timer_init(void);
void adc_start_conversion(void);
void adc_interrupt_enable(void);
void oled_display_init(void);
int main(void);

// PWM Initialization
void pwm_init(void)
{
    P2DIR |= BIT0;                // Set P2.0 as output
    P2SEL |= BIT0;                // Select Timer A1 output for P2.0
    TA1CCR0 = 4095;               // Set max count value (for 1 kHz PWM)
    TA1CCTL1 = OUTMOD_7;          // Set output mode to Reset/Set
    TA1CCR1 = TA1CCR0 / 2;        // Set initial duty cycle to 50%
    TA1CTL = TASSEL_2 | MC_1 | TACLR; // SMCLK, Up mode, Clear timer
}

// ADC Initialization
void adc_init(void)
{
    ADC12CTL0 &= ~ADC12ENC;           // Disable ADC before configuration
    ADC12CTL0 = ADC12SHT0_2 | ADC12ON; // Sample-and-hold time, ADC on
    ADC12CTL1 = ADC12SHP;              // Use sampling timer
    ADC12MCTL0 = ADC12INCH_12;         // Input channel A12 (P7.0)
    P7SEL |= BIT0;                     // Configure P7.0 for ADC input
    ADC12CTL0 |= ADC12ENC;             // Enable ADC
}

// Enable ADC Interrupt
void adc_interrupt_enable(void)
{
    ADC12IE = ADC12IE0;               // Enable interrupt for ADC12MEM0
}

// RC Filter Initialization
void rc_filter_init(void)
{
    // Configure RC filter output on P2.1
    P2DIR |= BIT1;                // Set P2.1 as output for filtered PWM signal
    P2SEL &= ~BIT1;               // Ensure P2.1 is in GPIO mode for analog filtering
}

// Timer A0 Initialization for Sampling Rate Control
void timer_init(void)
{
    TA0CCR0 = 3999;                // Set Timer A0 period for 1 ms interrupts (assuming SMCLK at 4 MHz)
    TA0CCTL0 = CCIE;               // Enable interrupt for Timer A0 CCR0
    TA0CTL = TASSEL_2 | MC_1 | TACLR; // SMCLK, Up mode, Clear timer
}

// OLED Display Initialization
void oled_display_init(void)
{
    ssd1306_init();                // Initialize SSD1306 OLED display
    ssd1306_clearDisplay();        // Clear the display
    // Removed call to ssd1306_display() as it's not defined in the provided library             // Update display with cleared content
}

// Start ADC Conversion
void adc_start_conversion(void)
{
    ADC12CTL0 |= ADC12SC;          // Start ADC Conversion
}

// Main Program
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;          // Stop watchdog timer
    pwm_init();                        // Initialize PWM
    adc_init();                        // Initialize ADC
    rc_filter_init();                  // Initialize RC filter
    timer_init();                      // Initialize Timer A0 for sampling rate control
    adc_interrupt_enable();            // Enable ADC interrupt
    oled_display_init();               // Initialize OLED display

    unsigned int adc_value;
    unsigned int duty_cycle;

    __enable_interrupt();              // Enable global interrupts

    while (1)
    {
        // Calculate new duty cycle based on dip-switch value
        duty_cycle = DIP_SWITCH_VALUE * 16; // Scale 8-bit dip-switch value to 12-bit range
        TA1CCR1 = duty_cycle;           // Set PWM duty cycle

        // Display duty cycle on OLED
        ssd1306_setPosition(0, 0);     // Set cursor position
        ssd1306_printUI32(0, 0, TA1CCR1, HCENTERUL_OFF);
        __delay_cycles(40000);          // Delay for stability

        // Measure and display filtered output
        P2OUT ^= BIT1;                  // Toggle filtered output on P2.1 (for oscilloscope measurement)
    }
} 

// Timer A0 Interrupt Service Routine for Sampling Control
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A(void)
{
    adc_start_conversion();              // Start ADC Conversion
}

// ADC Interrupt Service Routine for Reading ADC Value
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)
{
    if (ADC12IFG & ADC12IFG0)
    {
        unsigned int adc_value = ADC12MEM0; // Get ADC result
        ADC12IFG &= ~ADC12IFG0;             // Clear interrupt flag

        // Use ADC value to control PWM duty cycle dynamically
        TA1CCR1 = adc_value;                // Set PWM duty cycle based on ADC value

        // Update OLED display
        ssd1306_setPosition(0, 1);         // Set cursor position
        ssd1306_printUI32(0, 1, adc_value, HCENTERUL_OFF);
    }
}