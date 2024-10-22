#include <msp430f5529.h>
#include "ssd1306.h"
#include "i2c.h"
#include <stdio.h>
#include <pins_energia.h>

// Global variables
volatile unsigned char t_flag = 0;
volatile unsigned char button_flag = 0;
volatile unsigned char ss = 0;
volatile unsigned char mm = 0;
volatile unsigned char hh = 0;
volatile unsigned char set_time_stage = 0;
char tiden[16];

// Function prototypes
void reset_display(void);
void update_time_string(void);
unsigned char read_dip_switch(void);
void set_time(void);
void clear_time_area(void);

// Timer Interrupt Service Routine (ISR)
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A(void)
{
    if (set_time_stage == 0)
    {
        t_flag = 1;
        ss++;
        if (ss == 60)
        {
            ss = 0;
            mm++;
            if (mm == 60)
            {
                mm = 0;
                hh++;
                if (hh == 24)
                {
                    hh = 0;
                }
            }
        }
    }
    __bic_SR_register_on_exit(LPM0_bits);  // Exit low-power mode
}

// Port 2 Interrupt Service Routine (ISR) for Button Press
#pragma vector = PORT2_VECTOR
__interrupt void Port_2(void)
{
    if (P2IFG & BIT1)
    {
        __delay_cycles(100000);  // Increased debouncing delay
        if (!(P2IN & BIT1))      // Confirm button is still pressed
        {
            button_flag = 1;
            set_time_stage++;
            if (set_time_stage > 3)
            {
                set_time_stage = 1;  // Cycle back to 1
            }
        }
        P2IFG &= ~BIT1;  // Clear interrupt flag
    }
    __bic_SR_register_on_exit(LPM0_bits);  // Exit low-power mode
}

// Function to Reset the Display and Show Initial Time
void reset_display(void)
{
    ssd1306_clearDisplay();
    ssd1306_setPosition(0, 0);
    update_time_string();  // Display initial time
}

// Function to Update the Time String on the Display
void update_time_string(void)
{
    sprintf(tiden, "%02d:%02d:%02d", hh, mm, ss);
    ssd1306_setPosition(0, 0);
    ssd1306_printText(0, 0, tiden);
}

// Function to Read the DIP Switch Values
unsigned char read_dip_switch(void)
{
    unsigned char value = 0;
    value = P6IN & 0x7F;                   // Read P6.0 to P6.6
    value |= (P7IN & BIT0) ? 0x80 : 0x00;  // Read P7.0
    return ~value;  // Invert if DIP switches are active low
}

// Function to Set Time Based on DIP Switch and Stage
void set_time(void)
{
    unsigned char dip_value;
    if (set_time_stage == 0)
    {
        return;  // Not in time-setting mode
    }

    dip_value = read_dip_switch();

    switch (set_time_stage)
    {
        case 1:
            hh = dip_value % 24;
            sprintf(tiden, "Set Hour: %02d", hh);
            break;

        case 2:
            mm = dip_value % 60;
            sprintf(tiden, "Set Min: %02d", mm);
            break;

        case 3:
            ss = dip_value % 60;
            sprintf(tiden, "Set Sec: %02d", ss);
            set_time_stage = 0;  // Reset to normal operation
            break;
    }
    ssd1306_setPosition(0, 2);
    ssd1306_printText(0, 2, tiden);
}

// Main Function
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;  // Stop watchdog timer

    // Initialize I2C and OLED display
    i2c_init();
    ssd1306_init();

    // Configure DCO to 1 MHz
    UCSCTL3 = SELREF_2;                      // Set DCO FLL reference = REFO
    UCSCTL4 |= SELA_2;                       // Set ACLK = REFO

    __bis_SR_register(SCG0);                 // Disable FLL control loop
    UCSCTL0 = 0x0000;                        // Set lowest possible DCOx, MODx
    UCSCTL1 = DCORSEL_2;                     // Select DCO range 1MHz operation
    UCSCTL2 = FLLD_1 + 31;                   // Set DCO Multiplier for 1MHz
                                             // (N + 1) * FLLRef = Fdco
                                             // (31 + 1) * 32768 = 1MHz
    __bic_SR_register(SCG0);                 // Enable FLL control loop

    // Allow time for DCO to settle
    __delay_cycles(250000);

    // Timer configuration using SMCLK (~1MHz)
    TA0CTL = TASSEL_2 | ID_3 | MC_1; 
    TA0CCR0 = 16370;  
    TA0CCTL0 = CCIE; 
    TA0EX0 = TAIDEX_7; 

    // Configure P1.0 as output for LED (debugging)
    P1DIR |= BIT0;
    P1OUT &= ~BIT0;

    // Configure switchPin (P2.1) for external interrupt
    P2DIR &= ~BIT1;  // Set P2.1 as input
    P2REN |= BIT1;   // Enable pull-up resistor
    P2OUT |= BIT1;   // Set pull-up resistor
    P2IE |= BIT1;    // Enable interrupt on P2.1
    P2IES |= BIT1;   // Interrupt on falling edge

    // Configure DIP switch pins as inputs with pull-up resistors
    P6DIR &= ~0x7F;  // P6.0 to P6.6 as input
    P7DIR &= ~BIT0;  // P7.0 as input
    P6REN |= 0x7F;
    P7REN |= BIT0;
    P6OUT |= 0x7F;
    P7OUT |= BIT0;

    __bis_SR_register(GIE);  // Enable global interrupts

    reset_display();

    while (1)
    {
        if (button_flag)
        {
            button_flag = 0;
            set_time();
        }
        else if (t_flag && set_time_stage == 0)
        {
            t_flag = 0;
            update_time_string();
        }
        __bis_SR_register(LPM0_bits + GIE);  // Enter low-power mode
    }
    return 0;
}