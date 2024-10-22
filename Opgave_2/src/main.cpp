#include <msp430f5529.h>
#include "ssd1306.h"
#include "i2c.h"
#include <stdio.h>
#include <pins_energia.h>

// Globale variabler
volatile unsigned char t_flag = 0;
volatile unsigned char button_flag = 0;
volatile unsigned char ss = 0;
volatile unsigned char mm = 0;
volatile unsigned char hh = 0;
volatile unsigned char set_time_stage = 0;
volatile unsigned char prev_dip_value = 0;
char tiden[32];  // Forøget buffer størrelse for længere tekst
char blank_line[17] = "                ";  // 16 mellemrum + null terminator

// Function prototypes
void reset_display(void);
void update_time_string(void);
unsigned char read_dip_switch(void);
void set_time(void);
void clear_display_line(unsigned char line);
void display_setting_instructions(void);

#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A(void)
{
    static unsigned int update_counter = 0;
    
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
    else 
    {
        // Blink effekt under indstilling
        update_counter++;
        if (update_counter >= 5) {  // Omkring 0.5 sekund
            t_flag = 1;
            update_counter = 0;
        }
    }
    __bic_SR_register_on_exit(LPM0_bits);
}

#pragma vector = PORT2_VECTOR
__interrupt void Port_2(void)
{
    if (P2IFG & BIT1)
    {
        __delay_cycles(100000);
        if (!(P2IN & BIT1))
        {
            button_flag = 1;
            if (set_time_stage == 0) {
                set_time_stage = 1;  // Start indstilling
                display_setting_instructions();
            }
            else if (set_time_stage == 3) {
                set_time_stage = 0;  // Afslut indstilling
                ssd1306_clearDisplay();
            }
            else {
                set_time_stage++;    // Næste indstilling
                display_setting_instructions();
            }
        }
        P2IFG &= ~BIT1;
    }
    __bic_SR_register_on_exit(LPM0_bits);
}

void clear_display_line(unsigned char line)
{
    ssd1306_setPosition(0, line);
    ssd1306_printText(0, line, blank_line);
}

void display_setting_instructions(void)
{
    ssd1306_clearDisplay();
    switch(set_time_stage) {
        case 1:
            ssd1306_printText(0, 0, "Indstil Timer:");
            break;
        case 2:
            ssd1306_printText(0, 0, "Indstil Minutter:");
            break;
        case 3:
            ssd1306_printText(0, 0, "Indstil Sekunder:");
            break;
    }
}

void update_time_string(void)
{
    if (set_time_stage == 0) {
        sprintf(tiden, "%02d:%02d:%02d", hh, mm, ss);
        ssd1306_printText(0, 0, tiden);
    }
    else {
        // Under indstilling, vis den aktuelle værdi der ændres
        unsigned char current_value;
        switch(set_time_stage) {
            case 1: current_value = hh; break;
            case 2: current_value = mm; break;
            case 3: current_value = ss; break;
            default: return;
        }
        sprintf(tiden, "Vaerdi: %02d", current_value);
        ssd1306_printText(0, 2, tiden);
    }
}
12
unsigned char read_dip_switch(void)
{
    unsigned char value = 0;
    value = P6IN & 0x7F;                   // Read P6.0 to P6.6
    value |= (P7IN & BIT0) ? 0x80 : 0x00;  // Read P7.0
    return ~value;                          // Invert if DIP switches are active low
}

void set_time(void)
{
    unsigned char dip_value = read_dip_switch();
    
    // Kun opdater hvis DIP switch værdien er ændret
    if (dip_value != prev_dip_value) {
        prev_dip_value = dip_value;
        
        switch (set_time_stage)
        {
            case 1:
                hh = dip_value % 24;
                break;
            case 2:
                mm = dip_value % 60;
                break;
            case 3:
                ss = dip_value % 60;
                break;
        }
        t_flag = 1;  // Tving en display opdatering
    }
}

void reset_display(void)
{
    ssd1306_clearDisplay();
    update_time_string();
}

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
        if (set_time_stage > 0) {
            set_time();  // Tjek konstant for DIP switch ændringer i indstillingstilstand
        }
        
        if (t_flag)
        {
            t_flag = 0;
            update_time_string();
        }
        
        if (button_flag)
        {
            button_flag = 0;
        }
        
        __bis_SR_register(LPM0_bits + GIE);
    }
    return 0;
}