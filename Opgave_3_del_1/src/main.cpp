#include <Arduino.h>
#include <Energia.h>
#include "i2c.h"
#include "ssd1306.h"

void init_ports()
{
    P6SEL = 0x00;
    P7SEL = 0;
  
    pinMode(P1_1, INPUT_PULLUP);
    digitalWrite(P1_1, HIGH);
    pinMode(P4_7, OUTPUT);
    pinMode(P1_0, OUTPUT);
    pinMode(P2_1, INPUT_PULLUP);
    digitalWrite(P2_1, HIGH);
    P6DIR = 0;
    P6REN |= BIT6 | BIT5 | BIT4 | BIT3 | BIT2 | BIT1 | BIT0;
    P6OUT |= BIT6 | BIT5 | BIT4 | BIT3 | BIT2 | BIT1 | BIT0;
    P7REN |= BIT0;
    P7OUT |= BIT0;
}

void init_SMCLK_XT2()
{
    WDTCTL = WDTPW|WDTHOLD;
    P5SEL |= BIT2+BIT3;
    UCSCTL6 &= ~XT2OFF;
    UCSCTL4 |= SELA_2;
    UCSCTL4 |= SELS_5 + SELM_5;

    do
    {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
        SFRIFG1 &= ~OFIFG;
    }
    while (SFRIFG1&OFIFG);
}

void init_pwm()
{
    // Setup Timer A1 for edge-aligned PWM
    TA1CTL = TASSEL_2 | // SMCLK as clock source
             MC_1 |     // Up mode
             TACLR;     // Clear timer

    TA1CCR0 = 4095;    // PWM Period (Top value)
    TA1CCR1 = 2048;    // Initial 50% duty cycle

    // Setup PWM output mode
    TA1CCTL1 = OUTMOD_7;  // Reset/Set mode
    
    // Enable PWM output on P2.0
    pinMode(P2_0, OUTPUT);
    P2SEL |= BIT0;
}

void update_duty_cycle(uint8_t dip_value)
{
    // Simply multiply by 16 to map 8-bit to 12-bit range
    TA1CCR1 = dip_value << 4;  // Same as dip_value * 16
}

uint8_t calculate_duty_cycle()
{
    return (uint8_t)((((uint32_t)TA1CCR1 * 100) / TA1CCR0));
}

int main()
{
    WDTCTL = WDTPW | WDTHOLD;
    init_SMCLK_XT2();
    init_ports();
    init_pwm();
 
    i2c_init();
    ssd1306_init();
    reset_diplay();
    ssd1306_clearDisplay();
    
    char display_text[32];
    uint8_t duty_cycle;
    uint8_t prev_dip_value = 0;
    uint8_t prev_duty_cycle = 0;

    // Initial display setup
    ssd1306_printText(0, 0, "DIP:");
    ssd1306_printText(0, 2, "Duty:");
    ssd1306_printText(0, 4, "CCR1:");

    while(1)
    {
        // Read DIP switches
        uint8_t dip_value = (P6IN & 0x7F) | ((P7IN & BIT0) << 7);
        
        // Kun opdater hvis værdien har ændret sig
        if(dip_value != prev_dip_value)
        {
            update_duty_cycle(dip_value);
            duty_cycle = calculate_duty_cycle();
            
            // Opdater kun de specifikke værdier der vises
            sprintf(display_text, "%d   ", dip_value);  // Ekstra spaces for at overskrive gamle tal
            ssd1306_printText(40, 0, display_text);
            
            sprintf(display_text, "%d%%   ", duty_cycle);
            ssd1306_printText(40, 2, display_text);
            
            sprintf(display_text, "%d   ", TA1CCR1);
            ssd1306_printText(40, 4, display_text);
            
            prev_dip_value = dip_value;
            prev_duty_cycle = duty_cycle;
        }
        
        // Wait for CCR1 match
        while(!(TA1CCTL1 & CCIFG));
        TA1CCTL1 &= ~CCIFG;  // Clear the flag
        
        __delay_cycles(50000);  // Reduceret delay for bedre respons
    }
}