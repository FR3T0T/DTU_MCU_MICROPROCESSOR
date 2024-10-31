/*******************************************************************************
* DEL 1
* PWM Generator med DIP Switch Kontrol og OLED Display
*
* Dette program implementerer en PWM generator med følgende funktioner:
* - PWM output på P2.0 med frekvens omkring 1 kHz
* - Duty cycle styres via 8-bit DIP switch
* - OLED display viser aktuelle værdier
* - RC lavpasfilter kan tilsluttes for DC konvertering
*
* Hardware krav:
* - MSP430F5529 LaunchPad
* - 8-bit DIP switch på P6.0-P6.6 og P7.0
* - OLED display tilsluttet via I2C
* - RC lavpasfilter (12kΩ og 1µF) kan tilsluttes P2.0
*
* Udviklet til: DTU Engineering Technology ECS 2024
*******************************************************************************/

#include <Arduino.h>
#include <Energia.h>
#include "i2c.h"
#include "ssd1306.h"

/****************************************************************************
* Konstanter og Definitioner
****************************************************************************/
// System Clock og Timer Konstanter
#define SMCLK_FREQ      4000000.0f    // System clock frekvens (4 MHz)
#define TA1CCR0_VALUE   4095          // Timer A1 periode (12-bit opløsning)

// PWM og Filter Beregninger
#define PWM_FREQ        (SMCLK_FREQ / TA1CCR0_VALUE)  // ≈ 976.56 Hz
#define FILTER_RATIO    75            // Forhold mellem PWM og filter frekvens
#define CUTOFF_FREQ     (PWM_FREQ / FILTER_RATIO)     // ≈ 13 Hz

/*
* RC Filter Design Notes:
* - Ønsket cutoff frekvens: 13 Hz
* - Valgt kapacitor: C = 1µF
* - Beregnet modstand: R = 1/(2π * f * C) = 1/(2 * 3.14159 * 13 * 0.000001) ≈ 12.2kΩ
* - Bruger standard værdi: R = 12kΩ
* 
* Dette giver en faktisk cutoff frekvens på ca. 13.3 Hz
*/

/****************************************************************************
* Port Initialisering
* 
* Konfigurerer alle nødvendige porte for:
* - DIP switch input med pull-up modstande
* - PWM output
* - I2C kommunikation
* - Generelle I/O
****************************************************************************/
void init_ports()
{
    // Port Select Konfiguration
    P6SEL = 0x00;      // Port 6 som digital I/O
    P7SEL = 0;         // Port 7 som digital I/O
  
    // I2C Pin Setup
    pinMode(P1_1, INPUT_PULLUP);    // SDA med pull-up
    digitalWrite(P1_1, HIGH);
    
    // General I/O Setup
    pinMode(P4_7, OUTPUT);          // LED kontrol
    pinMode(P1_0, OUTPUT);          // Reserve output
    pinMode(P2_1, INPUT_PULLUP);    // Reserve input
    digitalWrite(P2_1, HIGH);
    
    // DIP Switch Konfiguration (P6.0-P6.6)
    P6DIR = 0;                      // Alle P6 pins som input
    P6REN |= 0x7F;                 // Enable pull-up/down for P6.0-P6.6
    P6OUT |= 0x7F;                 // Vælg pull-up
    
    // MSB DIP Switch (P7.0)
    P7REN |= BIT0;                 // Enable pull-up/down
    P7OUT |= BIT0;                 // Vælg pull-up
}

/****************************************************************************
* System Clock Initialisering
* 
* Konfigurerer XT2 høj-frekvens oscillator til 4 MHz
* Bruges som kilde for SMCLK (subsystem master clock)
****************************************************************************/
void init_SMCLK_XT2()
{
    WDTCTL = WDTPW|WDTHOLD;         // Stop watchdog timer
    
    P5SEL |= BIT2+BIT3;             // XT2 krystal pins
    UCSCTL6 &= ~XT2OFF;             // Enable XT2
    UCSCTL4 |= SELA_2;              // ACLK fra REFO
    UCSCTL4 |= SELS_5 + SELM_5;     // SMCLK og MCLK fra XT2
    
    // Oscillator fejl check loop
    do {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
        SFRIFG1 &= ~OFIFG;
    } while (SFRIFG1&OFIFG);
}

/****************************************************************************
* PWM Initialisering
* 
* Konfigurerer Timer A1 til PWM generation:
* - Frekvens: ~976.56 Hz (4MHz/4095)
* - Resolution: 12-bit (0-4095)
* - Output: P2.0
****************************************************************************/
void init_pwm()
{
    TA1CTL = TASSEL_2 |             // SMCLK som kilde
             MC_1 |                  // Up mode
             TACLR;                  // Clear timer

    TA1CCR0 = TA1CCR0_VALUE;        // PWM periode
    TA1CCR1 = TA1CCR0_VALUE/2;      // Initial 50% duty cycle
    
    TA1CCTL1 = OUTMOD_7;            // Reset/Set PWM mode
    
    pinMode(P2_0, OUTPUT);
    P2SEL |= BIT0;                  // P2.0 som PWM output
}

/****************************************************************************
* Duty Cycle Administration
****************************************************************************/
// Opdaterer PWM duty cycle baseret på DIP switch input
void update_duty_cycle(uint8_t dip_value)
{
    TA1CCR1 = dip_value << 4;       // Map 8-bit til 12-bit (gang med 16)
}

// Beregner aktuel duty cycle i procent
uint8_t calculate_duty_cycle()
{
    return (uint8_t)((((uint32_t)TA1CCR1 * 100) / TA1CCR0_VALUE));
}

// Beregner forventet output spænding i millivolt
uint16_t calculate_voltage_mv(uint8_t duty_cycle)
{
    return (3300 * duty_cycle) / 100;  // 3.3V = 3300mV
}

/****************************************************************************
* Hovedprogram
****************************************************************************/
int main()
{
    /* Initial system setup */
    WDTCTL = WDTPW | WDTHOLD;       // Stop watchdog timer
    init_SMCLK_XT2();               // Setup 4 MHz clock
    init_ports();                    // Konfigurer porte
    init_pwm();                      // Setup PWM
 
    /* Display initialisering */
    i2c_init();
    ssd1306_init();
    reset_diplay();
    ssd1306_clearDisplay();
    
    /* Variable deklarationer */
    char display_text[32];          // Display text buffer
    uint8_t duty_cycle;             // Aktuel duty cycle
    uint8_t prev_dip_value = 0;     // For ændring detektion
    uint8_t prev_duty_cycle = 0;    // For ændring detektion

    /* Display layout setup */
    ssd1306_printText(0, 0, "DIP:");    // DIP switch værdi
    ssd1306_printText(0, 2, "Duty:");   // Duty cycle procent
    ssd1306_printText(0, 4, "CCR1:");   // Timer værdi
    ssd1306_printText(0, 6, "DC V:");   // Beregnet spænding

    /* Hovedløkke */
    while(1)
    {
        // Læs og inverter DIP switch status
        uint8_t dip_value = ~((P6IN & 0x7F) | ((P7IN & BIT0) << 7));
        dip_value &= 0xFF;  // Mask til 8-bit
        
        // Kun opdater ved ændringer
        if(dip_value != prev_dip_value)
        {
            // Opdater PWM og beregn nye værdier
            update_duty_cycle(dip_value);
            duty_cycle = calculate_duty_cycle();
            
            /* Opdater display */
            // DIP værdi
            sprintf(display_text, "%d   ", dip_value);
            ssd1306_printText(40, 0, display_text);
            
            // Duty cycle
            sprintf(display_text, "%d%%   ", duty_cycle);
            ssd1306_printText(40, 2, display_text);
            
            // Timer værdi
            sprintf(display_text, "%d   ", TA1CCR1);
            ssd1306_printText(40, 4, display_text);
            
            // Beregnet spænding
            uint16_t voltage_mv = calculate_voltage_mv(duty_cycle);
            uint16_t volts = voltage_mv / 1000;
            uint16_t mv = voltage_mv % 1000;
            sprintf(display_text, "%d.%02dV  ", volts, mv/10);
            ssd1306_printText(40, 6, display_text);
            
            // Gem værdier til næste iteration
            prev_dip_value = dip_value;
            prev_duty_cycle = duty_cycle;
        }
        
        // Synkroniser med PWM timer
        while(!(TA1CCTL1 & CCIFG));    // Vent på CCR1 match
        TA1CCTL1 &= ~CCIFG;            // Clear flag
        
        __delay_cycles(50000);          // Delay for stabil operation
    }
}