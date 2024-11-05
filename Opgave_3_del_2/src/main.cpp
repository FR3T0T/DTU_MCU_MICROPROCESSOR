/*******************************************************************************
* DEL 2
* ADC12 PWM Generator med Potentiometer Kontrol og OLED Display
*
* Dette program implementerer en PWM generator med følgende funktioner:
* - PWM output på P2.0 med frekvens omkring 1 kHz
* - Duty cycle styres via 10kΩ potentiometer på P7.0 (A12)
* - OLED display viser ADC værdier og beregnede spændinger
* - RC lavpasfilter kan tilsluttes for DC konvertering
*
* Hardware krav:
* - MSP430F5529 LaunchPad
* - 10kΩ potentiometer tilsluttet P7.0 (A12)
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
#define SMCLK_FREQ      4000000.0f    // System clock frekvens (4 MHz)
#define TA1CCR0_VALUE   4095          // Timer A1 periode (12-bit opløsning)
#define PWM_FREQ        (SMCLK_FREQ / TA1CCR0_VALUE)  // ≈ 976.56 Hz
#define VFS             3.3f          // Reference spænding for ADC (3.3V)
#define ADC_BITS        12            // ADC opløsning (12-bit)

/****************************************************************************
* Port Initialisering
****************************************************************************/
void init_ports()
{
    // I2C Pin Setup
    pinMode(P1_1, INPUT_PULLUP);    // SDA med pull-up
    digitalWrite(P1_1, HIGH);
    
    // General I/O Setup
    pinMode(P4_7, OUTPUT);          // LED kontrol
    pinMode(P1_0, OUTPUT);          // Reserve output
    pinMode(P2_1, INPUT_PULLUP);    // Reserve input
    digitalWrite(P2_1, HIGH);
    
    // ADC Input Setup (P7.0/A12)
    P7SEL |= BIT0;                  // Analog funktion for P7.0
    P7DIR &= ~BIT0;                 // Input
    P7REN |= BIT0;                  // Enable pull-down modstand
    P7OUT &= ~BIT0;                 // Vælg pull-down
}

/****************************************************************************
* ADC12 Initialisering
****************************************************************************/
void init_adc()
{
    ADC12CTL0 &= ~ADC12ENC;           // Disable ADC12 før konfiguration
    
    // Nulstil alle ADC12 kontrolregistre
    ADC12CTL0 = 0;
    ADC12CTL1 = 0;
    ADC12CTL2 = 0;
    
    // Konfigurer ADC12
    ADC12CTL0 = ADC12ON |             // Tænd ADC12
                ADC12SHT0_8 |         // Længere sample tid (256 cycles)
                ADC12MSC;             // Multiple sample/conversion
                
    ADC12CTL1 = ADC12SHP |            // Sample timer
                ADC12SSEL_0 |         // ADC12OSC clock
                ADC12DIV_7 |          // Clock divider /8
                ADC12CONSEQ_0;        // Single channel mode
                
    ADC12CTL2 = ADC12RES_2 |          // 12-bit opløsning
                ADC12TCOFF;           // Temperatur sensor off
    
    // Input kanal A12 (P7.0)
    ADC12MCTL0 = ADC12INCH_12 |       // Channel A12
                 ADC12SREF_0;         // AVCC reference
    
    // Lad ADC stabilisere sig
    __delay_cycles(1000);
    
    ADC12CTL0 |= ADC12ENC;            // Enable ADC
}

/****************************************************************************
* ADC12 Sample Funktion
****************************************************************************/
uint16_t get_adc_sample()
{
    ADC12CTL0 |= ADC12ENC | ADC12SC;  // Start sampling/conversion
    
    while (!(ADC12IFG & BIT0));       // Vent på konvertering
    
    return ADC12MEM0;                 // Returner resultat
}

/****************************************************************************
* System Clock Initialisering
****************************************************************************/
void init_SMCLK_XT2()
{
    WDTCTL = WDTPW|WDTHOLD;         // Stop watchdog timer
    
    P5SEL |= BIT2+BIT3;             // XT2 krystal pins
    UCSCTL6 &= ~XT2OFF;             // Enable XT2
    UCSCTL4 |= SELA_2;              // ACLK fra REFO
    UCSCTL4 |= SELS_5 + SELM_5;     // SMCLK og MCLK fra XT2
    
    do {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
        SFRIFG1 &= ~OFIFG;
    } while (SFRIFG1&OFIFG);
}

/****************************************************************************
* PWM Initialisering
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
* ADC til PWM Konvertering
****************************************************************************/
void update_pwm_from_adc(uint16_t adc_value)
{
    // Direkte mapping fra 12-bit ADC til 12-bit PWM
    TA1CCR1 = adc_value;
}

// Beregn teoretisk NADC værdi
uint16_t calculate_theoretical_NADC(float Vin)
{
    return (uint16_t)((pow(2, ADC_BITS) * Vin) / VFS);
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
    init_adc();                      // Setup ADC12
 
    /* Display initialisering */
    i2c_init();
    ssd1306_init();
    reset_diplay();
    ssd1306_clearDisplay();
    
    /* Display layout setup */
    ssd1306_printText(0, 0, "ADC Val:");     // Rå ADC værdi
    ssd1306_printText(0, 2, "Volt In:");     // Målt spænding
    ssd1306_printText(0, 4, "Theo N:");      // Teoretisk NADC
    ssd1306_printText(0, 6, "PWM DC:");      // PWM duty cycle

    /* Hovedløkke */
    while(1)
    {
        // Læs ADC værdi
        uint16_t adc_value = get_adc_sample();
        
        // Beregn input spænding (V)
        float voltage = (adc_value * VFS) / (float)(1 << ADC_BITS);
        
        // Beregn teoretisk NADC
        uint16_t theo_nadc = calculate_theoretical_NADC(voltage);
        
        // Beregn duty cycle procent
        uint8_t duty_cycle = (adc_value * 100) / TA1CCR0_VALUE;
        
        // Opdater PWM
        update_pwm_from_adc(adc_value);
        
        /* Opdater display */
        // ADC værdi
        ssd1306_printUI32(64, 0, adc_value, 0);
        
        // Spænding (med 3 decimaler)
        char volt_str[16];
        sprintf(volt_str, "%d.%03dV", (int)voltage, 
                (int)((voltage - (int)voltage) * 1000));
        ssd1306_printText(64, 2, volt_str);
        
        // Teoretisk NADC
        ssd1306_printUI32(64, 4, theo_nadc, 0);
        
        // PWM duty cycle
        char duty_str[16];
        sprintf(duty_str, "%d%%", duty_cycle);
        ssd1306_printText(64, 6, duty_str);
        
        // Delay for stabil operation
        __delay_cycles(50000);
    }
}