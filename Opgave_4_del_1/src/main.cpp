/*******************************************************************************
* Co-Pilot, Chat-GPT & Claude AI er blevet brugt til fejlfinding og hjælp.
*
* DEL 1 & 1B
* Motor Driver og Encoder med PWM Kontrol og OLED Display
*
* Systemformål:
* - Læser analog input via ADC fra potentiometer
* - Genererer PWM signal med 10-bit opløsning (0-1023)
* - Styrer DC-motor hastighed via MOSFET
* - Viser real-time målinger på OLED display:
*   * ADC værdi (0-1023)
*   * PWM duty cycle (0-100%)
*   * Spænding (0-3.3V)
*
* Hardware Konfiguration:
* - MSP430 mikrocontroller
* - ADC input på P7.0 fra 10k potentiometer
* - PWM output på P2.0 til MOSFET gate
* - MOSFET IRF530 som motor driver
* - Beskyttelsesdiode 1N4006 over motor
* - OLED display via I2C
* - Ekstern motorforsyning (12V)
*
* Specielle funktioner:
* - 10-bit ADC opløsning
* - PWM frekvens ~10kHz ved 20MHz SMCLK
* - Digital filtrering for stabil motorstyring
* - Realtids display opdatering
*
* Udviklingsplatform: MSP430F5529 LaunchPad
*******************************************************************************/
#include <Arduino.h>
#include "i2c.h"
#include "ssd1306.h"
#include <msp430.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

/****************************************************************************
* System Konstanter og Konfiguration
****************************************************************************/ 
#define FILTER_SIZE 16      
#define ADC_THRESHOLD 3     
#define TA1CCR0_VALUE 1024  // 10-bit resolution for PWM

#define LABEL_X_POS 0       
#define VALUE_X_POS 72      
#define ADC_Y_POS 0         
#define PWM_Y_POS 1   
#define VOLT_Y_POS 2   

/****************************************************************************
* Globale Variable og Buffere
****************************************************************************/
volatile uint16_t adc_data = 0;     
volatile uint8_t adc_flag = 0;      

static uint16_t filter_buffer[FILTER_SIZE] = {0};  
static uint8_t filter_index = 0;    
static uint32_t filter_sum = 0;     

/****************************************************************************
* Filter Functions
****************************************************************************/
uint16_t filter_adc(uint16_t new_sample)
{
    filter_sum -= filter_buffer[filter_index];
    filter_buffer[filter_index] = new_sample;
    filter_sum += new_sample;
    filter_index = (filter_index + 1) % FILTER_SIZE;
    return (uint16_t)(filter_sum / FILTER_SIZE);  // Altid divider med FILTER_SIZE
}

uint16_t diff(uint16_t a, uint16_t b)
{
    return (a > b) ? (a - b) : (b - a);
}

/****************************************************************************
* Hardware Initialization
****************************************************************************/
void init_SMCLK_20MHz()  // Rettet navn til at matche faktisk frekvens
{
    P5SEL |= BIT2 + BIT3 + BIT4 + BIT5;  // Select XT2 for SMCLK (Pins 5.2 and 5.3)
    
     // Configure DCO to 25 MHz
    __bis_SR_register(SCG0); // Disable FLL control loop
    UCSCTL0 = 0x0000;        // Set lowest possible DCOx and MODx
    UCSCTL1 = DCORSEL_7;     // Select DCO range (DCORSEL_7 for max range)
    UCSCTL2 = FLLD_0 + 610;  // FLLD = 1, Multiplier N = 762 for ~25 MHz DCO   - 610 for 20Mhz

    //calculated by f DCOCLK  =32.768kHz×610=20MHz
    __bic_SR_register(SCG0); // Enable FLL control loop

    // Loop until XT2, XT1, and DCO stabilize
    do
    {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);         // Clear fault flags
        SFRIFG1 &= ~OFIFG;                                  // Clear oscillator fault flags
    }
    while (SFRIFG1 & OFIFG);                                // Wait until stable

    UCSCTL3 = SELREF__REFOCLK;                              // Set FLL reference to REFO           
    UCSCTL4 = SELA__XT1CLK | SELS__DCOCLK | SELM__DCOCLK;   // Set ACLK = XT1; SMCLK = DCO; MCLK = DCO
    UCSCTL5 = DIVS__1;                                      // Set SMCLK divider to 1 (no division)
}

void adc_init(void)
{
    ADC12CTL0 &= ~ADC12ENC;            // Disable ADC
    
    ADC12CTL0 = ADC12SHT0_3            // Sample time
              | ADC12ON                 // Turn on ADC12
              | ADC12MSC;              // Multiple sample and conversion
              
    ADC12CTL1 = ADC12SHP               // Sample timer
              | ADC12CONSEQ_2          // Repeat single channel
              | ADC12SSEL_0;           // ADC12OSC clock source
              
    ADC12CTL2 = ADC12RES_1;            // 10-bit resolution (0-1023)
    
    ADC12MCTL0 = ADC12INCH_12          // Input channel A12
                | ADC12SREF_0;         // VR+ = AVCC and VR- = AVSS
    
    ADC12IFG = 0x0000;                 // Clear any pending interrupts
    
    ADC12IE |= ADC12IE0;               // Enable ADC12 interrupt
    
    P7SEL |= BIT0;                     // P7.0 ADC option select
    
    ADC12CTL0 |= ADC12ENC;             // Enable conversion
    ADC12CTL0 |= ADC12SC;              // Start conversion
}

void timer_init(void)
{
    P2DIR |= BIT0;                     // P2.0 output
    P2SEL |= BIT0;                     // P2.0 PWM function
    
    TA1CCR0 = 1024;                    // PWM Period (10-bit)
    TA1CCTL1 = OUTMOD_2;               // Reset/Set PWM mode
    TA1CCR1 = 0;                       // Initial duty cycle 0%
    
    TA1CTL = TASSEL_2                  // SMCLK source
           | MC_3                      // Up mode
           | TACLR;                    // Clear timer
}

/****************************************************************************
* Display Functions
****************************************************************************/
void update_display(uint16_t adc_val, uint16_t duty, float voltage)
{
    char str[16];
    
    // ADC value
    snprintf(str, 5, "%4u", adc_val);
    ssd1306_printText(VALUE_X_POS, ADC_Y_POS, str);
    
    // Duty cycle
    snprintf(str, 5, "%3u%%", duty);
    ssd1306_printText(VALUE_X_POS, PWM_Y_POS, str);
    
    // Voltage - split i heltal og decimal del
    int volt_whole = (int)voltage;
    int volt_frac = (int)((voltage - volt_whole) * 100);
    
    snprintf(str, 8, "%d.%02dV", volt_whole, volt_frac);
    ssd1306_printText(VALUE_X_POS, VOLT_Y_POS, str);
}

/****************************************************************************
* Interrupt Service Routines
****************************************************************************/
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)
{
    if(ADC12IFG & BIT0)
    {
        adc_data = ADC12MEM0 & 0x3FF;  // Mask to 10 bits
        adc_flag = 1;
        ADC12IFG &= ~BIT0;
    }
}

/****************************************************************************
* Main Function
****************************************************************************/
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;         // Stop watchdog timer
    
    // Initialize all systems
    init_SMCLK_20MHz();               // Clock setup
    
    // Single delay after alle initialiseringer
    __delay_cycles(300000);           // Combined delay for all inits
    
    i2c_init();                       // I2C for display
    ssd1306_init();                   // Display init
    timer_init();                     // PWM setup
    adc_init();                       // ADC setup
    
    // Setup display
    ssd1306_clearDisplay();
    ssd1306_printText(LABEL_X_POS, ADC_Y_POS, "ADC:");
    ssd1306_printText(LABEL_X_POS, PWM_Y_POS, "Duty:");
    ssd1306_printText(LABEL_X_POS, VOLT_Y_POS, "Volt:");
    
    // Enable global interrupts
    __bis_SR_register(GIE);
    
    uint16_t last_filtered_value = 0;
    
    // Main loop
    while(1)
    {
        if (adc_flag)
        {
            uint16_t filtered_value = filter_adc(adc_data);
            
            if (diff(filtered_value, last_filtered_value) > ADC_THRESHOLD)
            {
                // Update PWM with filtered value
                TA1CCR1 = filtered_value;
                
                // Calculate display values
                uint16_t duty_cycle = (filtered_value * 100UL) / 1024;
                float voltage = (filtered_value * 3.3f) / 1024.0f;
                
                // Update display
                update_display(filtered_value, duty_cycle, voltage);
                
                last_filtered_value = filtered_value;
            }
            
            adc_flag = 0;
            ADC12CTL0 |= ADC12SC;     // Start next conversion
            __delay_cycles(10000);     // Stability delay
        }
    }
    return 0;
}