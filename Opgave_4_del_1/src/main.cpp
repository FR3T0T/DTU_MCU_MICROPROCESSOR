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
#define MAX_DIGITS 6        
#define VOLT_DIGITS 8       
#define DUTY_DIGITS 8       
#define CHAR_WIDTH 6        

#define FILTER_SIZE 16      
#define ADC_THRESHOLD 3     
#define TA1CCR0_VALUE 1024  // 10-bit resolution for PWM

#define LABEL_X_POS 0       
#define VALUE_X_POS 72      
#define ADC_Y_POS 0         
#define PWM_Y_POS 2   
#define VOLT_Y_POS 4   

/****************************************************************************
* Globale Variable og Buffere
****************************************************************************/
volatile uint16_t adc_data = 0;     
volatile uint8_t adc_flag = 0;      

static uint16_t filter_buffer[FILTER_SIZE] = {0};  
static uint8_t filter_index = 0;    
static uint32_t filter_sum = 0;     
static uint8_t filter_count = 0;    

/****************************************************************************
* Filter Functions
****************************************************************************/
uint16_t filter_adc(uint16_t new_sample) {
    filter_sum -= filter_buffer[filter_index];
    filter_buffer[filter_index] = new_sample;
    filter_sum += new_sample;
    filter_index = (filter_index + 1) % FILTER_SIZE;
    if (filter_count < FILTER_SIZE) filter_count++;
    return (uint16_t)(filter_sum / filter_count);
}

uint16_t diff(uint16_t a, uint16_t b) {
    return (a > b) ? (a - b) : (b - a);
}

/****************************************************************************
* Hardware Initialization
****************************************************************************/
void init_SMCLK_25MHz() {
    WDTCTL = WDTPW | WDTHOLD; 

    P5SEL |= BIT2 + BIT3; 
    P5SEL |= BIT4 + BIT5;
    
    __bis_SR_register(SCG0); 
    UCSCTL0 = 0x0000;        
    UCSCTL1 = DCORSEL_7;     
    UCSCTL2 = FLLD_0 + 610;  // For 20MHz
    __bic_SR_register(SCG0); 

    do {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG); 
        SFRIFG1 &= ~OFIFG;                          
    } while (SFRIFG1 & OFIFG); 

    UCSCTL3 = SELREF__REFOCLK;                            
    UCSCTL4 = SELA__XT1CLK | SELS__DCOCLK | SELM__DCOCLK; 
    UCSCTL5 = DIVS__1;                                    
}

void adc_init(void) {
    ADC12CTL0 &= ~ADC12ENC;            // Disable ADC
    
    // Reset ADC12CTL0 to known state
    ADC12CTL0 = 0x0000;
    
    ADC12CTL0 = ADC12SHT0_3            // Sample time
              | ADC12ON                 // Turn on ADC12
              | ADC12MSC;              // Multiple sample and conversion
              
    ADC12CTL1 = ADC12SHP               // Sample timer
              | ADC12CONSEQ_2          // Repeat single channel
              | ADC12SSEL_0;           // ADC12OSC clock source
              
    // Explicitly set 10-bit resolution
    ADC12CTL2 = ADC12RES_1;            // 10-bit resolution (0-1023)
    
    // Clear any previous channel settings
    ADC12MCTL0 = 0x0000;
    
    // Set input channel to A12 (P7.0)
    ADC12MCTL0 = ADC12INCH_12          // Input channel A12
                | ADC12SREF_0;         // VR+ = AVCC and VR- = AVSS
    
    // Clear any pending interrupts
    ADC12IFG = 0x0000;
    
    ADC12IE |= ADC12IE0;               // Enable ADC12 interrupt
    
    P7SEL |= BIT0;                     // P7.0 ADC option select
    
    ADC12CTL0 |= ADC12ENC;             // Enable conversion
    ADC12CTL0 |= ADC12SC;              // Start conversion
}

void timer_init(void) {
    // PWM Setup
    P2DIR |= BIT0;                     // P2.0 output
    P2SEL |= BIT0;                     // P2.0 PWM function
    
    TA1CCR0 = 1024;                    // PWM Period (10-bit)
    TA1CCTL1 = OUTMOD_7;               // Reset/Set PWM mode
    TA1CCR1 = 0;                       // Initial duty cycle 0%
    
    TA1CTL = TASSEL_2                  // SMCLK source
           | MC_1                      // Up mode
           | TACLR;                    // Clear timer
}

/****************************************************************************
* Display Functions
****************************************************************************/
void update_display(uint16_t adc_val, uint16_t duty, float voltage) {
    char str[16];
    
    // ADC value
    uint16_t bounded_adc = adc_val & 0x3FF;
    snprintf(str, 5, "%4u", bounded_adc);
    ssd1306_printText(VALUE_X_POS, ADC_Y_POS, str);
    
    // Duty cycle
    snprintf(str, 5, "%3u%%", duty);
    ssd1306_printText(VALUE_X_POS, PWM_Y_POS, str);
    
    // Voltage - split i heltal og decimal del
    int volt_whole = (int)voltage;
    int volt_frac = (int)((voltage - volt_whole) * 100); // Get 2 decimal places
    
    // Format voltage med både heltal og decimaler
    snprintf(str, 8, "%d.%02dV", volt_whole, volt_frac);
    ssd1306_printText(VALUE_X_POS, VOLT_Y_POS, str);
}

/****************************************************************************
* Interrupt Service Routines
****************************************************************************/
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void) {
    if(ADC12IFG & BIT0) {
        // Ensure we don't exceed 10-bit range (0-1023)
        adc_data = ADC12MEM0 & 0x3FF;  // Mask to 10 bits
        adc_flag = 1;
        ADC12IFG &= ~BIT0;
    }
}

/****************************************************************************
* Main Function
****************************************************************************/
int main(void) {
    WDTCTL = WDTPW | WDTHOLD;         // Stop watchdog timer
    
    // Initialize all systems
    init_SMCLK_25MHz();               // Clock setup
    __delay_cycles(100000);
    
    i2c_init();                       // I2C for display
    __delay_cycles(100000);
    
    ssd1306_init();                   // Display init
    __delay_cycles(100000);
    
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
    while(1) {
        if (adc_flag) {
            uint16_t bounded_adc = adc_data & 0x3FF;  // Ensure 10-bit value
            uint16_t filtered_value = filter_adc(bounded_adc);
            
            if (diff(filtered_value, last_filtered_value) > ADC_THRESHOLD) {
                // Update PWM with bounded value
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