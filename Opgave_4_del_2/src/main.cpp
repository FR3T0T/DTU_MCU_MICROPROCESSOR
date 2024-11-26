#include <msp430.h>
#include "i2c.h"
#include "ssd1306.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>

/****************************************************************************
* Konstanter og Konfiguration
****************************************************************************/ 
#define FILTER_SIZE 16      
#define ADC_THRESHOLD 3     
#define TA1CCR0_VALUE 1024  // 10-bit PWM resolution

// Display position defines - alle samlet her
#define LABEL_X_POS 0
#define VALUE_X_POS 72
#define ADC_Y_POS 0
#define PWM_Y_POS 1
#define VOLT_Y_POS 2
#define ENC1_Y_POS 3
#define ENC2_Y_POS 4

/****************************************************************************
* Globale Variable
****************************************************************************/
volatile uint16_t adc_data = 0;     
volatile uint8_t adc_flag = 0;      
static uint16_t filter_buffer[FILTER_SIZE] = {0};  
static uint8_t filter_index = 0;    
static uint32_t filter_sum = 0;     
volatile uint16_t captured_value1 = 0;
volatile uint16_t captured_value2 = 0;
volatile uint8_t capture_flag1 = 0;
volatile uint8_t capture_flag2 = 0;

/****************************************************************************
* Function Prototypes
****************************************************************************/
void init_clock(void);
void init_adc(void);
void init_pwm(void);
void init_timerA0_capture(void);
uint16_t filter_adc(uint16_t new_sample);
uint16_t diff(uint16_t a, uint16_t b);
float calculate_frequency(uint16_t capture_value);
void update_display(uint16_t adc_val, uint16_t duty, float voltage, float freq1, float freq2);

/****************************************************************************
* Implementation
****************************************************************************/
void init_clock(void) {
    P5SEL |= BIT2 + BIT3 + BIT4 + BIT5;
    
    __bis_SR_register(SCG0);
    UCSCTL0 = 0x0000;
    UCSCTL1 = DCORSEL_7;
    UCSCTL2 = FLLD_0 + 610;
    __bic_SR_register(SCG0);
    
    do {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
        SFRIFG1 &= ~OFIFG;
    } while (SFRIFG1 & OFIFG);
}

void init_pwm(void) {
    P2DIR |= BIT0;                     
    P2SEL |= BIT0;                     
    
    TA1CCR0 = TA1CCR0_VALUE;          
    TA1CCTL1 = OUTMOD_7;              
    TA1CCR1 = 0;                      
    
    TA1CTL = TASSEL_2                 
           | MC_3                     
           | TACLR;                   
}

void init_timerA0_capture(void) {
    TA0CTL = TASSEL_1    
           + MC_2        
           + TACLR;      

    P1DIR &= ~(BIT2 | BIT3);    
    P1SEL |= (BIT2 | BIT3);     

    TA0CCTL1 = CM_3     
             + CCIS_0   
             + CAP      
             + CCIE     
             + SCS;     

    TA0CCTL2 = CM_3     
             + CCIS_0   
             + CAP      
             + CCIE     
             + SCS;     

    P2DIR |= (BIT2 | BIT3);    
    P2OUT &= ~(BIT2 | BIT3);   
}

void init_adc(void) {
    ADC12CTL0 &= ~ADC12ENC;           
    
    ADC12CTL0 = ADC12SHT0_3           
               | ADC12ON               
               | ADC12MSC;            
              
    ADC12CTL1 = ADC12SHP              
               | ADC12CONSEQ_2         
               | ADC12SSEL_0;          
              
    ADC12CTL2 = ADC12RES_1;           
    
    ADC12MCTL0 = ADC12INCH_12         
                | ADC12SREF_0;        
    
    ADC12IFG = 0x0000;                
    ADC12IE |= ADC12IE0;              
    P7SEL |= BIT0;                    
    
    ADC12CTL0 |= ADC12ENC;            
    ADC12CTL0 |= ADC12SC;             
}

uint16_t filter_adc(uint16_t new_sample) {
    filter_sum -= filter_buffer[filter_index];
    filter_buffer[filter_index] = new_sample;
    filter_sum += new_sample;
    filter_index = (filter_index + 1) % FILTER_SIZE;
    return (uint16_t)(filter_sum / FILTER_SIZE);
}

uint16_t diff(uint16_t a, uint16_t b) {
    return (a > b) ? (a - b) : (b - a);
}

float calculate_frequency(uint16_t capture_value) {
    if (capture_value == 0) return 0.0f;
    return (32768.0f * 2.0f) / (float)capture_value;
}

void update_display(uint16_t adc_val, uint16_t duty, float voltage, float freq1, float freq2) {
    char str[32];
    
    // ADC værdi (uændret)
    snprintf(str, sizeof(str), "ADC: %4u", adc_val);
    ssd1306_printText(0, ADC_Y_POS, str);
    
    // PWM duty cycle (uændret)
    snprintf(str, sizeof(str), "PWM: %3u%%", duty);
    ssd1306_printText(0, PWM_Y_POS, str);
    
    // Spænding - konverter til hele og decimal del
    int16_t volt_whole = (int16_t)voltage;
    int16_t volt_frac = (int16_t)((voltage - volt_whole) * 100);
    snprintf(str, sizeof(str), "V: %d.%02dV", volt_whole, volt_frac);
    ssd1306_printText(0, VOLT_Y_POS, str);
    
    // Frekvens 1 - konverter til hele og decimal del
    int16_t freq1_whole = (int16_t)freq1;
    int16_t freq1_frac = (int16_t)((freq1 - freq1_whole) * 10);
    snprintf(str, sizeof(str), "F1: %4d.%1dHz", freq1_whole, freq1_frac);
    ssd1306_printText(0, ENC1_Y_POS, str);
    
    // Frekvens 2 - konverter til hele og decimal del
    int16_t freq2_whole = (int16_t)freq2;
    int16_t freq2_frac = (int16_t)((freq2 - freq2_whole) * 10);
    snprintf(str, sizeof(str), "F2: %4d.%1dHz", freq2_whole, freq2_frac);
    ssd1306_printText(0, ENC2_Y_POS, str);
}

#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void) {
    if(ADC12IFG & BIT0) {
        adc_data = ADC12MEM0 & 0x3FF;  
        adc_flag = 1;
        ADC12IFG &= ~BIT0;
    }
}

#pragma vector=TIMER0_A1_VECTOR
__interrupt void Timer_A1_ISR(void) {
    static uint16_t last1 = 0;
    static uint16_t last2 = 0;
    
    switch(TA0IV) {
        case 0x02:  // CCR1 interrupt (P1.2)
            captured_value1 = TA0CCR1 - last1;
            last1 = TA0CCR1;
            P2OUT ^= BIT2;  
            capture_flag1 = 1;
            break;
            
        case 0x04:  // CCR2 interrupt (P1.3)
            captured_value2 = TA0CCR2 - last2;
            last2 = TA0CCR2;
            P2OUT ^= BIT3;  
            capture_flag2 = 1;
            break;
    }
}

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;     // Stop watchdog timer
    
    init_clock();                  // Initialize clock system
    
    __delay_cycles(300000);        // Delay for stability
    
    // Initialize I2C and OLED
    i2c_init();
    __delay_cycles(50000);         // Add delay after I2C init
    
    ssd1306_init();
    __delay_cycles(50000);         // Add delay after OLED init
    
    // Clear display and add a small delay
    ssd1306_clearDisplay();
    __delay_cycles(50000);
    
    // Initial display test
    ssd1306_printText(0, 0, "System Start");
    __delay_cycles(1000000);
    
    // Initialize other peripherals
    init_pwm();
    init_adc();
    init_timerA0_capture();
    
    __bis_SR_register(GIE);        // Enable global interrupts
    
    uint16_t last_filtered_value = 0;
    float freq1 = 0, freq2 = 0;
    
    while(1) {
        if (adc_flag) {
            uint16_t filtered_value = filter_adc(adc_data);
            
            if (diff(filtered_value, last_filtered_value) > ADC_THRESHOLD) {
                // Update PWM
                TA1CCR1 = filtered_value;
                
                // Calculate values
                uint16_t duty_cycle = (filtered_value * 100UL) / 1024;
                float voltage = (filtered_value * 3.3f) / 1024.0f;
                
                // Update frequencies if new values available
                if (capture_flag1) {
                    freq1 = calculate_frequency(captured_value1);
                    capture_flag1 = 0;
                }
                if (capture_flag2) {
                    freq2 = calculate_frequency(captured_value2);
                    capture_flag2 = 0;
                }
                
                // Update display
                update_display(filtered_value, duty_cycle, voltage, freq1, freq2);
                
                last_filtered_value = filtered_value;
            }
            
            adc_flag = 0;
            ADC12CTL0 |= ADC12SC;     
            __delay_cycles(10000);     
        }
    }
    
    return 0;
}