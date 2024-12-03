#include <msp430.h>
#include "i2c.h"
#include "ssd1306.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>

/****************************************************************************
 * Konstanter og Konfiguration
 ****************************************************************************/
// Display relaterede konstanter
#define MAX_DIGITS 8
#define LABEL_X_POS 0
#define FREQ1_LINE 0
#define FREQ2_LINE 4
#define MOTOR_SPEED_LINE 2
#define TARGET_SPEED_LINE 3
#define ADC_Y_POS 0
#define PWM_Y_POS 1

// Filter og PWM konstanter
#define FILTER_SIZE 16
#define ADC_THRESHOLD 3
#define TA1CCR0_VALUE 512

/****************************************************************************
 * Globale Variable
 ****************************************************************************/
volatile uint16_t captured_value1 = 0;
volatile uint16_t captured_value2 = 0;
volatile uint8_t t_flag1 = 0;
volatile uint8_t t_flag2 = 0;
volatile float current_speed = 0.0f;
volatile float target_speed = 0.0f;
volatile uint16_t adc_data = 0;
volatile uint8_t adc_flag = 0;

static uint16_t filter_buffer[FILTER_SIZE] = {0};
static uint8_t filter_index = 0;
static uint32_t filter_sum = 0;

/****************************************************************************
 * Funktioner til ADC filter
 ****************************************************************************/
uint16_t filter_adc(uint16_t new_sample)
{
    filter_sum -= filter_buffer[filter_index];
    filter_buffer[filter_index] = new_sample;
    filter_sum += new_sample;
    filter_index = (filter_index + 1) % FILTER_SIZE;
    return (uint16_t)(filter_sum / FILTER_SIZE);
}

uint16_t diff(uint16_t a, uint16_t b)
{
    return (a > b) ? (a - b) : (b - a);
}

/****************************************************************************
 * Hardware Initialisering
 ****************************************************************************/
void adc_init(void)
{
    ADC12CTL0 &= ~ADC12ENC;
    ADC12CTL0 = ADC12SHT0_3 | ADC12ON | ADC12MSC;
    ADC12CTL1 = ADC12SHP | ADC12CONSEQ_2 | ADC12SSEL_0;
    ADC12CTL2 = ADC12RES_1;
    ADC12MCTL0 = ADC12INCH_12 | ADC12SREF_0;
    ADC12IFG = 0x0000;
    ADC12IE |= ADC12IE0;
    P7SEL |= BIT0;
    ADC12CTL0 |= ADC12ENC;
    ADC12CTL0 |= ADC12SC;
}

void timer_init(void)
{
    P2DIR |= BIT0;
    P2SEL |= BIT0;
    TA1CCR0 = TA1CCR0_VALUE;
    TA1CCTL1 = OUTMOD_7;
    TA1CCR1 = 0;
    TA1CTL = TASSEL_2 | MC_1 | TACLR;
}

void init_timerA0_capture(void)
{
    TA0CTL = TASSEL_1 + MC_2 + ID_0;
    P1DIR &= ~(BIT2 | BIT3);
    P1SEL |= (BIT2 | BIT3);
    TA0CCTL1 = CM_3 + CCIS_0 + CAP + CCIE + SCS;
    TA0CCTL2 = CM_3 + CCIS_0 + CAP + CCIE + SCS;
    P6DIR |= BIT0 | BIT1 | BIT2;
    P6OUT |= BIT0;
}

void init_SMCLK_20MHz(void)
{
    P5SEL |= BIT2 + BIT3 + BIT4 + BIT5;
    __bis_SR_register(SCG0);
    UCSCTL0 = 0x0000;
    UCSCTL1 = DCORSEL_7;
    UCSCTL2 = FLLD_0 + 610;
    __bic_SR_register(SCG0);
    do
    {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
        SFRIFG1 &= ~OFIFG;
    } while (SFRIFG1 & OFIFG);
    UCSCTL3 = SELREF__REFOCLK;
    UCSCTL4 = SELA__XT1CLK | SELS__DCOCLK | SELM__DCOCLK;
    UCSCTL5 = DIVS__1;
}

void init_all_hardware(void)
{
    init_SMCLK_20MHz();
    i2c_init();
    ssd1306_init();
    adc_init();
    init_timerA0_capture();
    timer_init();
}

/****************************************************************************
 * Motor Control Functions
 ****************************************************************************/
void update_motor_control(void)
{
    float speed_error = target_speed - current_speed;
    int16_t pwm_adjustment = (int16_t)(speed_error * 50.0f);
    uint16_t new_pwm = TA1CCR1;
    if (pwm_adjustment > 0 && new_pwm < (TA1CCR0_VALUE - pwm_adjustment))
        new_pwm += pwm_adjustment;
    else if (pwm_adjustment < 0 && new_pwm > (-pwm_adjustment))
        new_pwm += pwm_adjustment;
    if (new_pwm > (TA1CCR0_VALUE - 1)) new_pwm = TA1CCR0_VALUE - 1;
    if (new_pwm < 1) new_pwm = 1;
    TA1CCR1 = new_pwm;
}

float calculate_frequency(uint16_t capture_value)
{
    if (capture_value == 0 || capture_value > 32768) return 0.0f;
    return 32768.0f / (float)capture_value;
}

/****************************************************************************
 * Display Functions
 ****************************************************************************/
void update_display(float freq1, float freq2)
{
    char str[32];
    
    // Vis ADC og PWM værdier
    snprintf(str, sizeof(str), "ADC: %4u PWM: %3u%%", 
        adc_data, 
        (TA1CCR1 * 100UL) / TA1CCR0_VALUE);
    ssd1306_printText(LABEL_X_POS, FREQ1_LINE, str);
    
    // Vis frekvenser som i del 2
    if (freq1 > 0)
    {
        int freq1_whole = (int)freq1;
        int freq1_frac = (int)((freq1 - freq1_whole) * 10);
        snprintf(str, sizeof(str), "F1: %d.%dHz    ", freq1_whole, freq1_frac);
        ssd1306_printText(LABEL_X_POS, FREQ2_LINE, str);
    }
    
    if (freq2 > 0)
    {
        int freq2_whole = (int)freq2;
        int freq2_frac = (int)((freq2 - freq2_whole) * 10);
        snprintf(str, sizeof(str), "F2: %d.%dHz    ", freq2_whole, freq2_frac);
        ssd1306_printText(LABEL_X_POS, FREQ2_LINE + 1, str);
    }
}

/****************************************************************************
 * Interrupt Service Rutiner
 ****************************************************************************/
#pragma vector=TIMER0_A1_VECTOR
__interrupt void Timer_A1_ISR(void)
{
    static unsigned int last1 = 0;
    static unsigned int last2 = 0;
    switch(TA0IV)
    {
        case 0x02:
            if(last1 > TA0CCR1)
                captured_value1 = TA0CCR1 + (65535 - last1);
            else
                captured_value1 = TA0CCR1 - last1;
            last1 = TA0CCR1;
            P6OUT |= BIT1;
            P2OUT ^= BIT2;
            t_flag1 = 1;
            break;
        case 0x04:
            captured_value2 = TA0CCR2 - last2;
            last2 = TA0CCR2;
            P6OUT |= BIT2;
            P2OUT ^= BIT3;
            t_flag2 = 1;
            break;
    }
}

#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)
{
    if(ADC12IFG & BIT0)
    {
        adc_data = ADC12MEM0 & 0x3FF;
        adc_flag = 1;
        ADC12IFG &= ~BIT0;
    }
}

/****************************************************************************
 * Main Program
 ****************************************************************************/
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;
    
    init_all_hardware();
    ssd1306_clearDisplay();
    
    __bis_SR_register(GIE);
    
    uint16_t last_filtered_value = 0;
    float freq1 = 0.0f, freq2 = 0.0f;
    
    while(1)
    {
        if (adc_flag)
        {
            uint16_t filtered_value = filter_adc(adc_data);
            
            if (diff(filtered_value, last_filtered_value) > ADC_THRESHOLD)
            {
                TA1CCR1 = filtered_value;  // Direkte PWM kontrol
                last_filtered_value = filtered_value;
            }
            
            adc_flag = 0;
            ADC12CTL0 |= ADC12SC;
        }
        
        if (t_flag1)
        {
            freq1 = calculate_frequency(captured_value1);
            t_flag1 = 0;
        }
        
        if (t_flag2)
        {
            freq2 = calculate_frequency(captured_value2);
            t_flag2 = 0;
        }
        
        update_display(freq1, freq2);
        __delay_cycles(5000);
    }
    return 0;
}