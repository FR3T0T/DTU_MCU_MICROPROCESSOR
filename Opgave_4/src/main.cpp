/*******************************************************************************
* DEL. 4
* ADC-baseret PWM Controller med OLED Display
*
* Systemformål:
* - Læser analog input via ADC
* - Genererer PWM signal baseret på ADC værdier
* - Viser real-time målinger på OLED display
* - Implementerer digital filtrering for stabil operation
*
* Hardware Konfiguration:
* - MSP430 mikrocontroller
* - ADC input på P7.0
* - PWM output på P2.0
* - OLED display via I2C
* - Samplingfrekvens styret af Timer A0
*
* Udviklingsplatform: MSP430 LaunchPad
*******************************************************************************/

/* Nødvendige Header Filer */
#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include "i2c.h"
#include "ssd1306.h"

// System konstanter
#define PWM_PERIOD      1024    // 10-bit opløsning for PWM
#define ADC_THRESHOLD   3       // Minimum ændring før PWM update
#define TIMER_FREQ      32768   // ACLK frekvens for capture timing

// Display formattering
#define MAX_STR_LEN     20
#define FREQ_Y_POS      2
#define PWM_Y_POS       4
#define ADC_Y_POS       6

// Globale variable for ADC
volatile uint16_t adc_data = 0;
volatile uint8_t adc_flag = 0;

// Globale variable for encoder/capture
volatile uint16_t last1 = 0;
volatile uint16_t last2 = 0;
volatile uint16_t captured_value1 = 0;
volatile uint16_t captured_value2 = 0;
volatile uint8_t i = 0;
volatile uint8_t j = 0;
volatile uint16_t freq1 = 0;
volatile uint16_t freq2 = 0;
volatile uint8_t t_flag1 = 0;
volatile uint8_t t_flag2 = 0;

// Buffer til display
char displayBuffer[MAX_STR_LEN];

/*******************************************************************************
* SMCLK Initialisering til 25MHz
*******************************************************************************/
void init_SMCLK_25MHz(void)
{
    WDTCTL = WDTPW | WDTHOLD; // Stop the watchdog timer

    P5SEL |= BIT2 + BIT3; // Select XT2 for SMCLK (Pins 5.2 and 5.3)
    P5SEL |= BIT4 + BIT5;
    
    // Configure DCO to 25 MHz
    __bis_SR_register(SCG0); // Disable FLL control loop
    UCSCTL0 = 0x0000;        // Set lowest possible DCOx and MODx
    UCSCTL1 = DCORSEL_7;     // Select DCO range
    UCSCTL2 = FLLD_0 + 610;  // Set for 20MHz
    __bic_SR_register(SCG0); // Enable FLL control loop

    // Loop until XT2, XT1, and DCO stabilize
    do {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
        SFRIFG1 &= ~OFIFG;
    } while (SFRIFG1 & OFIFG);

    UCSCTL3 = SELREF__REFOCLK;
    UCSCTL4 = SELA__XT1CLK | SELS__DCOCLK | SELM__DCOCLK;
    UCSCTL5 = DIVS__1;
}

/*******************************************************************************
* ADC12 Initialisering
*******************************************************************************/
void adc_init(void)
{
    ADC12CTL0 &= ~ADC12ENC;                 // Disable ADC12

    ADC12CTL0 = ADC12SHT0_3 |               // Sample time
                ADC12ON;                     // Turn on ADC12
                
    ADC12CTL1 = ADC12SHP |                  // Sample timer
                ADC12SSEL_0;                 // ADC12OSC
                
    ADC12CTL2 = ADC12RES_1;                 // 10-bit resolution

    ADC12MCTL0 = ADC12INCH_12;              // P7.0 = A12

    ADC12IE |= ADC12IE0;                    // Enable ADC12 interrupt

    P7SEL |= BIT0;                          // P7.0 ADC option select
    
    ADC12CTL0 |= ADC12ENC;                  // Enable conversions
}

/*******************************************************************************
* Timer Initialisering
*******************************************************************************/
void timer_init(void)
{
    // Timer A0 setup - Capture mode for encoder
    TA0CTL = TASSEL_1 |                     // ACLK = 32768 Hz
             MC_2 |                         // Continuous mode
             TACLR;                         // Clear TAR
             
    // Capture setup for P1.2 (B-encoder)
    TA0CCTL1 = CM_3 |                       // Both edges
               CCIS_0 |                     // CCIxA
               SCS |                        // Synchronous capture
               CAP |                        // Capture mode
               CCIE;                        // Enable interrupt
               
    // Capture setup for P1.3 (A-encoder)
    TA0CCTL2 = CM_3 |                       // Both edges
               CCIS_0 |                     // CCIxA
               SCS |                        // Synchronous capture
               CAP |                        // Capture mode
               CCIE;                        // Enable interrupt

    // Timer A1 setup - PWM generation
    TA1CCR0 = PWM_PERIOD - 1;               // PWM Period (1024 for 10-bit)
    TA1CCTL1 = OUTMOD_7;                    // Reset/set PWM mode
    TA1CTL = TASSEL_2 |                     // SMCLK
             MC_1 |                         // Up mode
             TACLR;                         // Clear TAR

    // GPIO Setup
    P1SEL |= BIT2 + BIT3;                   // P1.2 & P1.3 = TA0.1 & TA0.2
    P2DIR |= BIT0 + BIT2 + BIT3;            // P2.0 (PWM), P2.2/P2.3 (monitor) outputs
    P2SEL |= BIT0;                          // P2.0 = TA1.1
}

/*******************************************************************************
* Display Update Functions
*******************************************************************************/
void updateDisplay(uint16_t adc_val, uint16_t pwm_val, uint16_t freq)
{
    char tempBuffer[8];
    
    // Opdater ADC værdi
    strcpy(displayBuffer, "ADC: ");
    itoa(adc_val, tempBuffer, 10);
    strcat(displayBuffer, tempBuffer);
    ssd1306_printText(0, ADC_Y_POS, displayBuffer);

    // Opdater PWM duty cycle
    strcpy(displayBuffer, "PWM: ");
    itoa(pwm_val, tempBuffer, 10);
    strcat(displayBuffer, tempBuffer);
    ssd1306_printText(0, PWM_Y_POS, displayBuffer);

    // Opdater frekvens
    strcpy(displayBuffer, "Freq: ");
    itoa(freq, tempBuffer, 10);
    strcat(displayBuffer, "Hz");
    ssd1306_printText(0, FREQ_Y_POS, displayBuffer);
}

/*******************************************************************************
* Interrupt Service Routines
*******************************************************************************/
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=ADC12_VECTOR
__interrupt void ADC12_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC12_VECTOR))) ADC12_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(ADC12IV)
    {
        case  0: break;                     // Vector  0:  No interrupt
        case  6:                            // Vector  6:  ADC12IFG0
            adc_data = ADC12MEM0;           // Move results
            adc_flag = 1;                   // Set flag for main loop
            break;
        default: break;
    }
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A1_VECTOR
__interrupt void TIMER0_A1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A1_VECTOR))) TIMER0_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(TA0IV)
    {
        case  0: break;                     // No interrupt
        case  2:                            // CCR1 - B-encoder
            captured_value1 = captured_value1 + TA0CCR1 - last1;
            last1 = TA0CCR1;
            i++;
            if (i == 2)
            {
                freq1 = TIMER_FREQ/captured_value1;
                captured_value1 = 0;
                i = 0;
                t_flag1 = 1;
                P2OUT ^= BIT2;              // Toggle P2.2 for monitoring
            }
            break;
        case  4:                            // CCR2 - A-encoder
            captured_value2 = captured_value2 + TA0CCR2 - last2;
            last2 = TA0CCR2;
            j++;
            if (j == 2)
            {
                freq2 = TIMER_FREQ/captured_value2;
                captured_value2 = 0;
                j = 0;
                t_flag2 = 1;
                P2OUT ^= BIT3;              // Toggle P2.3 for monitoring
            }
            break;
        default: break;
    }
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) TIMER0_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    ADC12CTL0 |= ADC12SC;                  // Start ADC conversion
}

/*******************************************************************************
* Main Program
*******************************************************************************/
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;               // Stop watchdog timer
    
    // Initialiser hardware
    init_SMCLK_25MHz();                     // Setup SMCLK to 25MHz
    __delay_cycles(100000);
    
    adc_init();                             // Initialize ADC
    timer_init();                           // Initialize timers
    
    __delay_cycles(100000);
    i2c_init();                             // Initialize I2C
    __delay_cycles(100000);
    
    ssd1306_init();                         // Initialize display
    __delay_cycles(1000000);
    ssd1306_clearDisplay();                 // Clear display
    __delay_cycles(1000000);
    
    // Enable interrupts
    __bis_SR_register(GIE);
    
    // Control variable
    uint16_t last_adc = 0;
    float Xd = 0;                           // Ønsket hastighed
    float Xe = 0;                           // Hastighedsfejl
    const float G = 10.0;                   // Loop gain - juster efter behov
    
    // Main loop
    while(1)
    {
        // ADC og PWM kontrol
        if (adc_flag)
        {
            int16_t diff = (int16_t)adc_data - (int16_t)last_adc;
            if (diff < 0) diff = -diff;
            
            if (diff > ADC_THRESHOLD)
            {
                Xd = (float)adc_data * freq1 / 1023.0f;  // Beregn ønsket hastighed
                
                if (t_flag1)  // Ny frekvens måling tilgængelig
                {
                    Xe = Xd - freq1;  // Beregn hastighedsfejl
                    uint16_t pwm_value = (uint16_t)(G * Xe);
                    
                    // Begræns PWM værdi
                    if (pwm_value > PWM_PERIOD-1)
                        pwm_value = PWM_PERIOD-1;
                    
                    TA1CCR1 = pwm_value;    // Opdater PWM duty cycle
                    
                    // Opdater display
                    updateDisplay(adc_data, pwm_value, freq1);
                    
                    t_flag1 = 0;
                }
                last_adc = adc_data;
            }
            adc_flag = 0;
        }
    }
}