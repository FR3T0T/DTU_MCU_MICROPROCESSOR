#include <Arduino.h>
#include "i2c.h"
#include "ssd1306.h"
#include <msp430f5529.h>
#include <stdint.h>

// Constants for motor control
#define FREQ_MAX 500             // Maximum encoder frequency
#define MAX_DUTY_CYCLE 1023      // 10-bit resolution for PWM
#define TIMER_CLK 32768          // ACLK frequency for Timer A0
#define SAMPLES_AVG 1           // Exactly 10 samples as required
#define NOMINAL_VOLTAGE 12.0     // Nominal voltage for motor
#define USE_FREQ_SCALING 0       // 0 for digital voltage scaling (default), 1 for frequency
#define PWM_TOP 1023            // 10-bit resolution
#define SLEW_RATE_LIMIT 0.5     // Slew rate limiting factor

// Global Variables
volatile int adcResult = 0;              // ADC result
volatile char adc_flag = 0;              // ADC conversion flag
float G = 0.5;                         // Initial proportional gain
volatile float Gm;                     // Motor gain scaling
volatile float Gc;                     // Corrected gain = G * (V2/V1)
volatile float currentVoltage = 12.0;  // Current motor voltage
//volatile int direction = 1;            // 1 forward, -1 reverse

// Encoder variables
volatile unsigned int captured_value1 = 0;  // For encoder A
volatile unsigned int captured_value2 = 0;  // For encoder B
volatile char t_flag1 = 0;
volatile char t_flag2 = 0;
volatile long Xd = 0;                     // Desired speed (from pot)
volatile long Xf_A = 0;                   // Actual speed from encoder A
volatile long Xf_B = 0;                   // Actual speed from encoder B
volatile long Xf = 0;                     // Average encoder speed
volatile long Xe = 0;                     // Error
volatile long Xc = 0;                     // Control output

// Moving average arrays
volatile int freq_buffer_A[SAMPLES_AVG] = {0};
volatile int freq_buffer_B[SAMPLES_AVG] = {0};
volatile int buffer_index = 0;
volatile int samples_collected = 0;

// Capture values for encoders
volatile unsigned int last1 = 0;         
volatile unsigned int last2 = 0;         
volatile int freq1 = 0;                  
volatile int freq2 = 0;                  
volatile float freq1_prev = 0;           
volatile float freq2_prev = 0;
volatile float slew = SLEW_RATE_LIMIT;

// Display formatting
char Xdshow[10], Xfshow[10], Xeshow[10], Xcshow[10];

void init_ports(void) {
    P7SEL = 0;                    // Configure Port 7 as GPIO
    P1DIR &= ~(BIT1 + BIT2 + BIT3);     // Set P1.1-3 as inputs
    P1REN |= BIT1 + BIT2 + BIT3;        // Enable pull-up/down
    P1OUT |= BIT1 + BIT2 + BIT3;        // Set as pull-up
    P4DIR |= BIT7;                       // P4.7 as output
    P1DIR |= BIT0;                       // P1.0 as output
    P2DIR |= BIT2 + BIT3;               // P2.2-3 as outputs
    P2OUT &= ~(BIT2 + BIT3);            // Initialize P2.2-3 low
}

void setupADC12(void) {
    ADC12CTL0 &= ~ADC12ENC;             
    ADC12CTL0 = ADC12SHT0_2 | ADC12ON;  
    ADC12CTL1 = ADC12SHP;               
    ADC12MCTL0 = ADC12INCH_12;          // P7.0
    ADC12CTL0 |= ADC12ENC;              
    ADC12IE |= ADC12IE0;                
    ADC12CTL2 = ADC12RES_1;             // 10-bit resolution
}

void init_SMCLK_20MHz(void) { 
    P5SEL |= BIT2 + BIT3;       
    P5SEL |= BIT4 + BIT5;
    
    __bis_SR_register(SCG0);     
    UCSCTL0 = 0x0000;           
    UCSCTL1 = DCORSEL_7;        
    UCSCTL2 = FLLD_0 + 610;     // 20MHz
    __bic_SR_register(SCG0);     
    
    do {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
        SFRIFG1 &= ~OFIFG;
    } while (SFRIFG1 & OFIFG);
    
    UCSCTL3 = SELREF__REFOCLK;
    UCSCTL4 = SELA__XT1CLK | SELS__DCOCLK | SELM__DCOCLK;
    UCSCTL5 = DIVS__1;
}

void setupPWM(void) {
    TA1CCR0 = PWM_TOP;          
    TA1CCTL1 = OUTMOD_7;        
    TA1CCR1 = 0;                
    TA1CTL = TASSEL_2 | MC_1;   
    P2DIR |= BIT0;              
    P2SEL |= BIT0;              
}

void setupTimerA2(void) {
    TA2CCTL0 = CCIE;                   
    TA2CCR0 = 200;                     // ~10 Hz sampling
    TA2CTL = TASSEL_1 | ID_0 | MC_1;   
}

void init_capture(void) {
    P1DIR &= ~(BIT2 + BIT3);       
    P1SEL |= BIT2 + BIT3;          
    
    TA0CTL = TASSEL_1 | MC_2;      // ACLK, Continuous mode
    
    TA0CCTL1 = CM_3 | CCIS_0 | CAP | CCIE;  // Capture both edges
    TA0CCTL2 = CM_3 | CCIS_0 | CAP | CCIE;  
    
    TA0CCTL1 &= ~CCIFG;
    TA0CCTL2 &= ~CCIFG;
}

void update_gain(void) {
    // Beregn gain kompensation baseret på spændingsændring
    Gm = 1023.0 / FREQ_MAX;  // PWM scaling factor
    Gc = G * (currentVoltage / NOMINAL_VOLTAGE);
}

void calculate_speed_setpoint(void) {
    if (USE_FREQ_SCALING) {
        // Option 1: Frekvens-baseret skalering
        Xd = ((long)adcResult * FREQ_MAX) / 1023;
    } else {
        // Option 2: Digital spændings-ækvivalent (standard)
        Xd = adcResult;
    }
}

void improve_stability(void) {
    static float prev_error = 0;
    static float output_prev = 0;
    float alpha = 0.7;
    
    update_gain();
    
    // Ændret error beregning
    Xe = Xd - Xf;
    
    Xe = (alpha * Xe) + ((1-alpha) * prev_error);
    prev_error = Xe;
    
    float output = Xd + (Gc * Xe);
    Xc = output_prev + slew * (output - output_prev);
    output_prev = Xc;
    
    if (Xc > PWM_TOP) {
        Xc = PWM_TOP;
    } else if (Xc < 0) {
        Xc = 0;
    }
}

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;   
    
    i2c_init();
    init_ports();
    setupADC12();
    init_SMCLK_20MHz();
    setupPWM();
    setupTimerA2();
    init_capture();
    
    ssd1306_init();
    ssd1306_clearDisplay();
    
    update_gain();  // Initialize gains
    
    __enable_interrupt();

    while(1) {
        Xd = adcResult;
        if (t_flag1 || t_flag2) {  // New encoder data
            t_flag1 = t_flag2 = 0;
            
            // Slew rate limiting på frekvenser
            freq1 = freq1_prev + slew * (freq1 - freq1_prev);
            freq2 = freq2_prev + slew * (freq2 - freq2_prev);
            
            // Opdater buffers
            freq_buffer_A[buffer_index] = freq1;
            freq_buffer_B[buffer_index] = freq2;
            buffer_index = (buffer_index + 1) % SAMPLES_AVG;
            
            if (samples_collected < SAMPLES_AVG) {
                samples_collected++;
            }
            
            // Beregn gennemsnit når vi har præcis 10 samples
            if (samples_collected == SAMPLES_AVG) {
                // Beregn ét samlet gennemsnit
                long sum = 0;
                for(int i = 0; i < SAMPLES_AVG; i++) {
                     sum += freq_buffer_A[i] + freq_buffer_B[i];
                }
                Xf = sum / (SAMPLES_AVG * 2);
            }
            
            freq1_prev = freq1;
            freq2_prev = freq2;
        }

        if (TA1CTL & TAIFG) {  // PWM period complete
            TA1CTL &= ~TAIFG;   
            
            calculate_speed_setpoint();
            improve_stability();
            TA1CCR1 = Xc;      // Update PWM
        }

        // Update display
        sprintf(Xdshow, "%04d", (int)Xd);
        sprintf(Xfshow, "%04d", (int)Xf);
        sprintf(Xeshow, "%04d", (int)Xe);
        sprintf(Xcshow, "%04d", (int)Xc);
        
        ssd1306_printText(0,0,"Xd:");
        ssd1306_printText(30,0,"     ");  
        ssd1306_printText(30,0,Xdshow);
        
        ssd1306_printText(0,2,"Xf:");
        ssd1306_printText(30,2,"     ");  
        ssd1306_printText(30,2,Xfshow);
        
        ssd1306_printText(0,4,"Xe:");
        ssd1306_printText(30,4,"     ");  
        ssd1306_printText(30,4,Xeshow);
        
        ssd1306_printText(0,6,"Xc:");
        ssd1306_printText(30,6,"     ");  
        ssd1306_printText(30,6,Xcshow);
    }
}

#pragma vector = TIMER0_A1_VECTOR
__interrupt void TIMER0_A1_ISR(void) {
    static int n1 = 0, n2 = 0;
    
    switch (TA0IV) {
        case 0x02:  // CCR1 - Encoder A
            TA0CCTL1 &= ~CCIFG;
            
            if (last1 > TA0CCR1)
                captured_value1 = 65535 - last1 + TA0CCR1;
            else
                captured_value1 = TA0CCR1 - last1;
                
            last1 = TA0CCR1;
            
            n1++;
            if (n1 == 2) {
                if (captured_value1 != 0) {
                    freq1 = (int)(32768 / captured_value1);
                }
                captured_value1 = 0;
                n1 = 0;
            }
            
            P2OUT ^= BIT2;
            t_flag1 = 1;
            break;
            
        case 0x04:  // CCR2 - Encoder B
            TA0CCTL2 &= ~CCIFG;
            
            if (last2 > TA0CCR2)
                captured_value2 = 65535 - last2 + TA0CCR2;
            else
                captured_value2 = TA0CCR2 - last2;
                
            last2 = TA0CCR2;
            
            n2++;
            if (n2 == 2) {
                if (captured_value2 != 0) {
                    freq2 = (int)(32768 / captured_value2);
                }
                captured_value2 = 0;
                n2 = 0;
            }
            
            P2OUT ^= BIT3;
            t_flag2 = 1;
            break;
    }
}

#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void) {
    ADC12CTL0 &= ~ADC12ENC;
    if (ADC12IFG != ADC12IFG0) {
        for (;;) {
        }
    }
    
    adcResult = ADC12MEM0;
    adc_flag = 1;
    ADC12CTL0 |= ADC12ENC;
}

#pragma vector = TIMER2_A0_VECTOR
__interrupt void TIMER2_A0_ISR(void) {
    TA2CCTL0 &= ~CCIFG;
    ADC12CTL0 |= ADC12SC;
}