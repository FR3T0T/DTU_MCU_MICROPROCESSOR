/****************************************************************************
 * Co-pilot, Chat-GPT & Claude AI er blevt brugt som hjælp til fejlfinding
 * 
 * Opgave 4, del 3
 * 
 * MSP430 Motor Kontrol System
 * 
 * Dette program implementerer et motor kontrol system med feedback fra to encodere.
 * Systemet bruger PWM til motorstyring og viser status på et SSD1306 OLED display.
 * Mikrocontrolleren kører på 20MHz med præcis hastighedsregulering.
 * 
 * Hardware forbindelser:
 * Encodere:
 * - Encoder A Signal: P1.2 (Timer A0.1 capture input)
 * - Encoder B Signal: P1.3 (Timer A0.2 capture input)
 * 
 * Motor styring:
 * - PWM Output: P2.0 (Timer A1.1 output)
 * - Motor Enable: P2.2, P2.3 (GPIO outputs)
 * 
 * Display (I2C):
 * - SCL: P3.0
 * - SDA: P3.1
 * 
 * ADC Input:
 * - Hastighedsreference: P7.0 (ADC12 kanal 12)
 * 
 * Debug outputs:
 * - P2.2: Toggle ved Encoder A interrupt
 * - P2.3: Toggle ved Encoder B interrupt
 ****************************************************************************/
#include <stdio.h>          // Standard C bibliotek - bruges til sprintf funktion
#include "i2c.h"            // Driver til I2C kommunikation - bruges til at snakke med OLED displayet
#include "ssd1306.h"        // Driver til OLED display - giver funktioner til at vise tekst og grafik
#include <msp430f5529.h>    // MSP430 mikrocontroller definitioner - giver adgang til alle hardware funktioner
#include <stdint.h>         // Standard integer typer - sikrer præcise størrelser på variable (8-bit, 16-bit, etc.)

/****************************************************************************
* Konstanter og Konfiguration
****************************************************************************/
// System parametre
#define FREQ_MAX 500            // Maksimal encoder frekvens i Hz
#define MAX_DUTY_CYCLE 1023     // 10-bit PWM opløsning
#define PWM_TOP 1023            // Maksimal PWM værdi

// Timer og sampling parametre
#define TIMER_CLK 32768         // ACLK frekvens (32.768 kHz)
#define SAMPLES_AVG 1           // Antal samples for midling

// Motor parametre
#define NOMINAL_VOLTAGE 9.0     // Motor nominel spænding
#define SLEW_RATE_LIMIT 0.5     // Hastighedsbegrænsning for blød start/stop

/****************************************************************************
* Globale Variable
****************************************************************************/
// ADC variable
volatile int adcResult = 0;            // ADC konverteringsresultat
volatile char adc_flag = 0;            // Flag for ny ADC værdi

// Reguleringsvariable
float G = 0.5;                         // Proportional reguleringsforstærkning
volatile float Gm;                     // Motor gain scaling faktor
volatile float Gc;                     // Spændingskorrigeret forstærkning
volatile float currentVoltage = 12.0;  // Aktuel forsyningsspænding

// Encoder målinger
volatile unsigned int captured_value1 = 0;  // Timer capture for encoder A
volatile unsigned int captured_value2 = 0;  // Timer capture for encoder B
volatile char t_flag1 = 0;                  // Ny måling flag encoder A
volatile char t_flag2 = 0;                  // Ny måling flag encoder B

// Reguleringsværdier
volatile long Xd = 0;                  // Ønsket hastighed (setpoint)
volatile long Xf_A = 0;                // Målt hastighed encoder A
volatile long Xf_B = 0;                // Målt hastighed encoder B
volatile long Xf = 0;                  // Gennemsnitlig hastighed
volatile long Xe = 0;                  // Reguleringsfejl
volatile long Xc = 0;                  // Kontrolsignal (PWM værdi)

/****************************************************************************
* Buffer System til Midling
****************************************************************************/
// Cirkulære buffers til frekvensværdier
volatile int freq_buffer_A[SAMPLES_AVG] = {0};  // Buffer for encoder A
volatile int freq_buffer_B[SAMPLES_AVG] = {0};  // Buffer for encoder B
volatile int buffer_index = 0;                  // Aktuel buffer position
volatile int samples_collected = 0;             // Antal indsamlede samples

// Encoder frekvensberegning
volatile unsigned int last1 = 0;         // Sidste capture værdi A
volatile unsigned int last2 = 0;         // Sidste capture værdi B
volatile int freq1 = 0;                  // Beregnet frekvens A
volatile int freq2 = 0;                  // Beregnet frekvens B
volatile float freq1_prev = 0;           // Forrige frekvens A
volatile float freq2_prev = 0;           // Forrige frekvens B
volatile float slew = SLEW_RATE_LIMIT;   // Aktuel slew rate

// Display formatering
char Xdshow[10], Xfshow[10], Xeshow[10], Xcshow[10];

/****************************************************************************
* System Initialisering
****************************************************************************/
void init_ports(void)
{
    P7SEL = 0;                          // Port P7.0 som GPIO for ADC input
    P1DIR &= ~(BIT2 + BIT3);            // P1.2 & P1.3 som encoder inputs
    P1REN |= BIT2 + BIT3;               // Pullup modstande på P1.2-P1.3
    P1OUT |= BIT2 + BIT3;               // Aktiver pullup på P1.2-P1.3
    P4DIR |= BIT7;                      // P4.7 som status LED output
    P1DIR |= BIT0;                      // P1.0 som status LED output
    P2DIR |= BIT2 + BIT3;               // P2.2 og P2.3 som motor kontrol outputs
    P2OUT &= ~(BIT2 + BIT3);            // P2.2 og P2.3 startes i lav tilstand
}

void setupADC12(void)
{
    ADC12CTL0 &= ~ADC12ENC;             // Disable for konfiguration
    ADC12CTL0 = ADC12SHT0_2 | ADC12ON;  // Sample tid og power on
    ADC12CTL1 = ADC12SHP;               // Sample timing kontrol
    ADC12MCTL0 = ADC12INCH_12;          // ADC input kanal
    ADC12CTL0 |= ADC12ENC;              // Enable konvertering
    ADC12IE |= ADC12IE0;                // Enable interrupt
    ADC12CTL2 = ADC12RES_1;             // 10-bit opløsning
}

/****************************************************************************
* Timere og PWM Konfiguration
****************************************************************************/
void init_SMCLK_20MHz(void)
{ 
    P5SEL |= BIT2 + BIT3;               // XT2 krystal pins
    P5SEL |= BIT4 + BIT5;               // XT2 krystal pins
    
    __bis_SR_register(SCG0);            // Disable FLL
    UCSCTL0 = 0x0000;                   // Reset tuning
    UCSCTL1 = DCORSEL_7;                // DCO frekvensområde
    UCSCTL2 = FLLD_0 + 610;             // DCOCLK = 20MHz
    __bic_SR_register(SCG0);            // Enable FLL
    
    // Vent på stabil oscillator
    do {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
        SFRIFG1 &= ~OFIFG;
    } while (SFRIFG1 & OFIFG);
    
    // Clock konfiguration
    UCSCTL3 = SELREF__REFOCLK;         // FLL reference
    UCSCTL4 = SELA__XT1CLK |           // ACLK = XT1
              SELS__DCOCLK |           // SMCLK = DCO
              SELM__DCOCLK;            // MCLK = DCO
    UCSCTL5 = DIVS__1;                 // SMCLK deling
}

/****************************************************************************
* Motor Kontrol Funktioner
****************************************************************************/
void setupPWM(void)
{
    TA1CCR0 = PWM_TOP;                 // PWM periode
    TA1CCTL1 = OUTMOD_7;               // PWM output mode
    TA1CCR1 = 0;                       // Start med 0% duty cycle
    TA1CTL = TASSEL_2 | MC_1;          // SMCLK kilde, up mode
    P2DIR |= BIT0;                     // PWM output pin
    P2SEL |= BIT0;                     // Timer funktion
}

void setupTimerA2(void)
{
    TA2CCTL0 = CCIE;                   // Interrupt enable
    TA2CCR0 = 200;                     // Periode for 10 Hz sampling
    TA2CTL = TASSEL_1 | ID_0 | MC_1;   // ACLK, ingen deling, up mode
}

/****************************************************************************
* Encoder Interface
****************************************************************************/
void init_capture(void)
{
    P1DIR &= ~(BIT2 + BIT3);           // Encoder inputs
    P1SEL |= BIT2 + BIT3;              // Timer capture funktion
    
    TA0CTL = TASSEL_1 | MC_2;          // ACLK, continuous mode
    
    // Capture konfiguration
    TA0CCTL1 = CM_3 |                  // Både stigende og faldende flanke
               CCIS_0 |                // CCIxA input 
               CAP |                   // Capture mode
               CCIE;                   // Interrupt enable
               
    TA0CCTL2 = CM_3 | CCIS_0 | CAP | CCIE;  // Same for kanal 2
    
    // Clear interrupt flags
    TA0CCTL1 &= ~CCIFG;
    TA0CCTL2 &= ~CCIFG;
}

/****************************************************************************
* Regulering og Stabilitet
****************************************************************************/
void update_gain(void) 
{
    // Beregn system forstærkning
    Gm = 1023.0 / FREQ_MAX;                         // PWM til frekvens skalering
    Gc = G * (currentVoltage / NOMINAL_VOLTAGE);    // Spændingskompensation
}

void improve_stability(void)
{
    static float prev_error = 0;        // Forrige reguleringsfejl
    static float output_prev = 0;       // Forrige output
    float alpha = 0.7;                  // Filterkonstant
    
    update_gain();
    
    // Reguleringsalgoritme
    Xe = Xd - Xf;                      // Beregn fejl
    Xe = (alpha * Xe) +                // Fejlfiltrering
         ((1-alpha) * prev_error);     
    prev_error = Xe;
    
    // Beregn styresignal med slew rate begrænsning
    float output = Xd + (Gc * Xe);
    Xc = output_prev + slew * (output - output_prev);
    output_prev = Xc;
    
    // Begræns output
    if (Xc > PWM_TOP)
    {
        Xc = PWM_TOP;
    }
    else if (Xc < 0)
    {
        Xc = 0;
    }
}

/****************************************************************************
* Hovedprogram
****************************************************************************/
int main(void)
{
    // System initialisering
    WDTCTL = WDTPW | WDTHOLD;           // Stop watchdog
    
    i2c_init();                         // I2C til display
    init_ports();                       // GPIO setup
    setupADC12();                       // ADC konfiguration
    init_SMCLK_20MHz();                 // System clock
    setupPWM();                         // Motor PWM
    setupTimerA2();                     // Sampling timer
    init_capture();                     // Encoder capture
    
    // Display initialisering
    ssd1306_init();
    ssd1306_clearDisplay();
    
    update_gain();                      // Initial gain beregning
    
    __enable_interrupt();               // Global interrupt enable

    // Hovedløkke
    while(1)
    {
        // Encoder databehandling
        if (t_flag1 || t_flag2) 
        {
            t_flag1 = t_flag2 = 0;
            
            // Frekvens filtrering
            freq1 = freq1_prev + slew * (freq1 - freq1_prev);
            freq2 = freq2_prev + slew * (freq2 - freq2_prev);
            
            // Buffer opdatering
            freq_buffer_A[buffer_index] = freq1;
            freq_buffer_B[buffer_index] = freq2;
            buffer_index = (buffer_index + 1) % SAMPLES_AVG;
            
            if (samples_collected < SAMPLES_AVG) 
            {
                samples_collected++;
            }
            
            // Beregn middelværdi
            if (samples_collected == SAMPLES_AVG) 
            {
                long sum = 0;
                for(int i = 0; i < SAMPLES_AVG; i++) 
                {
                     sum += freq_buffer_A[i] + freq_buffer_B[i];
                }
                Xf = sum / (SAMPLES_AVG * 2);
            }
            
            freq1_prev = freq1;
            freq2_prev = freq2;
        }

        // PWM opdatering
        if (TA1CTL & TAIFG)
        {
            TA1CTL &= ~TAIFG;
            Xd = adcResult;
            improve_stability();
            TA1CCR1 = Xc;
        }

        // Display opdatering
        sprintf(Xdshow, "%04d", (int)Xd);
        sprintf(Xfshow, "%04d", (int)Xf);
        sprintf(Xeshow, "%04d", (int)Xe);
        sprintf(Xcshow, "%04d", (int)Xc);
        
        // Display opdatering fortsætter
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

/****************************************************************************
* Interrupt Service Rutiner
****************************************************************************/

/**
 * Timer0_A1 Interrupt Service Rutine
 */
#pragma vector = TIMER0_A1_VECTOR
__interrupt void TIMER0_A1_ISR(void)
{
    static int n1 = 0, n2 = 0;  // Tællere for antal captures
    
    switch (TA0IV)
    {
        case 0x02:               // CCR1 - Encoder A
            TA0CCTL1 &= ~CCIFG;  // Clear interrupt flag
            
            // Beregn tid mellem pulser med overflow håndtering
            if (last1 > TA0CCR1)
                captured_value1 = 65535 - last1 + TA0CCR1;
            else
                captured_value1 = TA0CCR1 - last1;
                
            last1 = TA0CCR1;  // Gem aktuel værdi til næste gang
            
            // Frekvensberegning hver anden capture (komplet periode)
            n1++;
            if (n1 == 2)
            {
                if (captured_value1 != 0)
                {
                    freq1 = (int)(32768 / captured_value1);
                }
                captured_value1 = 0;
                n1 = 0;
            }
            
            P2OUT ^= BIT2;  // Toggle debug output
            t_flag1 = 1;    // Indiker ny måling klar
            break;
            
        case 0x04:  // CCR2 - Encoder B
            TA0CCTL2 &= ~CCIFG;  // Clear interrupt flag
            
            // Beregn tid mellem pulser med overflow håndtering
            if (last2 > TA0CCR2)
                captured_value2 = 65535 - last2 + TA0CCR2;
            else
                captured_value2 = TA0CCR2 - last2;
                
            last2 = TA0CCR2;  // Gem aktuel værdi til næste gang
            
            // Frekvensberegning hver anden capture (komplet periode)
            n2++;
            if (n2 == 2)
            {
                if (captured_value2 != 0)
                {
                    freq2 = (int)(32768 / captured_value2);
                }
                captured_value2 = 0;
                n2 = 0;
            }
            
            P2OUT ^= BIT3;  // Toggle debug output
            t_flag2 = 1;    // Indiker ny måling klar
            break;
    }
}

/**
 * ADC12 Interrupt Service Rutine
 */
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)
{
    ADC12CTL0 &= ~ADC12ENC;  // Disable ADC
    
    // Verificer korrekt interrupt kilde
    if (ADC12IFG != ADC12IFG0)
    {
        for (;;)
        {  // Fejlhåndtering - stop hvis forkert interrupt
        }
    }
    
    adcResult = ADC12MEM0;  // Gem konverteringsresultat
    adc_flag = 1;           // Indiker ny værdi klar
    ADC12CTL0 |= ADC12ENC;  // Genaktiver ADC
}

/**
 * Timer2_A0 Interrupt Service Rutine
 */
#pragma vector = TIMER2_A0_VECTOR
__interrupt void TIMER2_A0_ISR(void)
{
    TA2CCTL0 &= ~CCIFG;     // Clear interrupt flag
    ADC12CTL0 |= ADC12SC;   // Start ny ADC konvertering
}