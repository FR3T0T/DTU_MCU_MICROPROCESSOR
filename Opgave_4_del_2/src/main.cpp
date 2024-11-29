/****************************************************************************
* Includes og Header Files
****************************************************************************/
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
#define MAX_DIGITS 8          // Maksimalt antal cifre for numeriske værdier
#define CHAR_WIDTH 6          // Bredden af hver karakter i pixels
#define LABEL_X_POS 0        // Start X position for labels
#define VALUE_X_POS 72       // Start X position for værdier

// Display linje positioner
#define FREQ1_LINE 0         // Linje for frekvens 1
#define CV1_LINE 1           // Linje for captured value 1
#define PULSE1_LINE 2        // Linje for pulse count 1
#define FREQ2_LINE 4         // Linje for frekvens 2
#define CV2_LINE 5           // Linje for captured value 2
#define PULSE2_LINE 6        // Linje for pulse count 2

/****************************************************************************
* Globale Variable
****************************************************************************/
// Encoder måleværdier
volatile uint16_t captured_value1 = 0;  // Captured værdi fra encoder 1
volatile uint16_t captured_value2 = 0;  // Captured værdi fra encoder 2
volatile uint8_t t_flag1 = 0;          // Flag for ny måling på encoder 1
volatile uint8_t t_flag2 = 0;          // Flag for ny måling på encoder 2

/****************************************************************************
* Hjælpefunktioner til String Håndtering
****************************************************************************/
// Konverterer et tal til en string
void number_to_string(uint32_t value, char* str) {
    // Håndter special case for 0
    if (value == 0) {
        str[0] = '0';
        str[1] = '\0';
        return;
    }
    
    // Konverter tal til string, et ciffer ad gangen
    char temp[MAX_DIGITS + 1];
    uint8_t i = 0;
    while (value > 0 && i < MAX_DIGITS) {
        temp[i++] = '0' + (value % 10);
        value /= 10;
    }
    
    // Vend string om (cifre er baglæns)
    uint8_t j;
    uint8_t len = i;
    for (j = 0; j < i; j++) {
        str[j] = temp[len - 1 - j];
    }
    str[j] = '\0';  // Afslut med null-terminator
}

/****************************************************************************
* System Initialisering
****************************************************************************/
// Initialiserer system clock til 20MHz
void init_SMCLK_20MHz(void) {
    P5SEL |= BIT2 + BIT3 + BIT4 + BIT5;  // Vælg XT2 til SMCLK
    
    // Konfigurer DCO til 20 MHz
    __bis_SR_register(SCG0);              // Deaktiver FLL
    UCSCTL0 = 0x0000;                     // Laveste DCOx og MODx
    UCSCTL1 = DCORSEL_7;                  // Vælg DCO range
    UCSCTL2 = FLLD_0 + 610;               // Sæt multiplier for 20MHz

    __bic_SR_register(SCG0);              // Aktiver FLL igen

    // Vent på oscillator stabilitet
    do {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
        SFRIFG1 &= ~OFIFG;
    } while (SFRIFG1 & OFIFG);

    // Konfigurer clock kilder
    UCSCTL3 = SELREF__REFOCLK;
    UCSCTL4 = SELA__XT1CLK | SELS__DCOCLK | SELM__DCOCLK;
    UCSCTL5 = DIVS__1;
}

// Initialiserer Timer A0 til capture mode
void init_timerA0_capture(void) {
    // Timer A0 konfiguration
    TA0CTL = TASSEL_1     // ACLK (32768 Hz)
           + MC_2         // Continuous mode
           + ID_0;        // Input divider /2

    // Setup capture pins
    P1DIR &= ~(BIT2 | BIT3);    // P1.2 og P1.3 som inputs
    P1SEL |= (BIT2 | BIT3);     // Vælg timer funktionalitet

    // Capture konfiguration for P1.2 (Encoder 1)
    TA0CCTL1 = CM_1       // Capture på stigende flanke
             + CCIS_0     // CCIxA input signal
             + CAP        // Capture mode
             + CCIE       // Capture interrupt enable
             + SCS;       // Synkroniseret capture

    // Capture konfiguration for P1.3 (Encoder 2)
    TA0CCTL2 = CM_1 + CCIS_0 + CAP + CCIE + SCS;

    // LED setup for debugging
    P6DIR |= BIT0 | BIT1 | BIT2;    // Setup LED pins
    P6OUT |= BIT0;                   // Tænd første LED
}

/****************************************************************************
* Hovedfunktioner
****************************************************************************/
// Beregner frekvensen ud fra capture værdi
float calculate_frequency(uint16_t capture_value) {
    if (capture_value == 0 || capture_value > 32768) return 0.0f;
    return 32768.0f / (float)capture_value;
}

// Opdaterer display med alle værdier
void update_display(float freq1, float freq2) {
    char str[32];
    static uint32_t pulse_count1 = 0;
    static uint32_t pulse_count2 = 0;
    
    // Opdater pulse counts kun når der er nye data
    if (t_flag1) pulse_count1++;
    if (t_flag2) pulse_count2++;
    
    // Vis kun frekvenser hvis de er gyldige (større end 0)
    if (freq1 > 0) {
        int freq1_whole = (int)freq1;
        int freq1_frac = (int)((freq1 - freq1_whole) * 10);
        snprintf(str, sizeof(str), "F1: %d.%dHz    ", freq1_whole, freq1_frac);
        ssd1306_printText(LABEL_X_POS, FREQ1_LINE, str);
    }
    
    if (freq2 > 0) {
        int freq2_whole = (int)freq2;
        int freq2_frac = (int)((freq2 - freq2_whole) * 10);
        snprintf(str, sizeof(str), "F2: %d.%dHz    ", freq2_whole, freq2_frac);
        ssd1306_printText(LABEL_X_POS, FREQ2_LINE, str);
    }
}

/****************************************************************************
* Interrupt Service Rutiner
****************************************************************************/
#pragma vector=TIMER0_A1_VECTOR
__interrupt void Timer_A1_ISR(void) {
    static unsigned int last1 = 0;
    static unsigned int last2 = 0;
    static int i=0;
    switch(TA0IV) {
        case 0x02:  // CCR1 interrupt (P1.2)
           
           if(last1>TA0CCR1)
             captured_value1 = TA0CCR1 +(65535 -last1);  //corunter turn arround
           else
            captured_value1 = TA0CCR1 - last1;
            last1 = TA0CCR1;
            P6OUT |= BIT1;      // Tænd LED for encoder 1 aktivitet
            P2OUT^=BIT2;


            t_flag1 = 1;
            break;

        case 0x04:  // CCR2 interrupt (P1.3)
            captured_value2 = TA0CCR2 - last2;
            last2 = TA0CCR2;
            P6OUT |= BIT2;      // Tænd LED for encoder 2 aktivitet
                 P2OUT^=BIT3;
            t_flag2 = 1;
            break;
    }
}

/****************************************************************************
* Main Program
****************************************************************************/
int main(void) {
    WDTCTL = WDTPW | WDTHOLD;     
    
    init_SMCLK_20MHz();
    __delay_cycles(300000);
    
    i2c_init();
    __delay_cycles(50000);
    
    ssd1306_init();
    __delay_cycles(50000);
    
    // Ryd display én gang ved opstart
    ssd1306_clearDisplay();
    __delay_cycles(50000);
    
    // Initialiser frekvensværdier til 0
    float freq1 = 0.0f, freq2 = 0.0f;
    
    init_timerA0_capture();
    __bis_SR_register(GIE);
    
    P2DIR|=BIT2|BIT3;
    
    // Hovedløkke - opdater kun når der er nye data
    while(1) {
        if (t_flag1) {
            freq1 = calculate_frequency(captured_value1);
            t_flag1 = 0;
            P6OUT &= ~BIT1;
            update_display(freq1, freq2);  // Opdater kun ved nye data
        }
        
        if (t_flag2) {
            freq2 = calculate_frequency(captured_value2);
            t_flag2 = 0;
            P6OUT &= ~BIT2;
            update_display(freq1, freq2);  // Opdater kun ved nye data
        }
        
        P6OUT ^= BIT0;
        __delay_cycles(50000);
    }
    
    return 0;
}