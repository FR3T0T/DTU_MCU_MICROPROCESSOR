/* MSP430 Digital Ur med OLED Display
 * Dette program implementerer et digitalt ur med mulighed for at indstille tiden via DIP switches
 * Hardware krav:
 * - MSP430F5529 mikrocontroller
 * - SSD1306 OLED display
 * - 8 DIP switches (7 på P6.0-P6.6, 1 på P7.0)
 * - 1 trykknap på P2.1
 */

#include <msp430f5529.h>
#include "ssd1306.h"
#include "i2c.h"
#include <stdio.h>
#include <pins_energia.h>

// Globale variabler til tidsstyring og displayhåndtering
volatile unsigned char t_flag = 0;        // Flag til timing opdateringer
volatile unsigned char ss = 0;            // Sekunder (0-59)
volatile unsigned char mm = 0;            // Minutter (0-59)
volatile unsigned char hh = 0;            // Timer (0-23)
volatile unsigned char set_time_stage = 0; // Indstillingsfase (0=normal, 1=timer, 2=min, 3=sek)
volatile unsigned char prev_dip_value = 0; // Forrige DIP switch værdi
char tiden[16];                           // Buffer til tidsstreng

// Function prototypes
void reset_display(void);
void update_time_string(void);
unsigned char read_dip_switch(void);
void set_time(void);
void clear_display_line(unsigned char line);
void display_setting_instructions(void);

/* Timer A0 Interrupt Service Routine
 * Håndterer tidsoptælling og display opdateringer
 * Kører hvert sekund når uret er i normal tilstand
 */
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A(void)
{
    if (set_time_stage == 0)  // Normal ur-tilstand
    {
        t_flag = 1;
        ss++;                  // Increment sekunder
        if (ss == 60)         // Håndter overflow for sekunder
        {
            ss = 0;
            mm++;             // Increment minutter
            if (mm == 60)     // Håndter overflow for minutter
            {
                mm = 0;
                hh++;         // Increment timer
                if (hh == 24) // Håndter overflow for timer
                {
                    hh = 0;
                }
            }
        }
    }
    else  // Indstillingstilstand
    {
        t_flag = 1;  // Hold displayet opdateret
    }
    __bic_SR_register_on_exit(LPM0_bits);  // Exit low-power mode
}

/* Port 2 Interrupt Service Routine
 * Håndterer knaptryk for at skifte mellem indstillingstilstande
 * Debouncing implementeret med delay
 */
#pragma vector = PORT2_VECTOR
__interrupt void Port_2(void)
{
    if (P2IFG & BIT1)  // Tjek om det er vores knap der udløste interrupt
    {
        __delay_cycles(100000);  // Debouncing delay
        if (!(P2IN & BIT1))     // Bekræft at knappen stadig er trykket
        {
            set_time_stage = (set_time_stage + 1) % 4;  // Cykler gennem 0-3
            if (set_time_stage == 0)
            {
                ssd1306_clearDisplay();  // Ryd display ved afslutning af indstilling
            }
            else 
            {
                display_setting_instructions();  // Vis instruktioner for ny indstilling
            }
        }
        P2IFG &= ~BIT1;  // Ryd interrupt flag
    }
    __bic_SR_register_on_exit(LPM0_bits);
}

/* Rydder en specifik linje på displayet
 * Parametere:
 * line: Linjenummer der skal ryddes (0-7)
 */
void clear_display_line(unsigned char line)
{
    ssd1306_setPosition(0, line);
    ssd1306_printText(0, line, "                ");  // 16 mellemrum
}

/* Viser instruktioner for den aktuelle indstillingstilstand
 * Opdaterer display baseret på set_time_stage
 */
void display_setting_instructions(void)
{
    ssd1306_clearDisplay();
    switch(set_time_stage)
    {
        case 1:
            ssd1306_printText(0, 0, "Indstil Timer:");
            break;
        case 2:
            ssd1306_printText(0, 0, "Indstil Minutter:");
            break;
        case 3:
            ssd1306_printText(0, 0, "Indstil Sekunder:");
            break;
    }
}

/* Opdaterer tidsdisplayet
 * Normal tilstand: Viser HH:MM:SS
 * Indstillingstilstand: Viser den aktuelle værdi der indstilles
 */
void update_time_string(void)
{
    if (set_time_stage == 0)  // Normal ur-visning
    {
        sprintf(tiden, "%02d:%02d:%02d", hh, mm, ss);
        ssd1306_printText(0, 0, tiden);
    }
    else  // Indstillingstilstand
    {
        unsigned char current_value;
        switch(set_time_stage)
        {
            case 1: current_value = hh; break;
            case 2: current_value = mm; break;
            case 3: current_value = ss; break;
            default: return;
        }
        sprintf(tiden, "Vaerdi: %02d", current_value);
        ssd1306_printText(0, 2, tiden);
    }
}

/* Læser værdien fra DIP switches
 * Returnerer: 8-bit værdi fra switches (inverteret da switches er active low)
 */
unsigned char read_dip_switch(void)
{
    unsigned char value = 0;
    value = P6IN & 0x7F;                   // Læs P6.0 til P6.6
    value |= (P7IN & BIT0) ? 0x80 : 0x00;  // Læs P7.0
    return ~value;                          // Inverter værdien
}

/* Håndterer tidsindstilling via DIP switches
 * Validerer input og sætter nye værdier for timer, minutter eller sekunder
 * Viser fejlmeddelelser ved ugyldige værdier
 */
void set_time(void)
{
    unsigned char dip_value = read_dip_switch();
    
    if (dip_value != prev_dip_value)  // Kun opdater hvis værdien er ændret
    {
        prev_dip_value = dip_value;
        
        switch (set_time_stage)
        {
            case 1:  // Indstil timer
                if (dip_value > 23)
                {
                    hh = 23;  // Begræns til maksimal værdi
                    ssd1306_printText(0, 4, "FEJL: Max vaerdi 23");
                    ssd1306_printText(0, 5, "Vaerdi saettes til 23");
                }
                else
                {
                    hh = dip_value;
                    clear_display_line(3);
                }
                break;
                
            case 2:  // Indstil minutter
                if (dip_value > 59)
                {
                    mm = 59;
                    ssd1306_printText(0, 4, "FEJL: Max vaerdi 59");
                    ssd1306_printText(0, 5, "Vaerdi saettes til 59");
                }
                else
                {
                    mm = dip_value;
                    clear_display_line(3);
                }
                break;
                
            case 3:  // Indstil sekunder
                if (dip_value > 59)
                {
                    ss = 59;
                    ssd1306_printText(0, 4, "FEJL: Max vaerdi 59");
                    ssd1306_printText(0, 5, "Vaerdi saettes til 59");
                }
                else
                {
                    ss = dip_value;
                    clear_display_line(3);
                }
                break;
        }
        t_flag = 1;  // Opdater display
    }
}

/* Nulstiller display og viser aktuel tid */
void reset_display(void)
{
    ssd1306_clearDisplay();
    update_time_string();
}

/* Hovedprogram
 * Initialiserer hardware og kører hovedloop
 */
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;  // Stop watchdog timer

    // Initialiser I2C og OLED display
    i2c_init();
    ssd1306_init();

    // Konfigurer DCO til 1 MHz
    UCSCTL3 = SELREF_2;         // Sæt DCO FLL reference = REFO
    UCSCTL4 |= SELA_2;          // Sæt ACLK = REFO
    __bis_SR_register(SCG0);    // Deaktiver FLL
    UCSCTL0 = 0x0000;           // Sæt laveste mulige DCOx, MODx
    UCSCTL1 = DCORSEL_2;        // Vælg DCO range 1MHz
    UCSCTL2 = FLLD_1 + 31;      // Sæt DCO Multiplier for 1MHz
    __bic_SR_register(SCG0);    // Aktiver FLL

    __delay_cycles(250000);      // Vent på at DCO stabiliserer

    // Timer konfiguration (~1 sekund interrupt)
    TA0CTL = TASSEL_2 | ID_3 | MC_1; 
    TA0CCR0 = 16370;  
    TA0CCTL0 = CCIE; 
    TA0EX0 = TAIDEX_7; 

    // Konfigurer LED (til debug)
    P1DIR |= BIT0;
    P1OUT &= ~BIT0;

    // Konfigurer trykknap med pull-up
    P2DIR &= ~BIT1;  // Input
    P2REN |= BIT1;   // Enable pull-up/down
    P2OUT |= BIT1;   // Pull-up
    P2IE |= BIT1;    // Enable interrupt
    P2IES |= BIT1;   // Interrupt på falling edge

    // Konfigurer DIP switches med pull-up
    P6DIR &= ~0x7F;  // P6.0-P6.6 som input
    P7DIR &= ~BIT0;  // P7.0 som input
    P6REN |= 0x7F;   // Enable pull-up/down
    P7REN |= BIT0;
    P6OUT |= 0x7F;   // Set pull-up
    P7OUT |= BIT0;

    __bis_SR_register(GIE);  // Aktiver globale interrupts

    reset_display();

    // Hovedloop
    while (1)
    {
        if (set_time_stage > 0)  // Hvis i indstillingstilstand
        {
            set_time();  // Tjek for DIP switch ændringer
        }
        
        if (t_flag)  // Tid til display opdatering
        {
            t_flag = 0;
            update_time_string();
        }
        
        __bis_SR_register(LPM0_bits + GIE);  // Gå i low-power mode
    }
    return 0;
}