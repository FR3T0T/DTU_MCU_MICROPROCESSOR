#include <Arduino.h>
#include <Energia.h>
#include "i2c.h"
#include "ssd1306.h"

// ===== PWM og Filter Konstanter =====
// SMCLK frekvens er 4 MHz (4,000,000 Hz)
#define SMCLK_FREQ      4000000.0f
// Timer periode er 4095 for 12-bit opløsning
#define TA1CCR0_VALUE   4095
// PWM frekvens = SMCLK_freq / Timer_periode ≈ 976.56 Hz
#define PWM_FREQ        (SMCLK_FREQ / TA1CCR0_VALUE)
// Vi ønsker filter frekvens 75x lavere end PWM frekvens
#define FILTER_RATIO    75
// Dette giver en filter cutoff på ca. 13 Hz
#define CUTOFF_FREQ     (PWM_FREQ / FILTER_RATIO)

// ===== RC Filter Design =====
// For et 13 Hz lavpasfilter med:
// C = 1µF
// R = 1/(2π * f * C)
// R = 1/(2 * 3.14159 * 13 * 0.000001) ≈ 12.2kΩ
// Vi bruger 12kΩ som standard værdi

// ===== Port Initialisering =====
void init_ports()
{
    // Konfigurer port select registre til digital I/O mode
    P6SEL = 0x00;      // Deaktiver special funktioner på Port 6
    P7SEL = 0;         // Deaktiver special funktioner på Port 7
  
    // Setup for I2C kommunikation (OLED display)
    pinMode(P1_1, INPUT_PULLUP);    // SDA pin med pullup
    digitalWrite(P1_1, HIGH);        // Sæt SDA høj
    
    // Setup for generelle I/O pins
    pinMode(P4_7, OUTPUT);          // LED output
    pinMode(P1_0, OUTPUT);          // Generel output
    pinMode(P2_1, INPUT_PULLUP);    // Input med pullup
    digitalWrite(P2_1, HIGH);       // Sæt høj
    
    // Konfigurer Port 6 for DIP switches (P6.0-P6.6)
    P6DIR = 0;         // Sæt hele Port 6 som input
    
    // Aktiver pull-down modstande på P6.0-P6.6
    // BIT0-BIT6 repræsenterer pins 0-6 (0x7F = 0b01111111)
    P6REN |= BIT6 | BIT5 | BIT4 | BIT3 | BIT2 | BIT1 | BIT0;  // Aktiver modstande
    // Sæt pull-down (0) i stedet for pull-up (1)
    // ~ operator inverterer alle bits, så vi trækker fra (clear) i stedet for at sætte
    P6OUT &= ~(BIT6 | BIT5 | BIT4 | BIT3 | BIT2 | BIT1 | BIT0);
    
    // Konfigurer P7.0 for sidste DIP switch bit
    P7REN |= BIT0;    // Aktiver modstand på P7.0
    P7OUT &= ~BIT0;   // Sæt pull-down på P7.0
    
    // Med denne konfiguration:
    // - Når en DIP switch er ON (lukket): Pin læser '1'
    // - Når en DIP switch er OFF (åben): Pin læser '0'
    // Dette giver en mere intuitiv mapping hvor ON = høj værdi
}

// ===== System Clock Setup =====
void init_SMCLK_XT2()
{
    WDTCTL = WDTPW|WDTHOLD;       // Stop watchdog timer
    
    // Konfigurer XT2 oscillator
    P5SEL |= BIT2+BIT3;           // Port select for XT2
    UCSCTL6 &= ~XT2OFF;           // Aktiver XT2
    UCSCTL4 |= SELA_2;            // ACLK = REFO
    UCSCTL4 |= SELS_5 + SELM_5;   // SMCLK = MCLK = XT2
    
    // Vent på at oscillator er stabil
    do {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
        SFRIFG1 &= ~OFIFG;
    } while (SFRIFG1&OFIFG);
}

// ===== PWM Setup =====
void init_pwm()
{
    // Konfigurer Timer A1 for edge-aligned PWM
    TA1CTL = TASSEL_2 |    // SMCLK kilde (4 MHz)
             MC_1 |         // Up mode
             TACLR;         // Clear timer

    // PWM periode og duty cycle
    TA1CCR0 = TA1CCR0_VALUE;     // Timer periode (4095)
    TA1CCR1 = TA1CCR0_VALUE/2;   // Start med 50% duty cycle
    
    // Setup PWM output mode
    TA1CCTL1 = OUTMOD_7;         // Reset/Set mode
    
    // Aktiver PWM output på P2.0
    pinMode(P2_0, OUTPUT);
    P2SEL |= BIT0;               // Vælg Timer funktion på P2.0
}

// ===== Duty Cycle Opdatering =====
void update_duty_cycle(uint8_t dip_value)
{
    // Konverter 8-bit (0-255) til 12-bit (0-4080)
    // ved at gange med 16 (bit shift 4 pladser)
    TA1CCR1 = dip_value << 4;    // dip_value * 16
}

// ===== Duty Cycle Beregning =====
uint8_t calculate_duty_cycle()
{
    // Beregn duty cycle i procent
    return (uint8_t)((((uint32_t)TA1CCR1 * 100) / TA1CCR0_VALUE));
}

// ===== Spændings Beregning =====
uint16_t calculate_voltage_mv(uint8_t duty_cycle)
{
    // Beregn forventet spænding i millivolt
    // 3.3V = 3300mV
    return (3300 * duty_cycle) / 100;
}

// ===== Hovedprogram =====
int main()
{
    // Basis initialisering
    WDTCTL = WDTPW | WDTHOLD;    // Stop watchdog
    init_SMCLK_XT2();            // Setup system clock
    init_ports();                // Initialiser porte
    init_pwm();                  // Setup PWM
 
    // Display initialisering
    i2c_init();                  // Start I2C
    ssd1306_init();             // Init OLED
    reset_diplay();             // Reset display
    ssd1306_clearDisplay();     // Clear screen
    
    // Variable definitioner
    char display_text[32];      // Buffer til display tekst
    uint8_t duty_cycle;         // Aktuel duty cycle
    uint8_t prev_dip_value = 0; // Tidligere DIP værdi
    uint8_t prev_duty_cycle = 0; // Tidligere duty cycle

    // Setup display layout
    ssd1306_printText(0, 0, "DIP:");
    ssd1306_printText(0, 2, "Duty:");
    ssd1306_printText(0, 4, "CCR1:");
    ssd1306_printText(0, 6, "DC V:");

    // Hovedløkke
    while(1)
    {
        // Læs DIP switches (sammensæt 8-bit værdi)
        uint8_t dip_value = (P6IN & 0x7F) | ((P7IN & BIT0) << 7);
        
        // Opdater kun hvis DIP værdi har ændret sig
        if(dip_value != prev_dip_value)
        {
            // Opdater PWM og beregn værdier
            update_duty_cycle(dip_value);
            duty_cycle = calculate_duty_cycle();
            
            // Vis DIP switch værdi
            sprintf(display_text, "%d   ", dip_value);
            ssd1306_printText(40, 0, display_text);
            
            // Vis duty cycle procent
            sprintf(display_text, "%d%%   ", duty_cycle);
            ssd1306_printText(40, 2, display_text);
            
            // Vis Timer A1 CCR1 værdi
            sprintf(display_text, "%d   ", TA1CCR1);
            ssd1306_printText(40, 4, display_text);
            
            // Beregn og vis spænding
            uint16_t voltage_mv = calculate_voltage_mv(duty_cycle);
            uint16_t volts = voltage_mv / 1000;      // Hele volt
            uint16_t mv = voltage_mv % 1000;         // Resterende millivolt
            sprintf(display_text, "%d.%02dV  ", volts, mv/10);
            ssd1306_printText(40, 6, display_text);
            
            // Gem værdier til næste sammenligning
            prev_dip_value = dip_value;
            prev_duty_cycle = duty_cycle;
        }
        
        // Vent på Timer A1 CCR1 match
        while(!(TA1CCTL1 & CCIFG));
        TA1CCTL1 &= ~CCIFG;  // Clear interrupt flag
        
        // Kort delay for bedre system respons
        __delay_cycles(50000);  // Ca. 12.5ms ved 4MHz
    }
}