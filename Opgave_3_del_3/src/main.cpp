/*******************************************************************************
* Co-Pilot, Chat-GPT & Claude AI er blevet brugt til fejlfinding og hjælp.
*
* DEL 3
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
#include <msp430.h>          // MSP430 hardware definitioner
#include <stdint.h>          // Standard integer typer
#include <string.h>          // String manipulation funktioner
#include <stdio.h>           // Standard I/O funktioner
#include "i2c.h"            // I2C kommunikation driver
#include "ssd1306.h"        // OLED display driver

/****************************************************************************
* System Konstanter og Konfiguration
****************************************************************************/
// ADC og Reference Parametre
#define VFS 3.3f            // System reference spænding (3.3V)
#define ADC_BITS 10         // ADC opløsning (10-bit)
#define ADC_MAX_VALUE (1 << ADC_BITS)  // Maksimal ADC værdi (4096)

// Display Formattering
#define MAX_DIGITS 6        // Maximum antal cifre for numerisk display
#define VOLT_DIGITS 8       // Buffer størrelse for spændingsvisning ("x.xxx V\0")
#define DUTY_DIGITS 8       // Buffer størrelse for duty cycle ("xxx.xx %\0")
#define CHAR_WIDTH 6        // Pixel bredde per karakter

// Signal Processing
#define FILTER_SIZE 16      // Moving average filter længde
                            // Større værdi = mere udglatning
#define ADC_THRESHOLD 3     // Minimum ændring før display 
                            // Reducerer display flimmer
#define TA1CCR0_VALUE 1023  // PWM periode (10-bit)

/****************************************************************************
* Display Layout Konfiguration
****************************************************************************/
#define LABEL_X_POS 0       // Start position for labels
#define VALUE_X_POS 72      // Start position for værdier (12 * 6 pixels)
#define LINE_SPACING 2      // Vertikal afstand mellem linjer
#define ADC_Y_POS 0         // Y-position for ADC værdier
#define PWM_Y_POS (ADC_Y_POS + LINE_SPACING)   // Y-position for PWM data
#define VOLT_Y_POS (PWM_Y_POS + LINE_SPACING)  // Y-position for spænding


// System Konstanter
#define TARGET_VOLTAGE 1.0f  // Ønsket output spænding (V)
#define TARGET_ADC ((uint16_t)((TARGET_VOLTAGE/VFS) * TA1CCR0_VALUE))  // Konverteret til 12-bit værdi

/****************************************************************************
* Globale Variable og Buffere
****************************************************************************/
// ADC Interrupt Variable
volatile uint16_t adc_data = 0;    // Holder seneste ADC måling
volatile uint8_t adc_flag = 0;     // Signalerer ny data til main loop

// Moving Average Filter Data
static uint16_t filter_buffer[FILTER_SIZE] = {0};  // Cirkulær sample buffer
static uint8_t filter_index = 0;     // Aktuel buffer position
static uint32_t filter_sum = 0;      // Løbende sum af samples
static uint8_t filter_count = 0;     // Antal aktive samples

// Del 3
volatile float digital_Vo = 0;
volatile float digital_Ve_error = 0;
volatile uint16_t digital_Vd_volts = 0;
volatile float compVal = 0;

/****************************************************************************
* Data Strukturer til Display Formattering
****************************************************************************/
// Generel Numerisk Display
typedef struct
{
    char text[MAX_DIGITS + 1];  // Text buffer med null terminator
    uint8_t length;             // Aktuel text længde
} DisplayValue;

// Spændings Display Format
typedef struct
{
    char text[VOLT_DIGITS];     // Buffer til spændingsværdi
    uint8_t length;             // Aktuel længde
} VoltageDisplay;

// Duty Cycle Display Format
typedef struct
{
    char text[DUTY_DIGITS];     // Buffer til duty cycle
    uint8_t length;             // Aktuel længde
} DutyDisplay;

/****************************************************************************
* Signal Processing Funktioner
****************************************************************************/
// Moving Average Filter Implementation
uint16_t filter_adc(uint16_t new_sample)
{
    // Fjern ældste sample fra summen før tilføjelse af nyt
    filter_sum -= filter_buffer[filter_index];
    
    // Tilføj nyt sample til bufferen
    filter_buffer[filter_index] = new_sample;
    filter_sum += new_sample;
    
    // Opdater buffer index med wrap-around
    filter_index = (filter_index + 1) % FILTER_SIZE;
    
    // Håndter opstartsperiode hvor bufferen fyldes
    if (filter_count < FILTER_SIZE) filter_count++;
    
    // Returner gennemsnittet af alle samples
    return (uint16_t)(filter_sum / filter_count);
}

// Absolut Differens Beregning
uint16_t diff(uint16_t a, uint16_t b)
{
    // Returner positiv forskel mellem to værdier
    // Bruges til at detektere signifikante ændringer
    return (a > b) ? (a - b) : (b - a);
}

// ADC til Spændings Konvertering
float adc_to_voltage(uint16_t adc_value)
{
    // Konverter 12-bit ADC værdi til aktuel spænding
    // Bruger VFS (3.3V) som reference
    return (float)adc_value * VFS / ADC_MAX_VALUE;
}

/****************************************************************************
* Hardware Initialisering
****************************************************************************/
// ADC System Setup
void adc_init(void)
{
    ADC12CTL0 &= ~ADC12ENC;            // Disable ADC for konfiguration
    
    // ADC12CTL0 Register Setup
    ADC12CTL0 = ADC12SHT0_3            // Sample-and-hold time = 32 cycles
              | ADC12ON;                // Tænd for ADC12
              
    // ADC12CTL1 Register Setup
    ADC12CTL1 = ADC12SHP               // Sample-and-hold pulse mode
              | ADC12SSEL_0;           // ADC12OSC som clock kilde
              
    // ADC12CTL2 Register Setup
    ADC12CTL2 = ADC12RES_1;            // Changed from ADC12RES_2 to ADC12RES_1 for 10-bit
    
    // Input Kanal Konfiguration
    ADC12MCTL0 = ADC12INCH_12          // Input på kanal 12
                | ADC12SREF_0;         // VR+ = AVCC, VR- = AVSS
    
    // Interrupt Setup
    ADC12IE |= ADC12IE0;               // Enable interrupt for kanal 0
    
    // GPIO Konfiguration
    P7SEL |= BIT0;                     // P7.0 til analog funktion
    
    // Enable ADC
    ADC12CTL0 |= ADC12ENC;             // Enable konvertering
}

/****************************************************************************
* Nye PWM og Duty Cycle Funktioner
****************************************************************************/
// Opdaterer PWM duty cycle baseret på ADC værdi
void update_duty_cycle(uint16_t adc_value)
{
    // Direkte mapping da både ADC og PWM er 12-bit
    TA1CCR1 = adc_value;
}

// Beregner aktuel duty cycle i procent
uint8_t calculate_duty_cycle(void)
{
    return (uint8_t)((((uint32_t)TA1CCR1 * 100) / TA1CCR0_VALUE));
}

// Beregner forventet output spænding i millivolt
uint16_t calculate_voltage_mv(uint8_t duty_cycle)
{
    return (3300 * duty_cycle) / 100;  // 3.3V = 3300mV
}

void timer_init(void)
{
    // Timer A0 Konfiguration (ADC Trigger) - behold som er
    TA0CTL = TASSEL_2 | MC_1 | TACLR;  // SMCLK, Up mode, clear timer
    TA0CCR0 = 3999;                     // Set period
    TA0CCTL0 = CCIE;                    // Enable interrupt

    // Timer A1 Konfiguration (PWM Generator)
    TA1CTL = MC_0;                      // Stop timer først
    
    // PWM Setup
    TA1CCR0 = TA1CCR0_VALUE;           // PWM periode (12-bit)
    TA1CCTL1 = OUTMOD_7;               // Reset/Set PWM mode
    TA1CCR1 = 0;                       // Start med 0% duty cycle
    
    // Start timer
    TA1CTL = TASSEL_2                  // SMCLK som clock kilde
           | MC_1                      // Up mode
           | TACLR;                    // Clear timer
    
    // PWM Output Setup - Updated for MSP430F5529
    P2DIR |= BIT0;                     // P2.0 som output
    P2SEL |= BIT0;                     // P2.0 til PWM funktion (TA1.1)
    P2OUT &= ~BIT0;                    // Start lav
}

/****************************************************************************
* String Formattering og Konvertering
****************************************************************************/
// Numerisk til String Konvertering
void my_itoa(uint32_t value, char* str)
{
    char temp[MAX_DIGITS + 1];         // Temporær buffer
    uint8_t i = 0;
    
    // Håndter special case for 0
    if (value == 0)
    {
        str[0] = '0';
        str[1] = '\0';
        return;
    }
    
    // Konverter cifre et ad gangen
    while (value > 0 && i < MAX_DIGITS)
    {
        temp[i++] = '0' + (value % 10);  // Konverter til ASCII
        value /= 10;                      // Næste ciffer
    }
    
    // Vend string om (cifre er baglæns)
    uint8_t j;
    uint8_t len = i;
    for (j = 0; j < i; j++)
    {
        str[j] = temp[len - 1 - j];
    }
    str[j] = '\0';                     // Null-terminator
}

// Numerisk Display Formattering
void formatNumber(uint32_t number, DisplayValue* display)
{
    char temp[MAX_DIGITS + 1];
    my_itoa(number, temp);              // Konverter til string
    uint8_t len = strlen(temp);
    
    // Begræns længde til display bredde
    if (len > MAX_DIGITS) len = MAX_DIGITS;
    
    // Højrejuster tal med spaces
    memset(display->text, ' ', MAX_DIGITS);
    memcpy(display->text + MAX_DIGITS - len, temp, len);
    display->text[MAX_DIGITS] = '\0';
    display->length = MAX_DIGITS;
}

// Spændings Formattering
void formatVoltageDisplay(float voltage, VoltageDisplay* display)
{
    // Split voltage i heltal og decimaler
    uint16_t vol_int = (uint16_t)voltage;
    uint16_t vol_dec = (uint16_t)((voltage - vol_int) * 1000);
    
    // Formatér med 3 decimaler
    snprintf(display->text, VOLT_DIGITS, "%d.%03d V", vol_int, vol_dec);
    display->length = strlen(display->text);
}

// Duty Cycle Formattering
void formatDutyDisplay(uint32_t duty_percent, DutyDisplay* display)
{
    // Formatér med procent symbol
    snprintf(display->text, DUTY_DIGITS, "%3lu %%", duty_percent);
    display->length = strlen(display->text);
}

/****************************************************************************
* Display Update System
****************************************************************************/
void updateDisplayValue(uint8_t x, uint8_t y, void* new_value, void* old_value, uint8_t max_length)
{
    char* new_text = (char*)new_value;
    char* old_text = (char*)old_value;
    
    // Opdater kun ændrede karakterer
    for (uint8_t i = 0; i < max_length; i++)
    {
        if (new_text[i] != old_text[i])
        {
            // Slet gammel karakter
            ssd1306_printText(x + (i * CHAR_WIDTH), y, " ");
            
            // Skriv ny karakter hvis ikke null
            if (new_text[i] != '\0')
            {
                char temp[2] = {new_text[i], '\0'};
                ssd1306_printText(x + (i * CHAR_WIDTH), y, temp);
            }
            // Opdater historik
            old_text[i] = new_text[i];
        }
    }
}

/****************************************************************************
* Interrupt Service Routines
****************************************************************************/
// ADC Konversion Færdig Interrupt
__attribute__((interrupt(ADC12_VECTOR)))
void ADC12_ISR(void)
{
    if (ADC12IFG & BIT0)              // Verificer interrupt kilde
    {
        adc_data = ADC12MEM0;         // Gem ADC resultat
        adc_flag = 1;                 // Signal til main loop
        ADC12IFG &= ~BIT0;            // Clear interrupt flag
    }
}

// Timer A0 Overflow Interrupt (ADC Trigger)
__attribute__((interrupt(TIMER0_A0_VECTOR)))
void TIMER0_A0_ISR(void) {
    ADC12CTL0 |= ADC12ENC | ADC12SC;  // Start ny ADC konversion
}

/****************************************************************************
* System Initialisering
****************************************************************************/
void init_SMCLK_XT2()
{
    WDTCTL = WDTPW|WDTHOLD; // Stop watchdog timer
    P5SEL |= BIT2+BIT3;                       // Port select XT2
    UCSCTL6 &= ~XT2OFF;                       // Enable XT2
    UCSCTL4 |= SELA_2;                        // ACLK=REFO,SMCLK=DCO,MCLK=DCO
    UCSCTL4 |= SELS_5 + SELM_5;               // SMCLK=MCLK=XT2
    // Loop until XT1,XT2 & DCO stabilizes - in this case loop until XT2 settles
    do
    {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);     // Clear XT2,XT1,DCO fault flags
        SFRIFG1 &= ~OFIFG;                              // Clear fault flags
    }
    while (SFRIFG1&OFIFG);                              // Test oscillator fault flag
}

  void init_setup(void)
{
    // Basis System Protection
    WDTCTL = WDTPW | WDTHOLD;         // Stop watchdog timer
    
    // Hardware Initialisering
    i2c_init();                       // Setup I2C bus
    ssd1306_init();                   // Initialiser OLED
    adc_init();                       // Setup ADC system
    timer_init();                     // Setup timere
    init_SMCLK_XT2();
    // Display Layout Setup
    ssd1306_clearDisplay();
    ssd1306_printText(LABEL_X_POS, ADC_Y_POS, "ADC Value:");
    ssd1306_printText(LABEL_X_POS, PWM_Y_POS, "PWM Duty:");
    ssd1306_printText(LABEL_X_POS, VOLT_Y_POS, "Voltage:");
    
    // Enable Interrupts
    __bis_SR_register(GIE);           // Global interrupt enable
}

/****************************************************************************
* Opdateret Main Loop
****************************************************************************/
// Hoved program funktion
int main(void) {
    // Initialiser system hardware og konfiguration
    init_setup();
    
    // Opret statiske display buffere for ADC, duty cycle og spændings værdier
    // Både nuværende og sidste værdier gemmes for at detektere ændringer
    static DisplayValue curr_adc = {{0}, 0}, last_adc = {{0}, 0};        // ADC værdier
    static DutyDisplay curr_duty = {{0}, 0}, last_duty = {{0}, 0};       // Duty cycle værdier
    static VoltageDisplay curr_volt = {{0}, 0}, last_volt = {{0}, 0};    // Spændings værdier
    static uint16_t last_filtered_value = 0;                             // Sidste filtrerede ADC værdi
    
    // Regulerings parametre
    float G = 0.5;                     // Forstærkning for regulator
    uint16_t set_digital_vd = TARGET_ADC;  // Ønsket målværdi for ADC
    
    // Uendelig program løkke
    while(1) {
        // Tjek om ny ADC måling er tilgængelig
        if (adc_flag) {
            // Filtrer den nye ADC værdi gennem moving average filter
            uint16_t filtered_value = filter_adc(adc_data);
            
            // Hvis ændringen er større end grænseværdien, opdater system
            if (diff(filtered_value, last_filtered_value) > ADC_THRESHOLD) {
                // Beregn regulerings værdier
                digital_Vo = filtered_value;                    // Faktisk målt værdi
                digital_Ve_error = set_digital_vd - digital_Vo; // Beregn fejl
                compVal = G * digital_Ve_error;                 // Beregn kompensation
                
                // Begræns maksimal ændring per iteration
                float max_step = 50.0f;                        // Maksimal ændrings størrelse
                if (compVal > max_step) compVal = max_step;    // Begræns positiv ændring
                if (compVal < -max_step) compVal = -max_step;  // Begræns negativ ændring
                
                // Opdater PWM duty cycle med begrænsninger
                int32_t new_pwm = TA1CCR1;                     // Hent nuværende PWM værdi
                new_pwm += (int32_t)compVal;                   // Tilføj kompensation
                // Begræns PWM værdi til gyldigt område
                if (new_pwm > TA1CCR0_VALUE) new_pwm = TA1CCR0_VALUE;  
                if (new_pwm < 0) new_pwm = 0;
                TA1CCR1 = (uint16_t)new_pwm;                   // Opdater PWM register
                
                // Opdater ADC værdi på display
                formatNumber(filtered_value, &curr_adc);
                updateDisplayValue(VALUE_X_POS, ADC_Y_POS, 
                                 curr_adc.text, last_adc.text, MAX_DIGITS);
                
                // Opdater duty cycle værdi på display
                uint8_t duty_cycle = calculate_duty_cycle();
                formatDutyDisplay(duty_cycle, &curr_duty);
                updateDisplayValue(VALUE_X_POS, PWM_Y_POS,
                                 curr_duty.text, last_duty.text, DUTY_DIGITS);
                
                // Opdater spændings værdi på display
                float voltage = adc_to_voltage(filtered_value);
                formatVoltageDisplay(voltage, &curr_volt);
                updateDisplayValue(VALUE_X_POS, VOLT_Y_POS,
                                 curr_volt.text, last_volt.text, VOLT_DIGITS);
                
                // Gem den filtrerede værdi til næste sammenligning
                last_filtered_value = filtered_value;
            }
            
            // Nulstil ADC flag og vent kort tid for stabilitet
            adc_flag = 0;
            __delay_cycles(40);
        }
    }
    return 0;  // Returnér 0 ved normal afslutning (nås aldrig i praksis)
}