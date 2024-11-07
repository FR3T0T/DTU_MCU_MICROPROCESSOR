#include <msp430.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "i2c.h"
#include "ssd1306.h"

// Definer konstanter globalt
#define VFS 3.3f
#define ADC_BITS 12
#define ADC_MAX_VALUE (1 << ADC_BITS)
#define MAX_DIGITS 6         // Maximum antal cifre vi viser
#define VOLT_DIGITS 8        // Antal tegn for voltage display ("x.xxx V\0")
#define CHAR_WIDTH 6         // Bredde af hvert tegn i pixels
#define FILTER_SIZE 16       // Størrelse af moving average filter
#define ADC_THRESHOLD 3      // Minimum ændring før update

// Globale variable til ADC interrupt
volatile uint16_t adc_data = 0;
volatile uint8_t adc_flag = 0;

// Moving average filter buffer
static uint16_t filter_buffer[FILTER_SIZE] = {0};
static uint8_t filter_index = 0;
static uint32_t filter_sum = 0;
static uint8_t filter_count = 0;

// Structs til at holde formaterede værdier
typedef struct {
    char text[MAX_DIGITS + 1];  // +1 for null terminator
    uint8_t length;
} DisplayValue;

typedef struct {
    char text[VOLT_DIGITS];     // For voltage display med decimaler
    uint8_t length;
} VoltageDisplay;

// Funktions prototyper
void init_setup(void);
void adc_init(void);
void timer_init(void);
void formatNumber(uint32_t number, DisplayValue* display);
void updateDisplayValue(uint8_t x, uint8_t y, DisplayValue* new_value, DisplayValue* old_value);
void my_itoa(uint32_t value, char* str);
uint16_t filter_adc(uint16_t new_sample);
uint16_t diff(uint16_t a, uint16_t b);
float adc_to_voltage(uint16_t adc_value);
void formatVoltageDisplay(float voltage, VoltageDisplay* display);

// Moving average filter
uint16_t filter_adc(uint16_t new_sample) {
    filter_sum -= filter_buffer[filter_index];
    filter_buffer[filter_index] = new_sample;
    filter_sum += new_sample;
    
    filter_index = (filter_index + 1) % FILTER_SIZE;
    if (filter_count < FILTER_SIZE) filter_count++;
    
    return (uint16_t)(filter_sum / filter_count);
}

// Simple absolute difference
uint16_t diff(uint16_t a, uint16_t b) {
    return (a > b) ? (a - b) : (b - a);
}

// ADC til voltage konvertering
float adc_to_voltage(uint16_t adc_value) {
    return (float)adc_value * VFS / ADC_MAX_VALUE;
}

// ADC initialisering
void adc_init(void) {
    ADC12CTL0 &= ~ADC12ENC;          
    ADC12CTL0 = ADC12SHT0_3 | ADC12ON;
    ADC12CTL1 = ADC12SHP | ADC12SSEL_0;
    ADC12CTL2 = ADC12RES_2;          
    ADC12MCTL0 = ADC12INCH_12 | ADC12SREF_0;
    ADC12IE |= ADC12IE0;             
    P7SEL |= BIT0;                   
    ADC12CTL0 |= ADC12ENC;           
}

// Timer initialisering
void timer_init(void) {
    // Setup Timer A0 for ADC sampling control
    TA0CTL = TASSEL_2 | MC_1 | TACLR;  
    TA0CCR0 = 3999;                    
    TA0CCTL0 = CCIE;                   

    // Setup Timer A1 for PWM
    TA1CTL = TASSEL_2 | MC_1 | TACLR;  
    TA1CCR0 = 4095;                    
    TA1CCTL1 = OUTMOD_7;               
    TA1CCR1 = 0;                       
    P2DIR |= BIT0;                     
    P2SEL |= BIT0;                     
}

// Simple itoa implementation for uint32_t
void my_itoa(uint32_t value, char* str) {
    char temp[MAX_DIGITS + 1];
    uint8_t i = 0;
    
    if (value == 0) {
        str[0] = '0';
        str[1] = '\0';
        return;
    }
    
    while (value > 0 && i < MAX_DIGITS) {
        temp[i++] = '0' + (value % 10);
        value /= 10;
    }
    
    uint8_t j;
    uint8_t len = i;
    for (j = 0; j < i; j++) {
        str[j] = temp[len - 1 - j];
    }
    str[j] = '\0';
}

// Formatér tal med fast længde
void formatNumber(uint32_t number, DisplayValue* display) {
    char temp[MAX_DIGITS + 1];
    my_itoa(number, temp);
    uint8_t len = strlen(temp);
    
    if (len > MAX_DIGITS) len = MAX_DIGITS;
    
    memset(display->text, ' ', MAX_DIGITS);
    memcpy(display->text + MAX_DIGITS - len, temp, len);
    display->text[MAX_DIGITS] = '\0';
    display->length = MAX_DIGITS;
}

// Formatér spænding til display
void formatVoltageDisplay(float voltage, VoltageDisplay* display) {
    // Konverter til integer komponenter
    uint16_t vol_int = (uint16_t)voltage;
    uint16_t vol_dec = (uint16_t)((voltage - vol_int) * 1000);
    
    // Formater som "x.xxx V"
    snprintf(display->text, VOLT_DIGITS, "%d.%03d V", vol_int, vol_dec);
    display->length = strlen(display->text);
}

// Opdater display værdi (bruges for både tal og spænding)
void updateDisplayValue(uint8_t x, uint8_t y, void* new_value, void* old_value, uint8_t max_length) {
    char* new_text = (char*)new_value;
    char* old_text = (char*)old_value;
    
    for (uint8_t i = 0; i < max_length; i++) {
        if (new_text[i] != old_text[i]) {
            ssd1306_printText(x + (i * CHAR_WIDTH), y, " ");
            if (new_text[i] != '\0') {
                char temp[2] = {new_text[i], '\0'};
                ssd1306_printText(x + (i * CHAR_WIDTH), y, temp);
            }
            old_text[i] = new_text[i];
        }
    }
}

// ADC12 interrupt service routine
#ifdef __cplusplus
extern "C" {
#endif

__attribute__((interrupt(ADC12_VECTOR)))
void ADC12_ISR(void) {
    if (ADC12IFG & BIT0) {
        adc_data = ADC12MEM0;     
        adc_flag = 1;              
        ADC12IFG &= ~BIT0;         
    }
}

// Timer A0 interrupt service routine
__attribute__((interrupt(TIMER0_A0_VECTOR)))
void TIMER0_A0_ISR(void) {
    ADC12CTL0 |= ADC12ENC | ADC12SC;    
}

#ifdef __cplusplus
}
#endif

void init_setup(void) {
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer
    
    // Initialiser hardware
    i2c_init();
    ssd1306_init();
    adc_init();
    timer_init();
    
    // Initialiser display
    ssd1306_clearDisplay();
    ssd1306_printText(0, 0, "ADC Value:");
    ssd1306_printText(0, 2, "PWM Duty:");
    ssd1306_printText(0, 4, "Voltage:");
    
    __bis_SR_register(GIE);      // Enable global interrupts
}

int main(void) {
    init_setup();

    static DisplayValue curr_adc = {{0}, 0}, last_adc = {{0}, 0};
    static DisplayValue curr_duty = {{0}, 0}, last_duty = {{0}, 0};
    static VoltageDisplay curr_volt = {{0}, 0}, last_volt = {{0}, 0};
    static uint16_t last_filtered_value = 0;
    
    while(1) {
        if (adc_flag) {
            uint16_t filtered_value = filter_adc(adc_data);
            
            // Kun opdater hvis ændringen er større end threshold
            if (diff(filtered_value, last_filtered_value) > ADC_THRESHOLD) {
                if (TA1R == 0) {     
                    TA1CCR1 = filtered_value;
                }
                
                // Formater og vis ADC værdi
                formatNumber(filtered_value, &curr_adc);
                updateDisplayValue(0, 1, curr_adc.text, last_adc.text, MAX_DIGITS);
                
                // Beregn og vis duty cycle i procent
                uint32_t duty_percent = ((uint32_t)filtered_value * 100) / 4095;
                formatNumber(duty_percent, &curr_duty);
                updateDisplayValue(0, 3, curr_duty.text, last_duty.text, MAX_DIGITS);
                
                // Beregn og vis spænding
                float voltage = adc_to_voltage(filtered_value);
                formatVoltageDisplay(voltage, &curr_volt);
                updateDisplayValue(0, 5, curr_volt.text, last_volt.text, VOLT_DIGITS);
                
                last_filtered_value = filtered_value;
            }
            
            adc_flag = 0;
            __delay_cycles(40);  // 1ms delay ved 4MHz
        }
    }
    
    return 0;
}