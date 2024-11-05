#include <msp430.h>
#include <stdint.h>
#include <string.h>
#include "i2c.h"
#include "ssd1306.h"

// Definer konstanter globalt
#define VFS 3.3f
#define ADC_BITS 12
#define ADC_MAX_VALUE (1 << ADC_BITS)
#define ADC_THRESHOLD 3        // Mindre threshold da vi nu bruger averaging
#define FILTER_SIZE 8         // Størrelse af moving average filter
#define MAX_DIGITS 6         // Maximum antal cifre vi viser
#define CHAR_WIDTH 6         // Bredde af hvert tegn i pixels

// Struct til at holde formaterede værdier
typedef struct {
    char text[MAX_DIGITS + 1];  // +1 for null terminator
    uint8_t length;
} DisplayValue;

void init_ADC(void) {
    ADC12CTL0 &= ~ADC12ENC;
    ADC12CTL0 = ADC12SHT0_3 | ADC12ON;
    ADC12CTL1 = ADC12SHP | ADC12SSEL_0;
    ADC12CTL2 = ADC12RES_2;
    ADC12MCTL0 = ADC12INCH_12 | ADC12SREF_0;
    P7SEL |= BIT0;
    ADC12CTL0 |= ADC12ENC;
}

uint16_t get_sample(void) {
    ADC12CTL0 |= ADC12SC;
    while (!(ADC12IFG & BIT0));
    return ADC12MEM0;
}

// Moving average filter
uint16_t filter_adc(uint16_t new_sample) {
    static uint16_t buffer[FILTER_SIZE] = {0};
    static uint8_t index = 0;
    static uint32_t sum = 0;
    static uint8_t count = 0;
    
    sum -= buffer[index];
    buffer[index] = new_sample;
    sum += new_sample;
    
    index = (index + 1) % FILTER_SIZE;
    if (count < FILTER_SIZE) count++;
    
    return (uint16_t)(sum / count);
}

// Simple itoa implementation for uint32_t
void my_itoa(uint32_t value, char* str) {
    char temp[MAX_DIGITS + 1];
    uint8_t i = 0;
    
    // Handle special case of zero
    if (value == 0) {
        str[0] = '0';
        str[1] = '\0';
        return;
    }
    
    // Convert digits in reverse order
    while (value > 0 && i < MAX_DIGITS) {
        temp[i++] = '0' + (value % 10);
        value /= 10;
    }
    
    // Reverse the string
    uint8_t j;
    uint8_t len = i;
    for (j = 0; j < i; j++) {
        str[j] = temp[len - 1 - j];
    }
    str[j] = '\0';
}

// Konverter tal til string med fast længde
void formatNumber(uint32_t number, DisplayValue* display) {
    char temp[MAX_DIGITS + 1];
    my_itoa(number, temp);
    uint8_t len = strlen(temp);
    
    // Ensure we don't exceed buffer
    if (len > MAX_DIGITS) len = MAX_DIGITS;
    
    // Pad with spaces on the left
    memset(display->text, ' ', MAX_DIGITS);
    memcpy(display->text + MAX_DIGITS - len, temp, len);
    display->text[MAX_DIGITS] = '\0';
    display->length = MAX_DIGITS;
}

// Sammenlign og opdater kun ændrede cifre
void updateDisplayValue(uint8_t x, uint8_t y, DisplayValue* new_value, DisplayValue* old_value) {
    for (uint8_t i = 0; i < new_value->length; i++) {
        if (new_value->text[i] != old_value->text[i]) {
            // Opdater enkelt tegn ved at først skrive et mellemrum og så det nye tegn
            ssd1306_printText(x + (i * CHAR_WIDTH), y, " ");
            char temp[2] = {new_value->text[i], '\0'};
            ssd1306_printText(x + (i * CHAR_WIDTH), y, temp);
            old_value->text[i] = new_value->text[i];
        }
    }
}

// Simple absolute difference function
uint16_t diff(uint16_t a, uint16_t b) {
    return (a > b) ? (a - b) : (b - a);
}

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;
    
    // Initialize hardware
    i2c_init();
    ssd1306_init();
    ssd1306_clearDisplay();
    init_ADC();
    
    // Skriv statiske labels
    ssd1306_printText(0, 0, "ADC Value:");
    ssd1306_printText(0, 3, "Calc NADC:");
    ssd1306_printText(0, 6, "Voltage (mV):");
    
    // Initialiser display værdier
    DisplayValue curr_adc = {{0}, 0}, last_adc = {{0}, 0};
    DisplayValue curr_nadc = {{0}, 0}, last_nadc = {{0}, 0};
    DisplayValue curr_mv = {{0}, 0}, last_mv = {{0}, 0};
    
    uint16_t last_filtered_adc = 0;
    
    while(1) {
        // Get filtered ADC sample
        uint16_t filtered_adc = filter_adc(get_sample());
        
        // Check om værdien har ændret sig nok
        if (diff(filtered_adc, last_filtered_adc) > ADC_THRESHOLD) {
            // Beregn nye værdier
            float Vin = (filtered_adc * VFS) / ADC_MAX_VALUE;
            uint32_t NADC_calc = (uint32_t)((ADC_MAX_VALUE * Vin) / VFS);
            uint32_t voltage_mv = (uint32_t)(Vin * 1000.0f);
            
            // Formater nye værdier
            formatNumber(filtered_adc, &curr_adc);
            formatNumber(NADC_calc, &curr_nadc);
            formatNumber(voltage_mv, &curr_mv);
            
            // Opdater display kun for ændrede cifre
            updateDisplayValue(0, 1, &curr_adc, &last_adc);
            updateDisplayValue(0, 4, &curr_nadc, &last_nadc);
            updateDisplayValue(0, 7, &curr_mv, &last_mv);
            
            last_filtered_adc = filtered_adc;
        }
        
        __delay_cycles(10000); // Kortere delay for mere responsive opdateringer
    }
    
    return 0;
}