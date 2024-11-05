#include <Arduino.h>
#include <Energia.h>
#include <stdint.h>
#include "i2c.h"
#include "ssd1306.h"

void init_ADC(void) {
    // Reset og konfigurer ADC12
    ADC12CTL0 &= ~ADC12ENC;                   // Disable ADC12

    // Konfigurer ADC12
    ADC12CTL0 = ADC12SHT02 |                  // Sample time
                ADC12ON;                       // Turn on ADC12
    
    ADC12CTL1 = ADC12SHP |                    // Sample timer
                ADC12SSEL_0;                   // ADC12 clock source
    
    ADC12CTL2 = ADC12RES_2;                   // 12-bit resolution
    
    ADC12MCTL0 = ADC12INCH_12 |               // Input A12 (P7.0)
                 ADC12SREF_0;                  // VR+ = AVCC and VR- = AVSS
    
    // Konfigurer P7.0 som analog input
    P7SEL |= BIT0;                            // Select analog function for P7.0
}

uint16_t get_sample(void) {
    uint16_t result;
    
    ADC12CTL0 |= ADC12ENC | ADC12SC;          // Start sampling/conversion
    
    // Vent på konvertering er færdig
    while (!(ADC12IFG & BIT0));
    
    result = ADC12MEM0;                        // Læs resultat
    
    return result;
}

void delay_ms(uint32_t ms) {
    delay(ms);  // Brug Energia's delay funktion
}
void setup(){
    
}
void init1() {
    // Stop watchdog timer
    WDTCTL = WDTPW | WDTHOLD;
    
    // Initialiser I2C
    i2c_init();
    
    // Initialiser OLED display
    ssd1306_init();
    ssd1306_clearDisplay();
    
    // Initialiser ADC
    init_ADC();
    
    // Print header text
    ssd1306_printText(0, 0, "ADC Value:");
}

int main() {
    init1();

    while(1){
    uint16_t adc_value = get_sample();
    
    // Vis ADC værdi på OLED (linje 2)
    ssd1306_printUI32(0, 2, adc_value, 0);
    
    // Beregn spænding (Vin = (NADC * VFS) / 2^N)
    // VFS = 3.3V, N = 12
    float voltage = (adc_value * 3.3f) / 4096.0f;
    
    // Konverter til millivolt for visning
    uint32_t voltage_mv = (uint32_t)(voltage * 1000);
    
    // Vis spænding på linje 4
    ssd1306_printText(0, 4, "Voltage (mV):");
    ssd1306_printUI32(0, 6, voltage_mv, 0);
    
    __delay_cycles(1000000);  // Opdater hver 100ms

}
}