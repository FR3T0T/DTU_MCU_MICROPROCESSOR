#include <msp430.h>
#include <stdint.h>
#include "i2c.h"
#include "ssd1306.h"

// Definer konstanter globalt
#define VFS 3.3f
#define ADC_BITS 12
#define ADC_MAX_VALUE (1 << ADC_BITS)

void init_ADC(void) {
    // Disable ADC12 for configuration
    ADC12CTL0 &= ~ADC12ENC;
    
    // Configure ADC12
    ADC12CTL0 = ADC12SHT0_3 |    // 32 ADC12CLK cycles sample time
                ADC12ON;          // Turn on ADC12
    
    ADC12CTL1 = ADC12SHP |       // Use sampling timer
                ADC12SSEL_0;      // ADC12CLK source
    
    ADC12CTL2 = ADC12RES_2;      // 12-bit resolution
    
    ADC12MCTL0 = ADC12INCH_12 |  // Input channel A12 (P7.0)
                 ADC12SREF_0;     // VR+ = AVCC, VR- = AVSS
    
    // Configure P7.0 as analog input
    P7SEL |= BIT0;               // For MSP430F5529, use P7SEL
    
    // Enable ADC12
    ADC12CTL0 |= ADC12ENC;
}

uint16_t get_sample(void) {
    // Start conversion
    ADC12CTL0 |= ADC12SC;
    
    // Wait for conversion to complete
    while (!(ADC12IFG & BIT0));
    
    return ADC12MEM0;
}

int main(void) {
    // Stop watchdog timer
    WDTCTL = WDTPW | WDTHOLD;
    
    // Initialize I2C
    i2c_init();
    
    // Initialize OLED display
    ssd1306_init();
    ssd1306_clearDisplay();
    
    // Initialize ADC
    init_ADC();
    
    while(1) {
        // Get ADC sample
        uint16_t adc_value = get_sample();
        
        // Calculate actual voltage
        float Vin = (adc_value * VFS) / ADC_MAX_VALUE;
        
        // Calculate theoretical NADC
        uint32_t NADC_calc = (uint32_t)((ADC_MAX_VALUE * Vin) / VFS);
        
        // Display measured ADC value
        ssd1306_printText(0, 0, "ADC Value:");
        ssd1306_printUI32(0, 1, adc_value, 0);
        
        // Display theoretical NADC
        ssd1306_printText(0, 3, "Calc NADC:");
        ssd1306_printUI32(0, 4, NADC_calc, 0);
        
        // Display voltage in mV
        uint32_t voltage_mv = (uint32_t)(Vin * 1000.0f);
        ssd1306_printText(0, 6, "Voltage (mV):");
        ssd1306_printUI32(0, 7, voltage_mv, 0);
        
        // Delay mellem målinger (ca. 1 sekund)
        __delay_cycles(50000);
    }
    
    return 0;
}