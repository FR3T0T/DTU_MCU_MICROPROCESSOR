#include <Arduino.h>
#include <Energia.h>
#include "i2c.h"
#include "ssd1306.h"

void init_ports()
{
    // put your setup code here, to run once:
    P6SEL = 0x00;
    P7SEL = 0;
  
    pinMode(P1_1, INPUT_PULLUP);
    digitalWrite(P1_1, HIGH);
    pinMode(P4_7, OUTPUT);
    pinMode(P1_0, OUTPUT);
    pinMode(P2_1, INPUT_PULLUP); // Pull_up på switch til P2.1 => aktiv low
    digitalWrite(P2_1, HIGH);
    P6DIR=0;
    P6REN |= BIT6 | BIT5 | BIT4 | BIT3 | BIT2 | BIT1 | BIT0;
    P6OUT |= BIT6 | BIT5 | BIT4 | BIT3 | BIT2 | BIT1 | BIT0;
    P7REN |= BIT0;
    P7OUT |= BIT0;
}

unsigned char Dip_switch = 0;
char switch_value[20] = {0}; // der sættes plads af til tekstforklaring

unsigned char n = 3,i=0;

// for 4 Mhz SMCLK
void init_SMCLK_XT2()
{
    WDTCTL = WDTPW|WDTHOLD;                     // Stop watchdog timer
    P5SEL |= BIT2+BIT3;                         // Port select XT2
    UCSCTL6 &= ~XT2OFF;                         // Enable XT2
    UCSCTL4 |= SELA_2;                          // ACLK=REFO,SMCLK=DCO,MCLK=DCO
    UCSCTL4 |= SELS_5 + SELM_5;                 // SMCLK=MCLK=XT2

    // Loop until XT1,XT2 & DCO stabilizes - in this case loop until XT2 settles
    do
    {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);     // Clear XT2,XT1,DCO fault flags
        SFRIFG1 &= ~OFIFG;                              // Clear fault flags
    }
    while (SFRIFG1&OFIFG);                              // Test oscillator fault flag
}

void init_pwm()
{
    TA1CTL|=ID_0|TASSEL_2|MC_1;     // Select  SMCLK as timer clock and up, no clock scaling
    TA1CCTL1|= OUTMOD_3;            // Set/reset output  for P2.0  uses TA1CCR1 register for the duty cycle (TA1.1)
    TA1CCR1 =25;                    // The comparevalue for when the output will be set /reset =>duty cycle
    TA1CCR0=4095;                   // The top value for which the timer counter turns to decrement
    pinMode(P2_0, OUTPUT);          // For getting the PWM signal out
    P2SEL|=BIT0;                    // For PWM out -enable alternative function
}

int main()
{
    WDTCTL = WDTPW | WDTHOLD;   // Always disable watch for using delay function
    init_SMCLK_XT2();           // Select the 4 MHz clock for SMCLK
    init_ports();
    init_pwm();
 
    i2c_init();
    ssd1306_init();
    reset_diplay();
    ssd1306_printText(0, 0, "hej med dig/0");
    ssd1306_clearDisplay();

    while(1)
    {
        n = (P6IN&0x7F);                                // & (BIT6 | BIT5 | BIT4 | BIT3 | BIT2 | BIT1 | BIT0)); ////digitalRead(P6_0)+digitalRead(P6_1)+digitalRead(P6_2)+digitalRead(P6_3)+digitalRead(P6_4)+digitalRead(P6_5)+digitalRead(P6_6);//+digitalRead(P7_0)<<8;
        n=n + ((P7IN&BIT0)<<7);                         // Shift 7 times for biton msb place
        sprintf(switch_value, "switch er: %03d\0", n);
        ssd1306_printText(0, 0, switch_value);
        ssd1306_printUI32(0,5,TA1CCR1,0);               // Write the contents of the compareregister setting the duty cycle
 
        __delay_cycles(100000);
    }
}