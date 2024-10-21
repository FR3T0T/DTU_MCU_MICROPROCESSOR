#include <msp430f5529.h>
#include "ssd1306.h"
#include "i2c.h"
#include <stdio.h>
#include <pins_energia.h>

const int switchPin = P2_1;
const int operationSwitchPin = P1_1;
const int dipPin0 = P6_0;
const int dipPin1 = P6_1;
const int dipPin2 = P6_2;
const int dipPin3 = P6_3;
const int dipPin4 = P6_4;
const int dipPin5 = P6_5;
const int dipPin6 = P6_6;
const int dipPin7 = P7_0;

volatile unsigned char t_flag = 0;
volatile unsigned char button_flag = 0;
volatile unsigned char ss = 0;
volatile unsigned char mm = 0;
volatile unsigned char hh = 0;
volatile unsigned char set_time_stage = 0;
char tiden[9];

#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A(void)
{
    P1OUT ^= BIT0;
    if (set_time_stage == 0)
    {
        t_flag = 1;     
        ss++;
        if (ss == 60)
        {
            ss = 0;
            mm++;
            if (mm == 60) 
            {
                mm = 0;
                hh++;
                if (hh == 24) 
                {
                    hh = 0;
                }
            }
        }
    }
}

#pragma vector = PORT2_VECTOR
__interrupt void Port_2(void)
{
    if (P2IFG & BIT1)
    {
        button_flag = 1;
        set_time_stage++;
            if (set_time_stage > 3)
            {
            set_time_stage = 0;
            }
        P2IFG &= ~BIT1;  
    }
}

void update_time_string(void)
{
    sprintf(tiden, "%02d:%02d:%02d", hh, mm, ss);
}

unsigned char read_dip_switch(void)
{
    unsigned char value = 0;
    value |= (P6IN & BIT0) ? 1 : 0;
    value |= (P6IN & BIT1) ? 2 : 0;
    value |= (P6IN & BIT2) ? 4 : 0;
    value |= (P6IN & BIT3) ? 8 : 0;
    value |= (P6IN & BIT4) ? 16 : 0;
    value |= (P6IN & BIT5) ? 32 : 0;
    value |= (P6IN & BIT6) ? 64 : 0;
    value |= (P7IN & BIT0) ? 128 : 0;
    return value;
}

void set_time(void)
{
    unsigned char dip_value;
    switch (set_time_stage)
    {
        case 1:
            dip_value = read_dip_switch();
            hh = dip_value % 24;  // Ensure hours are 0-23
            sprintf(tiden, "Set HH:%02d", hh);
            break;
            
        case 2:
            dip_value = read_dip_switch();
            mm = dip_value % 60;  // Ensure minutes are 0-59
            sprintf(tiden, "Set MM:%02d", mm);
            break;
            
        case 3:
            dip_value = read_dip_switch();
            ss = dip_value % 60;  // Ensure seconds are 0-59
            sprintf(tiden, "Set SS:%02d", ss);
            break;
            
    }
    ssd1306_printText(0, 2, tiden);
}



int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;  
   

    i2c_init();
    ssd1306_init();
   


    TA0CTL = TASSEL_2 | ID_3 | MC_1; 
    TA0CCR0 = 16370;  
    TA0CCTL0 = CCIE; 
    TA0EX0 = TAIDEX_7;  

    P1DIR |= BIT0;
    P1OUT &= ~BIT0; 

    // Configure switchPin (P2.1)
    P2DIR &= ~BIT1;  
    P2REN |= BIT1;  
    P2OUT |= BIT1;   
    P2IE |= BIT1;    
    P2IES |= BIT1;   

    // Configure operationSwitchPin (P1.1)
    P1DIR &= ~BIT1;
    P1REN |= BIT1;
    P1OUT |= BIT1;

    // Configure DIP switch pins
    P6DIR &= ~(BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6);
    P7DIR &= ~BIT0;
    P6REN |= (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6);
    P7REN |= BIT0;
    P6OUT |= (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6);
    P7OUT |= BIT0;

    __bis_SR_register(GIE);
    
    reset_diplay();

    while (1)
    {
        if (button_flag)
        {
            button_flag = 0;
            set_time();
        }
        else if (t_flag && set_time_stage == 0)
        {
            t_flag = 0;
            update_time_string();
            ssd1306_printText(0, 0, tiden);
          


        
        }
    }
    return 0;
}