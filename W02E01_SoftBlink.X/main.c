/*
 * File:   main.c
 * Author: dtek0068
 *
 * Created on 13 November 2020, 14:56
 */

#define F_CPU 3333333

#include <util/delay.h>
#include <avr/io.h>

void pwm_period(int duty);

int main(void)
{
    uint8_t duty = 0;
    PORTF.DIR |= PIN5_bm;

    while (1)
    {
        for(;duty<256; duty++)
        {
            pwm_period(duty);
        }
        for(; duty>0; duty--)
        {
            pwm_period(duty);
        }
    }
    return 0;
}

void pwm_period(int duty)
{
    PORTF.OUTSET = PIN5_bm;
    for(int i=0; i<duty; i++)
    {
        _delay_us(7);
    }
    
    PORTF.OUTCLR = PIN5_bm;
    
    for(int j=256; j>duty; j--)
    {
        _delay_us(7);
    }
}