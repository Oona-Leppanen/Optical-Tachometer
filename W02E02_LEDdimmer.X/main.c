/*
 * File:   main.c
 * Author: dtek0068
 *
 * Created on 13 November 2020, 17:00
 */

#define F_CPU 3333333
#include <avr/io.h>
#include <util/delay.h>

void pwm_period(int duty);

int main(void)
{
    
    uint8_t duty = 0;
    
    PORTF.DIR |= PIN5_bm;
    PORTF.DIR &= ~PIN6_bm;
    int previous = 0;
    int current = 0;
    int button = 0;
    
    while (1)
    {
        if(PORTF.IN & PIN6_bm) //painetaanko nappia?
        {
            button = 1;
        }
        else
        {
            button = 0;
        }
        if(button == 1) //nappia painetaan
        {
            if(previous == 0) //nappia ei painettu edellisellä kierroksella
            {
                if(current==0) //jos himmeä
                {
                    current = 1;
                }
                else
                {
                    current = 0;
                }
            }
            if(current == 0) //kirkas?
            {
                if(duty>0)
                {
                    duty--;
                }
                pwm_period(duty);
            }
            else
            {
                if(duty<255)
                {
                   duty++;
                }
                pwm_period(duty);
            }
            previous = 1;
        }
        else
        {
            previous = 0;
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
    
    for(int j=255; j>duty; j--)
    {
        _delay_us(7);
    }
}