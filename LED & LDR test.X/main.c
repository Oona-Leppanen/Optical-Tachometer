/*
 * File:   main.c
 * Author: dtek0068
 *
 * Created on 03 December 2020, 16:43
 */


#define F_CPU 3333333
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>

int main(void)
{
    PORTE.DIRCLR = PIN0_bm;
    PORTF.DIRSET = PIN5_bm;

    while (1)
    {
        if (PORTE.IN & PIN0_bm)
        {
            PORTF.OUTSET = PIN5_bm;
        }
        else if (!(PORTE.IN & PIN0_bm))
        {
            PORTF.OUTCLR = PIN5_bm;   
        }
    }
    return 0;
}