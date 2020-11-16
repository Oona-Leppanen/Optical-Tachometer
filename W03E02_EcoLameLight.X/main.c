/*
 * File:   main.c
 * Author: dtek0068
 *
 * Created on 16 November 2020, 16:21
 */

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

int main(void)
{
    PORTF.DIRSET = PIN5_bm; //set PIN5 as output
    PORTF.DIRCLR = PIN6_bm; //set PIN6 as input
    PORTF.OUTSET = PIN5_bm;
    
    //trigger an interrupt when state is low
    PORTF.PIN6CTRL = PORT_ISC_FALLING_gc;
    
    set_sleep_mode(SLPCTRL_SMODE_IDLE_gc); //sleep mode
    
    sei();
    
    while (1)
    {
            sleep_mode();
    }
}

ISR(PORTF_PORT_vect)
{
    PORTF.OUTCLR = PIN5_bm; //set PIN5 output as 0
    while (!(PORTF.IN & PIN6_bm)) //button is pressed
    {
    }
    PORTF.OUTSET = PIN5_bm; //set PIN5 output as 1
    PORTF.INTFLAGS = 0xFF; //clear interrupt flags
}