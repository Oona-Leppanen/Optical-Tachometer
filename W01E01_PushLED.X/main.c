/*
 * File:   main.c
 * Author: dtek0068
 *
 * Created on 12 November 2020, 14:41
 */


#include <avr/io.h>

int main(void)
{
    PORTF.DIR |= PIN5_bm; //asetetaan PortF:n pin 5 ulostuloksi
    PORTF.DIR &= ~PIN6_bm; //asetetaan PortF:n pin 6 sis‰‰ntuloksi
    while (1)
    {
        if(PORTF.IN & PIN6_bm)//PORT.IN tarkkailee pin 6:ssa olevaa arvoa
        {
            PORTF.OUT |= PIN5_bm; //verrataan bittiarvoja toisiinsa OR-operaat.
            //valo palaa
        }
        else
        {
            PORTF.OUT &= ~PIN5_bm; //tulee aina 0; F.out:ssa jokin oletusarvo?
            //valo ei pala
        }
    }
}
