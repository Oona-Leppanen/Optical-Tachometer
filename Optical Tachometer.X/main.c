#define F_CPU 3333333

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdbool.h>
#include <avr/cpufunc.h>




/*Define macros for clarity*/
/*Set cursor at start of line 1*/
#define LCD_LINE_ONE            0x00
/*Set cursor at start of line 2*/
#define LCD_LINE_TWO            0x40
/*Replace all characters with ASCII 'space'*/
#define LCD_CLEAR               0b00000001
/*Return cursor to first position on first line*/
#define LCD_HOME                0b00000010
/*Shift cursor from left to right on read/write*/
#define LCD_ENTRY_MODE          0b00000110
/*Turn display off*/
#define LCD_DISPLAY_OFF         0b00001000
/*Display on, cursor off, don't blink character*/
#define LCD_DISPLAY_ON          0b00001100
/*Reset the LCD*/
#define LCD_FUNCTION_RESET      0b00110000
/*4-bit data, 2-line display, 5 x 7 font*/
#define LCD_FUNCTION_SET_4_BIT  0b00101000
/*Set cursor position*/
#define LCD_SET_CURSOR          0b10000000

/*Initialize global variables*/
/*Initial text to display on LCD*/
uint8_t lcd_text[]   = "RPM";
/*Volatile integer variable for storing RPM readings*/
volatile uint16_t counter = 0;
/*Counter as string variable for printing onto LCD*/
uint8_t c_string[6];
/*Integer variable for storing the current reading from ADC*/
uint16_t adcVal;

/*Function Prototypes*/
void lcd_write_4(uint8_t);
void lcd_write_instruction_4d(uint8_t);
void lcd_write_character_4d(uint8_t);
void lcd_write_string_4d(uint8_t *);
void lcd_init_4d(void);
void rtc_init(void);
void adc0_init(void);
void adc0_start(void);
bool adc0_conversionDone(void);
uint16_t adc0_read(void);

int main(void)
{
    /*Configure the microprocessor pins for the data lines*/
    /* 4 data lines as output for LCD*/
    PORTD.DIRSET = PIN7_bm | PIN6_bm | PIN5_bm | PIN4_bm;
    /*Configure PB3 to LCD E pin and PB4 to LCD RS pin, both as output*/
    PORTB.DIRSET = PIN3_bm | PIN4_bm;
    /*DC motor output pin*/
    PORTA.DIRSET = PIN2_bm;
    /*Input pin to read values to LDR*/
    PORTE.DIRCLR = PIN0_bm;
    /*AVR button for toggling DC motor on/off*/
    PORTF.DIRCLR = PIN6_bm;
    /*Trigger interrupt on AVR button press*/
    PORTF.PIN6CTRL = PORT_ISC_FALLING_gc;
    
    

    /*Initialize the LCD screen for 4-bit interface*/
    lcd_init_4d();
    /*Display initial information*/
    lcd_write_string_4d(lcd_text);
    
    /*Initialize RTC for 1 second periodic interrupts*/
    rtc_init();
    
    /*Initialize ADC*/
    adc0_init();
    /*Start ADC*/
    adc0_start();
    
    /*Enable interrupts*/
    sei();
    /*In the superloop we evaluate the readings from the ADC and count
     * how many passes our rotor blade has done between the LED and LDR*/
    while(1)
    {
        if (adc0_conversionDone()) /*Check if ADC has finished a conversion*/
        {
            /*Read from the ADC and store the value into adcVal*/
            adcVal = adc0_read();
            /*From our testing we found that the ADC readings with an 
             * ultrabright LED are between 280 and 320 during RPM readings. 
             * 300 was chosen as the mid-point.*/
            if (adcVal>300)
            {
                counter++;
                /*Keep the reading while adcVal is above 300 to 
                 * prevent overcounting to counter*/
                while(adcVal>300)
                {
                    /*If ADC has a new conversion, update adcVal*/
                    if (adc0_conversionDone())
                    {
                        adcVal = adc0_read();
                    }
                }
            }
        }
    }
    cli();
    
    return 0;
}

/*Initialize LCD-display*/
void lcd_init_4d(void)
{
    /* Power-up delay. All future delays relating to LCD instructions are 
     * for making sure the LCD has enough time to finish the instruction in 
     * question */
    _delay_ms(100);

    /*Set up the RS and E lines for the 'lcd_write_4' subroutine*/
    PORTB.OUTCLR = PIN4_bm; /* Select the Instruction Register (RS low)*/
    PORTB.OUTCLR = PIN3_bm; /* Make sure E is initially low*/
    
    /* Reset the LCD controller */
    lcd_write_4(LCD_FUNCTION_RESET);    /* First part of reset sequence*/
    _delay_ms(10);

    lcd_write_4(LCD_FUNCTION_RESET);    /* Second part of reset sequence*/
    _delay_us(200);

    lcd_write_4(LCD_FUNCTION_RESET);    /* Third part of reset sequence*/
    _delay_us(200);
 
    lcd_write_4(LCD_FUNCTION_SET_4_BIT);    /* set 4-bit mode*/
    _delay_us(80);

    /*set mode, lines, and font*/
    lcd_write_instruction_4d(LCD_FUNCTION_SET_4_BIT);
    _delay_us(80);

    lcd_write_instruction_4d(LCD_DISPLAY_OFF);  /* Turn display OFF*/
    _delay_us(80);

    lcd_write_instruction_4d(LCD_CLEAR);    /*Clear display*/
    _delay_ms(4);

    /* Set desired shift characteristics*/
    lcd_write_instruction_4d(LCD_ENTRY_MODE);
    _delay_us(80);
 
    lcd_write_instruction_4d(LCD_DISPLAY_ON);   /* Turn the display ON*/
    _delay_us(80);
}

/*Function to write a string to the LCD display*/
void lcd_write_string_4d(uint8_t theString[])
{
    volatile int i = 0;     /*Character counter*/
    while (theString[i] != 0)
    {
        /*Write each byte one character at at time*/
        lcd_write_character_4d(theString[i]);
        i++;
        _delay_us(80);
    }
}

/*Write a character to the LCD display*/
void lcd_write_character_4d(uint8_t theData)
{
    PORTB.OUTSET = PIN4_bm;     /* Select the Data Register (RS high)*/
    PORTB.OUTCLR = PIN3_bm;     /* Make sure E is initially low */
    lcd_write_4(theData);       /* Write the upper 4-bits of the data */
    lcd_write_4(theData << 4);  /* Write the lower 4-bits of the data */
}

/*Write an instruction to the LCD*/
void lcd_write_instruction_4d(uint8_t theInstruction)
{
    PORTB.OUTCLR = PIN4_bm;     /* Select the Instruction Register (RS low) */
    PORTB.OUTCLR = PIN3_bm;     /* Make sure E is initially low */
    lcd_write_4(theInstruction);        /* Write the upper 4-bits of the data */
    lcd_write_4(theInstruction << 4);   /* Write the lower 4-bits of the data */
}

/*Write 4 bits of data to the LCD*/
void lcd_write_4(uint8_t theByte)
{
    PORTD.OUTCLR = PIN7_bm; /* Assume that data is '0'*/
    if (theByte & 1<<7)     /* Make data = '1' if necessary*/
    {
        PORTD.OUTSET = PIN7_bm;
    }
    
    PORTD.OUTCLR = PIN6_bm; /* Repeat for each data bit*/
    if (theByte & 1<<6)
    {
        PORTD.OUTSET = PIN6_bm;
    }

    PORTD.OUTCLR = PIN5_bm;
    if (theByte & 1<<5)
    {
        PORTD.OUTSET = PIN5_bm;
    }

    PORTD.OUTCLR = PIN4_bm;
    if (theByte & 1<<4)
    {
        PORTD.OUTSET = PIN4_bm;
    }

    /* Enable pin high */
    PORTB.OUTSET = PIN3_bm;
    _delay_us(1);   /*Wait for data to go through*/
    /* Enable pin low */
    PORTB.OUTCLR = PIN3_bm;
    _delay_us(1);
}



/*RTC initialization function*/
void rtc_init(void)
{
	/*Temporary variable for setting different register bits */
    uint8_t temp;
	/*Initialize 32.768kHz	Oscillator */
	/*Disable oscillator */
    temp = CLKCTRL.XOSC32KCTRLA;
    temp &= ~CLKCTRL_ENABLE_bm;
	/*Writing to protected register */
    ccp_write_io((void*)&CLKCTRL.XOSC32KCTRLA, temp);

	/*Empty loop to wait until XOSC32KS becomes 0 */
    while(CLKCTRL.MCLKSTATUS & CLKCTRL_XOSC32KS_bm)
    {
    }
    /*Clearing the SEL bit in CLKCTRL.XOSC32KCTRLA 
     *register to select external oscillator */
    temp = CLKCTRL.XOSC32KCTRLA;
    temp &= ~CLKCTRL_SEL_bm;
    /*Writing to protected register */
    ccp_write_io((void*)&CLKCTRL.XOSC32KCTRLA, temp);

    /*Enable oscillator */
    temp = CLKCTRL.XOSC32KCTRLA;
    temp |= CLKCTRL_ENABLE_bm;
    /*Writing to protected register */
    ccp_write_io((void*)&CLKCTRL.XOSC32KCTRLA, temp);

    /*Initialize RTC */
	/*Empty loop to wait for all the registers to be synchronized */
    while (RTC.STATUS > 0)
    {
    }

    /*Selecting the 32.768 kHz External Crystal Oscillator (XOSC32KS) */
    RTC.CLKSEL = RTC_CLKSEL_TOSC32K_gc;
    /*Debug enabled, not sure if needed in this project */
    RTC.DBGCTRL = RTC_DBGRUN_bm;
    /*Enabling Periodic Interrupt */
    RTC.PITINTCTRL = RTC_PI_bm;
    /*Selecting RTC Clock Cycles 32678 and enabling RTC with bitmask macro */
    RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc | RTC_PITEN_bm;
}

/*ADC initialization function */
void adc0_init(void)
{
    /* Disable digital input buffer to have highest possible input impedance*/
    PORTE.PIN0CTRL &= ~PORT_ISC_gm;
    PORTE.PIN0CTRL |= PORT_ISC_INPUT_DISABLE_gc;
    /* Disable pull-up resistor to have highest possible input impedance*/
    PORTE.PIN0CTRL &= ~PORT_PULLUPEN_bm;
    ADC0.CTRLC = ADC_PRESC_DIV16_gc /*CLK_PER divided by 16 as instructed in earlier exercise materials */
        | ADC_REFSEL_INTREF_gc; /*Internal reference */
    VREF.CTRLA = VREF_ADC0REFSEL_2V5_gc; /* Reference voltage for ADC */
    ADC0.CTRLA = ADC_ENABLE_bm /*Enabling ADC */
        | ADC_RESSEL_10BIT_gc; /*10-bit ADC resolution mode */
    /*Select ADC channel AINn*/
    ADC0.MUXPOS = ADC_MUXPOS_AIN8_gc;
    /*Enable FreeRun mode to get RPM readings throughout running the program */
    ADC0.CTRLA |= ADC_FREERUN_bm;

}

/*Function to read ADC output */
uint16_t adc0_read(void)
{
    /*Clear the interrupt flag by writing 1 */
    ADC0.INTFLAGS = ADC_RESRDY_bm;
    return ADC0.RES;
}

/*Function to start ADC conversion */
void adc0_start(void)
{
    /*Start conversion */
    ADC0.COMMAND = ADC_STCONV_bm;
}

/*Function to check if conversion is done with usage of boolean from <stdbool.h> */
bool adc0_conversionDone(void)
{
    return (ADC0.INTFLAGS & ADC_RESRDY_bm);
}


/*Interrupt for updating LCD to display RPM using RTC PIT */
ISR(RTC_PIT_vect)
{
    /*Clear LCD instruction */
    lcd_write_instruction_4d(LCD_CLEAR);
    /*Delay 4ms to wait for LCD to finish instruction */
    _delay_ms(4);
    /*Upscale counter to represent real RPM value. Multiplying by 60 to get 
     *rotations per minute and dividing by 2 to account for 2 rotor blades */
    counter = counter*30;
    /*C library function to format integer to string for LCD to output */
    sprintf(c_string,"%d",counter);
    /*Write the string to LCD screen */
    lcd_write_string_4d(c_string);
    /*Delay 80 us for LCD to finish writing */
    _delay_us(80);
    /*Reset counter */
    counter = 0;
    /*Clear all RTC interrupt flags */
    RTC.PITINTFLAGS = RTC_PI_bm;
}

/*Interrupt for DC Motor control */
ISR(PORTF_PORT_vect)
{
    /* Toggle the DC motor ON/OFF */
    PORTA.OUTTGL = PIN2_bm;
    /* Clear all interrupt flags */
    PORTF.INTFLAGS = 0xFF;
}