#define F_CPU 3333333 //UL loppuun?

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdbool.h> //kokeilu
#include <avr/cpufunc.h>


// LCD module information
#define lcd_LineOne     0x00                    // start of line 1
#define lcd_LineTwo     0x40                    // start of line 2

// LCD instructions
#define lcd_Clear           0b00000001          // replace all characters with ASCII 'space'
#define lcd_Home            0b00000010          // return cursor to first position on first line
#define lcd_EntryMode       0b00000110          // shift cursor from left to right on read/write
#define lcd_DisplayOff      0b00001000          // turn display off
#define lcd_DisplayOn       0b00001100          // display on, cursor off, don't blink character
#define lcd_FunctionReset   0b00110000          // reset the LCD
#define lcd_FunctionSet4bit 0b00101000          // 4-bit data, 2-line display, 5 x 7 font
#define lcd_SetCursor       0b10000000          // set cursor position

uint8_t program_author[]   = "RPM";
uint8_t program_version[]  = "testaa";
uint8_t program_date[]     = "2020";
uint8_t c_string[6];
volatile int counter = 0;
uint16_t adcVal; //kokeilu
uint16_t ADC0_read(void); //kokeilu

// Function Prototypes
void lcd_write_4(uint8_t);
void lcd_write_instruction_4d(uint8_t);
void lcd_write_character_4d(uint8_t);
void lcd_write_string_4d(uint8_t *);
void lcd_init_4d(void);
void rtc_init(void);
void ADC0_init(void); //kokeilu
void ADC0_start(void);//kokeilu
bool ADC0_conversionDone(void);//kokeilu

int main(void)
{
    // configure the microprocessor pins for the data lines
    PORTD.DIRSET = PIN7_bm | PIN6_bm | PIN5_bm | PIN4_bm; // 8 data lines - output
    PORTB.DIRSET = PIN3_bm | PIN4_bm; //portB3 E:ss‰ ja portB4 RS:ss‰
    // E line - output
    // RS line - output
    PORTA.DIRSET = PIN2_bm;
    //PORTF.IN &= PIN6_bm;
    
    PORTE.DIRCLR = PIN0_bm; //LDR-johto
    PORTF.DIRCLR = PIN6_bm; //ISR (AVR nappi)
    PORTF.DIRSET = PIN5_bm; //ISR (AVR LED)
                            
    //When button goes to low state, trigger interrupt
    PORTF.PIN6CTRL = PORT_ISC_FALLING_gc;
    
    

// initialize the LCD controller as determined by the defines (LCD instructions)
    lcd_init_4d();                                  // initialize the LCD display for an 8-bit interface

// display the first line of information
    lcd_write_string_4d(program_author);

// set cursor to start of second line
    lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
    _delay_us(80);                                  // 40 uS delay (min)
    
    
    
     //perusasetus RTC kelloille
    rtc_init();
    
    ADC0_init(); //kokeilu
    //adcVal = ADC0_read(); //kokeilu
    ADC0_start();
    
    sei();
    
    // endless loop
    while(1)
    {
        if (ADC0_conversionDone()) //kokeilu
        {
            adcVal = ADC0_read();
            /* In FreeRun mode, the next conversion starts automatically */
            if (adcVal>400) //PORTE.IN & PIN0_bm vaihdettu
            {
                PORTF.OUTSET = PIN5_bm;
                counter++;
                while(adcVal>400) //PORTE.IN & PIN0_bm vaihdettu
                {
                    if (ADC0_conversionDone()) //kokeilu
                    {
                        adcVal = ADC0_read();
                        /* In FreeRun mode, the next conversion starts automatically */
                    }
                }
            }
            else if (!(PORTE.IN & PIN0_bm))
            {
                PORTF.OUTCLR = PIN5_bm;
            
            }
        }
    }
    cli();
    
    return 0;
}

void lcd_init_4d(void)
{
// Power-up delay
    _delay_ms(100);                                 // initial 40 mSec delay

// Set up the RS and E lines for the 'lcd_write_4' subroutine.
    PORTB.OUTCLR = PIN4_bm; // select the Instruction Register (RS low)
    PORTB.OUTCLR = PIN3_bm; // make sure E is initially low
    
// Reset the LCD controller
    lcd_write_4(lcd_FunctionReset);                 // first part of reset sequence
    _delay_ms(10);                                  // 4.1 mS delay (min)

    lcd_write_4(lcd_FunctionReset);                 // second part of reset sequence
    _delay_us(200);                                 // 100uS delay (min)

    lcd_write_4(lcd_FunctionReset);                 // third part of reset sequence
    _delay_us(200);                                 // this delay is omitted in the data sheet

// Preliminary Function Set instruction - used only to set the 4-bit mode.
// The number of lines or the font cannot be set at this time since the controller is still in the
//  8-bit mode, but the data transfer mode can be changed since this parameter is determined by one 
//  of the upper four bits of the instruction.
 
    lcd_write_4(lcd_FunctionSet4bit);               // set 4-bit mode
    _delay_us(80);                                  // 40uS delay (min)

// Function Set instruction
    lcd_write_instruction_4d(lcd_FunctionSet4bit);   // set mode, lines, and font
    _delay_us(80);                                  // 40uS delay (min)

// The next three instructions are specified in the data sheet as part of the initialization routine, 
//  so it is a good idea (but probably not necessary) to do them just as specified and then redo them 
//  later if the application requires a different configuration.

// Display On/Off Control instruction
    lcd_write_instruction_4d(lcd_DisplayOff);        // turn display OFF
    _delay_us(80);                                  // 40uS delay (min)

// Clear Display instruction
    lcd_write_instruction_4d(lcd_Clear);             // clear display RAM
    _delay_ms(4);                                   // 1.64 mS delay (min)

// ; Entry Mode Set instruction
    lcd_write_instruction_4d(lcd_EntryMode);         // set desired shift characteristics
    _delay_us(80);                                  // 40uS delay (min)

// This is the end of the LCD controller initialization as specified in the data sheet, but the display
//  has been left in the OFF condition.  This is a good time to turn the display back ON.
 
// Display On/Off Control instruction
    lcd_write_instruction_4d(lcd_DisplayOn);         // turn the display ON
    _delay_us(80);                                  // 40uS delay (min)
}

void lcd_write_string_4d(uint8_t theString[])
{
    volatile int i = 0;                             // character counter*/
    while (theString[i] != 0)
    {
        lcd_write_character_4d(theString[i]);
        i++;
        _delay_us(80);                              // 40 uS delay (min)
    }
}

void lcd_write_character_4d(uint8_t theData)
{
    PORTB.OUTSET = PIN4_bm; // select the Data Register (RS high)
    PORTB.OUTCLR = PIN3_bm; // make sure E is initially low
    lcd_write_4(theData);                           // write the upper 4-bits of the data
    lcd_write_4(theData << 4);                      // write the lower 4-bits of the data
}

void lcd_write_instruction_4d(uint8_t theInstruction)
{
    PORTB.OUTCLR = PIN4_bm; // select the Instruction Register (RS low)
    PORTB.OUTCLR = PIN3_bm; // make sure E is initially low
    lcd_write_4(theInstruction);                    // write the upper 4-bits of the data
    lcd_write_4(theInstruction << 4);               // write the lower 4-bits of the data
}


void lcd_write_4(uint8_t theByte)
{
    PORTD.OUTCLR = PIN7_bm; // assume that data is '0'
    if (theByte & 1<<7) // make data = '1' if necessary
    {
        PORTD.OUTSET = PIN7_bm;
    }
    
    PORTD.OUTCLR = PIN6_bm; // repeat for each data bit
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

// write the data
    // 'Address set-up time' (40 nS)
    // Enable pin high
    PORTB.OUTSET = PIN3_bm;                 //luetaan data E:ll‰ 
    _delay_us(1);                                   // implement 'Data set-up time' (80 nS) and 'Enable pulse width' (230 nS)
    // Enable pin low
    PORTB.OUTCLR = PIN3_bm;
    _delay_us(1);                                   // implement 'Data hold time' (10 nS) and 'Enable cycle time' (500 nS)
}




void rtc_init(void)
{
    //Asetetaan se 32.678kHz oskillaattori ja disabletaan ton prosun sis‰inen
    uint8_t temp;
    temp = CLKCTRL.XOSC32KCTRLA;
    temp &= ~CLKCTRL_ENABLE_bm;
    //T‰ on rekisteriin kirjotusmetodi, tuol matskuis lis‰‰ jos kiinnostaa
    ccp_write_io((void*)&CLKCTRL.XOSC32KCTRLA, temp);

    //Tyhj‰ loop, jonka ideana odottaa ett‰ XOSC32KS muuttuu 0
    //Joka meinaa siis et se on unstable (miks ikin‰ se sit pit‰‰k‰‰ olla nii)
    while(CLKCTRL.MCLKSTATUS & CLKCTRL_XOSC32KS_bm)
    {
        ;
    }
    //Valitaan oskillaattori mei‰n "external sourceks" kelloon
    temp = CLKCTRL.XOSC32KCTRLA;
    temp &= ~CLKCTRL_SEL_bm;
    //Sama rekisterilause
    ccp_write_io((void*)&CLKCTRL.XOSC32KCTRLA, temp);

    //Sitku oskillaattori valittu, se enabletaan t‰s
    temp = CLKCTRL.XOSC32KCTRLA;
    temp |= CLKCTRL_ENABLE_bm;
    //J‰lleen rekisteriin kirjotetaan
    ccp_write_io((void*)&CLKCTRL.XOSC32KCTRLA, temp);

    //T‰ss‰ venataan et "kaikki rekisterit ovat synkronoituneet"
    while (RTC.STATUS > 0)
    {
        ;
    }

    //Ja vihdoin t‰ss‰ kohtaa p‰‰st‰‰n valittee se 32.678 khz
    RTC.CLKSEL = RTC_CLKSEL_TOSC32K_gc;
    //Debug mode, en tii‰ tarvitaanko mei‰n projus
    RTC.DBGCTRL = RTC_DBGRUN_bm;
    //Enabletaan PIT (periodic interrupt)
    RTC.PITINTCTRL = RTC_PI_bm;
    //Syklien lukum‰‰r‰ (CYC32678 jos halutaan 1 per sekunti)
    RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc
            | RTC_PITEN_bm; //T‰ kohta on itse sen RTC:n k‰ynnistyslause

}

void ADC0_init(void) //kokeilu
{
 /* Disable digital input buffer */
 PORTE.PIN0CTRL &= ~PORT_ISC_gm;
 PORTE.PIN0CTRL |= PORT_ISC_INPUT_DISABLE_gc;
 /* Disable pull-up resistor */
 PORTE.PIN0CTRL &= ~PORT_PULLUPEN_bm;
 ADC0.CTRLC = ADC_PRESC_DIV16_gc /* CLK_PER divided by 16 */
         | ADC_REFSEL_VREFA_gc; /* Internal reference */
 ADC0.CTRLA = 0x4;//ADC_REFSEL_VREFA_gc;
 ADC0.CTRLA = ADC_ENABLE_bm /* ADC Enable: enabled */
    | ADC_RESSEL_10BIT_gc; /* 10-bit mode */
 /* Select ADC channel */
 ADC0.MUXPOS = ADC_MUXPOS_AIN8_gc;
 /* Enable FreeRun mode */
 ADC0.CTRLA |= ADC_FREERUN_bm;

}

uint16_t ADC0_read(void) //kokeilu
{
 /* Start ADC conversion */
 //ADC0.COMMAND = ADC_STCONV_bm;
 /* Wait until ADC conversion done */
 //while ( !(ADC0.INTFLAGS & ADC_RESRDY_bm) )
 //{
 //;
 //}
 /* Clear the interrupt flag by writing 1: */
 ADC0.INTFLAGS = ADC_RESRDY_bm;
 return ADC0.RES;
}

void ADC0_start(void) //kokeilu
{
 /* Start conversion */
 ADC0.COMMAND = ADC_STCONV_bm;
}

bool ADC0_conversionDone(void) //kokeilu
{
 return (ADC0.INTFLAGS & ADC_RESRDY_bm);
}


//Interrupt Service Routine with interrupt vector as parameter
ISR(RTC_PIT_vect)
{
    // Clear Display instruction
    lcd_write_instruction_4d(lcd_Clear);             // clear display RAM
    _delay_ms(4);
    counter = counter*60;
    // 1.64 mS delay (min)
    //itoa(counter, c_string, 10);
    sprintf(c_string,"%d",adcVal); //counter poistettu
    lcd_write_string_4d(c_string);
    _delay_us(80);
    //lcd_write_instruction_4d(lcd_Clear);
    counter = 0;
    //Clear all interrupt flags
    RTC.PITINTFLAGS = RTC_PI_bm;
}

ISR(PORTF_PORT_vect)
{
    PORTA.OUTTGL = PIN2_bm;
    PORTF.INTFLAGS = 0xFF; //clear interrupt flags
}