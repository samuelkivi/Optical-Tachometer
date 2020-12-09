/*
 * File:   main.c
 * Author: Roope Pouta, Samuel Kivi, Tomas Kobra, Jesse Järvi
 * Project: Optical tachometer
 */
#define F_CPU 3333333UL

#include <stdio.h>
#include <avr/io.h>	
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>	
#include <xc.h>

#define LCD_init 0x33 
#define LCD_8bit 0x38
#define	LCD_displayOn_CursorBlink 0x0F
#define	LCD_displayOn_CursorOff 0x0C 
#define	LCD_clear 0x01
#define LCD_cursor_1_line_start 0x80		
#define LCD_cursor_2_line_start 0xC0
#define LCD_cursor_increment 0x06
#define LCD_cursor_decrement 0x04
#define delay_enable 60
#define delay_lcd_init 100
#define delay_lcd_clear 2
#define delay_lcd_set 60
#define big_delay 2000
#define small_delay 500
#define black_box 0xFF
#define ADC_threshold 125

volatile uint16_t ADC = 0;      // Value of ADC
volatile uint8_t updateLCD = 0;     // Help variable for LCD-update 

volatile uint16_t count;     // Variable for counting interrupts per second
volatile uint16_t rpm;      // Variable for counting rpm

// Set LCD-ports
void LCD_Command( uint8_t cmnd )
{	
    PORTD.OUT = cmnd;
	PORTB.OUTCLR = PIN4_bm;				// RS=0, command reg.
	PORTB.OUTSET = PIN3_bm;				// Enable pulse
	_delay_us(delay_enable);
	PORTB.OUTCLR = PIN3_bm;             // Clears pin3

	_delay_us(delay_lcd_set);
}

// Set LCD-ports
void LCD_Char( uint8_t data )
{
	PORTD.OUT = data;
	PORTB.OUTSET = PIN4_bm;				// RS=1, command reg.
	PORTB.OUTSET = PIN3_bm;				// Enable pulse
	_delay_us(delay_enable);
	PORTB.OUTCLR = PIN3_bm;             // Clears pin3
	_delay_us(delay_lcd_set);
    LCD_Command(LCD_cursor_increment);  
}

// Initialize LCD-screen
void LCD_Init (void)					
{
    PORTB.DIRSET = (PIN3_bm | PIN4_bm | PIN5_bm); // Enables pins
    PORTB.OUTSET = PIN5_bm;                       // Sets pin 5 for output
    PORTB.OUTCLR = (PIN3_bm | PIN4_bm);           // Clears pins 3 & 4
	PORTD.DIR = 0xFF;                             // Enablses dataports 
    PORTD.OUT = 0x00;                             // Sets dataports for output
    _delay_us(delay_lcd_set);						
	
 
	LCD_Command(LCD_init);
    LCD_Command(LCD_8bit);
	LCD_Command(LCD_displayOn_CursorOff);              	             	
	LCD_Command(LCD_clear); 
    LCD_Command(LCD_cursor_1_line_start);
	_delay_us(delay_lcd_init);    
}

// Enables writing to the LCD-screen
void LCD_String (char *str)				
{
	int i;
	for(i=0;str[i]!='\0';i++)				
	{
		LCD_Char(str[i]);
	}
}

// Clears LCD-screen
void LCD_Clear()
{
	LCD_Command(LCD_clear);					
	_delay_ms(delay_lcd_clear);
	LCD_Command(LCD_cursor_1_line_start);	
    _delay_ms(delay_lcd_clear);
}

// Initialize ADC
void ADC0_init(void)
{
    // Disable digital input buffer
    PORTE.PIN0CTRL &= ~PORT_ISC_gm;
    PORTE.PIN0CTRL |= PORT_ISC_INPUT_DISABLE_gc;
    
    // Disable pull-up resistor
    PORTE.PIN0CTRL &= ~PORT_PULLUPEN_bm;
    
    // CLK_PER divided by 16, Internal reference
    ADC0.CTRLC = ADC_PRESC_DIV16_gc | ADC_REFSEL_INTREF_gc;
    
    // ADC enabled, 10-bit mode
    ADC0.CTRLA = ADC_ENABLE_bm | ADC_RESSEL_10BIT_gc;
    
    // Select ADC channel
    ADC0.MUXPOS = ADC_MUXPOS_AIN8_gc;
    
    // Set internal reference
    VREF.CTRLA |= VREF_ADC0REFSEL_1V5_gc; 
    
    // Enable interrupts
    ADC0.INTCTRL |= ADC_RESRDY_bm;    
}

int main(void) 
{   
    // Set up periodic cycle
    RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc;
    
    // Initialize ADC
    ADC0_init();	
    
    // Enable RTC interrupt timer
    RTC.PITINTCTRL = RTC_PI_bm;
    
    // Sets LDR pin
    PORTE.DIRCLR = PIN0_bm;
    
    // Sets dc-motor pin
    PORTA.DIRSET = PIN3_bm;
    
    // Initialize sleep_mode
    set_sleep_mode(SLPCTRL_SMODE_IDLE_gc);
    
    // Initialize LCD-screen
    cli();          // Disables interrupts
    LCD_Init();    
    sei();          // Enables interrupts

    // RTC enable
    RTC.PITCTRLA = RTC.PITCTRLA | RTC_PITEN_bm;
    
    // Old ADC-value for comparison
    uint8_t exValue = 0;
    
    // Prints RPM-count.
    LCD_Clear();
    LCD_String("RPM Count:");
    
    // Turns on ADC-conversion
    ADC0.COMMAND = ADC_STCONV_bm;       
    
    // Turns on dc-motor
    PORTA.OUT |= PIN3_bm;
    
    // Initialize string for LCD-screen text
    char string[5];
    
    while(1)
    {   
        if (updateLCD)
        {
            // Updating screen
            rpm = count * 30;   // 1s * (60 / 2) = RPM, because of 2 blades. 
            sprintf(string, "%-4u", rpm); // Send to LCD
            LCD_Command(LCD_cursor_2_line_start);
            LCD_String(string);     // Sets string to screen.
            count = 0;              // Reset counter
            updateLCD = 0;          // Reset counter
        }
        
        // Counter, if over 250 adds count
        else
        {   
            // Checks if Photoresistor is in dark.
            if (ADC > ADC_threshold && exValue == 0)
            {
                count++;
                exValue = 1;      // Comparison value
            }
            // Leaves out counts if loop is going too fast for 1 bit.
            else if (ADC > ADC_threshold && exValue == 1)
            {
                exValue = 1;      // Comparison value
            }
            else
            {
                exValue = 0;      // Comparison value
            }     
        }
        sleep_mode();
    }    
}

// Gives interrupt once per second to LCD-screen
ISR(RTC_PIT_vect)
{      
    RTC.PITINTFLAGS = RTC_PI_bm;
    updateLCD = 1;
}

// ADC-interrupt
ISR(ADC0_RESRDY_vect)
{
    // Set ADC value
    ADC = ADC0.RES;
    
    ADC0.COMMAND = ADC_STCONV_bm;
}