/*
 * File:   main.c
 * Author: Hari prasath
 *
 * Created on 12 January, 2021, 10:51 AM
 * Modified 04 September, 2021, 05:40 PM
 */

#include <string.h>
#include <stdlib.h>

#define _With_LCD
#define _PIC_18F46K22
#define _OSC_20MHZ
#define _Dummy_Out
#define _With_EEPROM
#define _Settings

#ifdef _OSC_20MHZ
#define _XTAL_FREQ 20000000
#endif

#ifdef _OSC_8MHZ
#define _XTAL_FREQ 8867233
#endif

#ifdef _With_LCD
//LCD Data and Control pins ---- OUTPUT
#define RS PORTBbits.RB2
#define EN PORTBbits.RB3
#define D4 PORTBbits.RB4
#define D5 PORTBbits.RB5
#define D6 PORTBbits.RB6
#define D7 PORTBbits.RB7
#endif

#ifdef _With_LCD
//LCD Back light control pin ---- OUTPUT
#define LED PORTBbits.RB1
#endif

#ifdef _Dummy_Out
//Relay control pin ---- OUTPUT
#define out PORTDbits.RD2
#endif

#ifdef _Settings
//Tactile button ---- INPUT
#define inc PORTCbits.RC0
#define dec PORTCbits.RC1
#define enter PORTCbits.RC2
#endif

#ifdef _PIC_16F887
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator: High-speed crystal/resonator on RA6/OSC2/CLKOUT and RA7/OSC1/CLKIN)
#pragma config WDTE = ON        // Watchdog Timer Enable bit (WDT enabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = ON       // RE3/MCLR pin function select bit (RE3/MCLR pin function is MCLR)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)
#endif

#ifdef _PIC_18F46K22


#endif

#include <xc.h>

#ifdef _With_LCD
char a,b,c,d,y,z,i;     //LCD functions variables
#endif

char j=0,k=0,l=0;      //for loop variables

char settings[8]="Settings";

char saved[5]="Saved";

char lux;
int status1,status2;

char backlight=1;     //lcd backlight flag
char relay=0;         //relay control flag


#ifdef _With_LCD
#include "lcd.h"
#endif

#ifdef _With_EEPROM
#include "eeprom.h"
#endif

#ifdef _With_RTC
#include "i2c.h"
#endif

#include "functions.h"
#include "enums.h"
 
void main(void)
{
    char no=0;
    TRISB=0X01;    // LCD DISPLAY CONNECTED
    TRISC=0X17;

 //----------------------------------------mode select switches setting-----------------------------------------------------------

    TRISD=0x03;
    PORTB=0xff;
    PORTC=0xff;
    PORTD=0xff;
    
    INTCONbits.GIE = set_bit.HIGH;
    __delay_ms(10);
    
    status1 = SSPSTAT;
    status2 = SSPCON2;
    
    //INTCONbits.GIE = set_bit.HIGH;
    //OPTION_REGbits.PSA = set_bit.HIGH;
    //OPTION_REGbits.PS = 0b111;
    __delay_ms(10);
    
    rtc_init();
    
    __delay_ms(10);
    lcd_init();
    __delay_ms(10);
    lcd_clear();
    __delay_ms(10);
    
    __delay_ms(10);
    status_check();
    __delay_ms(10);
    
    __delay_ms(10);
    welcome_msg();
    __delay_ms(2000);
    
    
    
    while(1)
    {
        __delay_ms(10);
        status_check();
        __delay_ms(1);
        
    }  
}
