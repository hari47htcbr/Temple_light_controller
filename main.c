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
// PIC18F46K22 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1H
#pragma config FOSC = HSHP      // Oscillator Selection bits (HS oscillator (high power > 16 MHz))
#pragma config PLLCFG = OFF     // 4X PLL Enable (Oscillator used directly)
#pragma config PRICLKEN = ON    // Primary clock enable bit (Primary clock is always enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRTEN = OFF     // Power-up Timer Enable bit (Power up timer disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled and controlled by software (SBOREN is enabled))
#pragma config BORV = 220       // Brown Out Reset Voltage bits (VBOR set to 2.20 V nominal)

// CONFIG2H
#pragma config WDTEN = ON       // Watchdog Timer Enable bits (WDT is always enabled. SWDTEN bit has no effect)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC1  // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<5:0> pins are configured as digital I/O on Reset)
#pragma config CCP3MX = PORTB5  // P3A/CCP3 Mux bit (P3A/CCP3 input/output is multiplexed with RB5)
#pragma config HFOFST = OFF     // HFINTOSC Fast Start-up (HFINTOSC output and ready status are delayed by the oscillator stable status)
#pragma config T3CMX = PORTC0   // Timer3 Clock input mux bit (T3CKI is on RC0)
#pragma config P2BMX = PORTD2   // ECCP2 B output mux bit (P2B is on RD2)
#pragma config MCLRE = EXTMCLR  // MCLR Pin Enable bit (MCLR pin enabled, RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection Block 0 (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection Block 1 (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection Block 2 (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection Block 3 (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection Block 0 (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection Block 1 (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection Block 2 (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection Block 3 (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection Block 0 (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection Block 1 (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection Block 2 (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection Block 3 (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
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
    
    INTCONbits.GIE = HIGH;
    __delay_ms(10);
    
    status1 = SSPSTAT;
    status2 = SSPCON2;
    
    //INTCONbits.GIE = set_bit.HIGH;
    //OPTION_REGbits.PSA = set_bit.HIGH;
    //OPTION_REGbits.PS = 0b111;
    __delay_ms(10);
#ifdef _With_RTC
    rtc_init();
#endif
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
