/*
 * File:   ME433_HW1_Part1.c
 * Author: derek_000
 *
 * Created on April 1, 2015, 10:13 PM
 */

/******************************************************************************
 * Software License Agreement
 *
 * Copyright © 2011 Microchip Technology Inc.  All rights reserved.
 * Microchip licenses to you the right to use, modify, copy and distribute
 * Software only when embedded on a Microchip microcontroller or digital
 * signal controller, which is integrated into your product or third party
 * product (pursuant to the sublicense terms in the accompanying license
 * agreement).
 *
 * You should refer to the license agreement accompanying this Software
 * for additional information regarding your rights and obligations.
 *
 * SOFTWARE AND DOCUMENTATION ARE PROVIDED Ã¬AS ISÃ® WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY
 * OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR
 * PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR
 * OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,
 * BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT
 * DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL,
 * INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA,
 * COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY
 * CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
 * OR OTHER SIMILAR COSTS.
 *
 *****************************************************************************/


#include <p32xxxx.h>
#include<xc.h> // processor SFR definitions
#include<sys/attribs.h> // __ISR macro

// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // not boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // free up secondary osc pins by turning sosc off
#pragma config FPBDIV = DIV_1 // divide CPU freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1048576 // slowest wdt
#pragma config WINDIS = OFF // no wdt window
#pragma config FWDTEN = OFF // wdt off by default
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the CPU clock to 40MHz
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL =  MUL_20 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 40MHz
#pragma config UPLLIDIV = DIV_2 // divide 8MHz input clock, then multiply by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 0 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = ON // allow only one reconfiguration
#pragma config IOL1WAY = ON // allow only one reconfiguration
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // controlled by USB module

//	Function Prototypes
int main(void);
int readADC(void);

int main(void) {
    
//Startup
    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that
    // kseg0 is cacheable (0x3) or uncacheable (0x2)
    // see Chapter 2 "CPU for Devices with M4K Core"
    // of the PIC32 reference manual
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // no cache on this chip!

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to be able to use TDI, TDO, TCK, TMS as digital
    DDPCONbits.JTAGEN = 0;

    __builtin_enable_interrupts();

ANSELBbits.ANSB13 = 0; // 0 for digital, 1 for analog
ANSELBbits.ANSB15 = 0; // 0 for digital, 1 for analog

    T2CONbits.TCKPS = 0;	//Setting prescaler to 1 (0 corresponds to 1)
    PR2 = 39999;            //Setting PR for timer 2 to 39999
    TMR2 = 0;				//Setting Timer 2 to 0
    OC1CONbits.OCTSEL = 0;	//Telling OC1 to use timer 2
    OC1CONbits.OCM = 0b110;	//Telling OC1 to use PWM without the fault
    OC1RS = 20000;			//Setting initial duty cycle to 20000/(39999+1)*100% = 50%
    OC1R = 20000;			//Updating duty cycles to 20000/(39999+1)*100% = 50%
    T2CONbits.ON = 1;		//turn on timer
    OC1CONbits.ON = 1;		//turn on OC code

// set up USER pin as input
    TRISBbits.TRISB13 = 1; // set pin B13 to be digital INPUT
    // U1RXRbits.U1RXR = 0b0011; // set U1RX to pin B13 (Input pin from User button)

// set up LED1 pin as a digital output
    TRISBbits.TRISB7 = 0; // set pin B7 to be digital OUTPUT
    // LATBbits.LATB7 = 1;
    // RPB7Rbits.RPB7R = 0b0001; //set B7 to U1TX (Output pin for LED1)
    
// set up LED2 as OC1 using Timer2 at 1kHz
    // TRISBbits.TRISB15 = 0; // set B15 to digital OUTPUT
    RPB15Rbits.RPB15R = 0b0101; // set B15 to U1TX (Output pin for OC1)

// set up A0 as AN0
    ANSELAbits.ANSA0 = 1;
    AD1CON3bits.ADCS = 3;
    AD1CHSbits.CH0SA = 0;
    AD1CON1bits.ADON = 1;
    
    while (1){
     // invert pin every 0.5s, set PWM duty cycle % to the pot voltage output %
        _CP0_SET_COUNT(0);
        LATBINV = 0b10000000;
        
            while(_CP0_GET_COUNT()<10000000){
                OC1RS = readADC()*(PR2+1)/1023; // delay for 10M core ticks, 0.5s
                if (PORTBbits.RB13 == 1){
                    ;// nothing
                }
                else{
                    LATBINV = 0b10000000;
                }
            }
        
    }
}

int readADC(void) {
    int elapsed = 0;
    int finishtime = 0;
    int sampletime = 20;
    int a = 0;

    AD1CON1bits.SAMP = 1;
    elapsed = _CP0_GET_COUNT();
    finishtime = elapsed + sampletime;
    while (_CP0_GET_COUNT() < finishtime) {
    }
    AD1CON1bits.SAMP = 0;
    while (!AD1CON1bits.DONE) {
    }
    a = ADC1BUF0;
    return a;
}



