/*******************************************************************************
  MPLAB Harmony Project Main Source File

  Company:
    Microchip Technology Inc.
  
  File Name:
    main.c

  Summary:
    This file contains the "main" function for an MPLAB Harmony project.

  Description:
    This file contains the "main" function for an MPLAB Harmony project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state 
    machines of all MPLAB Harmony modules in the system and it calls the 
    "SYS_Tasks" function from within a system-wide "super" loop to maintain 
    their correct operation. These two functions are implemented in 
    configuration-specific files (usually "system_init.c" and "system_tasks.c")
    in a configuration-specific folder under the "src/system_config" folder 
    within this project's top-level folder.  An MPLAB Harmony project may have
    more than one configuration, each contained within it's own folder under
    the "system_config" folder.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

//Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "system/common/sys_module.h"   // SYS function prototypes
#include <p32xxxx.h>
#include<xc.h> // processor SFR definitions
#include<sys/attribs.h> // __ISR macro
#include "i2c_display.h"
#include "i2c_master_int.h"
#include "accel.h"

// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

#define LENGTH 100

char buffer[LENGTH], buffer1[LENGTH], buffer2[LENGTH], message1[LENGTH], message2[LENGTH];
int x_line_point, y_line_point, i, j;

// lookup table for all of the ascii characters
static const char ASCII[96][5] = {
 {0x00, 0x00, 0x00, 0x00, 0x00} // 20  (space)
,{0x00, 0x00, 0x5f, 0x00, 0x00} // 21 !
,{0x00, 0x07, 0x00, 0x07, 0x00} // 22 "
,{0x14, 0x7f, 0x14, 0x7f, 0x14} // 23 #
,{0x24, 0x2a, 0x7f, 0x2a, 0x12} // 24 $
,{0x23, 0x13, 0x08, 0x64, 0x62} // 25 %
,{0x36, 0x49, 0x55, 0x22, 0x50} // 26 &
,{0x00, 0x05, 0x03, 0x00, 0x00} // 27 '
,{0x00, 0x1c, 0x22, 0x41, 0x00} // 28 (
,{0x00, 0x41, 0x22, 0x1c, 0x00} // 29 )
,{0x14, 0x08, 0x3e, 0x08, 0x14} // 2a *
,{0x08, 0x08, 0x3e, 0x08, 0x08} // 2b +
,{0x00, 0x50, 0x30, 0x00, 0x00} // 2c ,
,{0x08, 0x08, 0x08, 0x08, 0x08} // 2d -
,{0x00, 0x60, 0x60, 0x00, 0x00} // 2e .
,{0x20, 0x10, 0x08, 0x04, 0x02} // 2f /
,{0x3e, 0x51, 0x49, 0x45, 0x3e} // 30 0
,{0x00, 0x42, 0x7f, 0x40, 0x00} // 31 1
,{0x42, 0x61, 0x51, 0x49, 0x46} // 32 2
,{0x21, 0x41, 0x45, 0x4b, 0x31} // 33 3
,{0x18, 0x14, 0x12, 0x7f, 0x10} // 34 4
,{0x27, 0x45, 0x45, 0x45, 0x39} // 35 5
,{0x3c, 0x4a, 0x49, 0x49, 0x30} // 36 6
,{0x01, 0x71, 0x09, 0x05, 0x03} // 37 7
,{0x36, 0x49, 0x49, 0x49, 0x36} // 38 8
,{0x06, 0x49, 0x49, 0x29, 0x1e} // 39 9
,{0x00, 0x36, 0x36, 0x00, 0x00} // 3a :
,{0x00, 0x56, 0x36, 0x00, 0x00} // 3b ;
,{0x08, 0x14, 0x22, 0x41, 0x00} // 3c <
,{0x14, 0x14, 0x14, 0x14, 0x14} // 3d =
,{0x00, 0x41, 0x22, 0x14, 0x08} // 3e >
,{0x02, 0x01, 0x51, 0x09, 0x06} // 3f ?
,{0x32, 0x49, 0x79, 0x41, 0x3e} // 40 @
,{0x7e, 0x11, 0x11, 0x11, 0x7e} // 41 A
,{0x7f, 0x49, 0x49, 0x49, 0x36} // 42 B
,{0x3e, 0x41, 0x41, 0x41, 0x22} // 43 C
,{0x7f, 0x41, 0x41, 0x22, 0x1c} // 44 D
,{0x7f, 0x49, 0x49, 0x49, 0x41} // 45 E
,{0x7f, 0x09, 0x09, 0x09, 0x01} // 46 F
,{0x3e, 0x41, 0x49, 0x49, 0x7a} // 47 G
,{0x7f, 0x08, 0x08, 0x08, 0x7f} // 48 H
,{0x00, 0x41, 0x7f, 0x41, 0x00} // 49 I
,{0x20, 0x40, 0x41, 0x3f, 0x01} // 4a J
,{0x7f, 0x08, 0x14, 0x22, 0x41} // 4b K
,{0x7f, 0x40, 0x40, 0x40, 0x40} // 4c L
,{0x7f, 0x02, 0x0c, 0x02, 0x7f} // 4d M
,{0x7f, 0x04, 0x08, 0x10, 0x7f} // 4e N
,{0x3e, 0x41, 0x41, 0x41, 0x3e} // 4f O
,{0x7f, 0x09, 0x09, 0x09, 0x06} // 50 P
,{0x3e, 0x41, 0x51, 0x21, 0x5e} // 51 Q
,{0x7f, 0x09, 0x19, 0x29, 0x46} // 52 R
,{0x46, 0x49, 0x49, 0x49, 0x31} // 53 S
,{0x01, 0x01, 0x7f, 0x01, 0x01} // 54 T
,{0x3f, 0x40, 0x40, 0x40, 0x3f} // 55 U
,{0x1f, 0x20, 0x40, 0x20, 0x1f} // 56 V
,{0x3f, 0x40, 0x38, 0x40, 0x3f} // 57 W
,{0x63, 0x14, 0x08, 0x14, 0x63} // 58 X
,{0x07, 0x08, 0x70, 0x08, 0x07} // 59 Y
,{0x61, 0x51, 0x49, 0x45, 0x43} // 5a Z
,{0x00, 0x7f, 0x41, 0x41, 0x00} // 5b [
,{0x02, 0x04, 0x08, 0x10, 0x20} // 5c ¥
,{0x00, 0x41, 0x41, 0x7f, 0x00} // 5d ]
,{0x04, 0x02, 0x01, 0x02, 0x04} // 5e ^
,{0x40, 0x40, 0x40, 0x40, 0x40} // 5f _
,{0x00, 0x01, 0x02, 0x04, 0x00} // 60 `
,{0x20, 0x54, 0x54, 0x54, 0x78} // 61 a
,{0x7f, 0x48, 0x44, 0x44, 0x38} // 62 b
,{0x38, 0x44, 0x44, 0x44, 0x20} // 63 c
,{0x38, 0x44, 0x44, 0x48, 0x7f} // 64 d
,{0x38, 0x54, 0x54, 0x54, 0x18} // 65 e
,{0x08, 0x7e, 0x09, 0x01, 0x02} // 66 f
,{0x0c, 0x52, 0x52, 0x52, 0x3e} // 67 g
,{0x7f, 0x08, 0x04, 0x04, 0x78} // 68 h
,{0x00, 0x44, 0x7d, 0x40, 0x00} // 69 i
,{0x20, 0x40, 0x44, 0x3d, 0x00} // 6a j
,{0x7f, 0x10, 0x28, 0x44, 0x00} // 6b k
,{0x00, 0x41, 0x7f, 0x40, 0x00} // 6c l
,{0x7c, 0x04, 0x18, 0x04, 0x78} // 6d m
,{0x7c, 0x08, 0x04, 0x04, 0x78} // 6e n
,{0x38, 0x44, 0x44, 0x44, 0x38} // 6f o
,{0x7c, 0x14, 0x14, 0x14, 0x08} // 70 p
,{0x08, 0x14, 0x14, 0x18, 0x7c} // 71 q
,{0x7c, 0x08, 0x04, 0x04, 0x08} // 72 r
,{0x48, 0x54, 0x54, 0x54, 0x20} // 73 s
,{0x04, 0x3f, 0x44, 0x40, 0x20} // 74 t
,{0x3c, 0x40, 0x40, 0x20, 0x7c} // 75 u
,{0x1c, 0x20, 0x40, 0x20, 0x1c} // 76 v
,{0x3c, 0x40, 0x30, 0x40, 0x3c} // 77 w
,{0x44, 0x28, 0x10, 0x28, 0x44} // 78 x
,{0x0c, 0x50, 0x50, 0x50, 0x3c} // 79 y
,{0x44, 0x64, 0x54, 0x4c, 0x44} // 7a z
,{0x00, 0x08, 0x36, 0x41, 0x00} // 7b {
,{0x00, 0x00, 0x7f, 0x00, 0x00} // 7c |
,{0x00, 0x41, 0x36, 0x08, 0x00} // 7d }
,{0x10, 0x08, 0x08, 0x10, 0x08} // 7e ?
,{0x00, 0x06, 0x09, 0x09, 0x06} // 7f ?
}; // end char ASCII[96][5]

//	Function Prototypes
int main(void);
int readADC(void);
void display_write(int, int, char *);
// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

int main ( void )
{
    /* Initialize all MPLAB Harmony modules, including application(s). */
    SYS_Initialize ( NULL );

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
    // RPB15Rbits.RPB15R = 0b0101; // set B15 to U1TX (Output pin for OC1)

// set up A0 as AN0
    ANSELAbits.ANSA0 = 1;
    AD1CON3bits.ADCS = 3;
    AD1CHSbits.CH0SA = 0;
    AD1CON1bits.ADON = 1;
    
// Accelerometer
    acc_setup();
    volatile short accels[3]; // accelerations for the 3 axes
    volatile short mags[3]; // magnetometer readings for the 3 axes
    volatile short temp;

// Display
    display_init();
    
    while (1){
     // invert pin every 0.5s, set PWM duty cycle % to the pot voltage output %
        SYS_Tasks ( );
        /*
        if (accels[0]>0 && accels[1]>0){
            x_line_point = (float)((float)accels[0]/16000*64) + 64;
            y_line_point = (float)((float)accels[1]/16000*32) + 32;
  
                for (i=64;i<x_line_point;i++){
                    display_pixel_set(31, i, 1);
                    display_pixel_set(32, i, 1);
                    display_pixel_set(33, i, 1);
                    }
                for (j=32;j<y_line_point;j++){
                    display_pixel_set(j, 63, 1);
                    display_pixel_set(j, 64, 1);
                    display_pixel_set(j, 65, 1);
                }
        }
        else if (accels[0]>0 && accels[1]<0){
            x_line_point = (float)((float)accels[0]/16000*64) + 64;
            y_line_point = (float)((float)accels[1]/16000*32) + 32;
                for (i=64;i<x_line_point;i++){
                    display_pixel_set(31, i, 1);
                    display_pixel_set(32, i, 1);
                    display_pixel_set(33, i, 1);
                    }

                for (j=32;j>y_line_point;j--){
                    display_pixel_set(j, 63, 1);
                    display_pixel_set(j, 64, 1);
                    display_pixel_set(j, 65, 1);
                }
        }
        else if (accels[0]<0 && accels[1]>0){
            x_line_point = (float)((float)accels[0]/16000*64) + 64;
            y_line_point = (float)((float)accels[1]/16000*32) + 32;
                for (i=64;i>x_line_point;i--){
                    display_pixel_set(31, i, 1);
                    display_pixel_set(32, i, 1);
                    display_pixel_set(33, i, 1);
                    }

                for (j=32;j<y_line_point;j++){
                    display_pixel_set(j, 63, 1);
                    display_pixel_set(j, 64, 1);
                    display_pixel_set(j, 65, 1);
                }
        }
        else if (accels[0]<0 && accels[1]<0){
            x_line_point = (float)((float)accels[0]/16000*64) + 64;
            y_line_point = (float)((float)accels[1]/16000*32) + 32;
                for (i=64;i>x_line_point;i--){
                    display_pixel_set(31, i, 1);
                    display_pixel_set(32, i, 1);
                    display_pixel_set(33, i, 1);
                    }

                for (j=32;j>y_line_point;j--){
                    display_pixel_set(j, 63, 1);
                    display_pixel_set(j, 64, 1);
                    display_pixel_set(j, 65, 1);
                }
        }
        
        display_pixel_set(31,63,1);
        display_pixel_set(31,64,1);
        display_pixel_set(31,65,1);
        display_pixel_set(32,63,1);
        display_pixel_set(32,64,1);
        display_pixel_set(32,65,1);
        display_pixel_set(33,63,1);
        display_pixel_set(33,64,1);
        display_pixel_set(33,65,1);
        display_draw();
         * */
        
        }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE );
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

void display_write(int x_start, int y_start, char buffer1[]){
    int ASCII_string_row_index = 0;
    int ascii_byte;
    int rowindex;
    int columnindex;
    int x = x_start;
    int y = y_start;
    int number_of_rows = 1;
    
    while(buffer1[ASCII_string_row_index]){
        for(columnindex = 0; columnindex < 5; columnindex++){
            ascii_byte = ASCII[buffer1[ASCII_string_row_index]-0x20][columnindex]; // Extract desired byte from ASCII
            y = y_start;
            for(rowindex = 0; rowindex < 8; rowindex++){
                if(x < 128 && y < 64){
                    display_pixel_set(y,x,ascii_byte&1);      // Isolate Least Significant Bit
                    ascii_byte = ascii_byte>>1;               // Shift bits to the right by 1
                }
                y++;
                if(rowindex == 7){
                    y = y_start;                              // When y = 8, set y = y_start
                }

            }
            x++;
                if(x>127){                                    // Start a new row when x limit is reached
                    y_start = y_start+8;
                    x = x_start;
                    number_of_rows++;
                }
        }
        ASCII_string_row_index++;
    }
    display_draw();
}

/*******************************************************************************
 End of File
*/

