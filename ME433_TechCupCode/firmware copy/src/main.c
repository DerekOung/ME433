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


// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

int main ( void )
{
    /* Initialize all MPLAB Harmony modules, including application(s). */
    SYS_Initialize ( NULL );
   
    TRISBbits.TRISB7 = 0;
    TRISBbits.TRISB14 = 0;

    
    // PH Inputs: 1 = FORWARD, 0 = BACKWARD
                    LATBbits.LATB7 = 1; // LEFT
                    LATBbits.LATB14 = 1; // RIGHT
                    ANSELBbits.ANSB15 = 0; // 0 for digital, 1 for analog
                    //
                    // RIGHT: B15 PWM
                    T2CONbits.TCKPS = 0;	//Setting prescaler to 1 (0 corresponds to 1)
                    PR2 = 19999;            //Setting PR for timer 2 to 39999
                    TMR2 = 0;				//Setting Timer 2 to 0
                    OC1CONbits.OCTSEL = 0;	//Telling OC1 to use timer 2
                    OC1CONbits.OCM = 0b110;	//Telling OC1 to use PWM without the fault
                    OC1RS = 0.6 * 20000;			//Setting initial duty cycle to 20000/(39999+1)*100% = 50%
                    OC1R = 0.6 * 20000;			//Updating duty cycles to 20000/(39999+1)*100% = 50%
                    T2CONbits.ON = 1;		//turn on timer
                    OC1CONbits.ON = 1;		//turn on OC code
                    RPB15Rbits.RPB15R = 0b0101; // set B15 to U1TX (Output pin for OC1)
                    
                    //LEFT: B5 PWM
                    OC2CONbits.OCTSEL = 0;
                    OC2CONbits.OCM = 0b110;
                    OC2RS = 0.6* 20000;			//Setting initial duty cycle to 20000/(39999+1)*100% = 50%
                    OC2R = 0.6* 20000;			//Updating duty cycles to 20000/(39999+1)*100% = 50%
                    OC2CONbits.ON = 1;
                    RPB5Rbits.RPB5R = 0b0101; // set B5 to U1TX (Output pin for OC1)
                    
    while ( true )
    {
        
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks ( );
        
    }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE );
}


/*******************************************************************************
 End of File
*/

