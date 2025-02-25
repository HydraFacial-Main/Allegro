/*****************************************************************************
 * Simple UART driver
 *****************************************************************************
 * FileName:        UART.c
 * Dependencies:    MainDemo.h
 * Processor:       PIC24F, PIC24H, dsPIC, PIC32
 * Compiler:       	MPLAB C30, MPLAB C32
 * Linker:          MPLAB LINK30, MPLAB LINK32
 * Company:         Microchip Technology Incorporated
 *
 * Software License Agreement
 *
 * Copyright � 2008 Microchip Technology Inc.  All rights reserved.
 * Microchip licenses to you the right to use, modify, copy and distribute
 * Software only when embedded on a Microchip microcontroller or digital
 * signal controller, which is integrated into your product or third party
 * product (pursuant to the sublicense terms in the accompanying license
 * agreement).  
 *
 * You should refer to the license agreement accompanying this Software
 * for additional information regarding your rights and obligations.
 *
 * SOFTWARE AND DOCUMENTATION ARE PROVIDED �AS IS� WITHOUT WARRANTY OF ANY
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
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Anton Alkhimenok		01/08/07	...
 * Anton Alkhimenok		02/05/08	PIC32 support is added
 * Jayanth Murthy       06/25/09    dsPIC & PIC24H support 
 *****************************************************************************/
    #include "GenericTypeDefs.h"
    #include "Graphics/Graphics.h"
  //  #include "FlashProgrammer.h"
//#include "MainDemo.h"
#include "UART.h"
#include "string.h"

#define RECEIVE_MSG_SIZE	10
#define CR	0x0D

char charReceiveMessage[RECEIVE_MSG_SIZE];
BYTE rcvFlag = FALSE;
BYTE ptrRcv = 0;
char stage1Pressure[15];
char stage2Pressure[15];
char stage3Pressure[15];
char stage4Pressure[15];
char stage5Pressure[15];
char stage6Pressure[15];
char stage7Pressure[15];
char activePressure[15];
char regulatorPressure[15];
char SCBAPressure[15];
char activeStage[3];

BYTE stage1Flag = FALSE;
BYTE stage2Flag = FALSE;
BYTE stage3Flag = FALSE;
BYTE stage4Flag = FALSE;
BYTE stage5Flag = FALSE;
BYTE stage6Flag = FALSE;
BYTE stage7Flag = FALSE;
BYTE activeFlag = FALSE;
BYTE regulatorFlag = FALSE;
BYTE SCBAFlag = FALSE;

/*********************************************************************
* Function: void UARTInit(void)
*
* PreCondition: none
*
* Input: none
*
* Output: none
*
* Side Effects: none
*
* Overview: initializes UART 
*
* Note: none
*
********************************************************************/
void UARTInit(void)
{
    TX_TRIS = 0;
    RX_TRIS = 1;

    #if defined(__dsPIC33FJ128GP804__) || defined(__PIC24HJ128GP504__)

    	AD1PCFGL = 0xFFFF;
    	RPINR19bits.U2RXR = 2;  	// assign RP2 to RX
    	RPOR1bits.RP3R = 5;     	// assign RP3 to TX

    #endif

    #if defined(__PIC24FJ256GB110__) || defined(__PIC24FJ256GA110__) || defined(__PIC24FJ256GB210__)
    
	    __builtin_write_OSCCONL(OSCCON & 0xbf);
	    RPINR19bits.U2RXR = 10; 	// assign RP10 to RX
    	RPOR8bits.RP17R = 5;    	// assign RP17 to TX
		__builtin_write_OSCCONL(OSCCON | 0x40); 
		
    #endif

    #if defined(__PIC24FJ256DA210__)
    
	    __builtin_write_OSCCONL(OSCCON & 0xbf);
		_U2RXR = 11;				// Bring RX2 Input to RP11/RD0			
		_RP16R = 5; 				// Bring TX2 output on RP16/RF3
		__builtin_write_OSCCONL(OSCCON | 0x40); 
		
 		OSCCON = 0x3302;    		// Enable secondary oscillator
        CLKDIV = 0x0020;    		// Set PLL prescaler (1:1), PLLEN = 1
		
	#endif    
 
    #if defined(__PIC32MX)
    	U2BRG = (GetPeripheralClock() / 4 / BAUDRATE) - 1;
    #else
    
        #if defined(__dsPIC33F__) || defined(__PIC24H__)
    		// The processor frequency is set to 40MHz
    		#define BRG_TEMP    10 * GetSystemClock() / 8 / BAUDRATE	
    	#else
        	#define BRG_TEMP    10 * GetPeripheralClock() / 4 / BAUDRATE
		#endif
		
        #if (BRG_TEMP - ((BRG_TEMP / 10) * 10)) >= 5
		    U2BRG = BRG_TEMP / 10;
        #else
    		U2BRG = BRG_TEMP / 10 - 1;
        #endif
    #endif
/*
    if(GetHWButtonProgram() == HW_BUTTON_PRESS)
    {
    	U2BRG = 34;
    }
*/
    U2MODE = 0;
    U2STA = 0;
    U2MODEbits.UARTEN = 1;
    U2MODEbits.STSEL = 0;
    U2STAbits.UTXEN = 1;

	IEC1bits.U2RXIE = 1;

    #ifdef __PIC32MX
    U2STAbits.URXEN = 1;
    #endif
    U2MODEbits.BRGH = 1;

    U2STAbits.OERR = 0;
}

/*********************************************************************
* Function: void UARTPutChar(BYTE ch)
*
* PreCondition: none
*
* Input: none
*
* Output: none
*
* Side Effects: none
*
* Overview: puts character
*
* Note: none
*
********************************************************************/
void UARTPutChar(BYTE ch)
{

    // Wait for Tx buffer is not full
    while(U2STAbits.UTXBF == 1);
    U2TXREG = ch;
}

/*********************************************************************
* Function: void UARTPutHex(BYTE hex)
*
* PreCondition: none
*
* Input: none
*
* Output: none
*
* Side Effects: none
*
* Overview: puts HEX code of byte
*
* Note: none
*
********************************************************************/
void UARTPutByte(BYTE byte)
{
    UARTPutChar(Hex2Char(byte >> 4));
    UARTPutChar(Hex2Char(byte & 0x0f));
}

/*********************************************************************
* Function: void UARTPutWord(WORD_VAL word)
*
* PreCondition: none
*
* Input: none
*
* Output: none
*
* Side Effects: none
*
* Overview: puts HEX code of word
*
* Note: none
*
********************************************************************/
void UARTPutWord(WORD word)
{
    UARTPutByte(((WORD_VAL) word).v[1]);
    UARTPutByte(((WORD_VAL) word).v[0]);
}

/*********************************************************************
* Function: void UARTPutDWord(DWORD_VAL dword)
*
* PreCondition: none
*
* Input: none
*
* Output: none
*
* Side Effects: none
*
* Overview: puts HEX code of double word
*
* Note: none
*
********************************************************************/
void UARTPutDWord(DWORD dword)
{
    UARTPutWord(((DWORD_VAL) dword).w[1]);
    UARTPutWord(((DWORD_VAL) dword).w[0]);
}

/*********************************************************************
* Function: void UARTPutString(char* str)
*
* PreCondition: none
*
* Input: none
*
* Output: none
*
* Side Effects: none
*
* Overview: puts string
*
* Note: none
*
********************************************************************/
void UARTPutString(char *str)
{
    while(*str)
        UARTPutChar(*str++);
}

/*********************************************************************
* Function: BYTE UARTGetChar()
*
* PreCondition: none
*
* Input: none
*
* Output: last character received
*
* Side Effects: none
*
* Overview: returns last character received
*
* Note: none
*
********************************************************************/
BYTE UARTGetChar(void)
{
    BYTE    temp;

    temp = U2RXREG;

    // Clear error flag
    U2STAbits.OERR = 0;

    return (temp);
}

/*********************************************************************
* Function: BYTE UARTWaitChar()
*
* PreCondition: none
*
* Input: none
*
* Output: character received
*
* Side Effects: none
*
* Overview: wait for character
*
* Note: none
*
********************************************************************/
BYTE UARTWaitChar(void)
{

    // Clear error flag
    if(U2STAbits.OERR)
        U2STAbits.OERR = 0;

    // Wait for new data
    while(U2STAbits.URXDA == 0);

    return (UARTGetChar());
}

/*********************************************************************
* Function: BYTE Char2Hex(BYTE ch)
*
* PreCondition: none
*
* Input: ASCII to be converted
*
* Output: number
*
* Side Effects: none
*
* Overview: converts ASCII coded digit into number
*
* Note: none
*
********************************************************************/
BYTE Char2Hex(BYTE ch)
{

    // Wrong char
    if(ch > 102)
        return (0);

    // From a to f
    if(ch > 96)
        return (ch - 87);

    // Wrong char
    if(ch > 70)
        return (0);

    // From A to F
    if(ch > 64)
        return (ch - 55);

    // Wrong char
    if(ch > 57)
        return (0);

    // From 0 - 9
    if(ch > 47)
        return (ch - 48);
    else
        // Wrong char
        return (0);
}

/*********************************************************************
* Function: BYTE Hex2Char(BYTE hex)
*
* PreCondition: none
*
* Input: number
*
* Output: ASCII code
*
* Side Effects: none
*
* Overview: converts low nibble into ASCII coded digit
*
* Note: none
*
********************************************************************/
BYTE Hex2Char(BYTE hex)
{
    BYTE    h;
    h = hex & 0x0f;

    // From 0xa to 0xf
    if(h > 9)
        return (h + 55);
    else
        return (h + 48);
}

/*********************************************************************
 * Function:        void _ISR _U2RXInterrupt(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Copies bytes to and from the local UART TX and 
 *					RX FIFOs
 *
 * Note:            None
 ********************************************************************/

void _ISR __attribute__((__no_auto_psv__)) _U2RXInterrupt(void)
//void _ISR _U2RXInterrupt(void)
{
	BYTE i;

	// Store a received byte, if pending, if possible
	// Get the byte
	i = U2RXREG;
	
	// Clear the interrupt flag so we don't keep entering this ISR
	IFS1bits.U2RXIF = 0;

	charReceiveMessage[ptrRcv] = i;
	
	if(charReceiveMessage[ptrRcv] == CR)
	{
	    rcvFlag = TRUE;
		charReceiveMessage[ptrRcv] = 0;	//terminate with zero
		switch(charReceiveMessage[0])
		{

			case 'A':			 //1st stage
				strcpy(stage1Pressure, &charReceiveMessage[1]);
				stage1Flag = TRUE;
				break;		
	
			case 'B':	//2nd stage
				strcpy(stage2Pressure, &charReceiveMessage[1]);
				stage2Flag = TRUE;
				break;		
	
			case 'C':		//3rd stage
				strcpy(stage3Pressure, &charReceiveMessage[1]);
				stage3Flag = TRUE;
				break;
				
			case 'D':		//4th stage
				strcpy(stage4Pressure, &charReceiveMessage[1]);
				stage4Flag = TRUE;
				break;		
			
			case 'E': 		//5th stage
				strcpy(stage5Pressure, &charReceiveMessage[1]);
				stage5Flag = TRUE;
				break;		
	
			case 'F':		//6th stage
				strcpy(stage6Pressure, &charReceiveMessage[1]);
				stage6Flag = TRUE;
				break;		

			case 'G':	//7th stage
				strcpy(stage7Pressure, &charReceiveMessage[1]);
				stage7Flag = TRUE;
				break;		
	
			case 'H':	//active stage
				activeStage[0] = charReceiveMessage[1];
				activeStage[1] = 0;
				strcpy(activePressure, &charReceiveMessage[2]);
				activeFlag = TRUE;
				break;
				
			case 'I':
				strcpy(regulatorPressure, &charReceiveMessage[1]);
				regulatorFlag = TRUE;
				break;		
			
			case 'J': 
				strcpy(SCBAPressure, &charReceiveMessage[1]);
				SCBAFlag = TRUE;
				break;		
	
			default:
				break;
		}
	    ptrRcv = 0;
   
	}
	else
	{
	    ptrRcv++;

	    if(ptrRcv >= RECEIVE_MSG_SIZE)
	    {
		    ptrRcv = 0;
	    	
	    }
	    
	}



	
}
