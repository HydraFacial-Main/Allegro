/*****************************************************************************
 *
 * Basic access to parallel FLASH SST39LF400.
 *
 *****************************************************************************
 * FileName:        SST39LF400.c
 * Dependencies:    SST39LF400.h
 * Processor:       PIC24F
 * Compiler:       	MPLAB C30 V3.00
 * Linker:          MPLAB LINK30
 * Company:         Microchip Technology Incorporated
 *
 * Software License Agreement
 *
 * Copyright © 2009 Microchip Technology Inc.  All rights reserved.
 * Microchip licenses to you the right to use, modify, copy and distribute
 * Software only when embedded on a Microchip microcontroller or digital
 * signal controller, which is integrated into your product or third party
 * product (pursuant to the sublicense terms in the accompanying license
 * agreement).  
 *
 * You should refer to the license agreement accompanying this Software
 * for additional information regarding your rights and obligations.
 *
 * SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS” WITHOUT WARRANTY OF ANY
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
 * Date         Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * 4/12/2010	Original, ported from SST39LF400A
 * 8/11/2010	Removed dependency on Graphics file. 
 ********************************************************************/
#include "SST39LF400.h"

#if defined (USE_SST39LF400)

#define  	CS2_BASE_ADDRESS   		GFX_EPMP_CS2_BASE_ADDRESS  
#define 	SST39LF400_FLASH_SIZE	0x00400000ul


/************************************************************************
*   LOCAL PROTOTYPES
************************************************************************/
void lWrite16(DWORD address, WORD data);
WORD lRead16(DWORD address);


/************************************************************************
* Function:SST39LF400Init(WORD *pBuffer)                                                 
*                                                                       
* Overview: This function initializes IOs and EPMP. Before it
*           programs the EPMP port, it stores the current
*			EPMP SFR registers and saves it to the supplied 
*           array space.
*           
*                                                                       
* Input: Address of the array for the current EPMP SFR register
*		 contents. 
*                                                                       
* Output: none
*                                                                       
************************************************************************/
void SST39LF400Init(WORD *pBuffer)
{
	// if using PIC24FJ256DA210 Development Board, save the EPMP registers to the  
	// buffer so we can revert back when access to the parallel flash is done
	if (pBuffer != NULL)
	{
		// before modifying the registers, save the contents of the SFR first
		*pBuffer++ = PMCON2;
		*pBuffer++ = PMCON3;
		*pBuffer++ = PMCON4;
		*pBuffer++ = PMCS1CF;
		*pBuffer++ = PMCS1BS;
		*pBuffer++ = PMCS1MD;
		*pBuffer++ = PMCS2CF;
		*pBuffer++ = PMCS2BS;
		*pBuffer++ = PMCS2MD;
		*pBuffer++ = IFS2;
		*pBuffer++ = IEC2;
		*pBuffer = PMCON1;
	}
	
	// now that the SFR contents are saved, program the SFRs for the new values.
	PMCON1bits.PMPEN = 0;

	PMCS1CFbits.CSDIS = 1;      // disable CS1 functionality  
	PMCS2CFbits.CSDIS = 1;      // disable CS2 functionality  

	ANSDbits.ANSD7 = 0;   		// PMD15
	ANSDbits.ANSD6 = 0;   		// PMD14
	ANSFbits.ANSF0 = 0;   		// PMD11

	ANSBbits.ANSB15 = 0;  		// PMA0
	ANSBbits.ANSB14 = 0;  		// PMA1
	ANSGbits.ANSG9  = 0;  		// PMA2
	ANSBbits.ANSB13 = 0;  		// PMA10
	ANSBbits.ANSB12 = 0;  		// PMA11
	ANSBbits.ANSB11 = 0;  		// PMA12
	ANSBbits.ANSB10 = 0;  		// PMA13
	ANSAbits.ANSA7 = 0;   		// PMA17
	ANSGbits.ANSG6 = 0;   		// PMA18	

	PMCON1bits.ADRMUX = 0;	    // address is not multiplexed
	PMCON1bits.MODE = 3;        // master mode
	PMCON1bits.CSF = 0;         // PMCS1 pin used for chip select 1, PMCS2 pin used for chip select 2
	PMCON1bits.ALP = 1;         // set address latch strobes to high active level (for sn74lvc16373)
	PMCON1bits.ALMODE = 0;      // "smart" address strobes are not used
	PMCON1bits.BUSKEEP = 0;     // bus keeper is not used
	
	// CS1 is not used 
	// set CS2 start Address (this is where the parallel flash is connected on 
	// the PIC24FJ256DA210 Board
	PMCS2BS = (CS2_BASE_ADDRESS >> 8);				

	PMCON2bits.RADDR = 0xff;    // set CS2 end address

	PMCON2bits.MSTSEL = 0;		// set CPU as Master

	PMCON3bits.PTWREN = 1;      // enable write strobe port
	PMCON3bits.PTRDEN = 1;      // enable read strobe port
	PMCON3bits.PTBE0EN = 1;     // enable byte enable port
	PMCON3bits.PTBE1EN = 1;     // enable byte enable port
	PMCON3bits.AWAITM = 0;      // set address latch pulses width to 1/2 Tcy
	PMCON3bits.AWAITE = 0;      // set address hold time to 1/4 Tcy

	PMCON4 = 0xFFFF;            // PMA0 - PMA15 address lines are enabled
	PMCON3bits.PTEN16 = 1;		// enable PMA16
	PMCON3bits.PTEN17 = 1;		// enable PMA17

	PMCS2CFbits.CSDIS = 0;      // enable CS
	PMCS2CFbits.CSP = 0;        // CS active low (for SST39VF400A)
	PMCS2CFbits.CSPTEN = 1;     // enable CS port
	PMCS2CFbits.BEP = 0;        // byte enable active low (for SST39VF400A)
	PMCS2CFbits.WRSP = 0;       // write strobe active low (for SST39VF400A)
	PMCS2CFbits.RDSP = 0;       // read strobe active low (for SST39VF400A)
	PMCS2CFbits.SM = 0;         // read and write strobes on separate lines 
	PMCS2CFbits.PTSZ = 2;       // data bus width is 16-bit 

	PMCS2MDbits.ACKM = 0;        // PMACK is not used
	
	PMCS2MDbits.DWAITB = EPMPCS2_DWAITB;      				
	PMCS2MDbits.DWAITM = EPMPCS2_DWAITM;
	PMCS2MDbits.DWAITE = EPMPCS2_DWAITE;
	
	// Note: adjust this delay for slower devices 
	PMCS2MDbits.AMWAIT = EPMPCS2_AMWAIT;					

	PMCON1bits.PMPEN = 1;
}

/************************************************************************
* Function:SST39LF400DeInit(WORD *pBuffer)                                                 
*                                                                       
* Overview: This function de-initializes IOs and EPMP to recover from 
*           from the previous state. The SFR programmed values are 
*           supplied by the initialized array.
*           
*                                                                       
* Input: Address of the array for the current EPMP SFR register
*		 contents. 
*                                                                       
* Output: none
*                                                                       
************************************************************************/
void SST39LF400DeInit(WORD *pBuffer)
{
	// before modifying the registers, save the contents of the SFR first
	PMCON2 = *pBuffer++;
	PMCON3 = *pBuffer++;
	PMCON4 = *pBuffer++; 
	PMCS1CF = *pBuffer++;
	PMCS1BS = *pBuffer++;
	PMCS1MD = *pBuffer++;
	PMCS2CF = *pBuffer++;
	PMCS2BS = *pBuffer++;
	PMCS2MD = *pBuffer++;
	IFS2    = *pBuffer++;
	IEC2    = *pBuffer++;
	PMCON1  = *pBuffer;
	
}


/************************************************************************
* Function: BYTE SST39LV400WriteWord(DWORD address, WORD data)
*                                                                       
* Overview: this function writes one word
*                                                                       
* Input: address and word to be written
*                                                                       
* Output: non-zero if it's successful
*
* Notes: none
*                                                                       
************************************************************************/
BYTE SST39LF400WriteWord(DWORD address, WORD data)
{
	WORD temp;
	
    lWrite16(0x000005555,0x00aa);
    lWrite16(0x000002aaa,0x0055);
    lWrite16(0x000005555,0x00a0);
    lWrite16(address,data);
    
    SST39LF400WaitProgram();

    if((temp = SST39LF400ReadWord(address)) == data)
        if((temp = SST39LF400ReadWord(address)) == data)
            return (1);

    return (0); // failed
}

/************************************************************************
* Function: WORD SST39LF400ReadWord(DWORD address)
*                                                                       
* Overview: this function reads a word from address specified         
*                                                                       
* Input: address                                                     
*                                                                       
* Output: data read
*                                                                       
************************************************************************/
WORD SST39LF400ReadWord(DWORD address)
{
WORD temp;
	
	temp = lRead16(address);

    return (temp);
}

/************************************************************************
* Function: BYTE SST39LF400WriteArray(DWORD address, WORD* pData, nCount)
*                                                                       
* Overview: this function writes data array at the address specified
*                                                                       
* Input: flash memory address, pointer to the data array, data number
*                                                                       
* Output: return 1 if the operation was successfull
*
* Notes: none
*                                                                       
************************************************************************/
BYTE SST39LF400WriteArray(DWORD address, WORD *pData, WORD nCount)
{
    WORD    counter;
    WORD    *pD;
    DWORD   addr;

    pD = pData;
    addr = address;

    // write
    for(counter = 0; counter < nCount; counter++)
    {
        while(0 ==SST39LF400WriteWord(addr, *pD));
        addr++;
        pD++;
    }

    pD = pData;
    addr = address;

    // verify
    for(counter = 0; counter < nCount; counter++)
    {
        if(*pD++ !=SST39LF400ReadWord(addr++))
            return (0);
    }

    return (1);
}

/************************************************************************
* Function: void SST39LF400ReadArray(DWORD address, WORD* pData, nCount)
*                                                                       
* Overview: this function reads data array from the address specified
*                                                                       
* Input: flash memory address, pointer to the buffer receiving data, data number
*                                                                       
* Output: none
*                                                                       
************************************************************************/
void SST39LF400ReadArray(DWORD address, WORD *pData, WORD nCount)
{
    WORD    counter;
    WORD    temp;

    for(counter = 0; counter < nCount; counter++)
    {
		temp = lRead16(address);
		*pData++ = temp;
    }

}

/************************************************************************
* Function: void SST39LF400WaitProgram()
*                                                                       
* Overview: this function waits for program/erase operation completion 
*			using the Toggle Bit wait option.
*                                                                       
* Input: none                                                          
*                                                                       
* Output: none
*                                                                       
************************************************************************/
void SST39LF400WaitProgram(void)
{
WORD temp;
WORD tempPrev;

	temp = lRead16(0);
	temp &= 0x0040;
	tempPrev = ~temp;
	tempPrev &= 0x0040;
    
    while(temp != tempPrev)
    {
		tempPrev = temp;
		temp = lRead16(0);
		temp &= 0x0040;
    }   // end of while

}

/************************************************************************
* Function: void SST39LF400ChipErase(void)                                 
*
* Overview: erase whole chip
*
* Input: none
*            
* Output: none
*
************************************************************************/
void SST39LF400ChipErase(void)
{
    lWrite16(0x000005555,0x00aa);
    lWrite16(0x000002aaa,0x0055);
    lWrite16(0x000005555,0x0080);
    lWrite16(0x000005555,0x00aa);
    lWrite16(0x000002aaa,0x0055);
    lWrite16(0x000005555,0x0010);

    SST39LF400WaitProgram();
}

/************************************************************************
* Function: void SST39LF400SectorErase(DWORD address)                                 
*
* Overview: This function erases 2K Word section defined by address
*
* Input: address - The address location of the sector to be erased.
*				   The address is decided by Address[30:11] address lines.
*            
* Output: none
*
************************************************************************/
void SST39LF400SectorErase(DWORD address)
{
    lWrite16(0x000005555,0x00aa);
    lWrite16(0x000002aaa,0x0055);
    lWrite16(0x000005555,0x0080);
    lWrite16(0x000005555,0x00aa);
    lWrite16(0x000002aaa,0x0055);
    lWrite16((address>>11),0x0030);

    SST39LF400WaitProgram();
}

/************************************************************************
* Function: WORD SST39LF400CheckID()  
*
* Overview: Reads the product ID 
*
* Input: none
*            
* Output: Returns 1 if the expected product ID is read correctly
*         else it returns 0.
*
************************************************************************/
WORD SST39LF400CheckID()
{
	WORD testArray[2] = {0,0};
	
    lWrite16(0x000005555,0x00aa);
    lWrite16(0x000002aaa,0x0055);
    lWrite16(0x000005555,0x0090);
    testArray[0] = lRead16 (0x000000000);
    testArray[1] = lRead16 (0x000000001);

    // exit the query mode 
    lWrite16(0x000005555,0x00F0);
    
   	if ((testArray[0] == 0x00BF) && (testArray[1] == 0x2780))
   		return 1;
	else
		return 0;   		
}


/************************************************************************
************************************************************************/
void lWrite16(DWORD address, WORD data)
{
WORD pointer;
WORD temp;
		address <<= 1;
		address += (DWORD)CS2_BASE_ADDRESS;
		pointer = ((DWORD_VAL)address).w[0];
		pointer |= 0x8000;
		address <<= 1;
        temp = DSWPAG;
		DSWPAG = ((DWORD_VAL)address).v[2];
		*((WORD*)pointer) = data;
		while(PMCON2bits.BUSY);
        DSWPAG = temp;

}

/************************************************************************
************************************************************************/
WORD lRead16(DWORD address)
{
WORD pointer;
volatile WORD data;
WORD temp;
		address <<= 1;
		address += CS2_BASE_ADDRESS;
		pointer = ((DWORD_VAL)address).w[0];
		pointer |= 0x8000;
		address <<= 1;
        temp = DSRPAG;
		DSRPAG = ((DWORD_VAL)address).v[2];
		data = *((WORD*)pointer);
		while(PMCON2bits.BUSY);
		data = PMDIN1;
		DSRPAG = temp;
		return data;

}

#endif //USE_SST39LF400
