/*****************************************************************************
 *
 * Basic access to SPI Flash SST25VF016 located on 
 * Graphics LCD Controller PICtail Plus SSD1926 Board.
 *
 *****************************************************************************
 * FileName:        SST25VF016.c
 * Dependencies:    SST25VF016.h
 * Processor:       PIC24F, PIC24H, dsPIC, PIC32
 * Compiler:       	MPLAB C30 V3.00, MPLAB C32
 * Linker:          MPLAB LINK30, MPLAB LINK32
 * Company:         Microchip Technology Incorporated
 *
 * Software License Agreement
 *
 * Copyright © 2008 Microchip Technology Inc.  All rights reserved.
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
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Anton Alkhimenok		01/07/09	...
 *****************************************************************************/
#include "SST25VF016.h"

#if defined (USE_SST25VF016)

/************************************************************************
* Function: SST25Init                                                  
*                                                                       
* Overview: this function setup SPI and IOs connected to SST25
*                                                                       
* Input: none                                                          
*                                                                       
* Output: none
*                                                                       
************************************************************************/
void SST25Init(void)
{

		#if defined (EXPLORER_16)
/************************************************************************
* For Explorer 16 RD12 is connected to EEPROM chip select.
* To prevent a conflict between this EEPROM and SST25 flash
* the chip select of the EEPROM SPI should be pulled up.
************************************************************************/
    // Set IOs directions for EEPROM SPI
    EEPROM_CS_LAT = 1;			// set initial CS value to 1 (not asserted)
	EEPROM_CS_TRIS = 0;			// set CS pin to output
		#endif // #if defined (EXPLORER_16)
		
    // Initialize SPI
        #ifdef __PIC32MX
    SST25_SPISTAT = 0;
    SST25_SPICON = 0;
    SST25_SPIBRG = 0;
    SST25_SPICONbits.MSTEN = 1;
    SST25_SPICONbits.CKP = 0;
    SST25_SPICONbits.CKE = 1;
    SST25_SPICONbits.SMP = 1;
    SST25_SPIBRG = 1;
    SST25_SPICONbits.ON = 1;
    #else
        #if defined (GFX_PICTAIL_V3)
    SST25_SPISTAT = 0;
    SST25_SPICON1 = 0x001b;
    SST25_SPICON1bits.MSTEN = 1;
    SST25_SPICON2 = 0;
    SST25_SPICON1bits.MODE16 = 0;
    SST25_SPICON1bits.CKE = 0;
    SST25_SPICON1bits.CKP = 1;
    SST25_SPICON1bits.SMP = 1;
    SST25_SPISTATbits.SPIEN = 1;
        #elif defined (PIC24FJ256DA210_DEV_BOARD)
    SST25_SPISTAT = 0;
    #if defined (SPI_CLK_SPEED_8MHZ) 	// use 8 MHZ
		SST25_SPICON1bits.SPRE = 6; 
		SST25_SPICON1bits.PPRE = 3;
	    SST25_SPICON1bits.CKE = 1; 
    	SST25_SPICON1bits.CKP = 0; 
    	SST25_SPICON1bits.SMP = 0; 
	#else 								// use 16 MHZ 
		SST25_SPICON1bits.SPRE = 7;
		SST25_SPICON1bits.PPRE = 3;
	    SST25_SPICON1bits.CKE = 0;
	    SST25_SPICON1bits.CKP = 1;
	    SST25_SPICON1bits.SMP = 1;
	#endif		
    SST25_SPICON1bits.MSTEN = 1;
    SST25_SPICON2 = 0;
    SST25_SPICON1bits.MODE16 = 0;
    SST25_SPISTATbits.SPIEN = 1;
        #endif
	#endif
    // Set IOs directions for SST25 SPI
    #if defined (GFX_PICTAIL_V3) || defined (MULTI_MEDIA_BOARD_DM00123)
    
    SST25_CS_LAT = 1;
    SST25_CS_TRIS = 0;
    #ifndef __PIC32MX__
    SST25_SCK_TRIS = 0;
    SST25_SDO_TRIS = 0;
    SST25_SDI_TRIS = 1;
    #if defined(__PIC24FJ256GB210__)
	SST25_SDI_ANS = 0;
	#endif
    #endif
    
    #elif defined (PIC24FJ256DA210_DEV_BOARD)
    SST25_CS_LAT = 1;
    SST25_CS_TRIS = 0;
	SST25_SDI_ANS = 0;
    SST25_SDO_ANS = 0;
    SST25_SCK_TRIS = 0;
    SST25_SDO_TRIS = 0;
    SST25_SDI_TRIS = 1;
	#endif

        #if defined(__dsPIC33FJ128GP804__) || defined(__PIC24HJ128GP504__)
    AD1PCFGL = 0xFFFF;
    RPOR9bits.RP18R = 11;                   // assign RP18 for SCK2
    RPOR8bits.RP16R = 10;                   // assign RP16 for SDO2
    RPINR22bits.SDI2R = 17;                 // assign RP17 for SDI2	
        #elif defined(__PIC24FJ256GB110__) || defined(__PIC24FJ256GA110__) || defined (__PIC24FJ256GB210__)
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS
    RPOR10bits.RP21R = 11;                  // assign RP21 for SCK2
    RPOR9bits.RP19R = 10;                   // assign RP19 for SDO2
    RPINR22bits.SDI2R = 26;                 // assign RP26 for SDI2
    __builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS
        #elif defined(__PIC24FJ256DA210__)
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS
    #if defined (USE_SST25_SPI2)
	    RPOR1bits.RP2R = 11;                // assign RP2 for SCK2
	    RPOR0bits.RP1R = 10;                // assign RP1 for SDO2
	    RPINR22bits.SDI2R = 0;              // assign RP0 for SDI2
	#elif defined (USE_SST25_SPI1)
		RPOR8bits.RP17R = 8;       // assign RP16 for SCK1
		RPOR10bits.RP21R = 7;       // assign RP21 for SDO1
		RPINR20bits.SDI1R = 37;    // assign RPI37 for SDI1
	#endif
    __builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS
        #endif

    SST25ResetWriteProtection();
}

/************************************************************************
* Function SPIPut(BYTE data)                                                       
*                                                                       
* Overview:  this function sends a byte                     
*                                                                       
* Input: byte to be sent
*                                                                       
* Output: none
*                                                                       
************************************************************************/
void SPIPut(BYTE data)
{
        #ifdef __PIC32MX

    // Wait for free buffer
    while(!SST25_SPISTATbits.SPITBE);
    SST25_SPIBUF = data;

    // Wait for data byte
    while(!SST25_SPISTATbits.SPIRBF);
        #else

    // Wait for free buffer
    while(SST25_SPISTATbits.SPITBF);
    SST25_SPIBUF = data;

    // Wait for data byte
    while(!SST25_SPISTATbits.SPIRBF);
        #endif
}

/************************************************************************
* Macros SPIGet()                                                       
*                                                                       
* Overview:  this macros gets a byte from SPI                      
*                                                                       
* Input: none
*                                                                       
* Output: none
*                                                                       
************************************************************************/
#define SPIGet()     SST25_SPIBUF

/************************************************************************
* Function: void SST25WriteByte(BYTE data, DWORD address)                                           
*                                                                       
* Overview: this function writes a byte to the address specified
*                                                                       
* Input: data to be written and address
*                                                                       
* Output: none                                 
*                                                                       
************************************************************************/
void SST25WriteByte(BYTE data, DWORD address)
{
    SST25WriteEnable();
    SST25CSLow();

    SPIPut(SST25_CMD_WRITE);
    SPIGet();

    SPIPut(((DWORD_VAL) address).v[2]);
    SPIGet();

    SPIPut(((DWORD_VAL) address).v[1]);
    SPIGet();

    SPIPut(((DWORD_VAL) address).v[0]);
    SPIGet();

    SPIPut(data);
    SPIGet();

    SST25CSHigh();

    // Wait for write end
    while(SST25IsWriteBusy());
}

/************************************************************************
* Function: BYTE SST25ReadByte(DWORD address)             
*                                                                       
* Overview: this function reads a byte from the address specified         
*                                                                       
* Input: address                                                     
*                                                                       
* Output: data read
*                                                                       
************************************************************************/
BYTE SST25ReadByte(DWORD address)
{
    BYTE    temp;
    SST25CSLow();

    SPIPut(SST25_CMD_READ);
    SPIGet();

    SPIPut(((DWORD_VAL) address).v[2]);
    SPIGet();

    SPIPut(((DWORD_VAL) address).v[1]);
    SPIGet();

    SPIPut(((DWORD_VAL) address).v[0]);
    SPIGet();

    SPIPut(0);
    temp = SPIGet();

    SST25CSHigh();
    return (temp);
}

/************************************************************************
* Function: void SST25WriteWord(WODR data, DWORD address)                                           
*                                                                       
* Overview: this function writes a 16-bit word to the address specified
*                                                                       
* Input: data to be written and address
*                                                                       
* Output: none                                                         
*                                                                       
************************************************************************/
void SST25WriteWord(WORD data, DWORD address)
{
        #if defined(__dsPIC33FJ128GP804__) || defined(__PIC24HJ128GP504__)
    AD1PCFGLbits.PCFG6 = 1;
    AD1PCFGLbits.PCFG7 = 1;
    AD1PCFGLbits.PCFG8 = 1;
        #endif
    SST25WriteByte(((WORD_VAL) data).v[0], address);
    SST25WriteByte(((WORD_VAL) data).v[1], address + 1);
}

/************************************************************************
* Function: WORD SST25ReadWord(DWORD address)             
*                                                                       
* Overview: this function reads a 16-bit word from the address specified         
*                                                                       
* Input: address                                                     
*                                                                       
* Output: data read
*                                                                       
************************************************************************/
WORD SST25ReadWord(DWORD address)
{
    WORD_VAL    temp;

        #if defined(__dsPIC33FJ128GP804__) || defined(__PIC24HJ128GP504__)
    AD1PCFGLbits.PCFG6 = 1;
    AD1PCFGLbits.PCFG7 = 1;
    AD1PCFGLbits.PCFG8 = 1;
        #endif
    temp.v[0] = SST25ReadByte(address);
    temp.v[1] = SST25ReadByte(address + 1);

    return (temp.Val);
}

/************************************************************************
* Function: SST25WriteEnable()                                         
*                                                                       
* Overview: this function allows write/erase SST25. Must be called  
* before every write/erase command.                                         
*                                                                       
* Input: none                                                          
*                                                                       
* Output: none                                 
*                                                                       
************************************************************************/
void SST25WriteEnable(void)
{
    SST25CSLow();
    SPIPut(SST25_CMD_WREN);
    SPIGet();
    SST25CSHigh();
}

/************************************************************************
* Function: BYTE SST25IsWriteBusy(void)  
*                                                                       
* Overview: this function reads status register and chek BUSY bit for write operation
*                                                                       
* Input: none                                                          
*                                                                       
* Output: non zero if busy
*                                                                       
************************************************************************/
BYTE SST25IsWriteBusy(void)
{
    BYTE    temp;

    SST25CSLow();
    SPIPut(SST25_CMD_RDSR);
    SPIGet();

    SPIPut(0);
    temp = SPIGet();
    SST25CSHigh();

    return (temp & 0x01);
}

/************************************************************************
* Function: BYTE SST25WriteArray(DWORD address, BYTE* pData, nCount)
*                                                                       
* Overview: this function writes a data array at the address specified
*                                                                       
* Input: flash memory address, pointer to the data array, data number
*                                                                       
* Output: return 1 if the operation was successfull
*                                                                     
************************************************************************/
BYTE SST25WriteArray(DWORD address, BYTE *pData, WORD nCount)
{
    DWORD   addr;
    BYTE    *pD;
    WORD    counter;

    addr = address;
    pD = pData;

    // WRITE
    for(counter = 0; counter < nCount; counter++)
    {
        SST25WriteByte(*pD++, addr++);
    }

    // VERIFY
    for(counter = 0; counter < nCount; counter++)
    {
        if(*pData != SST25ReadByte(address))
            return (0);
        pData++;
        address++;
    }

    return (1);
}

/************************************************************************
* Function: void SST25ReadArray(DWORD address, BYTE* pData, nCount)
*                                                                       
* Overview: this function reads data into buffer specified
*                                                                       
* Input: flash memory address, pointer to the data buffer, data number
*                                                                       
************************************************************************/
void SST25ReadArray(DWORD address, BYTE *pData, WORD nCount)
{
    SST25CSLow();

    SPIPut(SST25_CMD_READ);
    SPIGet();

    SPIPut(((DWORD_VAL) address).v[2]);
    SPIGet();

    SPIPut(((DWORD_VAL) address).v[1]);
    SPIGet();

    SPIPut(((DWORD_VAL) address).v[0]);
    SPIGet();

    while(nCount--)
    {
        SPIPut(0);
        *pData++ = SPIGet();
    }

    SST25CSHigh();
}

/************************************************************************
* Function: void SST25ChipErase(void)
*                                                                       
* Overview: chip erase
*                                                                       
* Input: none
*                                                                       
************************************************************************/
void SST25ChipErase(void)
{
    SST25WriteEnable();

    SST25CSLow();

    SPIPut(SST25_CMD_ERASE);
    SPIGet();

    SST25CSHigh();

    // Wait for write end
    while(SST25IsWriteBusy());
}

/************************************************************************
* Function: void SST25ResetWriteProtection()
*                                                                       
* Overview: this function reset write protection bits
*                                                                       
* Input: none                                                     
*                                                                       
* Output: none
*                                                                       
************************************************************************/
void SST25ResetWriteProtection(void)
{
    SST25CSLow();

    SPIPut(SST25_CMD_EWSR);
    SPIGet();

    SST25CSHigh();

    SST25CSLow();

    SPIPut(SST25_CMD_WRSR);
    SPIGet();

    SPIPut(0);
    SPIGet();

    SST25CSHigh();
}

/************************************************************************
* Function: void SST25SectorErase(DWORD address)                                           
*                                                                       
* Overview: this function erases a 4Kb sector
*                                                                       
* Input: address within sector to be erased
*                                                                       
* Output: none                                 
*                                                                       
************************************************************************/
void SST25SectorErase(DWORD address)
{
    SST25WriteEnable();
    SST25CSLow();

    SPIPut(SST25_CMD_SER);
    SPIGet();

    SPIPut(((DWORD_VAL) address).v[2]);
    SPIGet();

    SPIPut(((DWORD_VAL) address).v[1]);
    SPIGet();

    SPIPut(((DWORD_VAL) address).v[0]);
    SPIGet();

    SST25CSHigh();

    // Wait for write end
    DelayMs(100);
    while(SST25IsWriteBusy());
}

#endif // #if defined (USE_SST25VF016)

