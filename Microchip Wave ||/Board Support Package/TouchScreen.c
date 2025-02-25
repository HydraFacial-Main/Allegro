/*****************************************************************************
 *
 * Simple 4 wire touch screen driver
 *
 *****************************************************************************
 * FileName:        TouchScreen.c
 * Dependencies:    TouchScreen.h
 * Processor:       PIC24, PIC32, dsPIC, PIC24H
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
 * Date        	Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * 01/08/07		...
 * 06/06/07		Basic calibration and GOL messaging are added
 * 02/05/08    	new PICtail support, portrait orientation is added
 * 02/07/08    	PIC32 support
 * 05/27/08    	More robust algorithm
 * 01/07/09    	Graphics PICtail Version 3 support is added
 * 06/25/09    	dsPIC & PIC24H support 
 * 06/29/09		Added event EVENT_STILLPRESS to support continuous press
 * 04/15/10		Qualified EVENT_STILLPRESS to be set only when
 *              an actual touch is occuring.
 *              Removed timer and timer ISR, made the touch detection
 *              a function.
 * 07/27/10    	Added support for parallel flash 
 *				(only on PIC24FJ256DA120 Development Board)
 *****************************************************************************/
#include "TouchScreen.h"
#include "C:/Microchip Wave II Images Touchscreen Codes/Microchip/Include/Graphics/gol.h"

#if defined (USE_RESISTIVE_TOUCH)

#if defined (USE_25LC256)
	#include "EEPROM.h"
#elif defined (USE_SST25VF016)	
	#include "SST25VF016.h"
#elif defined (USE_SST39LF400)	
	#include "SST39LF400.h"
#else
	#warning "Touchscreen is being used but calibration data will not be saved or retrieved. Enable the memory that will hold the calibration data in the Hardware Peofile."	
#endif	

	extern const FONT_FLASH FONTDEFAULT;

//////////////////////// LOCAL PROTOTYPES ////////////////////////////
void    TouchGetCalPoints(void);

#define WAIT_UNTIL_FINISH(x)    while(!x)

//////////////////////// GLOBAL VARIABLES ////////////////////////////
#define PRESS_THRESHOULD    100 // between 0-0x03ff the lesser this value the lighter the screen must be pressed
#define CALIBRATION_DELAY   300 // delay between calibration touch points

    // Max/Min ADC values for each derection
volatile WORD   	_calXMin = XMINCAL;
volatile WORD       _calXMax = XMAXCAL;
volatile WORD       _calYMin = YMINCAL;
volatile WORD       _calYMax = YMAXCAL;

// Current ADC values for X and Y channels
volatile SHORT      adcX = -1;
volatile SHORT      adcY = -1;
volatile SHORT      adcPot = 0;

BYTE InternationalMode = 1;
BYTE DaysLeft;
DWORD expirationTime;
//char serialNumber[12];
//char pinNumber[6];
BYTE continuousFlag = 1;
BYTE pinOKFlag = 0;
DWORD lastJulian = 0;
WORD oldMinutes = 0;
BYTE SDcard[100][14];
BYTE codeCount;

typedef enum
{
    SET_X,
    RUN_X,
    GET_X,
    RUN_CHECK_X,
    CHECK_X,
    SET_Y,
    RUN_Y,
    GET_Y,
    CHECK_Y,
    SET_VALUES,
    GET_POT,
    RUN_POT
} TOUCH_STATES;

volatile TOUCH_STATES state = SET_X;

void TouchProcessTouch(void)
{
    static SHORT    tempX, tempY;
    SHORT           temp;

    switch(state)
    {

        case SET_VALUES:
            if(!TOUCH_ADC_DONE)
                break;

            if((WORD) PRESS_THRESHOULD < (WORD) ADC1BUF0)
            {
                adcX = -1;
                adcY = -1;
            }
            else
            {
                adcX = tempX;
                adcY = tempY;
            }
        // If the hardware supports an analog pot, if not skip it
        #ifdef ADC_POT
            state = RUN_POT;

       case RUN_POT:
            #if defined(__dsPIC33F__) || defined(__PIC24H__)
            AD1CHS0 = ADC_POT;      // switch ADC channel
            #else
            AD1CHS = ADC_POT;       // switch ADC channel
            #endif
            AD1CON1bits.SAMP = 1;   // run conversion
            state = GET_POT;
            break;

        case GET_POT:
            if(!AD1CON1bits.DONE)
                break;

            adcPot = ADC1BUF0;
        #endif
            state = SET_X;

        case SET_X:
            #if defined(__dsPIC33F__) || defined(__PIC24H__)
            AD1CHS0 = ADC_XPOS;     // switch ADC channel
            #else
            AD1CHS = ADC_XPOS;      // switch ADC channel
            #endif
            TRIS_XPOS = 1;
            TRIS_YPOS = 1;
            TRIS_XNEG = 1;
            LAT_YNEG = 0;
            TRIS_YNEG = 0;

            AD1CON1bits.SAMP = 1;   // run conversion
            state = CHECK_X;
            break;

        case CHECK_X:
        case CHECK_Y:
            if(!TOUCH_ADC_DONE)
                break;

            if((WORD) PRESS_THRESHOULD > (WORD) ADC1BUF0)
            {
	            if (state == CHECK_X)
	            {
                	LAT_YPOS = 1;
                	TRIS_YPOS = 0;
                	tempX = -1;
                	state = RUN_X;
                } 
                else 
                {
	                LAT_XPOS = 1;
    	            TRIS_XPOS = 0;
        	        tempY = -1;
            	    state = RUN_Y;    
	            }   	
            }
            else
            {
                adcX = -1;
                adcY = -1;
		        #ifdef ADC_POT
            	    state = RUN_POT;
            	#else
            		state = SET_X;	
            	#endif	    
                break;
            }

        case RUN_X:
        case RUN_Y:
            AD1CON1bits.SAMP = 1;
            state = (state == RUN_X) ? GET_X : GET_Y;
            // no break needed here since the next state is either GET_X or GET_Y
            
        case GET_X:
        case GET_Y:
            if(!TOUCH_ADC_DONE)
                break;

            temp = ADC1BUF0;
            if (state == GET_X)
	        {
	            if(temp != tempX)
	            {
	                tempX = temp;
	                state = RUN_X;
	                break;
	            }
	        }
	        else
	        {
	            if(temp != tempY)
	            {
	                tempY = temp;
	                state = RUN_Y;
	                break;
	            }		        
		    }     

            if (state == GET_X) 
            	TRIS_YPOS = 1;
            else	
	            TRIS_XPOS = 1;
            AD1CON1bits.SAMP = 1;
            state = (state == GET_X) ? SET_Y : SET_VALUES;
            break;

        case SET_Y:
            if(!TOUCH_ADC_DONE)
                break;

            if((WORD) PRESS_THRESHOULD < (WORD) ADC1BUF0)
            {
                adcX = -1;
                adcY = -1;
		        #ifdef ADC_POT
            	    state = RUN_POT;
            	#else
                	state = SET_X;
                #endif	
                break;
            }

            #if defined(__dsPIC33F__) || defined(__PIC24H__)
            AD1CHS0 = ADC_YPOS;     // switch ADC channel
            #else
            AD1CHS = ADC_YPOS;      // switch ADC channel
            #endif
            TRIS_XPOS = 1;
            TRIS_YPOS = 1;
            LAT_XNEG = 0;
            TRIS_XNEG = 0;
            TRIS_YNEG = 1;

            AD1CON1bits.SAMP = 1;   // run conversion
            state = CHECK_Y;
            break;

        default:
            state = SET_X;
    }

}

/*********************************************************************
* Function: void TouchInit(void)
*
* PreCondition: none
*
* Input: none
*
* Output: none
*
* Side Effects: none
*
* Overview: sets ADC 
*
* Note: none
*
********************************************************************/
void TouchInit(void)
{
    // Initialize ADC
    AD1CON1 = 0x080E0;      // Turn on, auto-convert
    AD1CON2 = 0;            // AVdd, AVss, int every conversion, MUXA only
    AD1CON3 = 0x1FFF;       // 31 Tad auto-sample, Tad = 256*Tcy
    #if defined(__dsPIC33F__) || defined(__PIC24H__)
    AD1PCFGL = 0;           // All inputs are analog
    AD1PCFGLbits.PCFG11 = AD1PCFGLbits.PCFG12 = 1;
    #else
    #if !(defined(__PIC24FJ256DA210__) || defined(__PIC24FJ256GB210__))
    AD1PCFG = 0;            // All inputs are analog
    #endif
    #endif
    AD1CSSL = 0;            // No scanned inputs
    
}

/*********************************************************************
* Function: SHORT TouchGetX()
*
* PreCondition: none
*
* Input: none
*
* Output: x coordinate
*
* Side Effects: none
*
* Overview: returns x coordinate if touch screen is pressed
*           and -1 if not
*
* Note: none
*
********************************************************************/
SHORT TouchGetX(void)
{
    long    result;

    #ifdef SWAP_X_AND_Y
    result = ADCGetY();
    #else
    result = ADCGetX();
    #endif
    if(result >= 0)
    {
        #ifdef SWAP_X_AND_Y
        result = (GetMaxX() * (result - _calYMin)) / (_calYMax - _calYMin);
        #else
        result = (GetMaxX() * (result - _calXMin)) / (_calXMax - _calXMin);
        #endif
        #ifdef FLIP_X
        result = GetMaxX() - result;
        #endif
    }

    return (result);
}

/*********************************************************************
* Function: SHORT TouchGetY()
*
* PreCondition: none
*
* Input: none
*
* Output: y coordinate
*
* Side Effects: none
*
* Overview: returns y coordinate if touch screen is pressed
*           and -1 if not
*
* Note: none
*
********************************************************************/
SHORT TouchGetY(void)
{
    long    result;

    #ifdef SWAP_X_AND_Y
    result = ADCGetX();
    #else
    result = ADCGetY();
    #endif
    if(result >= 0)
    {
        #ifdef SWAP_X_AND_Y
        result = (GetMaxY() * (result - _calXMin)) / (_calXMax - _calXMin);
        #else
        result = (GetMaxY() * (result - _calYMin)) / (_calYMax - _calYMin);
        #endif
        #ifdef FLIP_Y
        result = GetMaxY() - result;
        #endif
    }

    return (result);
}

/*********************************************************************
* Function: void TouchGetMsg(GOL_MSG* pMsg)
*
* PreCondition: none
*
* Input: pointer to the message structure to be populated
*
* Output: none
*
* Side Effects: none
*
* Overview: populates GOL message structure
*
* Note: none
*
********************************************************************/
void TouchGetMsg(GOL_MSG *pMsg)
{
    static SHORT    prevX = -1;
    static SHORT    prevY = -1;

    SHORT           x, y;

    x = TouchGetX();
    y = TouchGetY();
    pMsg->type = TYPE_TOUCHSCREEN;
    pMsg->uiEvent = EVENT_INVALID;

    if(x == -1)
    {
        y = -1;
    }
    else
    {
        if(y == -1)
            x = -1;
    }

    if((prevX == x) && (prevY == y) && (x != -1) && (y != -1))
    {
        pMsg->uiEvent = EVENT_STILLPRESS;
        pMsg->param1 = x;
        pMsg->param2 = y;
        return;
    }

    if((prevX != -1) || (prevY != -1))
    {
        if((x != -1) && (y != -1))
        {

            // Move
            pMsg->uiEvent = EVENT_MOVE;
        }
        else
        {

            // Released
            pMsg->uiEvent = EVENT_RELEASE;
            pMsg->param1 = prevX;
            pMsg->param2 = prevY;
            prevX = x;
            prevY = y;
            return;
        }
    }
    else
    {
        if((x != -1) && (y != -1))
        {

            // Pressed
            pMsg->uiEvent = EVENT_PRESS;
        }
        else
        {

            // No message
            pMsg->uiEvent = EVENT_INVALID;
        }
    }

    pMsg->param1 = x;
    pMsg->param2 = y;
    prevX = x;
    prevY = y;
}

/*********************************************************************
* Function: void TouchStoreCalibration(void)
*
* PreCondition: EEPROMInit() must be called before
*
* Input: none
*
* Output: none
*
* Side Effects: none
*
* Overview: stores calibration parameters into EEPROM
*
* Note: none
*
********************************************************************/
void TouchStoreCalibration(void)
{
	BYTE intI;

	#if defined (USE_25LC256)  
    	EEPROMWriteWord(_calXMin, ADDRESS_XMIN);
    	EEPROMWriteWord(_calXMax, ADDRESS_XMAX);
    	EEPROMWriteWord(_calYMin, ADDRESS_YMIN);
    	EEPROMWriteWord(_calYMax, ADDRESS_YMAX);
    	EEPROMWriteWord(GRAPHICS_LIBRARY_VERSION, ADDRESS_VERSION);
    #elif defined (USE_SST39LF400)
		WORD tempArray[12];
		
		SST39LF400Init(tempArray);

	    SST39LF400SectorErase(ADDRESS_XMIN); // erase 4K sector
		SST39LF400WriteWord(_calXMin, ADDRESS_XMIN);
		SST39LF400WriteWord(_calXMax, ADDRESS_XMAX);
		SST39LF400WriteWord(_calYMin, ADDRESS_YMIN);
		SST39LF400WriteWord(_calYMax, ADDRESS_YMAX);
		SST39LF400WriteWord(GRAPHICS_LIBRARY_VERSION, ADDRESS_VERSION);
		
		SST39LF400DeInit(tempArray);
		
	#elif defined (USE_SST25VF016)
		SST25SectorErase(ADDRESS_XMIN); // erase 4K sector
		SST25WriteWord(_calXMin, ADDRESS_XMIN);
		SST25WriteWord(_calXMax, ADDRESS_XMAX);
		SST25WriteWord(_calYMin, ADDRESS_YMIN);
		SST25WriteWord(_calYMax, ADDRESS_YMAX);
		SST25WriteWord(GRAPHICS_LIBRARY_VERSION, ADDRESS_VERSION);
		SST25WriteWord(InternationalMode, ADDRESS_MODE);
		SST25WriteByte(DaysLeft, ADDRESS_DAYS_LEFT);
		SST25WriteByte(continuousFlag, ADDRESS_CONTINUOUS_FLAG);
		SST25WriteByte(pinOKFlag, ADDRESS_PIN_FLAG);
		SST25WriteWord((lastJulian & 0xFFFF), ADDRESS_LAST_JULIAN_LOW);
		SST25WriteWord((lastJulian >> 16), ADDRESS_LAST_JULIAN_HIGH);
		SST25WriteWord(oldMinutes, ADDRESS_OLD_MINUTES);
		SST25WriteByte(codeCount, ADDRESS_CODE_COUNT);

/*								 
		SST25WriteWord((expirationTime & 0xFFFF), ADDRESS_EXPIRATION_TIME_LOW);
		SST25WriteWord((expirationTime >> 16), ADDRESS_EXPIRATION_TIME_LOW);

		for(intI = 0; intI < 12; intI++)
		{
			SST25WriteByte(serialNumber[intI], ADDRESS_SERIAL_NUMBER + intI);

		}

		for(intI = 0; intI < 6; intI++)
		{
			SST25WriteByte(pinNumber[intI], ADDRESS_PIN_NUMBER + intI);

		}
*/

	#else 
		#error "Touch screen is being used but calibration data cannot be saved!"    
	#endif
}

/*********************************************************************
* Function: void StoreSD(void)
*
* PreCondition: EEPROMInit() must be called before
*
* Input: none
*
* Output: none
*
* Side Effects: none
*
* Overview: stores SD card into flash
*
* Note: none
*
********************************************************************/
void StoreSD(void)
{
	BYTE tempCode;
	volatile BYTE intI, intJ;

	SST25SectorErase(ADDRESS_SD_CARD); // erase 4K sector

	intI = SST25ReadByte(ADDRESS_SD_CARD);

	for(intJ = 0; intJ < codeCount; intJ++)
	{
		for(intI = 0; intI < 14; intI++)
		{
			tempCode = SDcard[intJ][intI];
			SST25WriteByte(tempCode, ADDRESS_SD_CARD + (DWORD)((intJ * 14) + intI));
			
		}
			
	}
}

/*********************************************************************
* Function: void ReadSD(void)
*
* PreCondition: EEPROMInit() must be called before
*
* Input: none
*
* Output: none
*
* Side Effects: none
*
* Overview: reads SD card from flash
*
* Note: none
*
********************************************************************/
void ReadSD(void)
{
	volatile BYTE intI, intJ;


	for(intJ = 0; intJ < codeCount; intJ++)
	{
		for(intI = 0; intI < 14; intI++)
		{
			SDcard[intJ][intI] = SST25ReadByte(ADDRESS_SD_CARD + (DWORD)((intJ * 14) + intI));
			
		}
	
	}

}
/*********************************************************************
* Function: void TouchLoadCalibration(void)
*
* PreCondition: EEPROMInit() must be called before
*
* Input: none
*
* Output: none
*
* Side Effects: none
*
* Overview: loads calibration parameters from EEPROM
*
* Note: none
*
********************************************************************/
void TouchLoadCalibration(void)
{
	BYTE intI;

	#if defined (USE_25LC256)  
    	_calXMin = EEPROMReadWord(ADDRESS_XMIN);
    	_calXMax = EEPROMReadWord(ADDRESS_XMAX);
    	_calYMin = EEPROMReadWord(ADDRESS_YMIN);
    	_calYMax = EEPROMReadWord(ADDRESS_YMAX);
    #elif defined (USE_SST39LF400)
		WORD tempArray[12];
		
		SST39LF400Init(tempArray);
    
		_calXMin = SST39LF400ReadWord(ADDRESS_XMIN);
		_calXMax = SST39LF400ReadWord(ADDRESS_XMAX);
		_calYMin = SST39LF400ReadWord(ADDRESS_YMIN);
		_calYMax = SST39LF400ReadWord(ADDRESS_YMAX);

		SST39LF400DeInit(tempArray);
		
	#elif defined (USE_SST25VF016)
	    _calXMin = SST25ReadWord(ADDRESS_XMIN);
	    _calXMax = SST25ReadWord(ADDRESS_XMAX);
	    _calYMin = SST25ReadWord(ADDRESS_YMIN);
	    _calYMax = SST25ReadWord(ADDRESS_YMAX);
		InternationalMode = SST25ReadByte(ADDRESS_MODE);
		DaysLeft = SST25ReadByte(ADDRESS_DAYS_LEFT);

/*
		expirationTime = SST25ReadWord(ADDRESS_EXPIRATION_TIME_LOW) & 0xFFFF;
		expirationTime |= (SST25ReadWord(ADDRESS_EXPIRATION_TIME_HIGH) << 16);

		for(intI = 0; intI < 12; intI++)
		{
			serialNumber[intI] = SST25ReadByte(ADDRESS_SERIAL_NUMBER + intI);

		}

		for(intI = 0; intI < 6; intI++)
		{
			pinNumber[intI] = SST25ReadByte(ADDRESS_PIN_NUMBER + intI);

		}
*/
		continuousFlag = SST25ReadByte(ADDRESS_CONTINUOUS_FLAG);
		pinOKFlag = SST25ReadByte(ADDRESS_PIN_FLAG);
		lastJulian = SST25ReadWord(ADDRESS_LAST_JULIAN_LOW) & 0xFFFF;
		lastJulian |= (SST25ReadWord(ADDRESS_LAST_JULIAN_HIGH) << 16);
		oldMinutes = SST25ReadWord(ADDRESS_OLD_MINUTES);
		codeCount = SST25ReadByte(ADDRESS_CODE_COUNT);

	#else 
		#error "Touch screen is being used but calibration data is not accessible!"    
	#endif
}

/*********************************************************************
* Function:  void TouchCalibration()
*
* PreCondition: InitGraph() must be called before
*
* Input: none
*
* Output: none
*
* Side Effects: none
*
* Overview: calibrates touch screen
*
* Note: none
*
********************************************************************/
void TouchCalibration(void)
{
    static const XCHAR  scr1StrLn1[] = {'I','M','P','O','R','T','A','N','T','.',0};
    static const XCHAR  scr1StrLn2[] = {'P','e','r','f','o','r','m','i','n','g',' ','t','o','u','c','h',0};
    static const XCHAR  scr1StrLn3[] = {'s','c','r','e','e','n',' ','c','a','l','i','b','r','a','t','i','o','n','.',0};
    static const XCHAR  scr1StrLn4[] = {'T','o','u','c','h','p','o','i','n','t','s',' ','E','X','A','C','T','L','Y',0};
    static const XCHAR  scr1StrLn5[] = {'a','t',' ','t','h','e',' ','p','o','s','i','t','i','o','n','s',' ','s','h','o','w','n',0};
    static const XCHAR  scr1StrLn6[] = {'b','y',' ','a','r','r','o','w','s','.',0};
    static const XCHAR  scr1StrLn7[] = {'T','o','u','c','h',' ','s','c','r','e','e','n',' ','t','o',0};
    static const XCHAR  scr1StrLn8[] = {'c','o','n','t','i','n','u','e','.',0};

#if defined (PIC24FJ256DA210_DEV_BOARD)
    static const XCHAR  scr2StrLn1[] = {'H','o','l','d',' ','S','1',' ','b','u','t','t','o','n',' ','a','n','d',0};
    static const XCHAR  scr2StrLn2[] = {'p','r','e','s','s',' ','M','C','L','R',' ','r','e','s','e','t',' ','t','o',0};
    static const XCHAR  scr2StrLn3[] = {'R','E','P','E','A','T',' ','t','h','e',' ','c','a','l','i','b','r','a','t','i','o','n',0};
    static const XCHAR  scr2StrLn4[] = {'p','r','o','c','e','d','u','r','e','.',0};
#else
    static const XCHAR  scr2StrLn1[] = {'H','o','l','d',' ','S','3',' ','b','u','t','t','o','n',' ','a','n','d',0};
    static const XCHAR  scr2StrLn2[] = {'p','r','e','s','s',' ','M','C','L','R',' ','r','e','s','e','t','(','S','1',')',' ','t','o',0};
    static const XCHAR  scr2StrLn3[] = {'R','E','P','E','A','T',' ','t','h','e',' ','c','a','l','i','b','r','a','t','i','o','n',0};
    static const XCHAR  scr2StrLn4[] = {'p','r','o','c','e','d','u','r','e','.',0};
#endif	
    SHORT               x, y;

    SHORT               textHeight, textStart;

    SetFont((void *) &FONTDEFAULT);
    
    textHeight = GetTextHeight((void *) &FONTDEFAULT);
    textStart =  (GetMaxY() - (textHeight*8)) >> 1;

    SetColor(WHITE);
    ClearDevice();

    SetColor(BRIGHTRED);
    WAIT_UNTIL_FINISH(OutTextXY((GetMaxX()-GetTextWidth((XCHAR *)scr1StrLn1, (void *) &FONTDEFAULT))>>1,  \
    							 textStart, (XCHAR *)scr1StrLn1));
    SetColor(BLACK);
    WAIT_UNTIL_FINISH(OutTextXY((GetMaxX()-GetTextWidth((XCHAR *)scr1StrLn2, (void *) &FONTDEFAULT))>>1,  \
    							 textStart + (1*textHeight), (XCHAR *)scr1StrLn2));
    WAIT_UNTIL_FINISH(OutTextXY((GetMaxX()-GetTextWidth((XCHAR *)scr1StrLn3, (void *) &FONTDEFAULT))>>1,  \
    							 textStart + (2*textHeight), (XCHAR *)scr1StrLn3));
    WAIT_UNTIL_FINISH(OutTextXY((GetMaxX()-GetTextWidth((XCHAR *)scr1StrLn4, (void *) &FONTDEFAULT))>>1,  \
    							 textStart + (3*textHeight), (XCHAR *)scr1StrLn4));
    WAIT_UNTIL_FINISH(OutTextXY((GetMaxX()-GetTextWidth((XCHAR *)scr1StrLn5, (void *) &FONTDEFAULT))>>1,  \
    							 textStart + (4*textHeight), (XCHAR *)scr1StrLn5));
    WAIT_UNTIL_FINISH(OutTextXY((GetMaxX()-GetTextWidth((XCHAR *)scr1StrLn6, (void *) &FONTDEFAULT))>>1,  \
    							textStart + (5*textHeight), (XCHAR *)scr1StrLn6));
    SetColor(BRIGHTRED);
    WAIT_UNTIL_FINISH(OutTextXY((GetMaxX()-GetTextWidth((XCHAR *)scr1StrLn7, (void *) &FONTDEFAULT))>>1,  \
    							textStart + (6*textHeight), (XCHAR *)scr1StrLn7));
    WAIT_UNTIL_FINISH(OutTextXY((GetMaxX()-GetTextWidth((XCHAR *)scr1StrLn8, (void *) &FONTDEFAULT))>>1,  \
    							textStart + (7*textHeight), (XCHAR *)scr1StrLn8));

    // Wait for touch
    do
    {
        x = ADCGetX();
        y = ADCGetY();
    } while((y == -1) || (x == -1));

    DelayMs(CALIBRATION_DELAY);

    SetColor(WHITE);
    ClearDevice();

    SetColor(BRIGHTRED);

    #ifdef SWAP_X_AND_Y
    WAIT_UNTIL_FINISH(Line(GetMaxX() - 5, 5, GetMaxX() - 5, 15));
    WAIT_UNTIL_FINISH(Line(GetMaxX() - 4, 5, GetMaxX() - 4, 15));
    WAIT_UNTIL_FINISH(Line(GetMaxX() - 6, 5, GetMaxX() - 6, 15));

    WAIT_UNTIL_FINISH(Line(GetMaxX() - 5, 5, GetMaxX() - 15, 5));
    WAIT_UNTIL_FINISH(Line(GetMaxX() - 5, 4, GetMaxX() - 15, 4));
    WAIT_UNTIL_FINISH(Line(GetMaxX() - 5, 6, GetMaxX() - 15, 6));

    WAIT_UNTIL_FINISH(Line(GetMaxX() - 5, 6, GetMaxX() - 15, 16));
    WAIT_UNTIL_FINISH(Line(GetMaxX() - 5, 4, GetMaxX() - 15, 14));
    WAIT_UNTIL_FINISH(Line(GetMaxX() - 5, 5, GetMaxX() - 15, 15));

    #else
    WAIT_UNTIL_FINISH(Line(5, 5, 5, 15));
    WAIT_UNTIL_FINISH(Line(4, 5, 4, 15));
    WAIT_UNTIL_FINISH(Line(6, 5, 6, 15));

    WAIT_UNTIL_FINISH(Line(5, 5, 15, 5));
    WAIT_UNTIL_FINISH(Line(5, 4, 15, 4));
    WAIT_UNTIL_FINISH(Line(5, 6, 15, 6));

    WAIT_UNTIL_FINISH(Line(5, 6, 15, 16));
    WAIT_UNTIL_FINISH(Line(5, 4, 15, 14));
    WAIT_UNTIL_FINISH(Line(5, 5, 15, 15));
    #endif
    _calXMin = 0xFFFF;
    _calXMax = 0;
    _calYMin = 0xFFFF;
    _calYMax = 0;

    TouchGetCalPoints();

    SetColor(WHITE);
    ClearDevice();

    SetColor(BRIGHTRED);

    #ifdef SWAP_X_AND_Y
    WAIT_UNTIL_FINISH(Line(5, 5, 5, 15));
    WAIT_UNTIL_FINISH(Line(4, 5, 4, 15));
    WAIT_UNTIL_FINISH(Line(6, 5, 6, 15));

    WAIT_UNTIL_FINISH(Line(5, 5, 15, 5));
    WAIT_UNTIL_FINISH(Line(5, 4, 15, 4));
    WAIT_UNTIL_FINISH(Line(5, 6, 15, 6));

    WAIT_UNTIL_FINISH(Line(5, 6, 15, 16));
    WAIT_UNTIL_FINISH(Line(5, 4, 15, 14));
    WAIT_UNTIL_FINISH(Line(5, 5, 15, 15));

    #else
    WAIT_UNTIL_FINISH(Line(5, GetMaxY() - 5, 5, GetMaxY() - 15));
    WAIT_UNTIL_FINISH(Line(4, GetMaxY() - 5, 4, GetMaxY() - 15));
    WAIT_UNTIL_FINISH(Line(6, GetMaxY() - 5, 6, GetMaxY() - 15));

    WAIT_UNTIL_FINISH(Line(5, GetMaxY() - 5, 15, GetMaxY() - 5));
    WAIT_UNTIL_FINISH(Line(5, GetMaxY() - 4, 15, GetMaxY() - 4));
    WAIT_UNTIL_FINISH(Line(5, GetMaxY() - 6, 15, GetMaxY() - 6));

    WAIT_UNTIL_FINISH(Line(5, GetMaxY() - 6, 15, GetMaxY() - 16));
    WAIT_UNTIL_FINISH(Line(5, GetMaxY() - 4, 15, GetMaxY() - 14));
    WAIT_UNTIL_FINISH(Line(5, GetMaxY() - 5, 15, GetMaxY() - 15));
    #endif
    TouchGetCalPoints();

    SetColor(WHITE);
    ClearDevice();

    SetColor(BRIGHTRED);

    #ifdef SWAP_X_AND_Y
    WAIT_UNTIL_FINISH(Line(GetMaxX() / 2 - 5, GetMaxY() - 5, GetMaxX() / 2 - 5, GetMaxY() - 15));
    WAIT_UNTIL_FINISH(Line(GetMaxX() / 2 - 4, GetMaxY() - 5, GetMaxX() / 2 - 4, GetMaxY() - 15));
    WAIT_UNTIL_FINISH(Line(GetMaxX() / 2 - 6, GetMaxY() - 5, GetMaxX() / 2 - 6, GetMaxY() - 15));

    WAIT_UNTIL_FINISH(Line(GetMaxX() / 2 - 5, GetMaxY() - 5, GetMaxX() / 2 - 15, GetMaxY() - 5));
    WAIT_UNTIL_FINISH(Line(GetMaxX() / 2 - 5, GetMaxY() - 4, GetMaxX() / 2 - 15, GetMaxY() - 4));
    WAIT_UNTIL_FINISH(Line(GetMaxX() / 2 - 5, GetMaxY() - 6, GetMaxX() / 2 - 15, GetMaxY() - 6));

    WAIT_UNTIL_FINISH(Line(GetMaxX() / 2 - 5, GetMaxY() - 6, GetMaxX() / 2 - 15, GetMaxY() - 16));
    WAIT_UNTIL_FINISH(Line(GetMaxX() / 2 - 5, GetMaxY() - 4, GetMaxX() / 2 - 15, GetMaxY() - 14));
    WAIT_UNTIL_FINISH(Line(GetMaxX() / 2 - 5, GetMaxY() - 5, GetMaxX() / 2 - 15, GetMaxY() - 15));

    #else
    WAIT_UNTIL_FINISH(Line(GetMaxX() - 5, GetMaxY() / 2 - 5, GetMaxX() - 5, GetMaxY() / 2 - 15));
    WAIT_UNTIL_FINISH(Line(GetMaxX() - 4, GetMaxY() / 2 - 5, GetMaxX() - 4, GetMaxY() / 2 - 15));
    WAIT_UNTIL_FINISH(Line(GetMaxX() - 6, GetMaxY() / 2 - 5, GetMaxX() - 6, GetMaxY() / 2 - 15));

    WAIT_UNTIL_FINISH(Line(GetMaxX() - 5, GetMaxY() / 2 - 5, GetMaxX() - 15, GetMaxY() / 2 - 5));
    WAIT_UNTIL_FINISH(Line(GetMaxX() - 5, GetMaxY() / 2 - 4, GetMaxX() - 15, GetMaxY() / 2 - 4));
    WAIT_UNTIL_FINISH(Line(GetMaxX() - 5, GetMaxY() / 2 - 6, GetMaxX() - 15, GetMaxY() / 2 - 6));

    WAIT_UNTIL_FINISH(Line(GetMaxX() - 5, GetMaxY() / 2 - 6, GetMaxX() - 15, GetMaxY() / 2 - 16));
    WAIT_UNTIL_FINISH(Line(GetMaxX() - 5, GetMaxY() / 2 - 4, GetMaxX() - 15, GetMaxY() / 2 - 14));
    WAIT_UNTIL_FINISH(Line(GetMaxX() - 5, GetMaxY() / 2 - 5, GetMaxX() - 15, GetMaxY() / 2 - 15));
    #endif
    TouchGetCalPoints();

    SetColor(WHITE);
    ClearDevice();

    SetColor(BLACK);
    WAIT_UNTIL_FINISH(OutTextXY((GetMaxX()-GetTextWidth((XCHAR *)scr2StrLn1, (void *) &FONTDEFAULT))>>1,  \
    							 textStart + (1*textHeight), (XCHAR *)scr2StrLn1));
    WAIT_UNTIL_FINISH(OutTextXY((GetMaxX()-GetTextWidth((XCHAR *)scr2StrLn2, (void *) &FONTDEFAULT))>>1,  \
    							 textStart + (2*textHeight), (XCHAR *)scr2StrLn2));
    WAIT_UNTIL_FINISH(OutTextXY((GetMaxX()-GetTextWidth((XCHAR *)scr2StrLn3, (void *) &FONTDEFAULT))>>1,  \
    							 textStart + (3*textHeight), (XCHAR *)scr2StrLn3));
    WAIT_UNTIL_FINISH(OutTextXY((GetMaxX()-GetTextWidth((XCHAR *)scr2StrLn4, (void *) &FONTDEFAULT))>>1,  \
    							 textStart + (4*textHeight), (XCHAR *)scr2StrLn4));
    SetColor(BRIGHTRED);
    WAIT_UNTIL_FINISH(OutTextXY((GetMaxX()-GetTextWidth((XCHAR *)scr1StrLn7, (void *) &FONTDEFAULT))>>1,  \
    							 textStart + (6*textHeight), (XCHAR *)scr1StrLn7));
    WAIT_UNTIL_FINISH(OutTextXY((GetMaxX()-GetTextWidth((XCHAR *)scr1StrLn8, (void *) &FONTDEFAULT))>>1,  \
    							 textStart + (7*textHeight), (XCHAR *)scr1StrLn8));

    // Wait for touch
    do
    {
        x = ADCGetX();
        y = ADCGetY();
    } while((y == -1) || (x == -1));

    DelayMs(CALIBRATION_DELAY);

    SetColor(BLACK);
    ClearDevice();
}

/*********************************************************************
* Function: void TouchGetCalPoints(void)
*
* PreCondition: InitGraph() must be called before
*
* Input: none
*
* Output: none
*
* Side Effects: none
*
* Overview: gets values for 3 touches
*
* Note: none
*
********************************************************************/
void TouchGetCalPoints(void)
{
    static const XCHAR  calStr[] = {'C','A','L','I','B','R','A','T','I','O','N',0};
    XCHAR               calTouchLeft[] = {'3',' ','t','o','u','c','h','e','s',' ','l','e','f','t',0};
    SHORT               counter;
    SHORT               x, y;
    WORD                ax[3], ay[3];

    SetFont((void *) &FONTDEFAULT);

    SetColor(BRIGHTRED);

    WAIT_UNTIL_FINISH
    (
        OutTextXY
            (
                (GetMaxX() - GetTextWidth((XCHAR *)calStr, (void *) &FONTDEFAULT)) >> 1,
                (GetMaxY() - GetTextHeight((void *) &FONTDEFAULT)) >> 1,
                (XCHAR *)calStr
            )
    );

    for(counter = 0; counter < 3; counter++)
    {

        SetColor(BRIGHTRED);

        calTouchLeft[0] = '3' - counter;

        WAIT_UNTIL_FINISH
        (
            OutTextXY
                (
                    (GetMaxX() - GetTextWidth(calTouchLeft, (void *) &FONTDEFAULT)) >> 1,
                    (GetMaxY() + GetTextHeight((void *) &FONTDEFAULT)) >> 1,
                    calTouchLeft
                )
        );

        // Wait for press
        do
        {
            x = ADCGetX();
            y = ADCGetY();
        } while((y == -1) || (x == -1));

        ax[counter] = x;
        ay[counter] = y;

        // Wait for release
        do
        {
            x = ADCGetX();
            y = ADCGetY();
        } while((y != -1) && (x != -1));

        SetColor(WHITE);

        WAIT_UNTIL_FINISH
        (
            OutTextXY
                (
                    (GetMaxX() - GetTextWidth(calTouchLeft, (void *) &FONTDEFAULT)) >> 1,
                    (GetMaxY() + GetTextHeight((void *) &FONTDEFAULT)) >> 1,
                    calTouchLeft
                )
        );

        DelayMs(CALIBRATION_DELAY);
    }

    for(counter = 0; counter < 3; counter++)
    {
        if(_calXMax < ax[counter])
            _calXMax = ax[counter];

        if(_calYMin > ay[counter])
            _calYMin = ay[counter];

        if(_calYMax < ay[counter])
            _calYMax = ay[counter];

        if(_calXMin > ax[counter])
            _calXMin = ax[counter];
    }
}

#endif // #if defined (USE_RESISTIVE_TOUCH)

