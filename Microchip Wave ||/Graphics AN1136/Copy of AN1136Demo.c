/*****************************************************************************
 * Microchip Graphics Library Demo Application
 * This program shows how to use the Graphics Objects Layer.
 *****************************************************************************
 * FileName:        AN1136Demo.c
 * Dependencies:    AN1136Demo.h
 * Processor:       PIC24F, PIC24H, dsPIC, PIC32F
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
 * Paolo A. Tamayo		07/20/07	...
 *****************************************************************************/
#include "AN1136Demo.h"
#include "FSconfig.h"
#include "UART.h"
#include "C:/Microchip Wave II Images Touchscreen Codes/Microchip/Include/MDD File System/FSIO.h"
#include "I2C.h"

// Configuration bits
#if defined(__dsPIC33F__) || defined(__PIC24H__)
_FOSCSEL(FNOSC_PRI);
_FOSC(FCKSM_CSECMD &OSCIOFNC_OFF &POSCMD_XT);
_FWDT(FWDTEN_OFF);
#elif defined(__PIC32MX__)
    #pragma config FPLLODIV = DIV_1, FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FWDTEN = OFF, FCKSM = CSECME, FPBDIV = DIV_1
    #pragma config OSCIOFNC = ON, POSCMOD = XT, FSOSCEN = ON, FNOSC = PRIPLL
    #pragma config CP = OFF, BWP = OFF, PWP = OFF
#else
    #if defined(__PIC24FJ256GB110__)
_CONFIG1(JTAGEN_OFF & GCP_OFF & GWRP_OFF & COE_OFF & FWDTEN_OFF & ICS_PGx2)
_CONFIG2(0xF7FF & IESO_OFF & FCKSM_CSDCMD & OSCIOFNC_OFF & POSCMOD_HS & FNOSC_PRIPLL & PLLDIV_DIV2 & IOL1WAY_OFF)
    #endif
    #if defined(__PIC24FJ256GA110__)
_CONFIG1(JTAGEN_OFF & GCP_OFF & GWRP_OFF & COE_OFF & FWDTEN_OFF & ICS_PGx2)
_CONFIG2(IESO_OFF & FCKSM_CSDCMD & OSCIOFNC_OFF & POSCMOD_HS & FNOSC_PRIPLL & IOL1WAY_OFF)
    #endif
    #if defined(__PIC24FJ128GA010__)
_CONFIG2(FNOSC_PRIPLL & POSCMOD_XT) // Primary XT OSC with PLL
_CONFIG1(JTAGEN_OFF & FWDTEN_OFF)   // JTAG off, watchdog timer off
    #endif
	#if defined (__PIC24FJ256GB210__)
_CONFIG1( WDTPS_PS32768 & FWPSA_PR128 & ALTVREF_ALTVREDIS & WINDIS_OFF & FWDTEN_OFF & ICS_PGx2 & GWRP_OFF & GCP_OFF & JTAGEN_OFF) 
_CONFIG2( POSCMOD_HS & IOL1WAY_OFF & OSCIOFNC_OFF & OSCIOFNC_OFF & FCKSM_CSDCMD & FNOSC_PRIPLL & PLL96MHZ_ON & PLLDIV_DIV2 & IESO_OFF)
_CONFIG3( WPFP_WPFP255 & SOSCSEL_SOSC & WUTSEL_LEG & WPDIS_WPDIS & WPCFG_WPCFGDIS & WPEND_WPENDMEM) 
	#endif
	#if defined (__PIC24FJ256DA210__)
_CONFIG1( WDTPS_PS32768 & FWPSA_PR128 & ALTVREF_ALTVREDIS & WINDIS_OFF & FWDTEN_OFF & ICS_PGx2 & GWRP_OFF & GCP_OFF & JTAGEN_OFF) 
_CONFIG2( POSCMOD_HS & IOL1WAY_OFF & OSCIOFNC_OFF & OSCIOFNC_OFF & FCKSM_CSDCMD & FNOSC_PRIPLL & PLL96MHZ_ON & PLLDIV_DIV2 & IESO_OFF)
_CONFIG3( WPFP_WPFP255 & SOSCSEL_SOSC & WUTSEL_LEG & ALTPMP_ALTPMPEN & WPDIS_WPDIS & WPCFG_WPCFGDIS & WPEND_WPENDMEM) 
	#endif	
#endif

/////////////////////////////////////////////////////////////////////////////
//                              OBJECT'S IDs
/////////////////////////////////////////////////////////////////////////////

#define ID_WHOLE 		20
#define ID_HF_DOMESTIC 		21
#define ID_AUX_DOMESTIC 		22
#define ID_CLEAN_DOMESTIC 		23

#define ID_HF_INTL 		24
#define ID_LED_INTL 		25
#define ID_AUX_INTL 		26
#define ID_CLEAN_INTL 		27

#define ID_MINUS 		28
#define ID_PLUS 		29
#define ID_ON 		30
#define ID_ON_GRAY 		31
#define ID_OFF 		32
#define ID_OFF_GRAY 		33

#define ID_START 		34
#define ID_STOP 		35
#define ID_RESET 		36

#define ID_YES 		37
#define ID_NO 		38

#define ID_HOME 		39
#define ID_STANDBY 		40

#define ID_0 		41
#define ID_1 		42
#define ID_2 		43
#define ID_3 		44
#define ID_4 		45
#define ID_5 		46
#define ID_6 		47
#define ID_7 		48
#define ID_8 		49
#define ID_9 		50
#define ID_BS 		51
#define ID_ENTER 		52

#define ID_TL_INVISIBLE 		53
#define ID_TR_INVISIBLE 		54
#define ID_BR_INVISIBLE 		55
#define ID_BL_INVISIBLE 		56

#define ID_START_GRAY 		57
#define ID_STOP_GRAY 		58

#define BOX_LEFT	58
#define BOX_TOP		28
#define BOX_RIGHT	420
#define BOX_BOTTOM	83
#define BOX_BACKGROUND	WHITE
#define TEXT_COLOR	GRAY4

#define CIRCLE_X	50
#define CIRCLE_Y	50
#define CIRCLE_SIZE	10

/////////////////////////////////////////////////////////////////////////////
//                            LOCAL PROTOTYPES
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
//                            IMAGES USED
/////////////////////////////////////////////////////////////////////////////
extern FONT_EXTERNAL EurostileLTStd;
extern const FONT_FLASH BigFonts;
extern const FONT_FLASH GOLMediumFont;
extern const FONT_FLASH GOLSmallFont;
/*
extern IMAGE_EXTERNAL Splash;
extern IMAGE_EXTERNAL HomeI;
extern IMAGE_EXTERNAL Home;
extern IMAGE_EXTERNAL Days;
extern IMAGE_EXTERNAL Code_Entry;
*/


extern IMAGE_EXTERNAL Splash;
extern IMAGE_EXTERNAL HomeI;
extern IMAGE_EXTERNAL HF_Mode;
extern IMAGE_EXTERNAL Home;
extern IMAGE_EXTERNAL Aux_Mode;
extern IMAGE_EXTERNAL LED_Mode;
extern IMAGE_EXTERNAL Clean_Mode;
extern IMAGE_EXTERNAL Yes;
extern IMAGE_EXTERNAL Stop_gray;
extern IMAGE_EXTERNAL Stop;
extern IMAGE_EXTERNAL Start_gray;
extern IMAGE_EXTERNAL Start;
extern IMAGE_EXTERNAL Standby;
extern IMAGE_EXTERNAL Reset_gray;
extern IMAGE_EXTERNAL Reminder;
extern IMAGE_EXTERNAL pumpO;
extern IMAGE_EXTERNAL pumpF;
extern IMAGE_EXTERNAL plusP;
extern IMAGE_EXTERNAL ON_gray;
extern IMAGE_EXTERNAL OFF_gray;
extern IMAGE_EXTERNAL No;
extern IMAGE_EXTERNAL minusM;
extern IMAGE_EXTERNAL Days;
extern IMAGE_EXTERNAL Code_Entry;

extern BYTE InternationalMode;
extern BYTE DaysLeft;
extern BYTE continuousFlag;
extern BYTE pinOKFlag;
extern DWORD lastJulian;
extern BYTE SDcard[100][14];
extern BYTE codeCount;


/////////////////////////////////////////////////////////////////////////////
//                                  MAIN
/////////////////////////////////////////////////////////////////////////////
GOL_SCHEME                  *altScheme; // alternative style scheme
WORD                        update = 0; // variable to update customized graphics
BYTE imageState = 0;
BYTE portB = 0;
BYTE portBsave;
BYTE portBsave1;
BYTE portA = 0x7F;
BYTE buttonReleased = 0;
BYTE vacuumLevel;
BYTE startFlag;
SHORT LEDseconds;
BYTE tempSeconds = 0;
BYTE ledBuzzerTimer;
BYTE buzzerTime;
BYTE buzzerFlag;
BYTE buzzerOff;
BYTE buttonTimer = 0;
BYTE buttonDelay = 0;
SHORT oldSeconds = 0;
BYTE hiddenButtonTimer = 0;
BYTE hiddenButtonFlagT = 0;
BYTE hiddenButtonFlagB = 0;
BYTE position = 0;
BYTE daysTimer;
char receiveBuffer[20];
BYTE seconds;
BYTE minutes;
WORD newMinutes;
BYTE hours;
BYTE date;
BYTE month;
BYTE year;
BYTE date1;
char String[20];
BYTE 	serialNumberFlag = 0;
WORD RTCtimer = 36;
FSFILE * pointer;
FSFILE * pointerSave;
char SDserialNumber[14];
char SDpinNumber[14];
BYTE pinFlag;
volatile WORD timer2 = 0;
volatile DWORD julianDate;
WORD daysLeftTimer = 50;	//5 sec
BYTE daysLeftOld = 0;
BYTE RTCflag = FALSE;

#define DEMODELAY 	2000
#define LED_BUZZER_TIME		40	//4 SEC
#define BUZZER_TIME		5	//0.5 SEC

void            TickInit(void);                 // starts tick counter
unsigned long CheckButtons(GOL_MSG *pMsg);
void HydraFacial(void);
void Auxillary(void);
void LightTherapy(void);
void SystemClean(void);
void ClearButtons(void);
void WriteDAC(SHORT data);
void ChangeVacuum(SHORT vacuumLevel);
void uitoa(WORD Value, BYTE* Buffer);
void ChangeTime(int seconds);
void StandbyImage(void);
void HomeSetup(void);
void CreditCode(void);
BYTE Debounce(void);
void uitoa(WORD Value, BYTE* Buffer);
void DisplayVacuum(SHORT level);
void ReminderImage(void);

/* */
int main(void)
{
	char tempPIN[10];
	char tempCode[20];
	char tempString[20];
	SHORT FScounter;
	BYTE intI, intJ;
	volatile WORD temp1;
	volatile DWORD temp2;
	BYTE difference;

    GOL_MSG msg;                    // GOL message structure to interact with GOL
    
     #if defined(PIC24FJ256DA210_DEV_BOARD)
    
    _ANSG8 = 0; /* S1 */
    _ANSE9 = 0; /* S2 */
    _ANSB5 = 0; /* S3 */
        
		// Configure SPI1 PPS pins (ENC28J60/ENCX24J600/MRF24WB0M or other PICtail Plus cards)
		__builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS

		RPOR8bits.RP16R = 8;       // assign RP16 for SCK1
		RPOR10bits.RP21R = 7;       // assign RP21 for SDO1
		RPINR20bits.SDI1R = 11;    // assign RP11 for SDI1
//		RPINR20bits.SDI1R = 37;    // assign RPI37 for SDI1

		__builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS                

    #else
    /////////////////////////////////////////////////////////////////////////////
    // ADC Explorer 16 Development Board Errata (work around 2)
    // RB15 should be output
    /////////////////////////////////////////////////////////////////////////////
    #ifndef MULTI_MEDIA_BOARD_DM00123
    LATBbits.LATB15 = 0;
    TRISBbits.TRISB15 = 0;
    #endif
    #endif
    /////////////////////////////////////////////////////////////////////////////
    #if defined(__dsPIC33F__) || defined(__PIC24H__)

    // Configure Oscillator to operate the device at 40Mhz
    // Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
    // Fosc= 8M*40(2*2)=80Mhz for 8M input clock
    PLLFBD = 38;                    // M=40
    CLKDIVbits.PLLPOST = 0;         // N1=2
    CLKDIVbits.PLLPRE = 0;          // N2=2
    OSCTUN = 0;                     // Tune FRC oscillator, if FRC is used

    // Disable Watch Dog Timer
    RCONbits.SWDTEN = 0;

    // Clock switching to incorporate PLL
    __builtin_write_OSCCONH(0x03);  // Initiate Clock Switch to Primary

    // Oscillator with PLL (NOSC=0b011)
    __builtin_write_OSCCONL(0x01);  // Start clock switching
    while(OSCCONbits.COSC != 0b011);

    // Wait for Clock switch to occur	
    // Wait for PLL to lock
    while(OSCCONbits.LOCK != 1)
    { };
    
    // Set PMD0 pin functionality to digital
    AD1PCFGL = AD1PCFGL | 0x1000;
    
    #elif defined(__PIC32MX__)
    INTEnableSystemMultiVectoredInt();
    SYSTEMConfigPerformance(GetSystemClock());
    #ifdef MULTI_MEDIA_BOARD_DM00123
    CPLDInitialize();
    CPLDSetGraphicsConfiguration(GRAPHICS_HW_CONFIG);
    CPLDSetSPIFlashConfiguration(SPI_FLASH_CHANNEL);
    #endif // #ifdef MULTI_MEDIA_BOARD_DM00123
    #endif // #if defined(__dsPIC33F__) || defined(__PIC24H__)

    GOLInit();                      // initialize graphics library &
								    // create default style scheme for GOL

    #if defined (GFX_PICTAIL_V1) || defined (GFX_PICTAIL_V2)
    EEPROMInit();                   // initialize Exp.16 EEPROM SPI
    BeepInit();
    #else
	    #if defined (USE_SST25VF016)
	    SST25Init();                    // initialize GFX3 SST25 flash SPI
	    #endif
    #endif

    SD_CS = 1;                     //Initialize Chip Select line
    SD_CS_TRIS = 0;           //Card Select - output

    UARTInit();
    
    TouchInit();                    // initialize touch screen
    TickInit();                     // initialize tick counter (for random number generation)
    HardwareButtonInit();           // Initialize the hardware buttons

//        TouchCalibration();
//        TouchStoreCalibration();

	
    // If it's a new board (EEPROM_VERSION byte is not programed) calibrate touch screen
    #if defined (GFX_PICTAIL_V1) || defined (GFX_PICTAIL_V2)
    if(GRAPHICS_LIBRARY_VERSION != EEPROMReadWord(ADDRESS_VERSION))
    {
        TouchCalibration();
        TouchStoreCalibration();
    }

    #else
	    #if defined (USE_SST25VF016)
	    if(GRAPHICS_LIBRARY_VERSION != SST25ReadWord(ADDRESS_VERSION))
	    {
	        TouchCalibration();
	        TouchStoreCalibration();
	    }
	    #elif defined (USE_SST39LF400)
		WORD tempArray[12], tempWord = 0x1234;
		
		SST39LF400Init(tempArray);
		tempWord = SST39LF400ReadWord(ADDRESS_VERSION);
		SST39LF400DeInit(tempArray);

	    if(GRAPHICS_LIBRARY_VERSION != tempWord)
	    {
	        TouchCalibration();
	        TouchStoreCalibration();
	    }
	    #endif
    #endif

    // Load touch screen calibration parameters from memory
    TouchLoadCalibration();
	SCL1_TRIS = 0;
//	DaysLeft = 30;

	//make sure RTC on
	seconds = ReadRTC(0x00);
	if((seconds & 0x80) == 0x80)
	{
		WriteRTC(0x00, 0x00);	//no - turn on

		date = ReadRTC(0x04) & 0x0F;
		date += (ReadRTC(0x04) >> 4) * 10;
		julianDate = (DWORD)date;

		month = ReadRTC(0x05) & 0x0F;
		month += (ReadRTC(0x05) >> 4) * 10;

		year = ReadRTC(0x06) & 0x0F;
		year = (ReadRTC(0x06) >> 4) * 12;

		switch(month)
		{
			case 1:
				break;

			case 2:
				julianDate += 31;
				break;

			case 3:
				julianDate += 59;
				break;

			case 4:
				julianDate += 90;
				break;

			case 5:
				julianDate += 120;
				break;

			case 6:
				julianDate += 151;
				break;

			case 7:
				julianDate += 181;
				break;

			case 8:
				julianDate += 212;
				break;

			case 9:
				julianDate += 243;
				break;

			case 10:
				julianDate += 273;
				break;

			case 11:
				julianDate += 304;
				break;

			case 12:
				julianDate += 334;
				break;

			default:
				break;

		}

		julianDate += (DWORD)year * 365;
		lastJulian = julianDate;
		TouchStoreCalibration();	//store in flash
		
	}

	// Configure SPI1 PPS pins (ENC28J60/ENCX24J600/MRF24WB0M or other PICtail Plus cards)
	__builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS

	RPOR8bits.RP16R = 8;       // assign RP16 for SCK1
	RPOR10bits.RP21R = 7;       // assign RP21 for SDO1
	RPINR20bits.SDI1R = 11;    // assign RP11 for SDI1
//		RPINR20bits.SDI1R = 37;    // assign RPI37 for SDI1

	__builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS                

	FScounter = 3;

	while(FScounter != 0)
	{
		if(!FSInit())	//try three times to read SD card
		{
			FScounter--;	
			
		}
		else
		{
			break;
		}
	}

	if(FScounter != 0)	//SD card found
	{
		// Open file 1 in read mode
		pointer = FSfopen ("Codes.txt", "r");
		if (pointer == NULL)
		  	while(1);

		intJ = 0;

	  	while(1)
	  	{
	  		// Check if this is the end of the file
	  		if (FSfeof (pointer))
	  		  break;

	  		// Read one 14-byte object
	  		if (FSfread (tempCode, 14, 1, pointer) != 1)
	  		  while(1);


	  		for(intI = 0; intI < 14; intI++)
	  		{
	  			SDcard[intJ][intI] = tempCode[intI];
	  			
	  		}

			intJ++;

		}

		codeCount = intJ;

		FSfclose (pointer);

		//now write to flash
		StoreSD();
		TouchStoreCalibration();
			
	}


	MPCInit();
	WriteMPCB(portB);
	WriteMPCA(portA);

 //   altScheme = GOLCreateScheme();  // create alternative style scheme
 //   altScheme->TextColor0 = BLACK;
//    altScheme->TextColor1 = BRIGHTBLUE;
    altScheme = GOLCreateScheme();  // create alternative style scheme
    altScheme->TextColor0 = GRAY0;
    altScheme->TextColor1 = GRAY0;
    altScheme->CommonBkColor = GRAY0;
//    altScheme->pFont = GRAY0;
    altScheme->EmbossDkColor = GRAY0;
    altScheme->EmbossLtColor = GRAY0;
    altScheme->Color0 = GRAY0;
    altScheme->Color1 = GRAY0;

	G1CON3bits.DPENOE = 0;		//disable GEN
	PMCON3bits.PTEN17 = 0;		// disable PMA17


    BtnCreate
    (
        ID_WHOLE,                    // object’s ID
        0,
        0,
        480,
        272,      // object’s dimension
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );                              // use alternative style scheme


    BtnCreate
    (
        ID_HF_DOMESTIC,                    // object’s ID
        15,
        96,
        158,
        189,      // object’s dimension
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );                              // use alternative style scheme

    BtnCreate
    (
        ID_AUX_DOMESTIC,                    // object’s ID
        169,
        96,
        312,
        189,      // object’s dimension
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );                              // use alternative style scheme

    BtnCreate
    (
        ID_CLEAN_DOMESTIC,                    // object’s ID
        321,
        96,
        464,
        189,      // object’s dimension
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );                              // use alternative style scheme

    BtnCreate
    (
        ID_HF_INTL,                    // object’s ID
        87,
        38,
        230,
        111,      // object’s dimension
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );                              // use alternative style scheme

    BtnCreate
    (
        ID_LED_INTL,                    // object’s ID
        87,
        140,
        230,
        233,      // object’s dimension
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );                              // use alternative style scheme

    BtnCreate
    (
        ID_AUX_INTL,                    // object’s ID
        249,
        38,
        392,
        111,      // object’s dimension
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );                              // use alternative style scheme

    BtnCreate
    (
        ID_CLEAN_INTL,                    // object’s ID
        249,
        140,
        392,
        233,      // object’s dimension
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );                              // use alternative style scheme

    BtnCreate
    (
        ID_MINUS,                    // object’s ID
        74,
        72,
        156,
        151,      // object’s dimension
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );                              // use alternative style scheme

    BtnCreate
    (
        ID_PLUS,                    // object’s ID
        317,
        72,
        399,
        151,      // object’s dimension
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );                              // use alternative style scheme

    BtnCreate
    (
        ID_ON,                    // object’s ID
        63,
        180,
        179,
        255,      // object’s dimension
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );                              // use alternative style scheme

    BtnCreate
    (
        ID_OFF,                    // object’s ID
        301,
        180,
        417,
        255,      // object’s dimension
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );                              // use alternative style scheme


    BtnCreate
    (
        ID_START,                    // object’s ID
        63,
        180,
        179,
        255,      // object’s dimension
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );                              // use alternative style scheme

    BtnCreate
    (
        ID_START_GRAY,                    // object’s ID
        63,
        180,
        179,
        255,      // object’s dimension
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );                              // use alternative style scheme

    BtnCreate
    (
        ID_STOP,                    // object’s ID
        182,
        180,
        298,
        255,      // object’s dimension
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );                              // use alternative style scheme

    BtnCreate
    (
        ID_STOP_GRAY,                    // object’s ID
        182,
        180,
        298,
        255,      // object’s dimension
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );                              // use alternative style scheme

    BtnCreate
    (
        ID_RESET,                    // object’s ID
        301,
        180,
        417,
        255,      // object’s dimension
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );                              // use alternative style scheme

    BtnCreate
    (
        ID_YES,                    // object’s ID
        63,
        180,
        179,
        255,      // object’s dimension
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );                              // use alternative style scheme

    BtnCreate
    (
        ID_NO,                    // object’s ID
        301,
        180,
        417,
        255,      // object’s dimension
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );                              // use alternative style scheme

    BtnCreate
    (
        ID_HOME,                    // object’s ID
        32,
        27,
        76,
        67,      // object’s dimension
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );                              // use alternative style scheme

    BtnCreate
    (
        ID_STANDBY,                    // object’s ID
        405,
        27,
        443,
        67,      // object’s dimension
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );                              // use alternative style scheme

    BtnCreate		
    (
		ID_1,		//1
    	56, 
    	85, 
    	145, 
    	142, 
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme

    );

    BtnCreate		
    (
		ID_2,		//2
    	147, 
    	85, 
    	236, 
    	142, 
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );

    BtnCreate		
    (
		ID_3,		//3
    	238, 
    	85, 
    	327, 
    	142, 
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );

    BtnCreate		
    (
		ID_4,		//4
    	56, 
    	142, 
    	145, 
    	199, 
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );

    BtnCreate		
    (
		ID_5,		//5
    	147, 
    	142, 
    	236, 
    	199, 
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );

    BtnCreate		
    (
		ID_6,		//6
    	238, 
    	142, 
    	327, 
    	199, 
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );

    BtnCreate		
    (
		ID_7,		//7
    	56, 
    	200, 
    	145, 
    	257, 
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );

    BtnCreate		
    (
		ID_8,		//8
    	147, 
    	200, 
    	236, 
    	257, 
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );

    BtnCreate		
    (
		ID_9,		//9
    	238, 
    	200, 
    	327, 
    	257, 
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
     );

    BtnCreate		
    (
		ID_0,		//0
    	329, 
    	200, 
    	418, 
    	257, 
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );

    BtnCreate	   
    (
		ID_BS,		//backspasce
    	329, 
    	85, 
    	418, 
    	142, 
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );

    BtnCreate		
    (
		ID_ENTER,		//enter
    	329, 
    	144, 
    	418, 
    	199, 
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );

    BtnCreate		
    (
		ID_TL_INVISIBLE,		//top left invisible
		0, 
		0, 
		70, 
		70, 
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );

    BtnCreate		
    (
		ID_TR_INVISIBLE,		//top right invisible
		200, 
		0, 
		270, 
		70, 
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );

    BtnCreate		
    (
		ID_BL_INVISIBLE,		//bottom left invisible
		0, 
		202, 
		70, 
		272, 
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );

    BtnCreate		
    (
		ID_BR_INVISIBLE,		//bottom right invisible
		410, 
		202, 
		480, 
		272, 
        0,                          // radius of the rounded edge
        BTN_DRAW,                   // draw the object after creation
        NULL,         // bitmap 
        "",                     // no text
        altScheme
    );

//    TouchCalibration();


    SetColor(BLACK);
    ClearDevice();

	if(!continuousFlag)
	{
		if(DaysLeft == 0)
		{
			CreditCode();
		
		}
		else
		{
			PutImage(0, 0, (void *) &Splash, 1);
		
		}

	}
	else
	{
		PutImage(0, 0, (void *) &Splash, 1);
		
	}

	
    update = 1;                     // to initialize the user graphics
    while(1)
    {
        if(GOLDraw())
        {                           // Draw GOL object
            TouchGetMsg(&msg);      // Get message from touch screen
            GOLMsg(&msg);           // Process message
        }
		if(RTCflag)
		{
			minutes = ReadRTC(0x01);
			minutes = ReadRTC(0x01) & 0x0F;
			minutes += (ReadRTC(0x01) >> 4) * 10;
			hours = ReadRTC(0x02);
			date = ReadRTC(0x04) & 0x0F;
			date += (ReadRTC(0x04) >> 4) * 10;
			julianDate = (DWORD)date;

			month = ReadRTC(0x05) & 0x0F;
			month += (ReadRTC(0x05) >> 4) * 10;

			year = ReadRTC(0x06) & 0x0F;
			year = (ReadRTC(0x06) >> 4) * 12;

			switch(month)
			{
				case 1:
					break;

				case 2:
					julianDate += 31;
					break;

				case 3:
					julianDate += 59;
					break;

				case 4:
					julianDate += 90;
					break;

				case 5:
					julianDate += 120;
					break;

				case 6:
					julianDate += 151;
					break;

				case 7:
					julianDate += 181;
					break;

				case 8:
					julianDate += 212;
					break;

				case 9:
					julianDate += 243;
					break;

				case 10:
					julianDate += 273;
					break;

				case 11:
					julianDate += 304;
					break;

				case 12:
					julianDate += 334;
					break;

				default:
					break;

			}

			julianDate += (DWORD)year * 365;

			lastJulian = (DWORD)SST25ReadWord(ADDRESS_LAST_JULIAN_LOW) & 0xFFFF;
			temp1 = SST25ReadWord(ADDRESS_LAST_JULIAN_HIGH);
			temp1 &= 0xFFFF;
			temp2 = temp1 << 16;

			lastJulian |= temp2;

			if(lastJulian != julianDate)	//if not the same
			{
				difference = (BYTE)(julianDate - lastJulian);

				if(difference > DaysLeft)
				{
					DaysLeft = 0;
				}
				else
				{
					DaysLeft -= difference;
					
				}

				lastJulian = julianDate;

				TouchStoreCalibration();	//store in flash


			}
				

			RTCflag = FALSE;
		}
    }
}

/////////////////////////////////////////////////////////////////////////////
// Function: WORD GOLMsgCallback(WORD objMsg, OBJ_HEADER* pObj, GOL_MSG* pMsg)
// Input: objMsg - translated message for the object,
//        pObj - pointer to the object,
//        pMsg - pointer to the non-translated, raw GOL message
// Output: if the function returns non-zero the message will be processed by default
// Overview: it's a user defined function. GOLMsg() function calls it each

//           time the valid message for the object received
/////////////////////////////////////////////////////////////////////////////
WORD GOLMsgCallback(WORD objMsg, OBJ_HEADER *pObj, GOL_MSG *pMsg)
{
    WORD    objectID;
    SHORT   width, height;
	BYTE	foundFlag;
	DWORD multiplier;
	DWORD checksum;
	DWORD expectedChecksum;
	char tempString[20];
	char tempPIN[10];
	char tempCode[20];
	volatile WORD intI, intJ, intK;

    objectID = GetObjID(pObj);


	if(imageState == 0)
	{
		if(buttonReleased == 0)
		{
	        if(objMsg == BTN_MSG_PRESSED)
	        {   // check if button is pressed
				buttonReleased = 1;
		//		buzzerFlag = 1;

				HomeSetup();
				return(1);
		        update = 1;
				
	        }
				
		}
		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
		{
			buttonReleased = 0;
			
		}


	}

	switch (imageState)
	{
		case HOME_IMAGE:
		    if(objectID == ID_HF_DOMESTIC)
		    {
				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {   // check if button is pressed
	    				buttonReleased = 1;

						HydraFacial();
				        update = 1;
						
			        }
						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }

		    if(objectID == ID_AUX_DOMESTIC)
		    {
				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
	    				buttonReleased = 1;
						Auxillary();
				        update = 1;
			        }
						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }

		    if(objectID == ID_CLEAN_DOMESTIC)
		    {
				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
	    				buttonReleased = 1;
						SystemClean();
				        update = 1;
			        }
						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }


		    if(objectID == ID_STANDBY)
		    {
//				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
	    				buttonReleased = 1;
						ReminderImage();
				        //update = 1;
			        }
						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }

		    if(objectID == ID_TL_INVISIBLE)
		    {
		 //		if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
	    				buttonReleased = 1;
						hiddenButtonFlagT = 1;
						hiddenButtonTimer = 30;	//3 sec
//				        SetColor(BRIGHTRED);	//red circle on
//						FillCircle(CIRCLE_X, CIRCLE_Y, CIRCLE_SIZE);
				        //update = 1;
			        }
						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }

		    if(objectID == ID_TR_INVISIBLE)
		    {
		   //		if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
	    				buttonReleased = 1;
						if(hiddenButtonFlagT == 1)	//second upper
						{
							hiddenButtonFlagT = 0;
							if(InternationalMode == 0)
							{
								InternationalMode = 1;
								
							}
							else
							{
								InternationalMode = 0;
								
							}
//   					        SetColor(GRAY0);		//red circle off
//  							FillCircle(CIRCLE_X, CIRCLE_Y, CIRCLE_SIZE);

							TouchStoreCalibration();	//store in flash
							
						}
				        //update = 1;
			        }
						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }


		    if(objectID == ID_BL_INVISIBLE)
		    {
			  //	if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
	    				buttonReleased = 1;
						hiddenButtonFlagB = 1;
						hiddenButtonTimer = 30;	//3 sec
//				        SetColor(BRIGHTRED);	//red circle on
//						FillCircle(CIRCLE_X, CIRCLE_Y, CIRCLE_SIZE);
			        //update = 1;
			        }
						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }

		    if(objectID == ID_BR_INVISIBLE)
		    {
		 //		if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
	    				buttonReleased = 1;
						if(hiddenButtonFlagB == 1)
						{
							hiddenButtonFlagB = 0;
//   					        SetColor(GRAY0);		//red circle off
//							FillCircle(CIRCLE_X, CIRCLE_Y, CIRCLE_SIZE);
					        
					        CreditCode();
							
						}
			        //update = 1;
			        }
			        else
			        {
						if(!continuousFlag)
						{
							uitoa(DaysLeft, (BYTE*)String);
							SetColor(BLACK);
							SetFont((void *) &BigFonts);
		
							while(!OutTextXY(400, 210, (XCHAR *)String));
							daysLeftTimer = 50;
								
						}

			        		
			        }

						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }

			break;

		case HOMEI_IMAGE:
		    if(objectID == ID_HF_INTL)
		    {
				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {   // check if button is pressed
	    				buttonReleased = 1;
						 HydraFacial();
				        update = 1;
			        }
						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }

		    if(objectID == ID_LED_INTL)
		    {
				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
	    				buttonReleased = 1;
						LightTherapy();
			    	    update = 1;
			        }
						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }

		    if(objectID == ID_AUX_INTL)
		    {
				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
	    				buttonReleased = 1;
						Auxillary();
			        	update = 1;
			        }
						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }

		    if(objectID == ID_CLEAN_INTL)
		    {
				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
	    				buttonReleased = 1;
						SystemClean();
				        update = 1;
			        }
						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }


		    if(objectID == ID_STANDBY)
		    {
				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
	    				buttonReleased = 1;
						ReminderImage();
				        //update = 1;
			        }
						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }

		    if(objectID == ID_TL_INVISIBLE)
		    {
		 //		if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
	    				buttonReleased = 1;
						hiddenButtonFlagT = 1;
						hiddenButtonTimer = 30;	//3 sec
//				        SetColor(BRIGHTRED);	//red circle on
//						FillCircle(CIRCLE_X, CIRCLE_Y, CIRCLE_SIZE);
				        //update = 1;
			        }
						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }

		    if(objectID == ID_TR_INVISIBLE)
		    {
		   //		if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
	    				buttonReleased = 1;
						if(hiddenButtonFlagT == 1)	//second upper
						{
							hiddenButtonFlagT = 0;
							if(InternationalMode == 0)
							{
								InternationalMode = 1;
								
							}
							else
							{
								InternationalMode = 0;
								
							}

//   					        SetColor(GRAY0);		//red circle off
//							FillCircle(CIRCLE_X, CIRCLE_Y, CIRCLE_SIZE);

							TouchStoreCalibration();	//store in flash
							
						}
				        //update = 1;
			        }
						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }

		    if(objectID == ID_BL_INVISIBLE)
		    {
			  //	if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
	    				buttonReleased = 1;
						hiddenButtonFlagB = 1;
						hiddenButtonTimer = 30;	//3 sec
//				        SetColor(BRIGHTRED);	//red circle on
//						FillCircle(CIRCLE_X, CIRCLE_Y, CIRCLE_SIZE);
			        //update = 1;
			        }
						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }

		    if(objectID == ID_BR_INVISIBLE)
		    {
		 //		if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
	    				buttonReleased = 1;
						if(hiddenButtonFlagB == 1)
						{
							hiddenButtonFlagB = 0;
//   					        SetColor(GRAY0);		//red circle off
//							FillCircle(CIRCLE_X, CIRCLE_Y, CIRCLE_SIZE);
					        
					        CreditCode();
							
						}
						else
						{
							if(!continuousFlag)
							{
								uitoa(DaysLeft, (BYTE*)String);
								SetColor(BLACK);
								SetFont((void *) &BigFonts);
			
								while(!OutTextXY(400, 210, (XCHAR *)String));
								daysLeftTimer = 50;
									
							}	
			        	
								
						}

			        //update = 1;
			        }
						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }

			break;

		case HF_IMAGE:
		    if(objectID == ID_MINUS)
		    {
				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {   // check if button is pressed
	    				buttonReleased = 1;
						buttonTimer = 10;

						if(vacuumLevel != 0)
						{
							vacuumLevel--;


							ChangeVacuum(vacuumLevel);

							DisplayVacuum(vacuumLevel);

						}

			    	    update = 1;
			        }
						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

	    		if (objMsg == BTN_MSG_STILLPRESSED)	//// check if button held
	    		{
					if(buttonTimer == 0)	//button held for one second
					{
						if(buttonDelay == 0)
						{
							buttonDelay = 1;
							if(vacuumLevel != 0)
								{
									vacuumLevel--;


									ChangeVacuum(vacuumLevel);

									DisplayVacuum(vacuumLevel);

								}

					    	    update = 1;

						}
						
					}
 	    			
	    		}

		    }

		    if(objectID == ID_PLUS)
		    {
				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
	    				buttonReleased = 1;
						buttonTimer = 10;

						if(vacuumLevel != 50)
						{
							vacuumLevel++;
							ChangeVacuum(vacuumLevel);

							DisplayVacuum(vacuumLevel);

						}

				        update = 1;

			        }
						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

	    		if (objMsg == BTN_MSG_STILLPRESSED)	//// check if button held
	    		{
					if(buttonTimer == 0)	//button held for one second
					{
						if(buttonDelay == 0)
						{
							buttonDelay = 1;
							if(vacuumLevel != 50)
								{
									vacuumLevel++;


									ChangeVacuum(vacuumLevel);

									DisplayVacuum(vacuumLevel);

								}

					    	    update = 1;

						}
						
					}
	    			
	    		}

		    }

		    if(objectID == ID_ON)
		    {
				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
		
	    				buttonReleased = 1;
					
					    PutImage(301, 180, (void *) &ON_gray, 1);

					    PutImage(63, 180, (void *) &pumpF, 1);

						portB &= PUMP_OFF;		//pump off
						WriteMPCB(portB);

				        update = 1;
			        }
 						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }


		    if(objectID == ID_OFF)
		    {
				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
	    				buttonReleased = 1;


					    PutImage(63, 180, (void *) &OFF_gray, 1);

					    PutImage(301, 180, (void *) &pumpO, 1);

						portB |= PUMP;		//pump on
						WriteMPCB(portB);

				        update = 1;

			        }
						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }


		    if(objectID == ID_HOME)
		    {
			 //	if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
	    				buttonReleased = 1;
						startFlag = 0;
	
						portB &= PUMP_OFF;		//pump off
						WriteMPCB(portB);

						portB &= VALVE3WAY_OFF;		//three-way valve off
						WriteMPCB(portB);

						ChangeVacuum(0);		//vacuum off
	
						HomeSetup();
				        update = 1;
			        }
						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }

		    if(objectID == ID_STANDBY)
		    {
			   //	if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
	    				buttonReleased = 1;
	
						portB &= PUMP_OFF;		//pump off
						WriteMPCB(portB);

						portB &= VALVE3WAY_OFF;		//three-way valve off
						WriteMPCB(portB);
					
						ChangeVacuum(0);		//vacuum off
	
						ReminderImage();
				        //update = 1;
			        }
						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }

			break;

		case AUX_IMAGE:
		    if(objectID == ID_MINUS)
		    {
				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {   // check if button is pressed
	    				buttonReleased = 1;
						buttonTimer = 10;

						if(vacuumLevel != 0)
						{
							vacuumLevel--;


							ChangeVacuum(vacuumLevel);

							DisplayVacuum(vacuumLevel);

						}

			    	    update = 1;
			        }
						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

	    		if (objMsg == BTN_MSG_STILLPRESSED)	//// check if button held
	    		{
					if(buttonTimer == 0)	//button held for one second
					{
						if(buttonDelay == 0)
						{
							buttonDelay = 1;
							if(vacuumLevel != 0)
								{
									vacuumLevel--;


									ChangeVacuum(vacuumLevel);

									DisplayVacuum(vacuumLevel);

								}

					    	    update = 1;

						}
						
					}
 	    			
	    		}

		    }

		    if(objectID == ID_PLUS)
		    {
				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
	    				buttonReleased = 1;
						buttonTimer = 10;

						if(vacuumLevel != 40)
						{
							vacuumLevel++;
							ChangeVacuum(vacuumLevel);

							DisplayVacuum(vacuumLevel);

						}

				        update = 1;

			        }
						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

	    		if (objMsg == BTN_MSG_STILLPRESSED)	//// check if button held
	    		{
					if(buttonTimer == 0)	//button held for one second
					{
						if(buttonDelay == 0)
						{
							buttonDelay = 1;
							if(vacuumLevel != 40)
								{
									vacuumLevel++;


									ChangeVacuum(vacuumLevel);

									DisplayVacuum(vacuumLevel);

								}

					    	    update = 1;

						}
						
					}
	    			
	    		}

		    }

		    if(objectID == ID_ON)
		    {
				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
		
	    				buttonReleased = 1;
					
					    PutImage(301, 180, (void *) &ON_gray, 1);

					    PutImage(63, 180, (void *) &pumpF, 1);
						portBsave = portBsave1;

						portBsave = portB;
						
						portBsave = PUMP_OFF;

						portB &= PUMP_OFF;		//pump off
						WriteMPCB(portB);

				        update = 1;
			        }
 						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }


		    if(objectID == ID_OFF)
		    {
				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
	    				buttonReleased = 1;


					    PutImage(63, 180, (void *) &OFF_gray, 1);

					    PutImage(301, 180, (void *) &pumpO, 1);
						portBsave = portB;

						portB |= PUMP;		//pump on
						WriteMPCB(portB);
						portBsave1 = portB;
				        update = 1;

			        }
						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }


		    if(objectID == ID_HOME)
		    {
//				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
	    				buttonReleased = 1;
						startFlag = 0;
	
						portB &= PUMP_OFF;		//pump off
						WriteMPCB(portB);

						portB &= VALVE3WAY_OFF;		//three-way valve off
						WriteMPCB(portB);

						ChangeVacuum(0);		//vacuum off
	
						HomeSetup();
				        update = 1;
			        }
						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }

		    if(objectID == ID_STANDBY)
		    {
//				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
	    				buttonReleased = 1;
	
						portB &= PUMP_OFF;		//pump off
						WriteMPCB(portB);

						portB &= VALVE3WAY_OFF;		//three-way valve off
						WriteMPCB(portB);
					
						ChangeVacuum(0);		//vacuum off
	
						ReminderImage();
				        //update = 1;
			        }
						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }

	
			break;

		case LT_IMAGE:
		    if(objectID == ID_START)
		    {
				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
		
	    				buttonReleased = 1;
						startFlag = 1;
					
					    PutImage(63, 180, (void *) &Start, 1);

					    PutImage(182, 180, (void *) &Stop_gray, 1);

				        update = 1;
			        }
 						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }

		    if(objectID == ID_STOP)
		    {
				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
		
	    				buttonReleased = 1;
					

						startFlag = 0;
					    PutImage(63, 180, (void *) &Start_gray, 1);

					    PutImage(182, 180, (void *) &Stop, 1);


				        update = 1;
			        }
 						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }

		    if(objectID == ID_RESET)
		    {
				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
		
	    				buttonReleased = 1;
					
						startFlag = 0;
						LEDseconds = 0;
						tempSeconds = 0;
						oldSeconds = 0;

						ChangeTime(LEDseconds);

					    PutImage(63, 180, (void *) &Start_gray, 1);

					    PutImage(182, 180, (void *) &Stop, 1);

				        update = 1;
			        }
 						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }

		    if(objectID == ID_HOME)
		    {
//				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
	    				buttonReleased = 1;
						startFlag = 0;
	
						LEDseconds = 0;
						tempSeconds = 0;
						oldSeconds = 0;

						portB &= BLUE_LED_OFF;		//blue LED off
						WriteMPCB(portB);

						portB &= RED_LED_OFF;		//red LED off
						WriteMPCB(portB);

						HomeSetup();
				        update = 1;
			        }
						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }

		    if(objectID == ID_STANDBY)
		    {
//				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
	    				buttonReleased = 1;
	
						startFlag = 0;
	
						LEDseconds = 0;
						tempSeconds = 0;
						oldSeconds = 0;

						portB &= BLUE_LED_OFF;		//blue LED off
						WriteMPCB(portB);

						portB &= RED_LED_OFF;		//red LED off
						WriteMPCB(portB);

						ReminderImage();
				        //update = 1;
			        }
						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }
			
			break;

		case SC_IMAGE:
		    if(objectID == ID_MINUS)
		    {
				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {   // check if button is pressed
	    				buttonReleased = 1;
						buttonTimer = 10;

						if(vacuumLevel != 0)
						{
							vacuumLevel--;


							ChangeVacuum(vacuumLevel);

							DisplayVacuum(vacuumLevel);

						}

			    	    update = 1;
			        }
						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

	    		if (objMsg == BTN_MSG_STILLPRESSED)	//// check if button held
	    		{
					if(buttonTimer == 0)	//button held for one second
					{
						if(buttonDelay == 0)
						{
							buttonDelay = 1;
							if(vacuumLevel != 0)
								{
									vacuumLevel--;


									ChangeVacuum(vacuumLevel);

									DisplayVacuum(vacuumLevel);

								}

					    	    update = 1;

						}
						
					}
 	    			
	    		}

		    }

		    if(objectID == ID_PLUS)
		    {
				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
	    				buttonReleased = 1;
						buttonTimer = 10;

						if(vacuumLevel != 50)
						{
							vacuumLevel++;
							ChangeVacuum(vacuumLevel);

							DisplayVacuum(vacuumLevel);

						}

				        update = 1;

			        }
						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

	    		if (objMsg == BTN_MSG_STILLPRESSED)	//// check if button held
	    		{
					if(buttonTimer == 0)	//button held for one second
					{
						if(buttonDelay == 0)
						{
							buttonDelay = 1;
							if(vacuumLevel != 50)
								{
									vacuumLevel++;


									ChangeVacuum(vacuumLevel);

									DisplayVacuum(vacuumLevel);

								}

					    	    update = 1;

						}
						
					}
	    			
	    		}

		    }

		    if(objectID == ID_ON)
		    {
				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
		
	    				buttonReleased = 1;
					
					    PutImage(301, 180, (void *) &ON_gray, 1);

					    PutImage(63, 180, (void *) &pumpF, 1);

						portB &= PUMP_OFF;		//pump off
						WriteMPCB(portB);

				        update = 1;
			        }
 						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }


		    if(objectID == ID_OFF)
		    {
				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
	    				buttonReleased = 1;


					    PutImage(63, 180, (void *) &OFF_gray, 1);

					    PutImage(301, 180, (void *) &pumpO, 1);

						portB |= PUMP;		//pump on
						WriteMPCB(portB);

				        update = 1;

			        }
						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }


		    if(objectID == ID_HOME)
		    {
//				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
	    				buttonReleased = 1;
						startFlag = 0;
	
						portB &= PUMP_OFF;		//pump off
						WriteMPCB(portB);

						portB &= VALVE3WAY_OFF;		//three-way valve off
						WriteMPCB(portB);

						ChangeVacuum(0);		//vacuum off
	
						HomeSetup();
				        update = 1;
			        }
						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }

		    if(objectID == ID_STANDBY)
		    {
//				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
	    				buttonReleased = 1;
	
						portB &= PUMP_OFF;		//pump off
						WriteMPCB(portB);

						portB &= VALVE3WAY_OFF;		//three-way valve off
						WriteMPCB(portB);
					
						ChangeVacuum(0);		//vacuum off
	
						ReminderImage();
				        //update = 1;
			        }
						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }

			break;

		case REMINDER_IMAGE:
		    if(objectID == ID_YES)
		    {
//				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
		
	    				buttonReleased = 1;
					
						StandbyImage();

				        update = 1;
			        }
 						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }

		    if(objectID == ID_NO)
		    {
  //				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
		
	    				buttonReleased = 1;
					
						SystemClean();

				        update = 1;
			        }
 						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }

		    if(objectID == ID_HOME)
		    {
		//		if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
	    				buttonReleased = 1;
						startFlag = 0;
	
						HomeSetup();
				        update = 1;
			        }
						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }


			break;

		case SB_IMAGE:
		    if(objectID == ID_STANDBY)
		    {
//				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
		
	    				buttonReleased = 1;
					
						HomeSetup();

				        update = 1;
			        }
 						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }


			break;

		case CREDIT_IMAGE:
		    if(objectID == ID_1)
		    {
				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
		
	    				buttonReleased = 1;
						String[position] = '1';
						String[position + 1] = 0;
			 	  		SetFont((void *) &BigFonts);
				   		SetColor(TEXT_COLOR);
				   		//SetColor(GRAY4);

						while(!OutTextXY(65, 30, (XCHAR *)String));
					
						position++;

				        update = 1;
			        }
 						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }

		    if(objectID == ID_2)
		    {
				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
		
	    				buttonReleased = 1;
						String[position] = '2';
						String[position + 1] = 0;
			 	  		SetFont((void *) &BigFonts);
				   		SetColor(TEXT_COLOR);
				   		//SetColor(GRAY4);

						while(!OutTextXY(65, 30, (XCHAR *)String));
					
						position++;

				        update = 1;
			        }
 						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }

		    if(objectID == ID_3)
		    {
				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
		
	    				buttonReleased = 1;
						String[position] = '3';
						String[position + 1] = 0;
			 	  		SetFont((void *) &BigFonts);
				   		SetColor(TEXT_COLOR);
				   		//SetColor(GRAY4);

						while(!OutTextXY(65, 30, (XCHAR *)String));
					
						position++;

				        update = 1;
			        }
 						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }

		    if(objectID == ID_4)
		    {
				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
		
	    				buttonReleased = 1;
						String[position] = '4';
						String[position + 1] = 0;
			 	  		SetFont((void *) &BigFonts);
				   		SetColor(TEXT_COLOR);
				   		//SetColor(GRAY4);

						while(!OutTextXY(65, 30, (XCHAR *)String));
					
						position++;

				        update = 1;
			        }
 						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }

		    if(objectID == ID_5)
		    {
				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
		
	    				buttonReleased = 1;
						String[position] = '5';
						String[position + 1] = 0;
			 	  		SetFont((void *) &BigFonts);
				   		SetColor(TEXT_COLOR);
				   		//SetColor(GRAY4);

						while(!OutTextXY(65, 30, (XCHAR *)String));
					
						position++;

				        update = 1;
			        }
 						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }

		    if(objectID == ID_6)
		    {
				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
		
	    				buttonReleased = 1;
						String[position] = '6';
						String[position + 1] = 0;
			 	  		SetFont((void *) &BigFonts);
				   		SetColor(TEXT_COLOR);
				   		//SetColor(GRAY4);

						while(!OutTextXY(65, 30, (XCHAR *)String));
					
						position++;

				        update = 1;
			        }
 						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }

		    if(objectID == ID_7)
		    {
				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
		
	    				buttonReleased = 1;
						String[position] = '7';
						String[position + 1] = 0;
			 	  		SetFont((void *) &BigFonts);
				   		SetColor(TEXT_COLOR);
				   		//SetColor(GRAY4);

						while(!OutTextXY(65, 30, (XCHAR *)String));
					
						position++;

				        update = 1;
			        }
 						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }

		    if(objectID == ID_8)
		    {
				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
		
	    				buttonReleased = 1;
						String[position] = '8';
						String[position + 1] = 0;
			 	  		SetFont((void *) &BigFonts);
				   		SetColor(TEXT_COLOR);
				   		//SetColor(GRAY4);

						while(!OutTextXY(65, 30, (XCHAR *)String));
					
						position++;

				        update = 1;
			        }
 						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }

		    if(objectID == ID_9)
		    {
				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
		
	    				buttonReleased = 1;
						String[position] = '9';
						String[position + 1] = 0;
			 	  		SetFont((void *) &BigFonts);
				   		SetColor(TEXT_COLOR);
				   		//SetColor(GRAY4);

						while(!OutTextXY(65, 30, (XCHAR *)String));
					
						position++;

				        update = 1;
			        }
 						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }

		    if(objectID == ID_0)
		    {
				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
		
	    				buttonReleased = 1;
						String[position] = '0';
						String[position + 1] = 0;
			 	  		SetFont((void *) &BigFonts);
				   		SetColor(TEXT_COLOR);
				   		//SetColor(GRAY4);

						while(!OutTextXY(65, 30, (XCHAR *)String));
					
						position++;

				        update = 1;
			        }
 						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }

		    if(objectID == ID_BS)
		    {
				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
						if(position != 0)
						{
							position--;
							//erase digit
							String[position] = 0;

							SetColor(BOX_BACKGROUND);
						 	while(!Bar(BOX_LEFT, BOX_TOP, BOX_RIGHT, BOX_BOTTOM));

						   	SetFont((void *) &BigFonts);
							SetColor(TEXT_COLOR);
						   	//SetColor(GRAY4);

						 	while(!OutTextXY(65, 30, (XCHAR *)String));
//							DrawDigit(String);


						}

				        update = 1;
			        }
 						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }

		    if(objectID == ID_ENTER)
		    {
				if(buttonReleased == 0)
				{
			        if(objMsg == BTN_MSG_PRESSED)
			        {
				        ReadSD();
				        for(intI = 0; intI < 14; intI++)
				        {
				        	SDserialNumber[intI] = SDcard[0][intI];
				        	
				        }

				        for(intI = 0; intI < 14; intI++)
				        {
				        	SDpinNumber[intI] = SDcard[1][intI];
				        	
				        }

						if(!pinOKFlag)
						{
							if(serialNumberFlag)
							{					
								serialNumberFlag = 0;
								//new serial number has been entered
								if(strncmp(SDserialNumber, String, 8) != 0)
								{
									SetColor(BOX_BACKGROUND);
									while(!Bar(BOX_LEFT, BOX_TOP, BOX_RIGHT, BOX_BOTTOM));

									SetFont((void *) &GOLMediumFont);
									SetColor(BLACK);
								
									while(!OutTextXY(65, 30, "Invalid serial number"));		//must delay~~~

									timer2 = 30;	//3 seconds

									while(timer2 != 0)
									{
										
									}
									HomeSetup();
									update = 1;
									return;	//not enough characters - bail out
								}
								else
								{
									update = 1;
									pinFlag = 1;
									position = 0;
									SetColor(BOX_BACKGROUND);
									while(!Bar(BOX_LEFT, BOX_TOP, BOX_RIGHT, BOX_BOTTOM));

									SetFont((void *) &GOLMediumFont);
									SetColor(BLACK);
								
									while(!OutTextXY(65, 30, "Enter PIN number"));
									return;
									
								}
							}
							if(pinFlag)
							{
								pinFlag = 0;
								//new serial number has been entered
								if(strncmp(SDpinNumber, String, 6) != 0)
								{
									SetColor(BOX_BACKGROUND);
									while(!Bar(BOX_LEFT, BOX_TOP, BOX_RIGHT, BOX_BOTTOM));

									SetFont((void *) &GOLMediumFont);
									SetColor(BLACK);
								
									while(!OutTextXY(65, 30, "Invalid PIN number"));
									timer2 = 30;	//3 seconds

									while(timer2 != 0)
									{
										
									}
									timer2 = 30;	//3 seconds

									while(timer2 != 0)
									{
										
									}

									HomeSetup();		//must delay
			
									update = 1;
									return;	//not enough characters - bail out
								}
								else
								{
									position = 0;
									pinOKFlag = 1;
									TouchStoreCalibration();	//store in flash

									SetColor(BOX_BACKGROUND);
									while(!Bar(BOX_LEFT, BOX_TOP, BOX_RIGHT, BOX_BOTTOM));

									SetFont((void *) &GOLMediumFont);
									SetColor(BLACK);
								
									while(!OutTextXY(65, 30, "Enter code"));
									update = 1;
									return;	
										
								}

							}
								
						}


						//Check string

						intJ = 0;

						for(intJ = 0; intJ < codeCount; intJ++)
						{
							for(intI = 0; intI < 14; intI++)
							{
								tempCode[intI] = SDcard[intJ][intI];
								
							}

							foundFlag = 0;

							if(strncmp("000000", String, 6) == 0)	//ignore erased codes
							{
								
							}
							else if(strncmp(&tempCode[5], String, 6) == 0)
							{
								if(tempCode[0] == 'C')
								{
									continuousFlag = 1;
									foundFlag = 1;
									DaysLeft = 0;
										
									SetColor(BOX_BACKGROUND);
									while(!Bar(BOX_LEFT, BOX_TOP, BOX_RIGHT, BOX_BOTTOM));
			
									SetFont((void *) &GOLMediumFont);
									SetColor(TEXT_COLOR);
			
									while(!OutTextXY(65, 30, "Valid Code"));


								}
								else
								{
									continuousFlag = 0;
									DaysLeft += 30;
									foundFlag = 1;
										
									SetColor(BOX_BACKGROUND);
									while(!Bar(BOX_LEFT, BOX_TOP, BOX_RIGHT, BOX_BOTTOM));
			
									SetFont((void *) &GOLMediumFont);
									SetColor(TEXT_COLOR);
			
									while(!OutTextXY(65, 30, "Valid Code"));
										
									width = GetImageWidth((void *) &Days);
									height = GetImageHeight((void *) &Days);
		
									PutImage(480 - width, 272 - height, (void *) &Days, 1);
									SetFont((void *) &GOLSmallFont);
		
									uitoa(DaysLeft, (BYTE*)String);
		
									while(!OutTextXY(365, 250, (XCHAR *)String));

									for(intI = 0; intI < 12; intI++)
									{
										SDcard[intJ][intI] = '0';
										
									}
									//store in flash
									StoreSD();


								}

								date = ReadRTC(0x04) & 0x0F;
								date += (ReadRTC(0x04) >> 4) * 10;
								julianDate = (DWORD)date;
						
								month = ReadRTC(0x05) & 0x0F;
								month += (ReadRTC(0x05) >> 4) * 10;
						
								year = ReadRTC(0x06) & 0x0F;
								year = (ReadRTC(0x06) >> 4) * 12;
						
								switch(month)
								{
									case 1:
										break;
						
									case 2:
										julianDate += 31;
										break;
						
									case 3:
										julianDate += 59;
										break;
						
									case 4:
										julianDate += 90;
										break;
						
									case 5:
										julianDate += 120;
										break;
						
									case 6:
										julianDate += 151;
										break;
						
									case 7:
										julianDate += 181;
										break;
						
									case 8:
										julianDate += 212;
										break;
						
									case 9:
										julianDate += 243;
										break;
						
									case 10:
										julianDate += 273;
										break;
						
									case 11:
										julianDate += 304;
										break;
						
									case 12:
										julianDate += 334;
										break;
						
									default:
										break;
						
								}
						
								julianDate += (DWORD)year * 365;
								lastJulian = julianDate;
								TouchStoreCalibration();	//store in flash

								timer2 = 30;	//3 seconds
	
								while(timer2 != 0)
								{
									
								}

								break;

							}

									
						}

						if(!foundFlag)
						{
						 	SetColor(BOX_BACKGROUND);
						 	while(!Bar(BOX_LEFT, BOX_TOP, BOX_RIGHT, BOX_BOTTOM));
	
						 	SetFont((void *) &GOLMediumFont);
						 	SetColor(TEXT_COLOR);
						 	
						 	while(!OutTextXY(65, 30, "Invalid Code")); //must delay
												 	
							timer2 = 30;	//3 seconds

						 	while(timer2 != 0)
						 	{
						 		
						 	}
	
						}

	   					HomeSetup();	

				        update = 1;
			        }
 						
				}
	    		if (objMsg == BTN_MSG_RELEASED)	//// check if button is pressed
	    		{
	    			buttonReleased = 0;
	    			
	    		}

		    }


			break;

		default:
			break;
	}


    return (1);
}

/////////////////////////////////////////////////////////////////////////////
// Function: WORD GOLDrawCallback()
// Output: if the function returns non-zero the draw control will be passed to GOL
// Overview: it's a user defined function. GOLDraw() function calls it each
//           time when GOL objects drawing is completed. User drawing should be done here.
//           GOL will not change color, line type and clipping region settings while

//           this function returns zero.
/////////////////////////////////////////////////////////////////////////////
WORD GOLDrawCallback(void)
{
    WORD        value, y, x;    // variables for the slider position
    static WORD prevValue = 0;

    if(update)
    {

        /* User defined graphics:	
		    This draws a series of bars indicating the value/position of the 
			slider's thumb. The height of the bars follows the equation of a 
			parabola "(y-k)^2 = 4a(x-h) with vertex at (k, h) at (60,100) on 
			the display. The value 110 is the 4*a constant. x & y are calculated
			based on the value of the slider thumb. The bars are drawn from 
			60 to 260 in the x-axis and 10 to 100 in the y-axis. Bars are drawn
			every 6 pixels with width of 4 pixels.
			
			Only the bars that are added or removed are drawn. Thus resulting 
			in an efficient customized drawing. 
		*/

        // prevValue will have the current value after drawing or removing bars.
        // reset the update flag


        update = 0;
    }

    return (1);
}

/*********************************************************************
* Function: WORD ExternalMemoryCallback(EXTDATA* memory, LONG offset, WORD nCount, void* buffer)
*
* PreCondition: none
*
* Input:  memory - pointer to the bitmap or font external memory structures
*                  (FONT_EXTERNAL or BITMAP_EXTERNAL)
*         offset - data offset
*         nCount - number of bytes to be transferred to the buffer
*         buffer - pointer to the buffer
*
* Output: number of bytes were transferred.
*
* Side Effects: none
*
* Overview: this function must be implemented in application. Graphics Library calls it
*           each time the data from external memory is required. The application must copy 
*           required data to the buffer provided.
*
* Note: none
*
********************************************************************/

// If there are several memories in the system they can be selected by IDs.
// In this demo ID for memory chip installed on Graphics PICTail board is assumed to be 0.
#define SST39_MEMORY    0

/* */

WORD ExternalMemoryCallback(IMAGE_EXTERNAL *memory, LONG offset, WORD nCount, void *buffer)
{
	
    if(memory->ID == SST39_MEMORY)
    {

        // Read data requested into buffer provided
        #if defined (GFX_PICTAIL_V3) || defined (PIC24FJ256DA210_DEV_BOARD)
        SST25ReadArray(memory->address + offset, // address to read from
        (BYTE *)buffer, nCount);
        #else
        SST39PMPInit();
        SST39ReadArray(memory->address + offset, // address to read from
        (BYTE *)buffer, nCount);
        LCDPMPInit();
        #endif
    }
	  
    return (nCount);
}

/////////////////////////////////////////////////////////////////////////////
// Function: Timer3 ISR
// Input: none
// Output: none
// Overview: increments tick counter. Tick is approx. 1 ms.
/////////////////////////////////////////////////////////////////////////////
#ifdef __PIC32MX__
    #define __T3_ISR    __ISR(_TIMER_3_VECTOR, ipl4)
#else
    #define __T3_ISR    __attribute__((interrupt, shadow, auto_psv))
#endif

/* */
void __T3_ISR _T3Interrupt(void)
{
    // Clear flag
    #ifdef __PIC32MX__
    mT3ClearIntFlag();
    #else
    IFS0bits.T3IF = 0;
    #endif

	TouchProcessTouch();    
}

/////////////////////////////////////////////////////////////////////////////
// Function: void TickInit(void)
// Input: none
// Output: none
// Overview: Initilizes the tick timer.
/////////////////////////////////////////////////////////////////////////////

/*********************************************************************
 * Section: Tick Delay
 *********************************************************************/
#define SAMPLE_PERIOD       500 // us
#define TICK_PERIOD			(GetPeripheralClock() * SAMPLE_PERIOD) / 4000000

/* */
void TickInit(void)
{

    TMR3 = 0;
    PR3 = TICK_PERIOD;
    IFS0bits.T3IF = 0;  //Clear flag
    IEC0bits.T3IE = 1;  //Enable interrupt
    T3CONbits.TON = 1;  //Run timer

    // Initialize Timer2
	
    TMR2 = 0;
    PR2 = 6250;		//100 ms timer
    IFS0bits.T2IF = 0;  //Clear flag
    IEC0bits.T2IE = 1;  //Enable interrupt
	T2CONbits.TCKPS1 = 1;	//256
	T2CONbits.TCKPS0 = 1;	//256
    T2CONbits.TON = 1;  //Run timer

    
}


/*****************************************************************************
  Function:
	void uitoa(WORD Value, BYTE* Buffer)

  Summary:
	Converts an unsigned integer to a decimal string.
	
  Description:
	Converts a 16-bit unsigned integer to a null-terminated decimal string.
	
  Precondition:
	None

  Parameters:
	Value	- The number to be converted
	Buffer	- Pointer in which to store the converted string

  Returns:
  	None
  ***************************************************************************/
void uitoa(WORD Value, BYTE* Buffer)
{
	BYTE i;
	WORD Digit;
	WORD Divisor;
	BOOL Printed = FALSE;

	if(Value)
	{
		for(i = 0, Divisor = 10000; i < 5u; i++)
		{
			Digit = Value/Divisor;
			if(Digit || Printed)
			{
				*Buffer++ = '0' + Digit;
				Value -= Digit*Divisor;
				Printed = TRUE;
			}
			Divisor /= 10;
		}
	}
	else
	{
		*Buffer++ = '0';
	}

	*Buffer = '\0';
}			    

void ChangeVacuum(SHORT vacuumLevel)
{
	switch (vacuumLevel)
	{
		case 0:
			WriteDAC(0);		
			break;

			case 1:															
				WriteDAC(370);												
				break;
	
			case 2:
				WriteDAC(629);		
				break;
	
			case 3:
				WriteDAC(888);		
				break;
	
			case 4:
				WriteDAC(1144);		
				break;
	
			case 5:
				WriteDAC(1399);		
				break;
	
			case 6:
				WriteDAC(1658);									 
				break;											 
																 
			case 7:
				WriteDAC(1914);		
				break;
	
			case 8:
				WriteDAC(2104);		
				break;
	
			case 9:
				WriteDAC(2425);		
				break;
	
			case 10:
				WriteDAC(2684);		
				break;
	
			case 11:
				WriteDAC(2939);								 
				break;										 
															 
			case 12:										 
				WriteDAC(3195);		
				break;
	
			case 13:
				WriteDAC(3454);		
				break;
	
			case 14:
				WriteDAC(3709);		
				break;
	
			case 15:
				WriteDAC(3965);		
				break;
	
			case 16:
				WriteDAC(4220);									
				break;											
																
			case 17:
				WriteDAC(4476);		
				break;
	
			case 18:
				WriteDAC(4735);		
				break;
	
			case 19:
				WriteDAC(4990);		
				break;
	
			case 20:
				WriteDAC(5243);		
				break;
	
			case 21:
				WriteDAC(5505);								
				break;										
															
			case 22:
				WriteDAC(5760);		
				break;
	
			case 23:
				WriteDAC(6019);		
				break;
	
			case 24:
				WriteDAC(6278);		
				break;
	
			case 25:
				WriteDAC(6530);		
				break;
	
			case 26:
				WriteDAC(6789);								 
				break;										 
															 
			case 27:										 
				WriteDAC(7048);		
				break;
	
			case 28:
				WriteDAC(7300);		
				break;
	
			case 29:
				WriteDAC(7536);		
				break;
	
	
			case 30:
				WriteDAC(7815);		
				break;
	
			case 31:
				WriteDAC(8070);		
				break;
	
			case 32:
				WriteDAC(8329);		
				break;
	
			case 33:
				WriteDAC(8604);								   
				break;										   
															   
			case 34:										   
				WriteDAC(8844);								   
				break;
	
			case 35:
				WriteDAC(9102);		
				break;
	
			case 36:
				WriteDAC(9358);		
				break;
	
			case 37:
				WriteDAC(9617);		
				break;
	
			case 38:
				WriteDAC(9869);		
				break;
	
			case 39:
				WriteDAC(10125);								 
				break;										 
															 
			case 40:										 
				WriteDAC(10384);		
				break;
	
			case 41:
				WriteDAC(10636);		
				break;
	
			case 42:
				WriteDAC(10895);		
				break;
	
			case 43:
				WriteDAC(11147);		
				break;
	
			case 44:
				WriteDAC(11406);		
				break;
	
			case 45:
				WriteDAC(11661);								 
				break;										 
															 
			case 46:										 
				WriteDAC(11914);		
				break;
	
			case 47:
				WriteDAC(12173);		
				break;
	
			case 48:
				WriteDAC(12428);		
				break;
	
			case 49:
				WriteDAC(12674);		
				break;
	
			case 50:
				WriteDAC(12933);		
				break;

		default:
			break;

	}

}

void WriteDAC(SHORT data)
{
    BYTE   clear,clear1,clear2;
	SHORT save1;
	SHORT shift;

 	save1 = SPI2CON1;

 	SPI2STATbits.SPIEN = 0;

    SPI2CON1bits.CKE = 1;
    SPI2CON1bits.CKP = 1;

    SPI2STATbits.SPIEN = 1;

	portA &= CS_DAC_OFF;
	WriteMPCA(portA);

	clear1 = (BYTE)((data >>8) & 0x3F);
	SPI2BUF = clear1;	//write
    while(!SPI2STATbits.SPIRBF);                     //Wait until cycle complete

	clear = SPI2BUF;

	clear2 = (BYTE)(data & 0xFF);

	SPI2BUF = clear2;	//write
    while(!SPI2STATbits.SPIRBF);                     //Wait until cycle complete

	clear = SPI2BUF;

	portA |= CS_DAC;
	WriteMPCA(portA);

 	SPI2STATbits.SPIEN = 0;
 	SPI2CON1 = save1;
 	SPI2STATbits.SPIEN = 1;

}


void HomeSetup(void)
{
	SHORT width, height;

    ClearDevice();

    if(!continuousFlag)
    {
    	if(DaysLeft == 0)
    	{
    		CreditCode();
    		return;
    	}

    }

	if(InternationalMode)
	{
        PutImage(0, 0, (void *) &HomeI, 1);
		imageState = HOMEI_IMAGE;
		
	}
	else
	{
        PutImage(0, 0, (void *) &Home, 1);
		imageState = HOME_IMAGE;
		
	}
}

void ReminderImage(void)
{
	imageState = REMINDER_IMAGE;

    ClearDevice();

    PutImage(0, 0, (void *) &Reminder, 1);
    PutImage(63, 180, (void *) &Yes, 1);
    PutImage(301, 180, (void *) &No, 1);

}
void StandbyImage(void)
{

	imageState = SB_IMAGE;

    SetColor(BLACK);

    ClearDevice();

    PutImage(0, 0, (void *) &Standby, 1);

	
}

void HydraFacial(void)
{
    SHORT   width, height;
	BYTE SW1detected = 0;
	BYTE SW2detected = 0;

	unsigned long buttonIndex;
	BYTE i;

	imageState = HF_IMAGE;
	vacuumLevel = 18;

    ClearDevice();

    PutImage(0, 0, (void *) &HF_Mode, 1);

	ChangeVacuum(vacuumLevel);

	DisplayVacuum(vacuumLevel);

	portB &= VALVE3WAY_OFF;		//three-way valve off
	WriteMPCB(portB);

//	portB |= PUMP;		//pump on
//	WriteMPCB(portB);

}


void Auxillary(void)
{
    SHORT   width, height;
	BYTE SW1detected = 0;
	BYTE SW2detected = 0;

	unsigned long buttonIndex;
	BYTE i;
	char String[4];

	vacuumLevel = 3;

	imageState = AUX_IMAGE;

    ClearDevice();

    PutImage(0, 0, (void *) &Aux_Mode, 1);

	ChangeVacuum(vacuumLevel);

	DisplayVacuum(vacuumLevel);

	portB |= VALVE3WAY;		//three-way valve on
	WriteMPCB(portB);


}

void LightTherapy(void)
{
	int seconds;

	unsigned long buttonIndex;
	BYTE i;

//	startFlag = 0;
//	LEDseconds = 0;
	imageState = LT_IMAGE;

    ClearDevice();

    PutImage(0, 0, (void *) &LED_Mode, 1);

    PutImage(63, 180, (void *) &Start_gray, 1);

    PutImage(182, 180, (void *) &Stop, 1);

	portB |= BLUE_LED;		//blue LED on
	WriteMPCB(portB);

	portB |= RED_LED;		//red LED on
	WriteMPCB(portB);
	
	LEDseconds = 0;
	tempSeconds = 0;
	oldSeconds = 0;
	seconds = LEDseconds;

	ChangeTime(seconds);



}

void SystemClean(void)
{
    SHORT   width, height;
	BYTE SW1detected = 0;
	BYTE SW2detected = 0;

	unsigned long buttonIndex;
	BYTE i;
	char String[4];

	imageState = SC_IMAGE;
	vacuumLevel = 26;

    ClearDevice();

    PutImage(0, 0, (void *) &Clean_Mode, 1);

	ChangeVacuum(vacuumLevel);

	DisplayVacuum(vacuumLevel);

}




//#define	CREDIT_IMAGE 	9

void CreditCode(void)
{
	unsigned long buttonIndex;
	BYTE i;
	BYTE enterFlag = 0;
	BYTE receiveBuffer[8];

	imageState = CREDIT_IMAGE;
	position = 0;

    ClearDevice();

    PutImage(0, 0, (void *) &Code_Entry, 1);

	String[0] = 0;

	SetColor(BOX_BACKGROUND);
	Bar(BOX_LEFT, BOX_TOP, BOX_RIGHT, BOX_BOTTOM);

	SetFont((void *) &GOLMediumFont);
	SetColor(BLACK);

	if(!pinOKFlag)
	{
		while(!OutTextXY(80, 35, "Enter Serial Number"));
		serialNumberFlag = 1;
			
	}
	else
	{
		while(!OutTextXY(65, 30, "Enter code"));
		update = 1;
		return;	
			
	}


}

    #define __T2_ISR    __attribute__((interrupt, shadow, auto_psv))


void __T2_ISR _T2Interrupt(void)	//100 ms
{
	SHORT height, width;
	BYTE temp;

    // Clear flag
    IFS0bits.T2IF = 0;

	if((!continuousFlag) && (DaysLeft != 0))
	{
		RTCtimer--;

		if(RTCtimer == 0)
		{
			RTCtimer = 36;
			RTCflag = TRUE;
				
		}
			
	}

	if(daysLeftTimer != 0)
	{
		daysLeftTimer--;
		if(daysLeftTimer == 0)
		{
			if((imageState ==  HOME_IMAGE) || (imageState ==  HOMEI_IMAGE))
			{
				HomeSetup();

			}
			
		}

		
	}

	if(timer2 != 0)
	{
		timer2--;
	}
	if(hiddenButtonFlagT || hiddenButtonFlagB)		//hidden button pressed?
	{
		if(hiddenButtonTimer != 0)	//second must be pressedd within 3 sec
		{
			hiddenButtonTimer--;
			if(hiddenButtonTimer == 0)
			{
//				SetColor(GRAY0);		//red circle off
//				FillCircle(CIRCLE_X, CIRCLE_Y, CIRCLE_SIZE);
				hiddenButtonFlagT = 0;	
				hiddenButtonFlagB = 0;	
			}
		}
	}

	if(!continuousFlag)
	{
		if((imageState ==  HOME_IMAGE) || (imageState ==  HOMEI_IMAGE))
		{
			if(DaysLeft < 11)
			{
				if(DaysLeft != daysLeftOld)
				{
					daysLeftOld = DaysLeft;
					//less than 11 dayss
					width = GetImageWidth((void *) &Days);
					height = GetImageHeight((void *) &Days);
	
					PutImage(480 - width, 272 - height, (void *) &Days, 1);
					SetFont((void *) &GOLSmallFont);
	
					uitoa(DaysLeft, (BYTE*)String);
	
					while(!OutTextXY(365, 250, (XCHAR *)String));
					daysLeftTimer = 50;


				}

			}
				
		}
			
	}


	if(startFlag)	//LED's on?
	{
		tempSeconds++;
		if(tempSeconds >= 10)
		{
			tempSeconds = 0;
			LEDseconds++;
		}

		if(oldSeconds != LEDseconds)
		{
			oldSeconds = LEDseconds;

			ChangeTime(oldSeconds);

		}

/*
		if(ledBuzzerTimer == 0)
		{
			ledBuzzerTimer = LED_BUZZER_TIME;
			buzzerFlag = 1;	
		}
		else
		{
			ledBuzzerTimer--;
		}
*/
	}

	if(buzzerFlag == 1)
	{
		buzzerFlag = 0;
		portA |= BUZZER;		//buzzer on
		WriteMPCA(portB);
		buzzerTime = BUZZER_TIME;
		buzzerOff = 0;

	}
	if(buzzerOff)
	{
		portA &= BUZZER_OFF;		//buzzer off
		WriteMPCA(portA);
		
	}

	if(buzzerTime)	//buzzer on?
	{
		buzzerTime--;
		if(buzzerTime == 0)
		{
			buzzerOff = 1;
		}
	}

	if(buttonTimer != 0)
	{
		buttonTimer--;
	}

	if(buttonDelay != 0)
	{
		buttonDelay--;
	}
}

void ChangeTime(int seconds)
{
    SHORT   width, height;
	BYTE minutes;
	BYTE second;
	BYTE temp;
	BYTE i;
	char character[4];
	char string[8];

	minutes = seconds / 60;

	temp = minutes * 60;

	second = seconds - temp;

    uitoa(minutes, (BYTE*)character);
	
	i = 0;

	if(strlen(character) < 2)
	{
		string[i++] = character[0];
		string[i++] = ':';
	}
	else
	{
		string[i++] = character[0];
		string[i++] = character[1];
		string[i++] = ':';
	}
	
    uitoa(second, (BYTE*)character);

	if(strlen(character) < 2)
	{
		string[i++] = '0';
		string[i++] = character[0];
	}
	else
	{
		string[i++] = character[0];
		string[i++] = character[1];
	}

	string[i] = 0;

	SetColor(GRAY0);
 	while(!Bar(86,66,399,144));		 

    SetFont((void *) &EurostileLTStd);

    // Get text width and height
    width = GetTextWidth(string, (void *) &EurostileLTStd);
    height = GetTextHeight((void *) &EurostileLTStd);

    SetFont((void *) &EurostileLTStd);

    SetColor(GRAY4);

    OutTextXY(242 - (width >> 1), 105 - (height >> 1), (XCHAR *)string);

	SetColor(BLACK);
	
}

void DisplayVacuum(SHORT level)
{
    SHORT   width, height;

 	uitoa(level, (BYTE*)String);

	SetColor(GRAY0);
 	while(!Bar(173,71,305,150));		 

    SetFont((void *) &EurostileLTStd);

    // Get text width and height
    width = GetTextWidth(String, (void *) &EurostileLTStd);
    height = GetTextHeight((void *) &EurostileLTStd);

    SetFont((void *) &EurostileLTStd);
	/*
Green to 00BB00
Yellow to FFCC00
Amber to FF6600
*/
	if(level == 0)
	{
	    SetColor(GRAY4);
		
	}
	else if(level < 25)
	{
	    SetColor(RGB565CONVERT(0,    187,    0));
		
	}
	else if(level < 37)
	{
	    SetColor(RGB565CONVERT(255,    153,    0));
		
	}

	else 
	{
	    SetColor(RGB565CONVERT(221,    0,    0));
		
	}

    OutTextXY(239 - (width >> 1), 110 - (height >> 1), (XCHAR *)String);
	SetColor(BLACK);

}


