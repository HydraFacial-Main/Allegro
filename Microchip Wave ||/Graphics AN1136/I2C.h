///////////////////////////////////////////////////////////////////////
//
//  File Name   : I2C.h
//
//  Copyright (C)  2001-, All Rights Reserved
//
//  Authored by : Alpha Resources, LLC
//                6717 Rovilla Rd.
//                Blacklick, OH 43004
//				  614-245-4059
//
//                Rob Reasons - Senior Software Engineer
//
//  Project     : Wave II
//
//  Description : This module contains all the utilites functions and variables.
//
//  Original Author : Rob Reasons
//
//
///////////////////////////////////////////////////////////////////////

#ifndef EEPROM_H

#define   EEPROM_H


void MCPInit(void);
void WriteMPCB(BYTE port);
void WriteRTC(BYTE address, BYTE data);
WORD ReadRTC(BYTE address);
void WriteMPCA(BYTE port);


#endif

