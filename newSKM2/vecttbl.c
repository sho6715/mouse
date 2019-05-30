/************************************************************************
*
* Device     : RX/RX600/RX63N,RX631
*
* File Name  : vecttbl.c
*
* Abstract   : Initialize of Vector Table and MDE register.
*
* History    : 0.10 (2011-02-21)  [Hardware Manual Revision : 0.01]
*            : 0.50 (2011-06-20)  [Hardware Manual Revision : 0.50]
*            : 1.00 (2013-02-18)  [Hardware Manual Revision : 1.00]
*
* NOTE       : THIS IS A TYPICAL EXAMPLE.
*
* Copyright (C) 2013 (2011) Renesas Electronics Corporation. and
* Renesas Solutions Corp. All rights reserved.
*
************************************************************************/

#include "vect.h"

#pragma section C FIXEDVECT

void (*const Fixed_Vectors[])(void) = {
//;0xffffffd0  Exception(Supervisor Instruction)
    Excep_SuperVisorInst,
//;0xffffffd4  Exception(Access Instruction)
    Excep_AccessInst,
//;0xffffffd8  Reserved
    Dummy,
//;0xffffffdc  Exception(Undefined Instruction)
    Excep_UndefinedInst,
//;0xffffffe0  Reserved
    Dummy,
//;0xffffffe4  Exception(Floating Point)
    Excep_FloatingPoint,
//;0xffffffe8  Reserved
    Dummy,
//;0xffffffec  Reserved
    Dummy,
//;0xfffffff0  Reserved
    Dummy,
//;0xfffffff4  Reserved
    Dummy,
//;0xfffffff8  NMI
    NonMaskableInterrupt,
//;0xfffffffc  RESET
//;<<VECTOR DATA START (POWER ON RESET)>>
//;Power On Reset PC
    /*(void*)*/ PowerON_Reset_PC                                                                                                                 
//;<<VECTOR DATA END (POWER ON RESET)>>
};

#pragma address id_code=0xffffffa0 // ID codes (Default)
const unsigned long id_code[4] = {
        0x45010203,
    	0x04050607,
    	0x08090A0B,
   		0x0C0D0E0F
};
//  è„à 8byte = 0x4501020304050607
//  â∫à 8byte = 0x08090A0B0C0D0E0F
//  ëSëÃòAåã  = 0x450102030405060708090A0B0C0D0E0F

#pragma address MDEreg=0xffffff80 // MDE register (Single Chip Mode)
#ifdef __BIG
	const unsigned long MDEreg = 0xfffffff8; // big
#else
	const unsigned long MDEreg = 0xffffffff; // little
#endif
