/**  
  **********************************************************************************
             Copyright(c) 2025 Focaltech Co. Ltd.
                      All Rights Reserved
  **********************************************************************************
  */

#ifndef EPORT_REG_H_
#define EPORT_REG_H_

#include "type.h"


typedef struct
{
	__IO unsigned short EPPAR;  //0x00
	__IO unsigned char  EPIER;  //0x02
	__IO unsigned char  EPDDR;  //0x03
	__IO unsigned char  EPPDR;  //0x04
	__IO unsigned char  EPDR;   //0x05
	__IO unsigned char  EPPUER; //0x06
	__IO unsigned char  EPFR;   //0x07
	__IO unsigned char  EPODER; //0x08
	__IO unsigned char  EPLPR;  //0x09

}EPORT_TypeDef;


#endif /* EPORT_REG_H_ */
