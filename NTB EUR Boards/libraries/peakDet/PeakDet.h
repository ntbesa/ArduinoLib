/***************************************************************************//**
 * @file PeakDet.h
 * @version 1.0
 * @author Samuel Kranz
 * @date 06.12.2016
 * @brief This is a spectrum analysis library for EUR II project
 * @note if ATmega16U4 is used make sure you have added the ATmega16u4 support 
 *       in the arduino core -> every #if defined(__AVR_ATmega32U4__) needs to be
 *       replaces with #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__)
 * @copyright Copyright 2016 NTB Buchs, http://www.ntb.ch/esa. All Rights reserved
 ******************************************************************************/

#ifndef PEAKDET_H_ /* Prevent loading library twice */
#define PEAKDET_H_

/**************************************************************************//**
*   Defines
*****************************************************************************/
#ifdef ARDUINO
	#if ARDUINO >= 100
		#include "Arduino.h"
	#else
		#include "WProgram.h" /* This is where the standard Arduino code lies */
	#endif
#else
	#include <stdlib.h>
	#include <stdio.h>
	#include <avr/io.h>
	#include <math.h>
	#include "defs.h"
	#include "types.h"
#endif

/**************************************************************************//**
*   TypeDefs
*****************************************************************************/
struct __attribute__((packed)) ValuePair_TypeDef {
  double val;
  uint16_t idx;
};


/**************************************************************************//**
*   Class
*****************************************************************************/
class PeakDet
{

public:
	/* Constructor */
	PeakDet::PeakDet(void) ;
	/* Destructor */
	PeakDet::~PeakDet(void) ;
	
	uint16_t PeakDet::findMajorPeak(double *vD, uint16_t len);
	void PeakDet::findPeaks(double *vD, uint16_t lenD, uint16_t *vP, uint8_t  lenP);
	
private:
	/* Functions */
	void PeakDet::SwapDouble(double *x, double *y);
	void PeakDet::SwapU16(uint16_t *x, uint16_t *y);
	void PeakDet::bubbleSort(ValuePair_TypeDef *vD, uint8_t len);
	
};

#endif /* PEAKDET_H_ */
