/***************************************************************************//**
* @file MatlabExporter.h
* @version 1.0
* @author Samuel Kranz
* @date 19.12.2016
* @brief This library prints arrays in matlab format
* @note if ATmega16U4 is used make sure you have added the ATmega16u4 support 
*       in the arduino core -> every #if defined(__AVR_ATmega32U4__) needs to be
*       replaces with #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__)
* @copyright Copyright 2016 NTB Buchs, http://www.ntb.ch/esa. All Rights reserved
******************************************************************************/

#ifndef MATLAB_EXPORTER_H_ /* Prevent loading library twice */
#define MATLAB_EXPORTER_H_

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
*   Class
*****************************************************************************/
class MatlabExporterClass
{

public:

  /* Constructor */
  MatlabExporterClass(void) {}
  
  /* Destructor */
  ~MatlabExporterClass(void){}

  static void exportArray(double *data, uint16_t len);
  static void exportArray(uint8_t *data, uint16_t len);
  static void exportArray(uint16_t *data, uint16_t len);
  static void exportArray(uint32_t *data, uint16_t len);

};

extern MatlabExporterClass MatlabExporter;

#endif /* MATLAB_EXPORTER_H_ */
