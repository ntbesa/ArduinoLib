/***************************************************************************//**
* @file MatlabExporter.cpp
* @version 1.0
* @author Samuel Kranz
* @date 19.12.2016
* @brief This library prints arrays in matlab format
* @note if ATmega16U4 is used make sure you have added the ATmega16u4 support 
*       in the arduino core -> every #if defined(__AVR_ATmega32U4__) needs to be
*       replaces with #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__)
* @copyright Copyright 2016 NTB Buchs, http://www.ntb.ch/esa. All Rights reserved
******************************************************************************/

/**************************************************************************//**
*   Defines
*****************************************************************************/
//#define MATLAB_EXPORTER_DEBUG


/**************************************************************************//**
*   Includes
*****************************************************************************/
#include "MatlabExporter.h"

MatlabExporterClass MatlabExporter;


/**************************************************************************//**
 * @brief
 *   Export double array
 *
 * @details
 *   Prints array to serial port in a matlab readable format.
 *
 * @param[in] double* data
 *   Pointer to array
 *
 * @param[in] uint16_t len
 *   Length of array
 *
 *****************************************************************************/
static void MatlabExporterClass::exportArray(double *data, uint16_t len)
{
  Serial.print("[");
  for(uint16_t i = 0; i < len-1; i++)
  {
    Serial.print(data[i]);
    Serial.print(";");
  }
  Serial.print(data[len-1]);
  Serial.println("];");
}

/**************************************************************************//**
 * @brief
 *   Export uint32_t array
 *
 * @details
 *   Prints array to serial port in a matlab readable format.
 *
 * @param[in] uint32_t* data
 *   Pointer to array
 *
 * @param[in] uint16_t len
 *   Length of array
 *
 *****************************************************************************/
static void MatlabExporterClass::exportArray(uint32_t *data, uint16_t len)
{
  Serial.print("[");
  for(uint16_t i = 0; i < len-1; i++)
  {
    Serial.print(data[i]);
    Serial.print(";");
  }
  Serial.print(data[len-1]);
  Serial.println("];");
}

/**************************************************************************//**
 * @brief
 *   Export uint16_t array
 *
 * @details
 *   Prints array to serial port in a matlab readable format.
 *
 * @param[in] uint16_t* data
 *   Pointer to array
 *
 * @param[in] uint16_t len
 *   Length of array
 *
 *****************************************************************************/
static void MatlabExporterClass::exportArray(uint16_t *data, uint16_t len)
{
  Serial.print("[");
  for(uint16_t i = 0; i < len-1; i++)
  {
    Serial.print(data[i]);
    Serial.print(";");
  }
  Serial.print(data[len-1]);
  Serial.println("];");
}

/**************************************************************************//**
 * @brief
 *   Export uint8_t array
 *
 * @details
 *   Prints array to serial port in a matlab readable format.
 *
 * @param[in] uint8_t* data
 *   Pointer to array
 *
 * @param[in] uint16_t len
 *   Length of array
 *
 *****************************************************************************/
static void MatlabExporterClass::exportArray(uint8_t *data, uint16_t len)
{
  Serial.print("[");
  for(uint16_t i = 0; i < len-1; i++)
  {
    Serial.print(data[i]);
    Serial.print(";");
  }
  Serial.print(data[len-1]);
  Serial.println("];");
}

