/***************************************************************************//**
* @file PeakDet.cpp
* @version 1.0
* @author Samuel Kranz
* @date 06.12.2016
* @brief This is a spectrum analysis library for EUR II project
* @note if ATmega16U4 is used make sure you have added the ATmega16u4 support 
*       in the arduino core -> every #if defined(__AVR_ATmega32U4__) needs to be
*       replaces with #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__)
* @copyright Copyright 2016 NTB Buchs, http://www.ntb.ch/esa. All Rights reserved
******************************************************************************/

/**************************************************************************//**
*   Defines
*****************************************************************************/
//#define PEAK_DET_DEBUG


/**************************************************************************//**
*   Includes
*****************************************************************************/
#include "PeakDet.h"

/**************************************************************************//**
* @brief
*   Constructor
*
*****************************************************************************/
PeakDet::PeakDet(void) {}

/**************************************************************************//**
* @brief
*   Destructor
*
*****************************************************************************/
PeakDet::~PeakDet(void){}

/**************************************************************************//**
* @brief
*   Searches for the major peak in the given vector
*
* @param[in] double* vD
*   Waveform vector
*
* @param[in] double len
*   Length of vector
*   
* @return uint16_t index
*   Index of major peak
*
*****************************************************************************/
uint16_t PeakDet::findMajorPeak(double *vD, uint16_t len) 
{
	double maxY = 0;
	uint16_t IndexOfMaxY = 0;
	
	for (uint16_t i = 1; i < ((len >> 1) - 1); i++) 
	{
		if ((vD[i-1] < vD[i]) && (vD[i] > vD[i+1])) 
		{
			if (vD[i] > maxY) 
			{
				maxY = vD[i];
				IndexOfMaxY = i;
			}
		}
	}

	return IndexOfMaxY;
}

/**************************************************************************//**
* @brief
*   Searches for all peaks in the given vector
*
* @param[in] double* vD
*   Input vector
*
* @param[in] double len
*   Length of vector
*   
* @param[out] uint16_t* vD
*   Index vector of peaks - highest first
*
* @param[in] uint8_t len
*   Length of result vector
*
*****************************************************************************/
void PeakDet::findPeaks(double *vD, uint16_t lenD, uint16_t *vP, uint8_t  lenP) 
{
  ValuePair_TypeDef pairs[lenP];
  memset(pairs,0,lenP*sizeof(double)+lenP*sizeof(uint16_t));
  memset(vP,0,lenP * 2);
  
  #ifdef PEAK_DET_DEBUG
    Serial.print("Empty Array :");
    Serial.print(pairs[0].val);
    Serial.print("; idx ");
    Serial.println(pairs[0].idx);
   
    Serial.print("Empty Array :");
    Serial.print(pairs[1].val);
    Serial.print("; idx ");
    Serial.println(pairs[1].idx);
  #endif

	for (uint16_t i = 1; i < (lenD - 1); i++) 
	{
		if ((vD[i-1] < vD[i]) && (vD[i] > vD[i+1])) 
		{
    
      #ifdef PEAK_DET_DEBUG
            Serial.print("Check if greater than least val :");
            Serial.print(vD[i]);
            Serial.print(" > ");
            Serial.println(pairs[lenP-1].val);
      #endif
      
		if (vD[i] > pairs[lenP-1].val) 
		{

      #ifdef PEAK_DET_DEBUG
             Serial.print("Possible peak @ ");
             Serial.print(vD[i]);
             Serial.print("; idx ");
             Serial.println(i);
      #endif

			pairs[lenP-1].val = vD[i];
			pairs[lenP-1].idx = i;
			bubbleSort(pairs,lenP);  

      #ifdef PEAK_DET_DEBUG
              Serial.print("Sorted Array :");
              Serial.print(pairs[0].val);
              Serial.print("; idx ");
              Serial.println(pairs[0].idx);
             
              Serial.print("Sorted Array :");
              Serial.print(pairs[1].val);
              Serial.print("; idx ");
              Serial.println(pairs[1].idx);
      #endif
			}
		}
	}

  
	//Todo use UNION or memcpy to optimize 
  for(uint16_t i = 0; i < lenP; i++)
  {
    vP[i] = pairs[i].idx;
  }
 
}

/* Private functions */


/**************************************************************************//**
* @brief
*   Bubblesort algorithm
*   
* @detail
*   Sorts the given vector using a simple bubblesort algorithm. 
*
* @param[inout] ValuePair_TypeDef* vD
*   Input vector - output vector sorted
*
* @param[in] double len
*   Length of vector
*
*****************************************************************************/
void PeakDet::bubbleSort(ValuePair_TypeDef *vD, uint8_t len)
{
	uint8_t n = len;
	do
	{
		uint8_t newn = 1;
		for (uint8_t i=0; i < n-1; ++i)
		{
    
      #ifdef PEAK_DET_DEBUG
              Serial.print("Check if ");
              Serial.print(vD[i].val);
              Serial.print(" < ");
              Serial.println(vD[i+1].val);
      #endif

		  if (vD[i].val < vD[i+1].val)
		  {
      
        #ifdef PEAK_DET_DEBUG
                Serial.print("Swapping ");
                Serial.print(vD[i].val);
                Serial.print(" and ");
                Serial.println(vD[i+1].val);
        #endif
        
				SwapDouble(&vD[i].val, &vD[i+1].val);
				SwapU16(&vD[i].idx, &vD[i+1].idx);
				newn = i+1;
			}
		}
		n = newn;
	} while (n > 1);
 }


/**************************************************************************//**
* @brief
*   Swapps two doubles
*
* @param[inout] double* x
*   Input double x - Output double y
*
* @param[inout] double* y
*   Input double y - Output double x
*
*****************************************************************************/
void PeakDet::SwapDouble(double *x, double *y) 
{
	double temp = *x;
	*x = *y;
	*y = temp;
}

/**************************************************************************//**
* @brief
*   Swapps two uint16_t
*
* @param[inout] uint16_t* x
*   Input uint16_t x - Output uint16_t y
*
* @param[inout] uint16_t* y
*   Input uint16_t y - Output uint16_t x
*
*****************************************************************************/
void PeakDet::SwapU16(uint16_t *x, uint16_t *y) 
{
  uint16_t temp = *x;
  *x = *y;
  *y = temp;
}
