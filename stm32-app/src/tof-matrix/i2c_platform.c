/*******************************************************************************
*
* Copyright (c) 2020 STMicroelectronics - All Rights Reserved
*
* License terms: STMicroelectronics Proprietary in accordance with licensing
* terms at www.st.com/sla0081
*
* STMicroelectronics confidential
* Reproduction and Communication of this document is strictly prohibited unless
* specifically authorized in writing by STMicroelectronics.
*
*******************************************************************************/

#include "i2c_platform.h"
#include "i2cdev.h"
#include "stm32fxxx.h" 
#include "FreeRTOS.h"
#include "task.h"


uint8_t RdByte(
	uint8_t  *dev_addr,
	uint16_t index,
	uint8_t  *pdata)
{
  int8_t status = 0;
  static uint8_t r8data;

  if (!i2cdevRead16(I2C1_DEV, *dev_addr>>1, index, 1, &r8data))
  {
    status = -1;
  }
  *pdata = r8data;

  return status;
}


uint8_t WrByte(
	uint8_t  *dev_addr,
	uint16_t index,
	uint8_t  data)
{
	int8_t status         = 0;

	if (!i2cdevWrite16(I2C1_DEV, *dev_addr>>1, index, 1, &data))
	{
	  status = -1;
	}

	return status;
}


uint8_t RdMulti(
	uint8_t  *dev_addr,
	uint16_t index,
	uint8_t  *pdata,
	uint32_t count)
{
	int8_t status         = 0;

	if (!i2cdevRead16(I2C1_DEV, *dev_addr>>1, index, count, pdata))
	{
		status = -1;
	}
	return status;
}


uint8_t WrMulti(
	uint8_t  *dev_addr,
	uint16_t index,
	uint8_t  *pdata,
	uint32_t count)
{
	int8_t status         = 0;

  if (!i2cdevWrite16(I2C1_DEV, *dev_addr>>1, index, count, pdata))
  {
    status = -1;
  }

	return status;
}


uint8_t Reset_Sensor(
		uint8_t dev)
{
	uint8_t status = 0;
	
	/* (Optional) Need to be implemented by customer. This function returns 0 if OK */
	
	/* Set pin LPN of 'dev' to LOW */
	/* Set pin AVDD of 'dev' to LOW */
	/* Set pin VDDIO of 'dev' to LOW */
	// WaitMs(100);

	/* Set pin LPN of 'dev' to HIGH */
	/* Set pin AVDD of 'dev' to HIGH */
	/* Set pin VDDIO of 'dev' to HIGH */
	// WaitMs(100);

	return status;
}

void SwapBuffer(
		uint8_t 		*buffer,
		uint16_t 	 	 size)
{
	uint32_t i, tmp;
	
	/* Example of possible implementation using <string.h> */
	for(i = 0; i < size; i = i + 4) 
	{
		tmp = (
		  buffer[i]<<24)
		|(buffer[i+1]<<16)
		|(buffer[i+2]<<8)
		|(buffer[i+3]);
		
		memcpy(&(buffer[i]), &tmp, 4);
	}
}	

uint8_t WaitMs(uint8_t  *dev_addr, uint32_t time_ms)
{
	uint8_t status = 255;

	/* Need to be implemented by customer. This function returns 0 if OK */
	vTaskDelay(M2T(time_ms));

	status = 0;
	return status;
}
