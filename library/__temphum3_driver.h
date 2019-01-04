/*
    __temphum3_driver.h

-----------------------------------------------------------------------------

  This file is part of mikroSDK.
  
  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

/**
@file   __temphum3_driver.h
@brief    TempHum3 Driver
@mainpage TempHum3 Click
@{

@image html sch.jpg

@}

@defgroup   TEMPHUM3
@brief      TempHum3 Click Driver
@{

| Global Library Prefix | **TEMPHUM3** |
|:---------------------:|:-----------------:|
| Version               | **1.0.0**    |
| Date                  | **Jan 2018.**      |
| Developer             | **MikroE Team**     |

*/
/* -------------------------------------------------------------------------- */

#include "stdint.h"

#ifndef _TEMPHUM3_H_
#define _TEMPHUM3_H_

/** 
 * @macro T_TEMPHUM3_P
 * @brief Driver Abstract type 
 */
#define T_TEMPHUM3_P    const uint8_t*

/** @defgroup TEMPHUM3_COMPILE Compilation Config */              /** @{ */

//  #define   __TEMPHUM3_DRV_SPI__                            /**<     @macro __TEMPHUM3_DRV_SPI__  @brief SPI driver selector */
   #define   __TEMPHUM3_DRV_I2C__                            /**<     @macro __TEMPHUM3_DRV_I2C__  @brief I2C driver selector */                                          
// #define   __TEMPHUM3_DRV_UART__                           /**<     @macro __TEMPHUM3_DRV_UART__ @brief UART driver selector */ 

                                                                       /** @} */
/** @defgroup TEMPHUM3_VAR Variables */                           /** @{ */

// START MODE
extern const uint8_t _TEMPHUM3_MODE_ACTIVE;
extern const uint8_t _TEMPHUM3_MODE_SLEEP;

extern const uint8_t _TEMPHUM3_REG_TEMPERATURE;
extern const uint8_t _TEMPHUM3_REG_HUMIDITY;
extern const uint8_t _TEMPHUM3_REG_INTERRUPT_DRDY;
extern const uint8_t _TEMPHUM3_REG_TEMPERATURE_MAX;
extern const uint8_t _TEMPHUM3_REG_HUMIDITY_MAX;
extern const uint8_t _TEMPHUM3_REG_INTERRUPT_MASK;
extern const uint8_t _TEMPHUM3_REG_TEMP_OFFSET_ADJUST;
extern const uint8_t _TEMPHUM3_REG_HUM_OFFSET_ADJUST;

extern const uint8_t _TEMPHUM3_REG_TEMP_THRESHOLD_HIGH;
extern const uint8_t _TEMPHUM3_REG_HUM_THRESHOLD_HIGH;
extern const uint8_t _TEMPHUM3_REG_TEMP_THRESHOLD_LOW;
extern const uint8_t _TEMPHUM3_REG_HUM_THRESHOLD_LOW;

extern const uint8_t _TEMPHUM3_REG_RST_DRDY_INT_CONF;
extern const uint8_t _TEMPHUM3_REG_MEASUREMENT_CONF;
extern const uint8_t _TEMPHUM3_REG_MANUFACTURER_ID;
extern const uint8_t _TEMPHUM3_REG_DEVICE_ID;

// STATUS
extern const uint8_t _TEMPHUM3_STATUS_DRDY;
extern const uint8_t _TEMPHUM3_STATUS_TEMP_HIGH;
extern const uint8_t _TEMPHUM3_STATUS_TEMP_LOW;
extern const uint8_t _TEMPHUM3_STATUS_HUM_HIGH;
extern const uint8_t _TEMPHUM3_STATUS_HUM_LOW;

// MASK
extern const uint8_t _TEMPHUM3_MASK_DRDY;
extern const uint8_t _TEMPHUM3_MASK_TEMP_HIGH;
extern const uint8_t _TEMPHUM3_MASK_TEMP_LOW;
extern const uint8_t _TEMPHUM3_MASK_HUM_HIGH;
extern const uint8_t _TEMPHUM3_MASK_HUM_LOW;

//---- Configuration ----

// MODE
extern const uint8_t _TEMPHUM3_CONF_NORMAL_MODE;
extern const uint8_t _TEMPHUM3_CONF_SOFT_RESET;
// OUTPUT DATA RANGE
extern const uint8_t _TEMPHUM3_CONF_ODR_NO_REPEATED;
extern const uint8_t _TEMPHUM3_CONF_ODR_REPEATED_2MIN;
extern const uint8_t _TEMPHUM3_CONF_ODR_REPEATED_1MIN;
extern const uint8_t _TEMPHUM3_CONF_ODR_REPEATED_10SEC;
extern const uint8_t _TEMPHUM3_CONF_ODR_REPEATED_5SEC;
extern const uint8_t _TEMPHUM3_CONF_ODR_REPEATED_1SEC;
extern const uint8_t _TEMPHUM3_CONF_ODR_REPEATED_500MS;
extern const uint8_t _TEMPHUM3_CONF_ODR_REPEATED_200MS;

extern const uint8_t _TEMPHUM3_CONF_HEATER_OFF;
extern const uint8_t _TEMPHUM3_CONF_HEATER_ON;
extern const uint8_t _TEMPHUM3_CONF_INT_DRDY_HIGH_Z;
extern const uint8_t _TEMPHUM3_CONF_INT_DRDY_ENABLE;
extern const uint8_t _TEMPHUM3_CONF_INT_POL_LOW;
extern const uint8_t _TEMPHUM3_CONF_INT_POL_HIGH;
extern const uint8_t _TEMPHUM3_CONF_INT_MODE_SENSITIVE;
extern const uint8_t _TEMPHUM3_CONF_INT_MODE_COMPARATOR;

//---- Mesurment Config ----
extern const uint8_t _TEMPHUM3_MCONF_TEMP_RES_14BIT;
extern const uint8_t _TEMPHUM3_MCONF_TEMP_RES_11BIT;
extern const uint8_t _TEMPHUM3_MCONF_TEMP_RES_9BIT;
extern const uint8_t _TEMPHUM3_MCONF_HUM_RES_14BIT;
extern const uint8_t _TEMPHUM3_MCONF_HUM_RES_11BIT;
extern const uint8_t _TEMPHUM3_MCONF_HUM_RES_9BIT;
extern const uint8_t _TEMPHUM3_MCONF_HUM_TEMP;
extern const uint8_t _TEMPHUM3_MCONF_TEMP_ONLY;
extern const uint8_t _TEMPHUM3_MCONF_HUM_ONLY;
extern const uint8_t _TEMPHUM3_MCONF_MEAS_START;
extern const uint8_t _TEMPHUM3_MCONF_MEAS_NO_ACTION;

                                                                       /** @} */
/** @defgroup TEMPHUM3_TYPES Types */                             /** @{ */



                                                                       /** @} */
#ifdef __cplusplus
extern "C"{
#endif

/** @defgroup TEMPHUM3_INIT Driver Initialization */              /** @{ */

#ifdef   __TEMPHUM3_DRV_SPI__
void temphum3_spiDriverInit(T_TEMPHUM3_P gpioObj, T_TEMPHUM3_P spiObj);
#endif
#ifdef   __TEMPHUM3_DRV_I2C__
void temphum3_i2cDriverInit(T_TEMPHUM3_P gpioObj, T_TEMPHUM3_P i2cObj, uint8_t slave);
#endif
#ifdef   __TEMPHUM3_DRV_UART__
void temphum3_uartDriverInit(T_TEMPHUM3_P gpioObj, T_TEMPHUM3_P uartObj);
#endif

// GPIO Only Drivers - remove in other cases
void temphum3_gpioDriverInit(T_TEMPHUM3_P gpioObj);
                                                                       /** @} */
/** @defgroup TEMPHUM3_FUNC Driver Functions */                   /** @{ */


// SETTINGS

/**
 * @brief This function settings Measurement
 *
 * Options:
      Temperature resolution (14, 11 or 9 bits),
      Humidity resolution (14, 11 or 9 bits),
      Measurement configuration (reads Humidity + Temperature, Temperature only or Humidity Only),
      Measurement trigger (no action or Start measurement).
 */
 
void temphum3_setMeasurement(uint8_t value);
/**
 * @brief This function configuration chip
 *
 * Options:
      Mode (Normal Operation mode or Soft Reset),
      Output Data Rate,
      Heater (Heater on or Heater off),
      DRDY/INT_EN pin configuration ( High Z or Enable),
      Interrupt polarity ( Active LOW or Active HIGH),
      Interrupt mode (Level sensitive or Comparator mode)
 */
void temphum3_Configuration(uint8_t value);

/**
 * @brief This function settings mode
 *
 * Options:
      Mode ACTIVE,
      Mode SLEEP
 */
void temphum3_setMode(uint8_t value);

// READ

/**
 * @brief This function reads temperature
 *
 * @return temperature data
 */
float temphum3_getTemperature();

/**
 * @brief This function reads huminidy
 *
 * @return huminidy data
 */
float temphum3_getHuminidy();

/**
 * @brief This function reads device ID
 *
 * @return device ID
 */
uint16_t temphum3_getID();

/**
 * @brief This function reads Manufacturer ID
 *
 * @return Manufacturer ID
 */
uint16_t temphum3_getManufacturerID();

// STATUS

/**
 * @brief This function reads status
 *
 * @param[int] status   which status you want to check
 * @return 1  - interrupt
 * @return 0  - No interrupt
 *
 * statuses you can check:
      DataReady bit status,
      Temperature threshold HIGH Interrupt status,
      Temperature threshold LOW Interrupt status,
      Humidity threshold HIGH Interrupt status,
      Humidity threshold LOW Interrupt status,
 */
uint8_t temphum3_getStatusInterrupt(uint8_t mask);
/**
 * @brief This function reads interrupt mask
 *
 * @param[int] status   which mask you want to check
 * @return 1  - interrupt
 * @return 0  - No interrupt
 *
 * statuses you can check:
      DataReady Interrupt mask,
      Temperature threshold HIGH Interrupt mask,
      Temperature threshold LOW Interrupt mask,
      Humidity threshold HIGH Interrupt mask,
      Humidity threshold LOW Interrupt mask,
 */
uint8_t temphum3_getInterruptMask(uint8_t mask);

// Write

/**
 * @brief This function settings offset temperature and huminidy
 *
 * @param[int] reg      register for setting offset
 * @param[int] value    data for writting to register
 */
void temphum3_setOffset(uint8_t reg, uint8_t value);

/**
 * @brief This functions settings max/min temperature and huminidy
 *
 * @param[int] _data     data for writting to register
 */
void temphum3_setHighTemp(uint8_t _data);
void temphum3_setLowTemp(uint8_t _data);
void temphum3_setHighHum(uint8_t _data);
void temphum3_setLowHum(uint8_t _data);

/**
 * @brief This function reads max huminidy
 *
 * @return max data
 */
float temphum3_getMaxHum();
/**
 * @brief This function reads max temperature
 *
 * @return max data
 */
float temphum3_getMaxTemp();


                                                                       /** @} */
#ifdef __cplusplus
} // extern "C"
#endif
#endif

/**
    @example Click_TempHum3_STM.c
    @example Click_TempHum3_TIVA.c
    @example Click_TempHum3_CEC.c
    @example Click_TempHum3_KINETIS.c
    @example Click_TempHum3_MSP.c
    @example Click_TempHum3_PIC.c
    @example Click_TempHum3_PIC32.c
    @example Click_TempHum3_DSPIC.c
    @example Click_TempHum3_AVR.c
    @example Click_TempHum3_FT90x.c
    @example Click_TempHum3_STM.mbas
    @example Click_TempHum3_TIVA.mbas
    @example Click_TempHum3_CEC.mbas
    @example Click_TempHum3_KINETIS.mbas
    @example Click_TempHum3_MSP.mbas
    @example Click_TempHum3_PIC.mbas
    @example Click_TempHum3_PIC32.mbas
    @example Click_TempHum3_DSPIC.mbas
    @example Click_TempHum3_AVR.mbas
    @example Click_TempHum3_FT90x.mbas
    @example Click_TempHum3_STM.mpas
    @example Click_TempHum3_TIVA.mpas
    @example Click_TempHum3_CEC.mpas
    @example Click_TempHum3_KINETIS.mpas
    @example Click_TempHum3_MSP.mpas
    @example Click_TempHum3_PIC.mpas
    @example Click_TempHum3_PIC32.mpas
    @example Click_TempHum3_DSPIC.mpas
    @example Click_TempHum3_AVR.mpas
    @example Click_TempHum3_FT90x.mpas
*/                                                                     /** @} */
/* -------------------------------------------------------------------------- */
/*
  __temphum3_driver.h

  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

3. All advertising materials mentioning features or use of this software
   must display the following acknowledgement:
   This product includes software developed by the MikroElektonika.

4. Neither the name of the MikroElektonika nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY MIKROELEKTRONIKA ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL MIKROELEKTRONIKA BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------- */