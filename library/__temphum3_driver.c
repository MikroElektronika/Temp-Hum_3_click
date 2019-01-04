/*
    __temphum3_driver.c

-----------------------------------------------------------------------------

  This file is part of mikroSDK.

  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

#include "__temphum3_driver.h"
#include "__temphum3_hal.c"

/* ------------------------------------------------------------------- MACROS */

// START MODE
const uint8_t _TEMPHUM3_MODE_ACTIVE  = 0x01;
const uint8_t _TEMPHUM3_MODE_SLEEP   = 0x00;

const uint8_t _TEMPHUM3_REG_TEMPERATURE         = 0x00;
const uint8_t _TEMPHUM3_REG_HUMIDITY            = 0x02;
const uint8_t _TEMPHUM3_REG_INTERRUPT_DRDY      = 0x04;
const uint8_t _TEMPHUM3_REG_TEMPERATURE_MAX     = 0x05;
const uint8_t _TEMPHUM3_REG_HUMIDITY_MAX        = 0x06;
const uint8_t _TEMPHUM3_REG_INTERRUPT_MASK      = 0x07;
const uint8_t _TEMPHUM3_REG_TEMP_OFFSET_ADJUST  = 0x08;
const uint8_t _TEMPHUM3_REG_HUM_OFFSET_ADJUST   = 0x09;

const uint8_t _TEMPHUM3_REG_TEMP_THRESHOLD_HIGH = 0x0A;
const uint8_t _TEMPHUM3_REG_HUM_THRESHOLD_HIGH  = 0x0C;
const uint8_t _TEMPHUM3_REG_TEMP_THRESHOLD_LOW  = 0x0B;
const uint8_t _TEMPHUM3_REG_HUM_THRESHOLD_LOW   = 0x0D;

const uint8_t _TEMPHUM3_REG_RST_DRDY_INT_CONF   = 0x0E;
const uint8_t _TEMPHUM3_REG_MEASUREMENT_CONF    = 0x0F;
const uint8_t _TEMPHUM3_REG_MANUFACTURER_ID     = 0xFC;
const uint8_t _TEMPHUM3_REG_DEVICE_ID           = 0xFE;

// STATUS
const uint8_t _TEMPHUM3_STATUS_DRDY       = 0x80;
const uint8_t _TEMPHUM3_STATUS_TEMP_HIGH  = 0x40;
const uint8_t _TEMPHUM3_STATUS_TEMP_LOW   = 0x20;
const uint8_t _TEMPHUM3_STATUS_HUM_HIGH   = 0x10;
const uint8_t _TEMPHUM3_STATUS_HUM_LOW    = 0x08;

// MASK
const uint8_t _TEMPHUM3_MASK_DRDY       = 0x80;
const uint8_t _TEMPHUM3_MASK_TEMP_HIGH  = 0x40;
const uint8_t _TEMPHUM3_MASK_TEMP_LOW   = 0x20;
const uint8_t _TEMPHUM3_MASK_HUM_HIGH   = 0x10;
const uint8_t _TEMPHUM3_MASK_HUM_LOW    = 0x08;

//---- Configuration ----

// MODE
const uint8_t _TEMPHUM3_CONF_NORMAL_MODE   = 0x00 << 7;
const uint8_t _TEMPHUM3_CONF_SOFT_RESET    = 0x01 << 7;
// OUTPUT DATA RANGE
const uint8_t _TEMPHUM3_CONF_ODR_NO_REPEATED     = 0x00 << 4;
const uint8_t _TEMPHUM3_CONF_ODR_REPEATED_2MIN   = 0x01 << 4;
const uint8_t _TEMPHUM3_CONF_ODR_REPEATED_1MIN   = 0x02 << 4;
const uint8_t _TEMPHUM3_CONF_ODR_REPEATED_10SEC  = 0x03 << 4;
const uint8_t _TEMPHUM3_CONF_ODR_REPEATED_5SEC   = 0x04 << 4;
const uint8_t _TEMPHUM3_CONF_ODR_REPEATED_1SEC   = 0x05 << 4;
const uint8_t _TEMPHUM3_CONF_ODR_REPEATED_500MS  = 0x06 << 4;
const uint8_t _TEMPHUM3_CONF_ODR_REPEATED_200MS  = 0x07 << 4;

const uint8_t _TEMPHUM3_CONF_HEATER_OFF          = 0x00 << 3;
const uint8_t _TEMPHUM3_CONF_HEATER_ON           = 0x01 << 3;
const uint8_t _TEMPHUM3_CONF_INT_DRDY_HIGH_Z     = 0x00 << 2;
const uint8_t _TEMPHUM3_CONF_INT_DRDY_ENABLE     = 0x01 << 2;
const uint8_t _TEMPHUM3_CONF_INT_POL_LOW         = 0x00 << 1;
const uint8_t _TEMPHUM3_CONF_INT_POL_HIGH        = 0x01 << 1;
const uint8_t _TEMPHUM3_CONF_INT_MODE_SENSITIVE  = 0x00;
const uint8_t _TEMPHUM3_CONF_INT_MODE_COMPARATOR = 0x01;

//---- Mesurment Config ----
const uint8_t _TEMPHUM3_MCONF_TEMP_RES_14BIT = 0x00 << 6;
const uint8_t _TEMPHUM3_MCONF_TEMP_RES_11BIT = 0x01 << 6;
const uint8_t _TEMPHUM3_MCONF_TEMP_RES_9BIT  = 0x02 << 6;
const uint8_t _TEMPHUM3_MCONF_HUM_RES_14BIT  = 0x00 << 5;
const uint8_t _TEMPHUM3_MCONF_HUM_RES_11BIT  = 0x01 << 5;
const uint8_t _TEMPHUM3_MCONF_HUM_RES_9BIT   = 0x02 << 5;
const uint8_t _TEMPHUM3_MCONF_HUM_TEMP       = 0x00 << 1;
const uint8_t _TEMPHUM3_MCONF_TEMP_ONLY      = 0x01 << 1;
const uint8_t _TEMPHUM3_MCONF_HUM_ONLY       = 0x02 << 1;
const uint8_t _TEMPHUM3_MCONF_MEAS_START     = 0x01;
const uint8_t _TEMPHUM3_MCONF_MEAS_NO_ACTION = 0x00;






/* ---------------------------------------------------------------- VARIABLES */

#ifdef   __TEMPHUM3_DRV_I2C__
static uint8_t _slaveAddress;
#endif

/* -------------------------------------------- PRIVATE FUNCTION DECLARATIONS */



/* --------------------------------------------- PRIVATE FUNCTION DEFINITIONS */



/* --------------------------------------------------------- PUBLIC FUNCTIONS */

#ifdef   __TEMPHUM3_DRV_SPI__

void temphum3_spiDriverInit(T_TEMPHUM3_P gpioObj, T_TEMPHUM3_P spiObj)
{
    hal_spiMap( (T_HAL_P)spiObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif
#ifdef   __TEMPHUM3_DRV_I2C__

void temphum3_i2cDriverInit(T_TEMPHUM3_P gpioObj, T_TEMPHUM3_P i2cObj, uint8_t slave)
{
    _slaveAddress = slave;
    hal_i2cMap( (T_HAL_P)i2cObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif
#ifdef   __TEMPHUM3_DRV_UART__

void temphum3_uartDriverInit(T_TEMPHUM3_P gpioObj, T_TEMPHUM3_P uartObj)
{
    hal_uartMap( (T_HAL_P)uartObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif

// GPIO Only Drivers - remove in other cases
void temphum3_gpioDriverInit(T_TEMPHUM3_P gpioObj)
{
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
}

/* ----------------------------------------------------------- IMPLEMENTATION */

void temphum3_setMeasurement(uint8_t value)
{
    uint8_t wReg[2];
    wReg[0] = 0x0F;
    wReg[1] = value;

    hal_i2cStart();
    hal_i2cWrite(_slaveAddress, wReg, 2, END_MODE_STOP);
    Delay_100ms();
}
void temphum3_Configuration(uint8_t value)
{
    uint8_t wReg[2];
    wReg[0] = 0x0E;
    wReg[1] = value;

    hal_i2cStart();
    hal_i2cWrite(_slaveAddress, wReg, 2, END_MODE_STOP);
    Delay_100ms();
}
void temphum3_setMode(uint8_t value)
{
    hal_gpio_csSet(value);
}

float temphum3_getTemperature()
{
    uint8_t wReg[1];
    uint8_t msb[1];
    uint8_t lsb[1];
    volatile int16_t tempData;
    volatile float temp;
    wReg[0] = 0x01;
    
    hal_i2cStart();
    hal_i2cWrite(_slaveAddress, wReg, 1, END_MODE_RESTART);
    Delay_100ms();
    hal_i2cRead(_slaveAddress, msb, 1, END_MODE_STOP);


    tempData = msb[0];        // MSB
    tempData <<= 8;
    wReg[0] = 0x00;
    
    hal_i2cStart();
    hal_i2cWrite(_slaveAddress, wReg, 1, END_MODE_RESTART);
    Delay_100ms();
    hal_i2cRead(_slaveAddress, lsb, 1, END_MODE_STOP);
    
    tempData = tempData | lsb[0]; // LSB
    temp = tempData / 65536.00;
    temp = temp * 165;
    temp = temp - 40.00;
    
    Delay_100ms();
    return temp;
}

float temphum3_getHuminidy()
{
    uint8_t wReg[1];
    uint8_t msb[1];
    uint8_t lsb[1];
    volatile int16_t humData;
    volatile float hum;
    wReg[0] = 0x03;

    hal_i2cStart();
    hal_i2cWrite(_slaveAddress, wReg, 1, END_MODE_RESTART);
    Delay_100ms();
    hal_i2cRead(_slaveAddress, msb, 1, END_MODE_STOP);


    humData = msb[0];        // MSB
    humData <<= 8;
    wReg[0] = 0x02;

    hal_i2cStart();
    hal_i2cWrite(_slaveAddress, wReg, 1, END_MODE_RESTART);
    Delay_100ms();
    hal_i2cRead(_slaveAddress, lsb, 1, END_MODE_STOP);
    Delay_100ms();

    humData = humData | lsb[0]; // LSB
    hum = humData / 65536.00;
    hum = hum * 100;
    Delay_100ms();
    
    return hum;
}
//------------------------------------------------------------------------- ID

uint16_t temphum3_getID()
{
    uint8_t wReg[1];
    uint8_t rReg[2];
    uint16_t Value = 0;;
    wReg[0] = 0xFE;

    while (Value != 0x07D0)
    {
        hal_i2cStart();
        hal_i2cWrite(_slaveAddress, wReg, 1, END_MODE_RESTART);
        hal_i2cRead(_slaveAddress, rReg, 2, END_MODE_STOP);
        Delay_100ms();

        Value = rReg[1]; // MSB
        Value <<= 8;
        Value = Value | rReg[0]; //LSB
    }
    return Value;
}

uint16_t temphum3_getManufacturerID()
{
    uint8_t wReg[1];
    uint8_t rReg[2];
    uint16_t Value = 0;;
    wReg[0] = 0xFC;

    while (Value != 0x5449)
    {
        hal_i2cStart();
        hal_i2cWrite(_slaveAddress, wReg, 1, END_MODE_RESTART);
        hal_i2cRead(_slaveAddress, rReg, 2, END_MODE_STOP);
        Delay_100ms();

        Value = rReg[1]; // MSB
        Value <<= 8;
        Value = Value | rReg[0]; //LSB
    }
    return Value;
}
//-----------------------------------------------------------------------------

uint8_t temphum3_getStatusInterrupt(uint8_t mask)
{
    uint8_t wReg[1];
    uint8_t rReg[1];
    uint8_t status;
    wReg[0] = _TEMPHUM3_REG_INTERRUPT_DRDY;

    hal_i2cStart();
    hal_i2cWrite(_slaveAddress, wReg, 1, END_MODE_RESTART);
    hal_i2cRead(_slaveAddress, rReg, 1, END_MODE_STOP);
    Delay_100ms();

    if( rReg[0] & mask )return 1;
    else return 0;
}

uint8_t temphum3_getInterruptMask(uint8_t mask)
{
    uint8_t wReg[1];
    uint8_t rReg[1];
    uint8_t status;
    wReg[0] = _TEMPHUM3_REG_INTERRUPT_MASK;

    hal_i2cStart();
    hal_i2cWrite(_slaveAddress, wReg, 1, END_MODE_RESTART);
    hal_i2cRead(_slaveAddress, rReg, 1, END_MODE_STOP);
    Delay_100ms();

    if( rReg[0] & mask )return 1;
    else return 0;
}

void temphum3_setOffset(uint8_t reg, uint8_t value)
{
    uint8_t wReg[2];
    wReg[0] = reg;
    wReg[1] = value;

    hal_i2cStart();
    hal_i2cWrite(_slaveAddress, wReg, 2, END_MODE_STOP);
    Delay_100ms();
}

// ---------------------------------------------------------------------------
void temphum3_setHighTemp(uint8_t _data)
{
    uint8_t wReg[2];
    wReg[0] = 0x0B;
    wReg[1] = (uint8_t)((_data + 40)/165 * 0xFF);

    hal_i2cStart();
    hal_i2cWrite(_slaveAddress, wReg, 2, END_MODE_STOP);
    Delay_100ms();
}
void temphum3_setLowTemp(uint8_t _data)
{
    uint8_t wReg[2];
    wReg[0] = 0x0A;
    wReg[1] = (uint8_t)((_data + 40)/165 * 0xFF);

    hal_i2cStart();
    hal_i2cWrite(_slaveAddress, wReg, 2, END_MODE_STOP);
    Delay_100ms();
}

void temphum3_setHighHum(uint8_t _data)
{
    uint8_t wReg[2];
    wReg[0] = 0x0C;
    wReg[1] = (uint8_t)( (_data / 100) * 0xFF);

    hal_i2cStart();
    hal_i2cWrite(_slaveAddress, wReg, 2, END_MODE_STOP);
    Delay_100ms();
}

void temphum3_setLowHum(uint8_t _data)
{
    uint8_t wReg[2];
    wReg[0] = 0x0D;
    wReg[1] = (uint8_t)( (_data / 100) * 0xFF);

    hal_i2cStart();
    hal_i2cWrite(_slaveAddress, wReg, 2, END_MODE_STOP);
    Delay_100ms();
}
//---------------------------------------------------------------------------

float temphum3_getMaxHum()
{
    uint8_t wReg[1];
    uint8_t rReg[1];
    float HumData;
    wReg[0] = _TEMPHUM3_REG_HUMIDITY_MAX;

    hal_i2cStart();
    hal_i2cWrite(_slaveAddress, wReg, 1, END_MODE_RESTART);
    hal_i2cRead(_slaveAddress, rReg, 1, END_MODE_STOP);
    Delay_100ms();

    HumData = (rReg[0] / 256.00) * 100;
    Delay_100ms();
    return HumData;
}

float temphum3_getMaxTemp()
{
    uint8_t wReg[1];
    uint8_t rReg[1];
    float TempData;
    wReg[0] = _TEMPHUM3_REG_TEMPERATURE_MAX;

    hal_i2cStart();
    hal_i2cWrite(_slaveAddress, wReg, 1, END_MODE_RESTART);
    hal_i2cRead(_slaveAddress, rReg, 1, END_MODE_STOP);
    Delay_100ms();

    TempData = (rReg[0] / 256.00) * 100;
    TempData = TempData - 40;
    
    Delay_100ms();
    return TempData;
}

/* -------------------------------------------------------------------------- */
/*
  __temphum3_driver.c

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