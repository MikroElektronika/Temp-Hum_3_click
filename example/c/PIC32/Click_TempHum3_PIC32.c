/*
Example for TempHum3 Click

    Date          : Jan 2018.
    Author        : MikroE Team

Test configuration PIC32 :
    
    MCU                : P32MX795F512L
    Dev. Board         : EasyPIC Fusion v7
    PIC32 Compiler ver : v4.0.0.0

---
Description :

The application is composed of three sections :

- System Initialization - Initializes I2C module and CS pin as OUTPUT and INT pin as INPUT
- Application Initialization - Initializes Driver init and settings chip mode ACTIVE and configuration Measurement and Interrupt, 
  then settings maximum / minimum possible Temperature and Huminidy.
- Application Task - (code snippet) - Reads the temperature and huminidy and logs to the USBUART every 500 ms.

*/

#include "Click_TempHum3_types.h"
#include "Click_TempHum3_config.h"

char temp_txt[256];
char hum_txt[256];
float  Temperature = 0;
float  Huminidy = 0;

void systemInit()
{
    mikrobus_gpioInit( _MIKROBUS1, _MIKROBUS_INT_PIN, _GPIO_INPUT );
    mikrobus_gpioInit( _MIKROBUS1, _MIKROBUS_CS_PIN, _GPIO_OUTPUT );
    mikrobus_i2cInit( _MIKROBUS1, &_TEMPHUM3_I2C_CFG[0] );
    mikrobus_logInit( _LOG_USBUART_A, 9600 );
    Delay_ms( 100 );
}

void applicationInit()
{   
    temphum3_i2cDriverInit( (T_TEMPHUM3_P)&_MIKROBUS1_GPIO, (T_TEMPHUM3_P)&_MIKROBUS1_I2C, 0x41 );
    temphum3_setMode(_TEMPHUM3_MODE_ACTIVE);
    Delay_100ms();
    temphum3_setMeasurement( _TEMPHUM3_MCONF_TEMP_RES_14BIT |
                             _TEMPHUM3_MCONF_HUM_RES_14BIT | 
                             _TEMPHUM3_MCONF_HUM_TEMP | 
                             _TEMPHUM3_MCONF_MEAS_START);
    Delay_100ms();
    
    temphum3_Configuration( _TEMPHUM3_CONF_NORMAL_MODE  |
                            _TEMPHUM3_CONF_ODR_REPEATED_1SEC |
                            _TEMPHUM3_CONF_HEATER_OFF |
                            _TEMPHUM3_CONF_INT_DRDY_HIGH_Z |
                            _TEMPHUM3_CONF_INT_POL_LOW |
                            _TEMPHUM3_CONF_INT_MODE_SENSITIVE);
                            
    mikrobus_logWrite("--- Init done---",_LOG_LINE);
    
    temphum3_setHighTemp(45);
    temphum3_setLowTemp(10);
    temphum3_setHighHum(30);
    temphum3_setLowHum(70);
    Delay_100ms();
    
    mikrobus_logWrite("--- Settings Temp&Hum done---",_LOG_LINE);
}

void applicationTask()
{
    Temperature = temphum3_getTemperature();
    Delay_100ms();
    Huminidy = temphum3_getHuminidy();
    FloatToStr(Temperature,temp_txt);
    mikrobus_logWrite("Temperature : ",_LOG_TEXT);
    mikrobus_logWrite(temp_txt,_LOG_LINE);
    FloatToStr(Huminidy,hum_txt);
    mikrobus_logWrite("Huminidy : ",_LOG_TEXT);
    mikrobus_logWrite(hum_txt,_LOG_LINE);
    Delay_ms( 500 );
}

void main()
{
    systemInit();
    applicationInit();

    while (1)
    {
            applicationTask();
    }
}