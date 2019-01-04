![MikroE](http://www.mikroe.com/img/designs/beta/logo_small.png)

---

# TempHum3 Click

- **CIC Prefix**  : TEMPHUM3
- **mikroSDK**    : YES
- **Author**      : Katarina Perendic
- **Verison**     : 1.0.0
- **Date**        : Jan 2018.

---

### Software Support

We provide a library for the TempHum3 Click on our [LibStock](https://libstock.mikroe.com/projects/view/2324/temp-amp-hum-3-click) 
page, as well as a demo application (example), developed using MikroElektronika 
[compilers](http://shop.mikroe.com/compilers). The demo can run on all the main 
MikroElektronika [development boards](http://shop.mikroe.com/development-boards).

**Library Description**

Library covers all functionalities of the click board.

Key functions :

- ``` float temphum3_getTemperature() ``` - Reads temperature from sensor
- ``` void temphum3_setMeasurement(uint8_t value) ``` - Setup measurement parameters
- ``` void temphum3_Configuration(uint8_t value) ``` - Setup sensor configuration

**Examples Description**

The application is composed of three sections :

- System Initialization - Initializes I2C module and CS pin as OUTPUT and INT pin as INPUT
- Application Initialization - Initializes Driver init and settings chip mode ACTIVE and configuration Measurement and Interrupt, 
  then settings maximum / minimum possible Temperature and Huminidy.
- Application Task - (code snippet) - Reads the temperature and huminidy and logs to the USBUART every 500 ms.


```.c
void applicationTask()
{
    Temperature = temphum3_getTemperature();
    Delay_100ms();

    Huminidy = temphum3_getHuminidy();
    Delay_100ms();

    FloatToStr(Temperature,temp_txt);
    mikrobus_logWrite("Temperature : ",_LOG_TEXT);
    mikrobus_logWrite(temp_txt,_LOG_LINE);
    
    FloatToStr(Huminidy,hum_txt);
    mikrobus_logWrite("Huminidy : ",_LOG_TEXT);
    mikrobus_logWrite(hum_txt,_LOG_LINE);
    
    Delay_ms( 500 );
}
```

The full application code, and ready to use projects can be found on our 
[LibStock](https://libstock.mikroe.com/projects/view/2324/temp-amp-hum-3-click) page.

mikroE Libraries used in the example:

- I2C
- UART
- Conversions

**Additional notes and informations**

Depending on the development board you are using, you may need 
[USB UART click](http://shop.mikroe.com/usb-uart-click), 
[USB UART 2 Click](http://shop.mikroe.com/usb-uart-2-click) or 
[RS232 Click](http://shop.mikroe.com/rs232-click) to connect to your PC, for 
development systems with no UART to USB interface available on the board. The 
terminal available in all Mikroelektronika 
[compilers](http://shop.mikroe.com/compilers), or any other terminal application 
of your choice, can be used to read the message.

---
### Architectures Supported

#### mikroC

| STM | KIN | CEC | MSP | TIVA | PIC | PIC32 | DSPIC | AVR | FT90x |
|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| x | x | x | x | x | x | x | x | x | x |

#### mikroBasic

| STM | KIN | CEC | MSP | TIVA | PIC | PIC32 | DSPIC | AVR | FT90x |
|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| x | x | x | x | x | x | x | x | x | x |

#### mikroPascal

| STM | KIN | CEC | MSP | TIVA | PIC | PIC32 | DSPIC | AVR | FT90x |
|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| x | x | x | x | x | x | x | x | x | x |

---
---
