# MCP2221 Library for TE MEAS Weather Station Raspberry Pi Hat
These are a collection of C++ libraries for interfacing the TE MEAS Weather Station Raspberry Pi hat with MCP2221 USB to UART/Ia2C bridge chip. The libraries are ported from Arduin libraries developed by TE, the original libraries can be found here: https://github.com/TEConnectivity

**About the porting process:**
The porting efforts focused on replacing Ardunio I2C APIs with MCP2221 ones, with minimum changes to variables and constants to accommodate the Microchip MCP2221 library APIs input/output arguments. Another reason these libraries are almost identical to the original TE Arduino libraries is to ease usage and integration for developers who have already Arduino experience. 

**Tools and other requirements:**
The project was developed and compiled with Visual Studio 2015.

![MCP2221_MEAS](https://user-images.githubusercontent.com/8460504/96188790-2cb80a80-0ef4-11eb-977c-e719eeb9de13.jpg)

The four sensors aboard the MEAS weather station Pi hat are:
- TSD305-1C55 contactless IR temperature and ambient temperature sensor.
- HTU21D temperature and humidity sensor.
- MS5637 pressure and temperature sensor.
- TSYS01 temperature sensor.

For more information please visit the TE MEAS weather station Pi hat documentation and datasheets here: https://www.te.com/global-en/product-CAT-DCS0036.html
