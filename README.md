# MCP2221 Library for TE MEAS Weather Station Raspberry Pi Hat
These are a collection of C++ libraries for interfacing the TE MEAS Weather Station Raspberry Pi hat with MCP2221 USB-to-I2C/UART bridge IC. All libraries listed here are ported from the Arduino libraries developed by TE. The original libraries can be found here: https://github.com/TEConnectivity

**About the porting process:**

The porting efforts focused on replacing Ardunio I2C APIs with MCP2221 ones, with minimum changes to variables and constants to accommodate the Microchip MCP2221 library APIs input/output arguments. The reason to keep the ported libraries almost identical to the original TE Arduino libraries is to ease usage and integration for developers who are new to the MCP2221 and/or already have Arduino experience. 

**Tools and other requirements:**

The project was developed and compiled with Visual Studio 2015.

**Hardware:**

![MCP2221_MEAS](https://user-images.githubusercontent.com/8460504/96188790-2cb80a80-0ef4-11eb-977c-e719eeb9de13.jpg)

**Output:**


<img width="674" alt="mcp2221_meas_screenshot" src="https://user-images.githubusercontent.com/8460504/96192810-b0292a00-0efb-11eb-99f6-dc7b5b470bcd.png">

The four sensors aboard the MEAS weather station Pi hat are:
- TSD305-1C55 contactless and ambient temperature sensor.
- HTU21D temperature and humidity sensor.
- MS5637 pressure and temperature sensor.
- TSYS01 temperature sensor.

For MCP2221 wiring guide, please visit this tutorial: http://elrayescampaign.blogspot.com/2018/06/mcp2221-i2c-interfacing-tutorial.html

For more information please visit the TE MEAS weather station Pi hat documentation and datasheets here: https://www.te.com/global-en/product-CAT-DCS0036.html
