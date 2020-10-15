/*	
*	Usage Example of TE MEAS Weather Station libraries with MCP2221 USB-UART/I2C bridge IC
*	This project compiles successfully with Visual Studio 2015
*/

#include <stdio.h>
#include <conio.h>
#include <Windows.h>
#include <iso646.h>
#include "mcp2221_dll_um.h"
#include "tsd305.h"
#include "htu21d.h"
#include "ms5637.h"
#include "tsys01.h"
#pragma comment(lib, "mcp2221_dll_um_x86.lib")

#define I2cAddr7bit 1
#define I2cAddr8bit 0

//Global variables
void *handle;
int i;

wchar_t SerNum = 0x0000075428;
wchar_t LibVer[6];
wchar_t MfrDescriptor[30];
wchar_t ProdDescrip[30];
int ver = 0;
int error = 0;
int flag = 0;
unsigned int delay = 0;
unsigned int ReqCurrent;
unsigned int PID = 0xDD;
unsigned int VID = 0x4D8;
unsigned int NumOfDev = 0;
unsigned char PowerAttrib;
unsigned char RxData[7] = { 0 };
unsigned char Addr = 0x00;
unsigned char DummyByte = 0x00;

//Functions prototypes
void ExitFunc();
void Mcp2221_config();

void ExitFunc()
{
	Mcp2221_I2cCancelCurrentTransfer(handle);
	Mcp2221_CloseAll();
	//_sleep(100);
}

void Mcp2221_config()
{
	ver = Mcp2221_GetLibraryVersion(LibVer);		//Get DLL version
	if (ver == 0)
	{
		printf("Library (DLL) version: %ls\n", LibVer);
	}
	else
	{
		error = Mcp2221_GetLastError();
		printf("Version can't be found, version: %d, error: %d\n", ver, error);
	}

	//Get number of connected devices with this VID & PID
	Mcp2221_GetConnectedDevices(VID, PID, &NumOfDev);
	if (NumOfDev == 0)
	{
		printf("No MCP2221 devices connected\n");
		//exit(0);
	}
	else
	{
		printf("Number of devices found: %d\n", NumOfDev);
	}

	//Open device by index
	handle = Mcp2221_OpenByIndex(VID, PID, NumOfDev - 1);
	if (error == NULL)
	{
		printf("Connection successful\n");
	}
	else
	{
		error = Mcp2221_GetLastError();
		printf("Error message is %s\n", error);
	}

	//Get manufacturer descriptor
	flag = Mcp2221_GetManufacturerDescriptor(handle, MfrDescriptor);
	if (flag == 0)
	{
		printf("Manufacturer descriptor: %ls\n", MfrDescriptor);
	}
	else
	{
		printf("Error getting descriptor: %d\n", flag);
	}

	//Get product descriptor
	flag = Mcp2221_GetProductDescriptor(handle, ProdDescrip);
	if (flag == 0)
	{
		printf("Product descriptor: %ls\n", ProdDescrip);
	}
	else
	{
		printf("Error getting product descriptor: %d\n", flag);
	}

	//Get power attributes
	flag = Mcp2221_GetUsbPowerAttributes(handle, &PowerAttrib, &ReqCurrent);
	if (flag == 0)
	{
		printf("Power Attributes, %x\nRequested current units = %d\nRequested current(mA) = %d\n", PowerAttrib, ReqCurrent, ReqCurrent * 2);
	}
	else
	{
		printf("Error getting power attributes: %d\n", flag);
	}

	//Set I2C bus
	flag = Mcp2221_SetSpeed(handle, 500000);    //set I2C speed to 400 KHz
	if (flag == 0)
	{
		printf("I2C is configured\n");
	}
	else
	{
		printf("Error setting I2C bus: %d\n", flag);
		Mcp2221_I2cCancelCurrentTransfer(handle);
	}

	//Set I2C advanced parameters
	flag = Mcp2221_SetAdvancedCommParams(handle, 1, 1000);  //1ms timeout, try 1000 times
	if (flag == 0)
	{
		printf("I2C advanced settings set\n");
	}
	else
	{
		printf("Error setting I2C advanced settings: %d\n", flag);
	}
}

int main(int argc, char *argv[])
{
	static ms5637 m_ms5637;
	tsd305 m_tsd305;
	htu21d m_htu21d;
	static tsys01 m_tsys01;

	bool connected;
	float ms5637_temperature = 0;
	float ms5637_pressure = 0;

	float tsd305_temperature = 0;
	float tsd305_object_temperature = 0;

	float htu21d_temperature;
	float htu21d_humidity;

	float tsys01_temperature = 0;

	htu21_battery_status battery_status_HTU21;
	ms5637_status status_MS5637;
	htu21_status status_HTU21;
	tsd305_status status_TSD305;

	//Register exit function
	atexit(ExitFunc);

	//Configure any connected MCP2221
	Mcp2221_config();
	m_ms5637.begin();
	m_tsd305.begin();
	m_htu21d.begin();
	m_tsys01.begin();

	if(m_ms5637.is_connected()) printf("MS5637 is connected.\n");
	if(m_tsd305.is_connected()) printf("TSD305 is connected.\n");
	if(m_htu21d.is_connected()) printf("HTU21D is connected.\n");
	if(m_tsys01.is_connected()) printf("TSYS01 is connected.\n");

	while (1)
	{
		status_MS5637 = m_ms5637.read_temperature_and_pressure(&ms5637_temperature, &ms5637_pressure);
		printf("\nMS5637: Temperature: %.2f C, Pressure: %.2f hPa\n", ms5637_temperature, ms5637_pressure);

		status_TSD305 = m_tsd305.read_temperature_and_object_temperature(&tsd305_temperature, &tsd305_object_temperature);
		printf("\nTSD305: Temperature: %.2f C, Object temperature: %.2f C\n", tsd305_temperature, tsd305_object_temperature);

		status_HTU21 = m_htu21d.read_temperature_and_relative_humidity(&htu21d_temperature, &htu21d_humidity);
		printf("\nHTU21D: Temperature: %.2f C, humidity: %.2f \%RH\n", htu21d_temperature, htu21d_humidity);

		m_tsys01.read_temperature(&tsys01_temperature);
		printf("\nTSYS01:: Temperature: %.2f C\n", tsys01_temperature);
		Sleep(2000);
	}
}
