#include <stdint.h>
#include <stdio.h>
#include <Windows.h>
#include "ms5637.h"
#include "mcp2221_dll_um.h"
#include "GetErrName.h"

#define I2cAddr7bit 1
#define I2cAddr8bit 0

// Constants

// MS5637 device address
#define MS5637_ADDR 0x76 // 0b1110110

// MS5637 device commands
#define MS5637_RESET_COMMAND 0x1E
#define MS5637_START_PRESSURE_ADC_CONVERSION 0x40
#define MS5637_START_TEMPERATURE_ADC_CONVERSION 0x50
#define MS5637_READ_ADC 0x00

#define MS5637_CONVERSION_OSR_MASK 0x0F

// MS5637 commands
#define MS5637_PROM_ADDRESS_READ_ADDRESS_0 0xA0
#define MS5637_PROM_ADDRESS_READ_ADDRESS_1 0xA2
#define MS5637_PROM_ADDRESS_READ_ADDRESS_2 0xA4
#define MS5637_PROM_ADDRESS_READ_ADDRESS_3 0xA6
#define MS5637_PROM_ADDRESS_READ_ADDRESS_4 0xA8
#define MS5637_PROM_ADDRESS_READ_ADDRESS_5 0xAA
#define MS5637_PROM_ADDRESS_READ_ADDRESS_6 0xAC
#define MS5637_PROM_ADDRESS_READ_ADDRESS_7 0xAE

// Coefficients indexes for temperature and pressure computation
#define MS5637_CRC_INDEX 0
#define MS5637_PRESSURE_SENSITIVITY_INDEX 1
#define MS5637_PRESSURE_OFFSET_INDEX 2
#define MS5637_TEMP_COEFF_OF_PRESSURE_SENSITIVITY_INDEX 3
#define MS5637_TEMP_COEFF_OF_PRESSURE_OFFSET_INDEX 4
#define MS5637_REFERENCE_TEMPERATURE_INDEX 5
#define MS5637_TEMP_COEFF_OF_TEMPERATURE_INDEX 6

//variables
extern int flag;
extern void *handle;
extern unsigned char DummyByte;

/**
* \brief Class constructor
*
*/
ms5637::ms5637(void) {}

/**
 * \brief Perform initial configuration. Has to be called once.
 */
void ms5637::begin(void) {
}

/**
* \brief Check whether MS5637 device is connected
*
* \return bool : status of MS5637
*       - true : Device is present
*       - false : Device is not acknowledging I2C address
*/
bool ms5637::is_connected(void) 
{ 
	bool conn_flag;
	flag = Mcp2221_I2cWrite(handle, sizeof(DummyByte), MS5637_ADDR, I2cAddr7bit, &DummyByte);    //issue start condition then address
	if (flag == 0) conn_flag = true;
	else conn_flag = false;
	return conn_flag;
}

/**
* \brief Writes the MS5637 8-bits command with the value passed
*
* \param[in] uint8_t : Command value to be written.
*
* \return ms5637_status : status of MS5637
*       - ms5637_status_ok : I2C transfer completed successfully
*       - ms5637_status_i2c_transfer_error : Problem with i2c transfer
*       - ms5637_status_no_i2c_acknowledge : I2C did not acknowledge
*/
enum ms5637_status ms5637::write_command(uint8_t cmd) {
  uint8_t i2c_status;
  
  flag = Mcp2221_I2cWrite(handle, 1, MS5637_ADDR, I2cAddr7bit, &cmd);
  if(flag != 0) printf("Error writing cmd to MS5637 (1): %s\n", Mcp2221_GetErrorName(flag));

  return ms5637_status_ok;
}

/**
* \brief Set  ADC resolution.
*
* \param[in] ms5637_resolution_osr : Resolution requested
*
*/
void ms5637::set_resolution(enum ms5637_resolution_osr res) {
  ms5637_resolution_osr = res;
}

/**
* \brief Reset the MS5637 device
*
* \return ms5637_status : status of MS5637
*       - ms5637_status_ok : I2C transfer completed successfully
*       - ms5637_status_i2c_transfer_error : Problem with i2c transfer
*       - ms5637_status_no_i2c_acknowledge : I2C did not acknowledge
*/
enum ms5637_status ms5637::reset(void) {
  return write_command(MS5637_RESET_COMMAND);
}

/**
* \brief Reads the ms5637 EEPROM coefficient stored at address provided.
*
* \param[in] uint8_t : Address of coefficient in EEPROM
* \param[out] uint16_t* : Value read in EEPROM
*
* \return ms5637_status : status of MS5637
*       - ms5637_status_ok : I2C transfer completed successfully
*       - ms5637_status_i2c_transfer_error : Problem with i2c transfer
*       - ms5637_status_no_i2c_acknowledge : I2C did not acknowledge
*       - ms5637_status_crc_error : CRC check error on the coefficients
*/
enum ms5637_status ms5637::read_eeprom_coeff(uint8_t command, uint16_t *coeff) {
  uint8_t buffer[2];
  uint8_t i;
  uint8_t i2c_status;

  buffer[0] = 0;
  buffer[1] = 0;

  /* Read data */
  flag = Mcp2221_I2cWrite(handle, 1, MS5637_ADDR, I2cAddr7bit, &command);
  if(flag != 0) printf("Error writing command to MS5637 (2): %s\n", Mcp2221_GetErrorName(flag));
  
  flag = Mcp2221_I2cRead(handle, 2, MS5637_ADDR, I2cAddr7bit, buffer);
  if(flag != 0) printf("Error reading from MS5637 (3): %d\n", Mcp2221_GetErrorName(flag));

  *coeff = (buffer[0] << 8) | buffer[1];

  return ms5637_status_ok;
}

/**
* \brief Reads the ms5637 EEPROM coefficients to store them for computation.
*
* \return ms5637_status : status of MS5637
*       - ms5637_status_ok : I2C transfer completed successfully
*       - ms5637_status_i2c_transfer_error : Problem with i2c transfer
*       - ms5637_status_no_i2c_acknowledge : I2C did not acknowledge
*       - ms5637_status_crc_error : CRC check error on the coefficients
*/
enum ms5637_status ms5637::read_eeprom(void) {
  enum ms5637_status status;
  uint8_t i;

  for (i = 0; i < MS5637_COEFFICIENT_COUNT; i++) {
    status = read_eeprom_coeff(MS5637_PROM_ADDRESS_READ_ADDRESS_0 + i * 2,
                               eeprom_coeff + i);
    if (status != ms5637_status_ok)
      return status;
  }
  if (!crc_check(eeprom_coeff, (eeprom_coeff[MS5637_CRC_INDEX] & 0xF000) >> 12))
    return ms5637_status_crc_error;

  coeff_read = true;

  return ms5637_status_ok;
}

/**
* \brief CRC check
*
* \param[in] uint16_t *: List of EEPROM coefficients
* \param[in] uint8_t : crc to compare with
*
* \return bool : TRUE if CRC is OK, FALSE if KO
*/
bool ms5637::crc_check(uint16_t *n_prom, uint8_t crc) {
  uint8_t cnt, n_bit;
  uint16_t n_rem, crc_read;

  n_rem = 0x00;
  crc_read = n_prom[0];
  n_prom[MS5637_COEFFICIENT_COUNT] = 0;
  n_prom[0] = (0x0FFF & (n_prom[0])); // Clear the CRC byte

  for (cnt = 0; cnt < (MS5637_COEFFICIENT_COUNT + 1) * 2; cnt++) {

    // Get next byte
    if (cnt % 2 == 1)
      n_rem ^= n_prom[cnt >> 1] & 0x00FF;
    else
      n_rem ^= n_prom[cnt >> 1] >> 8;

    for (n_bit = 8; n_bit > 0; n_bit--) {

      if (n_rem & 0x8000)
        n_rem = (n_rem << 1) ^ 0x3000;
      else
        n_rem <<= 1;
    }
  }
  n_rem >>= 12;
  n_prom[0] = crc_read;

  return (n_rem == crc);
}

/**
* \brief Triggers conversion and read ADC value
*
* \param[in] uint8_t : Command used for conversion (will determine Temperature
* vs Pressure and osr)
* \param[out] uint32_t* : ADC value.
*
* \return ms5637_status : status of MS5637
*       - ms5637_status_ok : I2C transfer completed successfully
*       - ms5637_status_i2c_transfer_error : Problem with i2c transfer
*       - ms5637_status_no_i2c_acknowledge : I2C did not acknowledge
*/
enum ms5637_status ms5637::conversion_and_read_adc(uint8_t cmd, uint32_t *adc) {
  enum ms5637_status status = ms5637_status_ok;
  uint8_t i2c_status;
  uint8_t buffer[3];
  uint8_t i;
 
  /* Read data */
  flag = Mcp2221_I2cWrite(handle, 1, MS5637_ADDR, I2cAddr7bit, &cmd); 
  if(flag != 0) printf("Error writing cmd to MS5637 (4): %s\n", Mcp2221_GetErrorName(flag));
  
  Sleep(conversion_time[(cmd & MS5637_CONVERSION_OSR_MASK) / 2]);
  
  uint8_t tCmd = 0x00;
  flag = Mcp2221_I2cWrite(handle, 1, MS5637_ADDR, I2cAddr7bit, &tCmd);
  if(flag != 0) printf("Error writing cmd to MS5637 (5): %s\n", Mcp2221_GetErrorName(flag));

  flag = Mcp2221_I2cRead(handle, 3, MS5637_ADDR, I2cAddr7bit, buffer);
  if(flag != 0) printf("Error reading from MS5637 (6): %s\n", Mcp2221_GetErrorName(flag));
  
  // delay conversion depending on resolution
  if (status != ms5637_status_ok)
    return status;

  // Send the read command
  if (status != ms5637_status_ok)
    return status;

  *adc = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];

  return status;
}

/**
* \brief Reads the temperature and pressure ADC value and compute the
* compensated values.
*
* \param[out] float* : Celsius Degree temperature value
* \param[out] float* : mbar pressure value
*
* \return ms5637_status : status of MS5637
*       - ms5637_status_ok : I2C transfer completed successfully
*       - ms5637_status_i2c_transfer_error : Problem with i2c transfer
*       - ms5637_status_no_i2c_acknowledge : I2C did not acknowledge
*       - ms5637_status_crc_error : CRC check error on the coefficients
*/
enum ms5637_status ms5637::read_temperature_and_pressure(float *temperature,
                                                         float *pressure) {
  enum ms5637_status status = ms5637_status_ok;
  uint32_t adc_temperature, adc_pressure;
  int32_t dT, TEMP;
  int64_t OFF, SENS, P, T2, OFF2, SENS2;
  uint8_t cmd;

  // If first time adc is requested, get EEPROM coefficients
  if (coeff_read == false)
    status = read_eeprom();

  if (status != ms5637_status_ok)
    return status;

  // First read temperature
  cmd = ms5637_resolution_osr * 2;
  cmd |= MS5637_START_TEMPERATURE_ADC_CONVERSION;
  status = conversion_and_read_adc(cmd, &adc_temperature);
  if (status != ms5637_status_ok)
    return status;

  // Now read pressure
  cmd = ms5637_resolution_osr * 2;
  cmd |= MS5637_START_PRESSURE_ADC_CONVERSION;
  status = conversion_and_read_adc(cmd, &adc_pressure);
  if (status != ms5637_status_ok)
    return status;

  if (adc_temperature == 0 || adc_pressure == 0)
    return ms5637_status_i2c_transfer_error;

  // Difference between actual and reference temperature = D2 - Tref
  dT = (int32_t)adc_temperature -
       ((int32_t)eeprom_coeff[MS5637_REFERENCE_TEMPERATURE_INDEX] << 8);

  // Actual temperature = 2000 + dT * TEMPSENS
  TEMP = 2000 +
         ((int64_t)dT *
              (int64_t)eeprom_coeff[MS5637_TEMP_COEFF_OF_TEMPERATURE_INDEX] >>
          23);

  // Second order temperature compensation
  if (TEMP < 2000) {
    T2 = (3 * ((int64_t)dT * (int64_t)dT)) >> 33;
    OFF2 = 61 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) / 16;
    SENS2 = 29 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) / 16;

    if (TEMP < -1500) {
      OFF2 += 17 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500);
      SENS2 += 9 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500);
    }
  } else {
    T2 = (5 * ((int64_t)dT * (int64_t)dT)) >> 38;
    OFF2 = 0;
    SENS2 = 0;
  }

  // OFF = OFF_T1 + TCO * dT
  OFF = ((int64_t)(eeprom_coeff[MS5637_PRESSURE_OFFSET_INDEX]) << 17) +
        (((int64_t)(eeprom_coeff[MS5637_TEMP_COEFF_OF_PRESSURE_OFFSET_INDEX]) *
          dT) >>
         6);
  OFF -= OFF2;

  // Sensitivity at actual temperature = SENS_T1 + TCS * dT
  SENS =
      ((int64_t)eeprom_coeff[MS5637_PRESSURE_SENSITIVITY_INDEX] << 16) +
      (((int64_t)eeprom_coeff[MS5637_TEMP_COEFF_OF_PRESSURE_SENSITIVITY_INDEX] *
        dT) >>
       7);
  SENS -= SENS2;

  // Temperature compensated pressure = D1 * SENS - OFF
  P = (((adc_pressure * SENS) >> 21) - OFF) >> 15;

  *temperature = ((float)TEMP - T2) / 100;
  *pressure = (float)P / 100;

  return status;
}
