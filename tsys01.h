/*#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif*/

#include <stdio.h>
#include <stdint.h>

#define TSYS01_ADDR_CSB_1 0x76 // 0b1110110
#define TSYS01_ADDR_CSB_0 0x77 // 0b1110111

#define PROM_ELEMENTS_NUMBER 8

#define COEFF_MUL_0 (float)(-1.5)
#define COEFF_MUL_1 (float)(1)
#define COEFF_MUL_2 (float)(-2)
#define COEFF_MUL_3 (float)(4)
#define COEFF_MUL_4 (float)(-2)

// enum
enum tsys01_status_code {
  tsys01_STATUS_OK = 0x00,
  tsys01_STATUS_ERR_OVERFLOW = 0x01,
  tsys01_STATUS_ERR_TIMEOUT = 0x02,
};
enum tsys01_address { tsys01_i2c_address_csb_1, tsys01_i2c_address_csb_0 };

enum tsys01_status {
  tsys01_status_ok,
  tsys01_status_no_i2c_acknowledge,
  tsys01_status_i2c_transfer_error,
  tsys01_status_crc_error
};

class tsys01 {

public:
  tsys01();

  /**
   * \brief Perform initial configuration. Has to be called once.
   */
  void begin();

  /**
   * \brief Check whether TSYS01 device is connected
   *
   * \return bool : status of TSYS01
   *       - true : Device is present
   *       - false : Device is not acknowledging I2C address
   */
  bool is_connected(void);

  /**
   * \brief Configures TSYS01 I2C address to be used depending on HW
   * configuration
   *
   * \param[in] address : TSYS01 I2C address
   *
   */
  void set_address(enum tsys01_address address);

  /**
   * \brief Reset the TSYS01 device
   *
   * \return tsys01_status : status of TSYS01
   *       - tsys01_status_ok : I2C transfer completed successfully
   *       - tsys01_status_i2c_transfer_error : Problem with i2c transfer
   *       - tsys01_status_no_i2c_acknowledge : I2C did not acknowledge
   */
  enum tsys01_status reset(void);

  /**
   * \brief Reads the temperature ADC value
   *
   * \param[out] uint32_t* : Temperature ADC value.
   *
   * \return tsys01_status : status of TSYS01
   *       - tsys01_status_ok : I2C transfer completed successfully
   *       - tsys01_status_i2c_transfer_error : Problem with i2c transfer
   *       - tsys01_status_no_i2c_acknowledge : I2C did not acknowledge
   *       - tsys01_status_crc_error : CRC error on PROM coefficients
   */
  enum tsys01_status read_temperature(float *temperature);

private:
  /**
   * \brief Reads the tsys01 EEPROM coefficient stored at address provided.
   *
   * \param[in] uint8_t : Address of coefficient in EEPROM
   * \param[out] uint16_t* : Value read in EEPROM
   *
   * \return tsys01_status : status of TSYS01
   *       - tsys01_status_ok : I2C transfer completed successfully
   *       - tsys01_status_i2c_transfer_error : Problem with i2c transfer
   *       - tsys01_status_no_i2c_acknowledge : I2C did not acknowledge
   */
  enum tsys01_status read_eeprom_coeff(uint8_t command, uint16_t *coeff);

  /**
   * \brief Reads the tsys01 EEPROM coefficient stored at address provided.
   *
   * \param[in] uint8_t : Address of coefficient in EEPROM
   * \param[out] uint16_t* : Value read in EEPROM
   *
   * \return tsys01_status : status of TSYS01
   *       - tsys01_status_ok : I2C transfer completed successfully
   *       - tsys01_status_i2c_transfer_error : Problem with i2c transfer
   *       - tsys01_status_no_i2c_acknowledge : I2C did not acknowledge
   */
  enum tsys01_status read_eeprom(void);

  /**
   * \brief CRC check
   *
   * \param[in] uint16_t *: List of EEPROM coefficients
   *
   * \return bool : TRUE if CRC is OK, FALSE if KO
   */
  bool crc_check(uint16_t *n_prom);

  /**
   * \brief Reads the temperature ADC value and compute the degree Celsius one.
   *
   * Perform algorithm computation based on adc 16 bits ( 24-bits value is
   * divided by 256)<BR>
   * \verbatim
   *  T (degC) =     (-2) * k4 * 10e-21 * adc^4 +
   *                    4 * k3 * 10e-16 * adc^3 +
   *                 (-2) * k2 * 10e-11 * adc^2 +
   *                    1 * k1 * 10e-6  * adc +
   *               (-1.5) * k0 * 10e-2
   * \endverbatim
   *
   * Factored into
   * \verbatim
   *  T (degC) = 10e-2.( a.k0 + 10e1 * 10e-5.adc.( b.k1 + 10e-5.adc ( c.k2 +
   * 10e-5.adc.( d.k3 + 10e-5.adc.e.k4 ) ) ) )
   * \endverbatim
   *
   * \param[out] float* : Celsius Degree temperature value
   *
   * \return tsys01_status : status of TSYS01
   *       - tsys01_status_ok : I2C transfer completed successfully
   *       - tsys01_status_i2c_transfer_error : Problem with i2c transfer
   *       - tsys01_status_no_i2c_acknowledge : I2C did not acknowledge
   *       - tsys01_status_crc_error : CRC error on PROM coefficients
   */
  enum tsys01_status conversion_and_read_adc(uint32_t *adc);

  /**
* \brief Writes the TSYS01 8-bits command with the value passed
*
* \param[in] uint8_t : Command value to be written.
*
* \return tsys01_status : status of TSYS01
*       - tsys01_status_ok : I2C transfer completed successfully
*       - tsys01_status_i2c_transfer_error : Problem with i2c transfer
*       - tsys01_status_no_i2c_acknowledge : I2C did not acknowledge
*/
  enum tsys01_status write_command(uint8_t cmd);

  uint8_t tsys01_i2c_address = TSYS01_ADDR_CSB_0;

  bool tsys01_crc_check(uint16_t *n_prom);
  const float coeff_mul[5];
  uint16_t eeprom_coeff[PROM_ELEMENTS_NUMBER];

  bool tsys01_coeff_read;
};
