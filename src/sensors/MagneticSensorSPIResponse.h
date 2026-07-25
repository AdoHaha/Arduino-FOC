#ifndef MAGNETICSENSORSPIRESPONSE_H
#define MAGNETICSENSORSPIRESPONSE_H

#include <stdint.h>

/**
 * Integrity status of a 16-bit SPI sensor response.
 *
 * MagneticSensorSPI only enables these checks for configurations that
 * explicitly enable full-frame even parity and describe an error bit.
 */
enum MagneticSensorSPIReadStatus : uint8_t {
  MAGNETIC_SENSOR_SPI_READ_OK = 0x00,
  MAGNETIC_SENSOR_SPI_PARITY_ERROR = 0x01,
  MAGNETIC_SENSOR_SPI_SENSOR_ERROR = 0x02
};

/**
 * Return true when the complete 16-bit word has even parity.
 *
 * AMS AS5047/AS5048/AS5147 responses use bit 15 to make the complete response
 * word even. Checking the complete word also detects a flipped parity bit.
 */
inline bool magneticSensorSPIHasEvenParity(uint16_t value) {
  uint8_t odd = 0;
  while (value) {
    odd ^= (uint8_t)(value & 0x1u);
    value >>= 1;
  }
  return odd == 0;
}

/**
 * Validate a response before its non-data bits are stripped.
 *
 * The error flag is interpreted only after parity succeeds. If parity is bad,
 * every response bit (including the error flag) is untrusted.
 *
 * A false parity flag or an error-bit position of zero disables the
 * corresponding optional check. This preserves source compatibility for
 * existing seven-field aggregate MagneticSensorSPIConfig_s initializers,
 * whose new trailing fields are zero-initialized.
 */
inline MagneticSensorSPIReadStatus magneticSensorSPIValidateResponse(
    uint16_t response, bool validate_even_parity, int response_error_bit) {
  if (validate_even_parity && !magneticSensorSPIHasEvenParity(response)) {
    return MAGNETIC_SENSOR_SPI_PARITY_ERROR;
  }
  if (response_error_bit > 0 &&
      (response & (uint16_t)(1u << response_error_bit))) {
    return MAGNETIC_SENSOR_SPI_SENSOR_ERROR;
  }
  return MAGNETIC_SENSOR_SPI_READ_OK;
}

#endif
