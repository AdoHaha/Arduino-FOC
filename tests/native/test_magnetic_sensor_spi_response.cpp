#include <assert.h>
#include <stdint.h>

#include "../../src/sensors/MagneticSensorSPIResponse.h"

static uint16_t withEvenParity(uint16_t lower_15_bits) {
  uint16_t response = lower_15_bits & 0x7FFFu;
  if (!magneticSensorSPIHasEvenParity(response)) {
    response |= 0x8000u;
  }
  return response;
}

int main() {
  const uint16_t valid_angle = withEvenParity(0x1234u);
  assert(magneticSensorSPIHasEvenParity(valid_angle));
  assert(magneticSensorSPIValidateResponse(valid_angle, 15, 14) ==
         MAGNETIC_SENSOR_SPI_READ_OK);

  assert(magneticSensorSPIValidateResponse(valid_angle ^ 0x0020u, 15, 14) ==
         MAGNETIC_SENSOR_SPI_PARITY_ERROR);
  assert(magneticSensorSPIValidateResponse(valid_angle ^ 0x8000u, 15, 14) ==
         MAGNETIC_SENSOR_SPI_PARITY_ERROR);

  const uint16_t sensor_error = withEvenParity(0x4000u | 0x0234u);
  assert(magneticSensorSPIValidateResponse(sensor_error, 15, 14) ==
         MAGNETIC_SENSOR_SPI_SENSOR_ERROR);

  // An error-flag bit in a word with bad parity is untrusted and is classified
  // as a link-integrity failure rather than a sensor-reported command error.
  assert(magneticSensorSPIValidateResponse(sensor_error ^ 0x0001u, 15, 14) ==
         MAGNETIC_SENSOR_SPI_PARITY_ERROR);

  // Existing generic configurations have the new trailing fields
  // zero-initialized, so neither validation rule is enabled.
  assert(magneticSensorSPIValidateResponse(0xFFFFu, 0, 0) ==
         MAGNETIC_SENSOR_SPI_READ_OK);
  return 0;
}
