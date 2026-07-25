#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <vector>

#include "../../src/sensors/MagneticSensorSPI.h"

SPIClass SPI;

unsigned long _micros() {
  static unsigned long now = 0;
  now += 100;
  return now;
}

static uint16_t withEvenParity(uint16_t lower_15_bits) {
  uint16_t response = lower_15_bits & 0x7FFFu;
  if (!magneticSensorSPIHasEvenParity(response)) {
    response |= 0x8000u;
  }
  return response;
}

static float rawToAngle(uint16_t raw) {
  return ((float)raw / 16384.0f) * 6.28318530718f;
}

int main() {
  MagneticSensorSPI sensor(AS5147_SPI, 10);
  const uint16_t initial_raw = 0x1234u;
  const uint16_t initial_response = withEvenParity(initial_raw);

  // Sensor::init() performs four reads. Each AMS read is a command transfer
  // followed by the transfer which returns its response.
  SPI.setResponses({
      0, initial_response, 0, initial_response,
      0, initial_response, 0, initial_response,
  });
  sensor.init(&SPI);
  assert(sensor.hasValidData());

  const uint16_t retry_raw = 0x2345u;
  const uint16_t retry_response = withEvenParity(retry_raw);
  const uint16_t bad_response = retry_response ^ 0x0020u;
  SPI.setResponses({0, bad_response, 0, retry_response});
  const float retried_angle = sensor.getSensorAngle();
  assert(fabsf(retried_angle - rawToAngle(retry_raw)) < 1e-6f);
  assert(SPI.transmitted.size() == 4);
  assert(sensor.parity_error_count == 1);
  assert(sensor.successful_retry_count == 1);
  assert(sensor.exhausted_retry_count == 0);
  assert(sensor.last_read_status == MAGNETIC_SENSOR_SPI_READ_OK);

  // After both attempts fail, direct callers receive the last validated
  // angle rather than a corrupt word or zero.
  SPI.setResponses({0, bad_response, 0, bad_response});
  const float retained_angle = sensor.getSensorAngle();
  assert(fabsf(retained_angle - retried_angle) < 1e-6f);
  assert(sensor.parity_error_count == 3);
  assert(sensor.successful_retry_count == 1);
  assert(sensor.exhausted_retry_count == 1);
  assert(sensor.last_read_status == MAGNETIC_SENSOR_SPI_PARITY_ERROR);

  // A parity-valid EF response is counted separately. Reading ERRFL clears
  // the AMS latch before the requested angle is retried.
  const uint16_t ef_response = withEvenParity(0x4000u | 0x0345u);
  const uint16_t error_flags = withEvenParity(0x0004u);
  const uint16_t recovered_raw = 0x3456u;
  const uint16_t recovered_response = withEvenParity(recovered_raw);
  SPI.setResponses({
      0, ef_response, 0, error_flags, 0, recovered_response,
  });
  assert(fabsf(sensor.getSensorAngle() - rawToAngle(recovered_raw)) < 1e-6f);
  assert(SPI.transmitted.size() == 6);
  assert(SPI.transmitted[2] == 0x4001u);
  assert(sensor.sensor_error_count == 1);
  assert(sensor.error_clear_count == 1);
  assert(sensor.error_clear_parity_error_count == 0);
  assert(sensor.last_error_flags == 0x0004u);
  assert(sensor.successful_retry_count == 2);
  assert(sensor.exhausted_retry_count == 1);
  assert(sensor.last_read_status == MAGNETIC_SENSOR_SPI_READ_OK);

  sensor.clearErrorCounters();
  assert(sensor.parity_error_count == 0);
  assert(sensor.sensor_error_count == 0);
  assert(sensor.error_clear_count == 0);
  assert(sensor.error_clear_parity_error_count == 0);
  assert(sensor.successful_retry_count == 0);
  assert(sensor.exhausted_retry_count == 0);
  assert(sensor.hasValidData());

  // A legacy seven-field aggregate remains source-compatible and leaves
  // response validation disabled for generic sensors.
  // A completely invalid cold start remains explicitly invalid and never
  // exposes the internal -1 sentinel through base Sensor state. The first
  // later valid update seeds tracking without inventing a full rotation.
  MagneticSensorSPI cold_sensor(AS5147_SPI, 12);
  const uint16_t cold_bad_response = initial_response ^ 0x0001u;
  std::vector<uint16_t> cold_start_responses;
  for (int i = 0; i < 8; i++) {
    cold_start_responses.push_back(0);
    cold_start_responses.push_back(cold_bad_response);
  }
  SPI.setResponses(cold_start_responses);
  cold_sensor.init(&SPI);
  assert(!cold_sensor.hasValidData());
  assert(cold_sensor.getMechanicalAngle() == 0.0f);
  assert(cold_sensor.getFullRotations() == 0);

  const uint16_t cold_recovered_raw = 0x3F00u;
  SPI.setResponses({0, withEvenParity(cold_recovered_raw)});
  cold_sensor.update();
  assert(cold_sensor.hasValidData());
  assert(fabsf(cold_sensor.getMechanicalAngle() -
               rawToAngle(cold_recovered_raw)) < 1e-6f);
  assert(cold_sensor.getFullRotations() == 0);

  MagneticSensorSPIConfig_s generic_config = {
      SPI_MODE0, 1000000, 14, 0, 15, 0, 0
  };
  MagneticSensorSPI generic_sensor(generic_config, 11);
  SPI.setResponses({
      0, 0xFFFFu, 0, 0xFFFFu, 0, 0xFFFFu, 0, 0xFFFFu,
  });
  generic_sensor.init(&SPI);
  assert(generic_sensor.hasValidData());
  assert(generic_sensor.parity_error_count == 0);
  assert(generic_sensor.sensor_error_count == 0);
  return 0;
}
