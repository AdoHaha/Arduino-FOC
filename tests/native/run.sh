#!/usr/bin/env sh
set -eu

response_test="${TMPDIR:-/tmp}/simplefoc_magnetic_sensor_spi_response_test"
retry_test="${TMPDIR:-/tmp}/simplefoc_magnetic_sensor_spi_retry_test"

"${CXX:-c++}" -std=c++11 -Wall -Wextra -Werror \
  tests/native/test_magnetic_sensor_spi_response.cpp \
  -o "$response_test"
"$response_test"

"${CXX:-c++}" -std=c++11 -Wall -Wextra -Werror \
  -Wno-overloaded-virtual -Wno-missing-field-initializers \
  -Itests/native/fakes \
  tests/native/test_magnetic_sensor_spi_retry.cpp \
  src/sensors/MagneticSensorSPI.cpp \
  src/common/base_classes/Sensor.cpp \
  -o "$retry_test"
"$retry_test"
