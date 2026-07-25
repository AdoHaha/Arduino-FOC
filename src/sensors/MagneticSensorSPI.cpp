
#include "MagneticSensorSPI.h"

/** Typical configuration for the 14bit AMS AS5147 magnetic sensor over SPI interface */
MagneticSensorSPIConfig_s AS5147_SPI = {
  .spi_mode = SPI_MODE1,
  .clock_speed = 1000000,
  .bit_resolution = 14,
  .angle_register = 0x3FFF,
  .data_start_bit = 13,
  .command_rw_bit = 14,
  .command_parity_bit = 15,
  .response_has_even_parity = true,
  .response_error_bit = 14,
  .response_error_clear_register = 0x0001
};
// AS5048 and AS5047 are the same as AS5147
MagneticSensorSPIConfig_s AS5048_SPI = AS5147_SPI;
MagneticSensorSPIConfig_s AS5047_SPI = AS5147_SPI;

/** Typical configuration for the 14bit MonolithicPower MA730 magnetic sensor over SPI interface */
MagneticSensorSPIConfig_s MA730_SPI = {
  .spi_mode = SPI_MODE0,
  .clock_speed = 1000000,
  .bit_resolution = 14,
  .angle_register = 0x0000,
  .data_start_bit = 15,
  .command_rw_bit = 0,  // not required
  .command_parity_bit = 0, // parity not implemented
  .response_has_even_parity = false,
  .response_error_bit = 0,
  .response_error_clear_register = 0
};


// MagneticSensorSPI(int cs, float _bit_resolution, int _angle_register)
//  cs              - SPI chip select pin
//  _bit_resolution   sensor resolution bit number
// _angle_register  - (optional) angle read register - default 0x3FFF
MagneticSensorSPI::MagneticSensorSPI(int cs, int _bit_resolution, int _angle_register){

  chip_select_pin = cs;
  // angle read register of the magnetic sensor
  angle_register = _angle_register ? _angle_register : DEF_ANGLE_REGISTER;
  // register maximum value (counts per revolution)
  cpr = _powtwo(_bit_resolution);
  spi_mode = SPI_MODE1;
  clock_speed = 1000000;
  bit_resolution = _bit_resolution;

  command_parity_bit = 15; // for backwards compatibilty
  command_rw_bit = 14; // for backwards compatibilty
  data_start_bit = 13; // for backwards compatibilty
  // Preserve the legacy generic constructor's behavior. Response validation
  // is enabled only by configurations which explicitly describe its bits.
  response_has_even_parity = false;
  response_error_bit = 0;
  response_error_clear_register = 0;
}

MagneticSensorSPI::MagneticSensorSPI(MagneticSensorSPIConfig_s config, int cs){
  chip_select_pin = cs;
  // angle read register of the magnetic sensor
  angle_register = config.angle_register ? config.angle_register : DEF_ANGLE_REGISTER;
  // register maximum value (counts per revolution)
  cpr = _powtwo(config.bit_resolution);
  spi_mode = config.spi_mode;
  clock_speed = config.clock_speed;
  bit_resolution = config.bit_resolution;

  command_parity_bit = config.command_parity_bit; // for backwards compatibilty
  command_rw_bit = config.command_rw_bit; // for backwards compatibilty
  data_start_bit = config.data_start_bit; // for backwards compatibilty
  response_has_even_parity = config.response_has_even_parity;
  response_error_bit = config.response_error_bit;
  response_error_clear_register = config.response_error_clear_register;
}

void MagneticSensorSPI::init(SPIClass* _spi){
  spi = _spi;
	// 1MHz clock (AMS should be able to accept up to 10MHz)
	settings = SPISettings(clock_speed, MSBFIRST, spi_mode);
	//setup pins
	pinMode(chip_select_pin, OUTPUT);
	//SPI has an internal SPI-device counter, it is possible to call "begin()" from different devices
	spi->begin();
	// do any architectures need to set the clock divider for SPI? Why was this in the code?
  //spi->setClockDivider(SPI_CLOCK_DIV8);
  digitalWrite(chip_select_pin, HIGH);

  this->Sensor::init(); // call base class init
  tracking_initialized = hasValidData();
  if (!tracking_initialized) {
    // Sensor::init() performs direct getSensorAngle() calls. If every initial
    // response was invalid, do not leave its protected tracking state at -1.
    angle_prev = 0.0f;
    vel_angle_prev = 0.0f;
    full_rotations = 0;
    vel_full_rotations = 0;
    velocity = 0.0f;
    angle_prev_ts = _micros();
    vel_angle_prev_ts = angle_prev_ts;
  }
}

void MagneticSensorSPI::update() {
  if (!tracking_initialized) {
    const float val = getSensorAngle();
    if (val < 0.0f) {
      return;
    }
    // Seed both position and velocity history from the delayed first valid
    // sample so it cannot look like a wrap from the temporary zero state.
    angle_prev = val;
    vel_angle_prev = val;
    full_rotations = 0;
    vel_full_rotations = 0;
    velocity = 0.0f;
    angle_prev_ts = _micros();
    vel_angle_prev_ts = angle_prev_ts;
    tracking_initialized = true;
    return;
  }
  this->Sensor::update();
}

//  Shaft angle calculation
//  angle is in radians [rad]
float MagneticSensorSPI::getSensorAngle(){
  int raw_count = getRawCount();
  if (raw_count < 0) {
    return -1.0f;
  }
  return (raw_count / (float)cpr) * _2PI;
}

// function reading the raw counter of the magnetic sensor
int MagneticSensorSPI::getRawCount(){
	return (int)MagneticSensorSPI::read(angle_register);
}

bool MagneticSensorSPI::hasValidData() const {
  return last_valid_count >= 0;
}

void MagneticSensorSPI::clearErrorCounters() {
  parity_error_count = 0;
  sensor_error_count = 0;
  error_clear_count = 0;
  error_clear_parity_error_count = 0;
  successful_retry_count = 0;
  exhausted_retry_count = 0;
}

// SPI functions 
/**
 * Utility function used to calculate even parity of word
 */
byte MagneticSensorSPI::spiCalcEvenParity(word value){
	byte cnt = 0;
	byte i;

	for (i = 0; i < 16; i++)
	{
		if (value & 0x1) cnt++;
		value >>= 1;
	}
	return cnt & 0x1;
}

  /*
  * Read a register from the sensor
  * Takes the address of the register as a 16 bit word
  * Returns the value of the register
  */
int MagneticSensorSPI::read(word angle_register){

  const word command = buildReadCommand(angle_register);

  const bool validate_response =
      response_has_even_parity || response_error_bit > 0;
  const uint16_t attempts =
      validate_response ? (uint16_t)response_retries + 1u : 1u;
  const word data_mask = 0xFFFF >> (16 - bit_resolution);

  MagneticSensorSPIReadStatus failed_status = MAGNETIC_SENSOR_SPI_READ_OK;
  for (uint16_t attempt = 0; attempt < attempts; attempt++) {
    last_response = transferResponse(command);

    const MagneticSensorSPIReadStatus status =
        magneticSensorSPIValidateResponse(
            last_response, response_has_even_parity, response_error_bit);
    if (status == MAGNETIC_SENSOR_SPI_READ_OK) {
      word register_value =
          last_response >> (1 + data_start_bit - bit_resolution);
      last_valid_count = register_value & data_mask;
      last_read_status = MAGNETIC_SENSOR_SPI_READ_OK;
      if (attempt > 0) {
        successful_retry_count++;
      }
      return last_valid_count;
    }

    failed_status = (MagneticSensorSPIReadStatus)(
        (uint8_t)failed_status | (uint8_t)status);
    if (status == MAGNETIC_SENSOR_SPI_PARITY_ERROR) {
      parity_error_count++;
    } else if (status == MAGNETIC_SENSOR_SPI_SENSOR_ERROR) {
      sensor_error_count++;
      if (response_error_clear_register > 0) {
        // AMS error flags are latched. Reading ERRFL returns their detail and
        // clears the latch so a subsequent angle retry can recover.
        const word clear_response =
            transferResponse(buildReadCommand(response_error_clear_register));
        error_clear_count++;
        const MagneticSensorSPIReadStatus clear_status =
            magneticSensorSPIValidateResponse(
                clear_response, response_has_even_parity, 0);
        if (clear_status == MAGNETIC_SENSOR_SPI_PARITY_ERROR) {
          parity_error_count++;
          error_clear_parity_error_count++;
          failed_status = (MagneticSensorSPIReadStatus)(
              (uint8_t)failed_status | (uint8_t)clear_status);
        } else {
          const word clear_data =
              clear_response >> (1 + data_start_bit - bit_resolution);
          last_error_flags = clear_data & data_mask;
        }
      }
    }
  }

  last_read_status = failed_status;
  exhausted_retry_count++;

  // Sensor::update() also retains its last valid angle for a negative result.
  // Keeping the raw count here additionally protects direct getSensorAngle()
  // callers and Sensor::init(), which performs direct reads.
  return last_valid_count;
}

word MagneticSensorSPI::buildReadCommand(word register_address) {
  word command = register_address;
  if (command_rw_bit > 0) {
    command |= (word)(1u << command_rw_bit);
  }
  if (command_parity_bit > 0) {
    command |=
        ((word)spiCalcEvenParity(command) << command_parity_bit);
  }
  return command;
}

word MagneticSensorSPI::transferResponse(word command) {
  // SPI - begin transaction
  spi->beginTransaction(settings);

  // Send the command. AMS sensors return the requested register value in the
  // following transfer, so the response received here belongs to an earlier
  // command and is intentionally ignored.
  digitalWrite(chip_select_pin, LOW);
  spi->transfer16(command);
  digitalWrite(chip_select_pin,HIGH);

#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32) // if ESP32 board
  delayMicroseconds(50); // why do we need to delay 50us on ESP32? In my experience no extra delays are needed, on any of the architectures I've tested...
#else
  delayMicroseconds(1); // delay 1us, the minimum time possible in plain arduino. 350ns is the required time for AMS sensors, 80ns for MA730, MA702
#endif

  // Clock out the response to the command above. 0x0000 is a valid even-parity
  // NOP frame for the supported AMS sensors.
  digitalWrite(chip_select_pin, LOW);
  word register_value = spi->transfer16(0x0000);
  digitalWrite(chip_select_pin, HIGH);

  spi->endTransaction();
  return register_value;
}

/**
 * Closes the SPI connection
 * SPI has an internal SPI-device counter, for each init()-call the close() function must be called exactly 1 time
 */
void MagneticSensorSPI::close(){
	spi->end();
}
