#ifndef MAGNETICSENSORSPI_LIB_H
#define MAGNETICSENSORSPI_LIB_H


#include "Arduino.h"
#include <SPI.h>
#include "../common/base_classes/Sensor.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"
#include "MagneticSensorSPIResponse.h"

#define DEF_ANGLE_REGISTER 0x3FFF

struct MagneticSensorSPIConfig_s  {
  int spi_mode;
  long clock_speed;
  int bit_resolution;
  int angle_register;
  int data_start_bit;
  int command_rw_bit;
  int command_parity_bit;
  bool response_has_even_parity;
  int response_error_bit;
  int response_error_clear_register;
};
// typical configuration structures
extern MagneticSensorSPIConfig_s AS5147_SPI,AS5048_SPI,AS5047_SPI, MA730_SPI;

class MagneticSensorSPI: public Sensor{
 public:
    /**
     *  MagneticSensorSPI class constructor
     * @param cs  SPI chip select pin 
     * @param bit_resolution   sensor resolution bit number
     * @param angle_register  (optional) angle read register - default 0x3FFF
     */
    MagneticSensorSPI(int cs, int bit_resolution, int angle_register = 0);
    /**
     *  MagneticSensorSPI class constructor
     * @param config   SPI config
     * @param cs  SPI chip select pin
     */
    MagneticSensorSPI(MagneticSensorSPIConfig_s config, int cs);

    /** sensor initialise pins */
    void init(SPIClass* _spi = &SPI);

    // implementation of abstract functions of the Sensor class
    /** update tracking, safely seeding it after a delayed first valid read */
    void update() override;
    /** get current angle (rad) */
    float getSensorAngle() override;

    /**
     * Number of complete command/response retries after an invalid response.
     *
     * Retrying the complete transaction is required by pipelined AMS sensors:
     * a register response is returned during the transfer following its
     * command. A value of zero disables retries but not validation.
     */
    uint8_t response_retries = 1;

    /** Raw response word from the most recent transfer. */
    uint16_t last_response = 0;
    /**
     * Status of the most recent logical read. A non-zero value after the read
     * means all attempts failed and the last valid angle was retained.
     */
    MagneticSensorSPIReadStatus last_read_status =
        MAGNETIC_SENSOR_SPI_READ_OK;

    /** Invalid response-frame counters. */
    uint32_t parity_error_count = 0;
    uint32_t sensor_error_count = 0;
    /** Automatic reads of the AMS ERRFL register and corrupt clear replies. */
    uint32_t error_clear_count = 0;
    uint32_t error_clear_parity_error_count = 0;
    /** Lower data bits returned by the latest parity-valid ERRFL read. */
    uint16_t last_error_flags = 0;
    /** Logical reads recovered by a retry. */
    uint32_t successful_retry_count = 0;
    /** Logical reads for which every attempt failed validation. */
    uint32_t exhausted_retry_count = 0;

    /** True after at least one validated angle has been received. */
    bool hasValidData() const;
    /** Reset diagnostic counters without changing the retained angle. */
    void clearErrorCounters();

    // returns the spi mode (phase/polarity of read/writes) i.e one of SPI_MODE0 | SPI_MODE1 | SPI_MODE2 | SPI_MODE3
    int spi_mode;
    
    /* returns the speed of the SPI clock signal */
    long clock_speed;
    

  private:
    float cpr; //!< Maximum range of the magnetic sensor
    // spi variables
    int angle_register; //!< SPI angle register to read
    int chip_select_pin; //!< SPI chip select pin
	  SPISettings settings; //!< SPI settings variable
    // spi functions
    /** Stop SPI communication */
    void close(); 
    /** Read one SPI register value, returning -1 before the first valid read. */
    int read(word angle_register);
    /** Execute one complete pipelined command/response transaction. */
    word transferResponse(word command);
    /** Build a read command including its outgoing parity bit. */
    word buildReadCommand(word register_address);
    /** Calculate parity value  */
    byte spiCalcEvenParity(word value);

    /**
     * Function getting current angle register value
     * it uses angle_register variable
     */
    int getRawCount();
    
    int bit_resolution; //!< the number of bites of angle data
    int command_parity_bit; //!< the bit where parity flag is stored in command
    int command_rw_bit; //!< the bit where read/write flag is stored in command
    int data_start_bit; //!< the the position of first bit
    bool response_has_even_parity; //!< validate even parity over all 16 bits
    int response_error_bit; //!< optional response error bit (zero disables)
    int response_error_clear_register; //!< register read to clear latched error
    int last_valid_count = -1; //!< validated fallback used after exhausted retries
    bool tracking_initialized = false; //!< base Sensor state has a valid seed

    SPIClass* spi;
};


#endif
