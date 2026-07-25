#ifndef SIMPLEFOC_NATIVE_TEST_SPI_H
#define SIMPLEFOC_NATIVE_TEST_SPI_H

#include <stdint.h>
#include <vector>

class SPISettings {
 public:
  SPISettings() {}
  SPISettings(long, int, int) {}
};

class SPIClass {
 public:
  void begin() {}
  void end() {}
  void beginTransaction(const SPISettings&) {}
  void endTransaction() {}

  uint16_t transfer16(uint16_t outgoing) {
    transmitted.push_back(outgoing);
    if (next_response >= responses.size()) {
      return 0;
    }
    return responses[next_response++];
  }

  void setResponses(const std::vector<uint16_t>& values) {
    responses = values;
    transmitted.clear();
    next_response = 0;
  }

  std::vector<uint16_t> responses;
  std::vector<uint16_t> transmitted;
  size_t next_response = 0;
};

extern SPIClass SPI;

#endif
