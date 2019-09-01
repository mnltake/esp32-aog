#pragma once
#include <stdint.h>
#include <string>
#include "io_provider.hpp"
#include "main.hpp"

class IoProvider_FXL6408 : public IoProvider {
public:
  IoProvider_FXL6408(uint8_t address);
  bool init() ;

  // configure a port, default nothing enabled
  bool configureAsDigitalInput(uint8_t port, bool usePullUpDown, bool pullDirectionUp) ;

  bool configureAsDigitalOutput(uint8_t port) ;
  bool getDigitalInput(uint8_t port)  ;

  void setDigitalOutput(uint8_t port, bool state) ;

private:
  uint8_t outputRegister = 0;
  uint8_t getByteI2C(int i2cregister) ;

  uint8_t setByteI2C(byte i2cregister, byte value) ;

uint8_t setBit(uint8_t byte, uint8_t position, bool value) ;

  uint8_t i2cAddress;
  String getName() const ;
  const portDefinition ports[32] = {
    {"GPIO 0", true, false, true, false, false},
    {"GPIO 1", true, false, true, false, false},
    {"GPIO 2", true, false, true, false, false},
    {"GPIO 3", true, false, true, false, false},
    {"GPIO 4", true, false, true, false, false},
    {"GPIO 5", true, false, true, false, false},
    {"GPIO 6", true, false, true, false, false},
    {"GPIO 7", true, false, true, false, false}};
};
