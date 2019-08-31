#include <stdint.h>
#include <string>
#include "io_provider.cpp"
#include "main.hpp"

class IoProvider_FXL6408 : public IoProvider {
public:
  IoProvider_FXL6408(uint8_t address) {
    i2cAddress = address;
  }
  bool init() {
    int returnValues = 0;
    returnValues += setByteI2C(0x07, 0b11111111); // Output High-Z (not driven)
    returnValues += setByteI2C(0x03, 0b11111111); // Everything Output
    returnValues += setByteI2C(0x05, 0b00000000); // (Disabled) Outputs to low)
    returnValues += setByteI2C(0x0B, 0b00000000); // No Pullup/down
    returnValues += setByteI2C(0x11, 0b11111111); // No interrupts
    return returnValues == 0;
  }

  // configure a port, default nothing enabled
  bool configureAsDigitalInput(uint8_t port, bool usePullUpDown, bool pullDirectionUp) {
    if (configuration[port] != unconfigured) {
      return false;
    }
    // set up the port as input, no interrupt, error handling is a TODO
    configuration[port] = digitalInput;

    int returnValues = 0;
    // pullUp/Down
    returnValues += setByteI2C(0x0D, setBit(getByteI2C(0x0D), port, pullDirectionUp));
    returnValues += setByteI2C(0x0B, setBit(getByteI2C(0x0B), port, usePullUpDown));
    // direction
    returnValues += setByteI2C(0x03, setBit(getByteI2C(0x03), port, false));

    return returnValues == 0;
  }

  bool configureAsDigitalOutput(uint8_t port) {
    if (configuration[port] != unconfigured) {
      return false;
    }
    configuration[port] = digitalOutput;

    int returnValues = 0;
    // default low
    outputRegister = setBit(outputRegister, port, false);
    returnValues += setByteI2C(0x05, outputRegister);
    // disable High-Z
    returnValues += setByteI2C(0x07, setBit(getByteI2C(0x07), port, false));
    // direction
    returnValues += setByteI2C(0x03, setBit(getByteI2C(0x03), port, true));

    return returnValues == 0;
  }

  bool getDigitalInput(uint8_t port)  {
    uint8_t value = getByteI2C(0xF);
    value = (value >> port) & 1; // shift so the port is at last bit, then mask
    return value == 1;
  }

  void setDigitalOutput(uint8_t port, bool state) {
    outputRegister = setBit(outputRegister, port, state);
    setByteI2C(0x05, outputRegister);
  };

private:
  uint8_t outputRegister = 0;
  uint8_t getByteI2C(int i2cregister) {
    uint8_t result;
    if ( xSemaphoreTake( i2cMutex, 1000 ) == pdTRUE ) {
      Wire.beginTransmission(i2cAddress);
      Wire.write(i2cregister);
      Wire.endTransmission(false);
      uint8_t state = Wire.requestFrom(i2cAddress, 1, (int)true);
      result = Wire.read();
      xSemaphoreGive( i2cMutex );
    }
    return result;
  }

  uint8_t setByteI2C(byte i2cregister, byte value) {
    uint8_t result = 255;
    if ( xSemaphoreTake( i2cMutex, 1000 ) == pdTRUE ) {
      Wire.beginTransmission(i2cAddress);
      Wire.write(i2cregister);
      Wire.write(value);
      result = Wire.endTransmission();
      xSemaphoreGive( i2cMutex );
    }
    return result;
  }

uint8_t setBit(uint8_t byte, uint8_t position, bool value) {
  uint8_t pattern = 0b00000001 << position;
  if (value) {
    return byte | pattern;
  }  else {
    pattern = ~pattern;
    return byte & pattern;
  }
}

  uint8_t i2cAddress;
  std::string getName() const {return "FXL6408"; }
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
