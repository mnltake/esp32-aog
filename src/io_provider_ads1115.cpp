#include <stdint.h>
#include <string>
#include "io_provider.cpp"
#include <Adafruit_ADS1015.h>
#include "main.hpp"

class IoProvider_ADS115 : public IoProvider {
public:
  IoProvider_ADS115(uint8_t address) {
    i2cAddress = address;
  }
  bool init() {
    if ( xSemaphoreTake( i2cMutex, 1000 ) == pdTRUE ) {
      ads = Adafruit_ADS1115( i2cAddress );
      ads.setGain( GAIN_TWOTHIRDS );
      ads.begin();
      ads.setSPS(ADS1115_DR_860SPS);      // for ADS1115 fastest samples per second is 860 (default is 128)
      xSemaphoreGive( i2cMutex );
      return true;
    }
    return false;
  }

  bool configureAsAnalogInput(uint8_t port) {
    if (configuration[port] != unconfigured) {
      return false;
    }
    configuration[port] = analogInput;
    // disable "shared" port
    switch (port) {
      case 0 ... 1:
        configuration[4] = disabled;
        break;
      case 2 ... 3:
        configuration[5] = disabled;
        break;
      case 4:
        configuration[0] = disabled;
        configuration[1] = disabled;
        break;
      case 5:
        configuration[2] = disabled;
        configuration[3] = disabled;
        break;
    }
  }

  int getAnalogInput(uint8_t port) {
    int result = INT_MAX;
    if ( xSemaphoreTake( i2cMutex, 1000 ) == pdTRUE ) {
      switch (port) {
        case 0 ... 3:
          result = ads.readADC_SingleEnded(port);
          break;
        case 5:
          result = ads.readADC_Differential_0_1();
          break;
        case 6:
          result = ads.readADC_Differential_2_3();
          break;
      }
      xSemaphoreGive( i2cMutex );
      return true;
    }
    return result;;
  }
  float getAnalogInputScaled(uint8_t port) {
    return (float)getAnalogInput(port) / INT16_MAX; //Int 16 signed ...
  }

private:
  uint8_t i2cAddress;
  std::string getName() const {return "ADS1115"; }
  Adafruit_ADS1115 ads = Adafruit_ADS1115();
  const portDefinition ports[32] = {
    {"Single 0", false, true, false, false, false},
    {"Single  1", false, true, false, false, false},
    {"Single  2", false, true, false, false, false},
    {"Single  3", false, true, false, false, false},
    {"Differential 0-1", false, true, false, false, false},
    {"Differential 2-3", false, true, false, false, false}};
};
