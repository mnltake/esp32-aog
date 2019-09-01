#include <stdint.h>
#include <Arduino.h>
#include "io_provider.hpp"
#include "io_provider_ads1115.hpp"
#include <Adafruit_ADS1015.h>
#include "main.hpp"


  IoProvider_ADS115::IoProvider_ADS115(uint8_t address) {
    i2cAddress = address;
  }
  bool IoProvider_ADS115::init() {
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

  bool IoProvider_ADS115::configureAsAnalogInput(uint8_t port) {
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

  int IoProvider_ADS115::getAnalogInput(uint8_t port) {
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
  float IoProvider_ADS115::getAnalogInputScaled(uint8_t port) {
    return (float)getAnalogInput(port) / INT16_MAX; //Int 16 signed ...
  }

  String IoProvider_ADS115::getName() const {return "ADS1115"; }
