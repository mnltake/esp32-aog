#pragma once
#include <stdint.h>
#include <Arduino.h>
#include "io_provider.hpp"
#include <Adafruit_ADS1015.h>
#include "main.hpp"

class IoProvider_ADS115 : public IoProvider {
public:
  IoProvider_ADS115(uint8_t address) ;
  bool init() ;

  bool configureAsAnalogInput(uint8_t port) ;
  int getAnalogInput(uint8_t port);
  float getAnalogInputScaled(uint8_t port);

private:
  uint8_t i2cAddress;
  String getName() const ;
  Adafruit_ADS1115 ads = Adafruit_ADS1115();
  const portDefinition ports[32] = {
    {"Single 0", false, true, false, false, false},
    {"Single  1", false, true, false, false, false},
    {"Single  2", false, true, false, false, false},
    {"Single  3", false, true, false, false, false},
    {"Differential 0-1", false, true, false, false, false},
    {"Differential 2-3", false, true, false, false, false}};
};
