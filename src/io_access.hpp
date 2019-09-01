#pragma once
#include <stdint.h>
#include <Arduino.h>

#include "io_provider.hpp"


enum IoProviders : uint8_t {
  NotUsed = 0,
  ESP32_IO = 1,
  ADS1115 = 8,
  FXL6408 = 16
} ;

class IoAccess {
  // mapping the internal uint8_t to different IO access providers.
  // Every access provider is responsible for 32 IOs
  // Value 255 is the reserved value for invalid pins
public:
  static bool addIoProvider(IoProviders type, uint8_t position, uint8_t options);
  static uint8_t initProvider();
  // returns the capabilities
  static bool isDigitalInput(uint8_t port);
  static bool isAnalogInput(uint8_t port);
  static bool isDigitalOutput(uint8_t port);
  static bool isPwmOutput(uint8_t port);
  static bool isRawIo(uint8_t port);
  static bool configureAsDigitalInput(uint8_t port, bool usePullUpDown, bool pullDirectionUp);
  static bool configureAsAnalogInput(uint8_t port);
  static bool configureAsDigitalOutput(uint8_t port);
  static bool configureAsPwmOutput(uint8_t port, uint16_t frequency);
  static bool configureAsRawIo(uint8_t port);

  // returns the pin number of a port on the microcontroler.
  // should only be used if necessary (e.g. setting up serial ports)
  // other application has to configure the pin
  static gpio_num_t getRawIo(uint8_t port);

  // normal interactions during runtime
  static bool getDigitalInput(uint8_t port);
  static int getAnalogInput(uint8_t port);
  static float getAnalogInputScaled(uint8_t port);
  static void setDigitalOutput(uint8_t port, bool state);
  static void setPwmOutput(uint8_t port, uint8_t dutyCycle);


  static const String getPortName(uint8_t port);

private:
  // interfaces contains the different IO access providers.
  // currently only 4 "slots" (0-31, 32-63, 64-95, 96-127), can be extended
  // to 8 slots
  static IoProvider *provider[8];
};
