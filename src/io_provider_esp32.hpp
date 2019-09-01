#pragma once
#include <stdint.h>
#include <Arduino.h>
#include <esp32-hal-adc.h>
#include "io_provider.hpp"

class IoProvider_ESP32 : public IoProvider {
public:
  IoProvider_ESP32() ;
  bool init() ;

  // configure a port, default nothing enabled
  bool configureAsDigitalInput(uint8_t port, bool usePullUpDown, bool pullDirectionUp) ;

  bool configureAsAnalogInput(uint8_t port);

  bool configureAsDigitalOutput(uint8_t port) ;

  bool configureAsPwmOutput(uint8_t port, uint16_t frequency) ;

  bool configureAsRawIo(uint8_t port) ;

// returns the pin number of a port on the microcontroler.
// should only be used if necessary (e.g. setting up serial ports)
// other application has to configure the pin
gpio_num_t getRawIo(uint8_t port) const ;

// normal interactions during runtime
bool getDigitalInput(uint8_t port)  const ;

int getAnalogInput(uint8_t port)  const ;
uint16_t getAnalogInputScaled(uint8_t port)  const ;
void setDigitalOutput(uint8_t port, bool state);
void setPwmOutput(uint8_t port, uint8_t dutyCycle) ;
private:
  String getName() const ;
  const portDefinition ports[32] = {
    {"GPIO  0", true, false, true, true, true},
    {"GPIO  1", true, false, true, true, true},
    {"GPIO  2", true, false, true, true, true},
    {"GPIO  3", true, false, true, true, true},
    {"GPIO  4", true, false, true, true, true},
    {"GPIO  5", true, false, true, true, true},
    {"GPIO 12", true, false, true, true, true},
    {"GPIO 13", true, false, true, true, true},
    {"GPIO 14", true, false, true, true, true},
    {"GPIO 15", true, false, true, true, true},
    {"GPIO 16", true, false, true, true, true},
    {"GPIO 17", true, false, true, true, true},
    {"GPIO 18", true, false, true, true, true},
    {"GPIO 19", true, false, true, true, true},
    {"GPIO 21", true, false, true, true, true},
    {"GPIO 22", true, false, true, true, true},
    {"GPIO 23", true, false, true, true, true},
    {"GPIO 25", true, false, true, true, true},
    {"GPIO 26", true, false, true, true, true},
    {"GPIO 27", true, false, true, true, true},
    {"GPIO 32", true, false, true, true, true},
    {"GPIO 33", true, false, true, false, true},
    {"GPIO 34", true, true, false, false, true},
    {"GPIO 35", true, true, false, false, true},
    {"GPIO 36", true, true, false, false, true},
    {"GPIO 39", true, true, false, false, true} };
  const gpio_num_t portMapping[32] = {
    GPIO_NUM_0, GPIO_NUM_1, GPIO_NUM_2, GPIO_NUM_3, GPIO_NUM_4, GPIO_NUM_5,
    GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_14, GPIO_NUM_15, GPIO_NUM_16, GPIO_NUM_17,
    GPIO_NUM_18, GPIO_NUM_19, GPIO_NUM_21, GPIO_NUM_22, GPIO_NUM_23, GPIO_NUM_25,
    GPIO_NUM_26, GPIO_NUM_27, GPIO_NUM_32, GPIO_NUM_33, GPIO_NUM_34, GPIO_NUM_35,
    GPIO_NUM_36, GPIO_NUM_39};
  struct pwmChannel {
    bool inUse = false;
    gpio_num_t usedBay;
    pwmChannel(){};
    };
  pwmChannel pwmChannels[4];
};
