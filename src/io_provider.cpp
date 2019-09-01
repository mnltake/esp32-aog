#include <stdint.h>
#include <Arduino.h>
#include <esp32-hal-adc.h>
#include "io_access.hpp"
#include "io_provider.hpp"


  // returns the capabilities
  bool IoProvider::isDigitalInput(uint8_t port) const {return ports[port].digitalInput;};
  bool IoProvider::isAnalogInput(uint8_t port) const {return ports[port].analogInput;};
  bool IoProvider::isDigitalOutput(uint8_t port) const {return ports[port].digitalOutput;};
  bool IoProvider::isPwmOutput(uint8_t port) const {return ports[port].pwmOutput;};
  bool IoProvider::isRawIo(uint8_t port) const {return ports[port].rawIo;};

  // configure a port, default nothing enabled
  bool IoProvider::configureAsDigitalInput(uint8_t port, bool usePullUpDown, bool pullDirectionUp) {return false;}
  bool IoProvider::configureAsAnalogInput(uint8_t port) {return false;}
  bool IoProvider::configureAsDigitalOutput(uint8_t port) {return false;}
  bool IoProvider::configureAsPwmOutput(uint8_t port, uint16_t frequency) {return false;}
  bool IoProvider::configureAsRawIo(uint8_t port) {return false;}

  // returns the pin number of a port on the microcontroler.
  // should only be used if necessary (e.g. setting up serial ports)
  // other application has to configure the pin
  gpio_num_t IoProvider::getRawIo(uint8_t port)  const {
    // for all except ESP32 always invalid
    return GPIO_NUM_0;
  }

  // normal interactions during runtime
  bool IoProvider::getDigitalInput(uint8_t port) {return false;}
  int IoProvider::getAnalogInput(uint8_t port) {return INT_MAX;}
  float IoProvider::getAnalogInputScaled(uint8_t port) {return 0.0f;}
  void IoProvider::setDigitalOutput(uint8_t port, bool state) {};
  void IoProvider::setPwmOutput(uint8_t port, uint8_t dutyCycle) {};


  const String IoProvider::getPortName(uint8_t port) const {
    return String(getName() + " " + ports[port].humanName);
  }
  const IoProvider::portDefinition IoProvider::ports[32] = {};
