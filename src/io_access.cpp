#include <stdint.h>
#include <Arduino.h>
#include "io_access.hpp"
#include "io_provider.hpp"
#include "io_provider_ads1115.hpp"
#include "io_provider_esp32.hpp"
#include "io_provider_fxl6408.hpp"


  bool IoAccess::addIoProvider(IoProviders type, uint8_t position, uint8_t options) {
    // check if position exists
    if (position >= sizeof(provider) / sizeof(provider[0])) {
      return false;
    }
    // check if free
    if (provider[position] != nullptr) {
      return false;
    }
    // looks good, create IO provider
    switch ( type ) {
      case NotUsed:
        return true;
      case ESP32_IO:
        provider[position] = new IoProvider_ESP32();
        return provider[position] -> init();
      case ADS1115:
        provider[position] = new IoProvider_ADS115(options);
        return provider[position] -> init();
      case FXL6408:
        provider[position] = new IoProvider_FXL6408(options);
        return provider[position] -> init();
      default:
        return false;
      }
  }
  uint8_t IoAccess::initProvider() {
    uint8_t result = 0;
    for (int i = 0; i < sizeof(provider) / sizeof(provider[0]); i++) {
      if (provider[i] != nullptr) {
        if (provider[i] -> init() != true) {
          result += 1 << i;
        }
      }
    }
    return result;
  }
  // returns the capabilities
  bool IoAccess::isDigitalInput(uint8_t port) {
    if (port == 255) {
      return true;;
    }
    return provider[port / 32] -> isDigitalInput(port % 32);
  };
  bool IoAccess::isAnalogInput(uint8_t port) {
    if (port == 255) {
      return true;;
    }
    return provider[port / 32] ->  isAnalogInput(port % 32);
  }
  bool IoAccess::isDigitalOutput(uint8_t port) {
    if (port == 255) {
      return true;;
    }
    return provider[port / 32] ->  isDigitalOutput(port % 32);
  }
  bool IoAccess::isPwmOutput(uint8_t port) {
    if (port == 255) {
      return true;;
    }
    return provider[port / 32] ->  isPwmOutput(port % 32);
  };
  bool IoAccess::isRawIo(uint8_t port) {
    if (port == 255) {
      return true;;
    }
    return provider[port / 32] ->  isRawIo(port % 32);
  };

  // configure a port, default nothing enabled
  bool IoAccess::configureAsDigitalInput(uint8_t port, bool usePullUpDown, bool pullDirectionUp) {
    if (port == 255 || provider[port / 32]  == nullptr) {
      return false;
    }
    return provider[port / 32] ->  configureAsDigitalInput(port % 32, usePullUpDown, pullDirectionUp);
  }
  bool IoAccess::configureAsAnalogInput(uint8_t port) {
    if (port == 255 || provider[port / 32]  == nullptr) {
      return false;
    }
    return provider[port / 32] ->  configureAsAnalogInput(port % 32);
  }
  bool IoAccess::configureAsDigitalOutput(uint8_t port) {
    if (port == 255 || provider[port / 32]  == nullptr) {
      return false;
    }
    return provider[port / 32] ->  configureAsDigitalOutput(port % 32);
  }
  bool IoAccess::configureAsPwmOutput(uint8_t port, uint16_t frequency) {
    if (port == 255 || provider[port / 32]  == nullptr) {
      return false;
    }
    return provider[port / 32] ->  configureAsPwmOutput(port % 32, frequency);
  }
  bool IoAccess::configureAsRawIo(uint8_t port) {
    if (port == 255 || provider[port / 32]  == nullptr) {
      return false;
    }
    return provider[port / 32] ->  configureAsRawIo(port % 32);
  }

  // returns the pin number of a port on the microcontroler.
  // should only be used if necessary (e.g. setting up serial ports)
  // other application has to configure the pin
  gpio_num_t IoAccess::getRawIo(uint8_t port) {
    if (provider[port / 32]  != nullptr) {
      return provider[port / 32] -> getRawIo(port % 32);
    }
    return GPIO_NUM_0;
  }

  // normal interactions during runtime
  bool IoAccess::getDigitalInput(uint8_t port) {
    if (provider[port / 32]  != nullptr) {
      return provider[port / 32] -> getDigitalInput(port % 32);
    }
    return false;
  }
  int IoAccess::getAnalogInput(uint8_t port) {
    if (provider[port / 32]  != nullptr) {
      return provider[port / 32] -> getAnalogInput(port % 32);
    }
    return -1;
  }
  float IoAccess::getAnalogInputScaled(uint8_t port) {
    if (provider[port / 32]  != nullptr) {
      return provider[port / 32] -> getAnalogInputScaled(port % 32);
    }
    return -1;
  } // FSR scaled to a 0-1
  void IoAccess::setDigitalOutput(uint8_t port, bool state) {
    if (provider[port / 32]  != nullptr) {
      provider[port / 32] -> setDigitalOutput(port % 32, state);
    }
  }
  void IoAccess::setPwmOutput(uint8_t port, uint8_t dutyCycle) {
    if (provider[port / 32]  != nullptr) {
      provider[port / 32] -> setPwmOutput(port % 32, dutyCycle);
    }
  }


  const String IoAccess::getPortName(uint8_t port) {
    if (port == 255) {
      return "None";
    }
    return provider[port / 32] -> getPortName(port  % 32);
  }

IoProvider *IoAccess::provider[8] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
