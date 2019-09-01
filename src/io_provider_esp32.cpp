#include <stdint.h>
#include <string>
#include <esp32-hal-adc.h>
#include "io_provider.hpp"
#include "io_provider_esp32.hpp"

  IoProvider_ESP32::IoProvider_ESP32() {
  }
  bool IoProvider_ESP32::init() {
    analogReadResolution(10); // Default of 12 is not very linear. Recommended to use 10 or 11 depending on needed resolution.
    analogSetAttenuation(ADC_11db); // Default is 11db which is very noisy. But needed for full scale range  Recommended to use 2.5 or 6.
    return true;
  }

  // configure a port, default nothing enabled
  bool IoProvider_ESP32::configureAsDigitalInput(uint8_t port, bool usePullUpDown, bool pullDirectionUp) {
    if (configuration[port] != unconfigured) {
      return false;
    }
    // set up the port as input, no interrupt, error handling is a TODO
    configuration[port] = digitalInput;
    gpio_pad_select_gpio(portMapping[port]);
    gpio_set_direction(portMapping[port], GPIO_MODE_INPUT);
    gpio_intr_disable(portMapping[port]);
    if (usePullUpDown) {
      if (pullDirectionUp) {
        gpio_set_pull_mode(portMapping[port], GPIO_PULLUP_ONLY);
      } else {
        gpio_set_pull_mode(portMapping[port], GPIO_PULLDOWN_ONLY);
      }
    } else {
      gpio_set_pull_mode(portMapping[port], GPIO_FLOATING);
    }
    return true;
  }

  bool IoProvider_ESP32::configureAsAnalogInput(uint8_t port) {
    if (configuration[port] != unconfigured) {
      return false;
    }
    configuration[port] = analogInput;
    gpio_pad_select_gpio(portMapping[port]);
    gpio_set_direction(portMapping[port], GPIO_MODE_INPUT);
    gpio_intr_disable(portMapping[port]);
    gpio_set_pull_mode(portMapping[port], GPIO_FLOATING);
    return true;
  }

  bool IoProvider_ESP32::configureAsDigitalOutput(uint8_t port) {
    if (configuration[port] != unconfigured) {
      return false;
    }
    configuration[port] = digitalOutput;
    gpio_pad_select_gpio(portMapping[port]);
    gpio_intr_disable(portMapping[port]);
    gpio_set_pull_mode(portMapping[port], GPIO_FLOATING);
    gpio_set_direction(portMapping[port], GPIO_MODE_OUTPUT);
    return true;
  }

  bool IoProvider_ESP32::configureAsPwmOutput(uint8_t port, uint16_t frequency) {
    if (configuration[port] != unconfigured) {
      return false;
    }
    for (int i = 0; i < 4; i++) {
      if (pwmChannels[i].inUse == false) {
        // internals
        configuration[port] = pwmOutput;
        pwmChannels[i].inUse = true;
        pwmChannels[i].usedBay = portMapping[port];
        // pin
        gpio_pad_select_gpio(portMapping[port]);
        gpio_intr_disable(portMapping[port]);
        gpio_set_pull_mode(portMapping[port], GPIO_FLOATING);
        gpio_set_direction(portMapping[port], GPIO_MODE_OUTPUT);
        // pwm
        ledcSetup(i, frequency, 8);
        ledcWrite(i, 0);
        ledcAttachPin(portMapping[port], i);
        return true;
      }
    }
    return false;
  }

  bool IoProvider_ESP32::configureAsRawIo(uint8_t port) {
    if (configuration[port] != unconfigured) {
      return false;
    }
    configuration[port] = rawIo;
    return true;
  }

// returns the pin number of a port on the microcontroler.
// should only be used if necessary (e.g. setting up serial ports)
// other application has to configure the pin
gpio_num_t IoProvider_ESP32::getRawIo(uint8_t port) const {
  return portMapping[port];
}

// normal interactions during runtime
bool IoProvider_ESP32::getDigitalInput(uint8_t port)  const {
  return digitalRead(portMapping[port]);
}

int IoProvider_ESP32::getAnalogInput(uint8_t port)  const {
  return (uint16_t)analogRead(portMapping[port]);
}
uint16_t IoProvider_ESP32::getAnalogInputScaled(uint8_t port)  const {
  return (float)getAnalogInput(port) / 1024.0;
}
void IoProvider_ESP32::setDigitalOutput(uint8_t port, bool state) {
  if (state) {
    digitalWrite(portMapping[port], HIGH);
  } else {
    digitalWrite(portMapping[port], LOW);
  }
};
void IoProvider_ESP32::setPwmOutput(uint8_t port, uint8_t dutyCycle) {
  for (int i = 0; i < 4; i++) {
    if (pwmChannels[i].inUse == true && pwmChannels[i].usedBay == portMapping[port]) {
      ledcWrite(i, dutyCycle);
    }
  }
};

  String IoProvider_ESP32::getName() const {return "ESP32"; }
