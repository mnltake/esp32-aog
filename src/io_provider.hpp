#pragma once
#include <stdint.h>
#include <Arduino.h>
#include <esp32-hal-adc.h>

class IoProvider {

public:
  // take care of (necessary) initialisation
  virtual bool init() = 0;

  // returns the capabilities
  bool isDigitalInput(uint8_t port) const ;
  bool isAnalogInput(uint8_t port) const ;
  bool isDigitalOutput(uint8_t port) const ;
  bool isPwmOutput(uint8_t port) const ;
  bool isRawIo(uint8_t port) const;

  // configure a port, default nothing enabled
  bool configureAsDigitalInput(uint8_t port, bool usePullUpDown, bool pullDirectionUp) ;
  bool configureAsAnalogInput(uint8_t port) ;
  bool configureAsDigitalOutput(uint8_t port) ;
  bool configureAsPwmOutput(uint8_t port, uint16_t frequency) ;
  bool configureAsRawIo(uint8_t port) ;

  // returns the pin number of a port on the microcontroler.
  // should only be used if necessary (e.g. setting up serial ports)
  // other application has to configure the pin
  gpio_num_t getRawIo(uint8_t port)  const;

  // normal interactions during runtime
  bool getDigitalInput(uint8_t port);
  int getAnalogInput(uint8_t port) ;
  float getAnalogInputScaled(uint8_t port) ;
  void setDigitalOutput(uint8_t port, bool state) ;
  void setPwmOutput(uint8_t port, uint8_t dutyCycle) ;


  const String getPortName(uint8_t port) const ;


protected:
  struct portDefinition {
    String humanName;
    bool digitalInput;
    bool analogInput;
    bool digitalOutput;
    bool pwmOutput;
    bool rawIo;
    portDefinition(){};
    portDefinition(String name, bool digitalIn, bool analogIn, bool digitalOut, bool pwmOut, bool raw ){
      humanName = name;
      digitalInput = digitalIn;
      analogInput = analogIn;
      digitalOutput = digitalOut;
      pwmOutput = pwmOut;
      rawIo = raw;
    };
  };

  enum portState {
    unconfigured,
    digitalInput,
    analogInput,
    digitalOutput,
    pwmOutput,
    rawIo,
    disabled
  };
  portState configuration[32];


private:
  virtual String getName() const = 0;
  static const portDefinition ports[32];
};
