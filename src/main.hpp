// MIT License
//
// Copyright (c) 2019 Christian Riggenbach
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <WiFi.h>
#include <WiFiMulti.h>

#include <HTTPClient.h>

#include <AsyncUDP.h>

#include <EEPROM32_Rotate.h>
#include <ESPUI.h>

#include <Adafruit_MMA8451.h>
#include <Adafruit_BNO055.h>

#include "average.hpp"

#ifndef MAIN_HPP
#define MAIN_HPP

extern uint16_t labelLoad;
extern uint16_t labelOrientation;
extern uint16_t labelWheelAngle;
extern uint16_t textNmeaToSend;

extern uint16_t labelStatusOutput;
extern uint16_t labelStatusAdc;
extern uint16_t labelStatusCan;
extern uint16_t labelStatusImu;
extern uint16_t labelStatusInclino;
extern uint16_t labelStatusGps;
extern uint16_t labelStatusNtrip;

extern uint32_t statusLedPattern;
extern uint8_t fxl6408_outputRegister;

extern SemaphoreHandle_t i2cMutex;

///////////////////////////////////////////////////////////////////////////
// Configuration
///////////////////////////////////////////////////////////////////////////

struct SteerConfig {

  enum class Gpio : int8_t {
    Default     = -1,
    None        = 0,
    Esp32Gpio4  = 4,
    Esp32Gpio5  = 5,
    Esp32Gpio12 = 12,
    Esp32Gpio13 = 13,
    Esp32Gpio14 = 14,
    Esp32Gpio15 = 15,
    Esp32Gpio21 = 21,
    Esp32Gpio22 = 22,
    Esp32Gpio23 = 23,
    Esp32Gpio25 = 25,
    Esp32Gpio26 = 26,
    Esp32Gpio27 = 27,
    Esp32Gpio32 = 32,
    Esp32Gpio33 = 33,
    Esp32Gpio34 = 34,
    Esp32Gpio35 = 35,
    Esp32Gpio36 = 36,
    Esp32Gpio39 = 39
  };

  enum class AnalogIn : uint8_t {
    None                    = 0,
    Esp32GpioA2             = 2,
    Esp32GpioA3             = 3,
    Esp32GpioA4             = 4,
    Esp32GpioA7             = 7,
    Esp32GpioA9             = 9,
    Esp32GpioA12            = 12,
    ADS1115A0Single         = 100,
    ADS1115A1Single         = 101,
    ADS1115A2Single         = 102,
    ADS1115A3Single         = 103,
    ADS1115A0A1Differential = 200,
    ADS1115A2A3Differential = 202
  };

  enum class NetworkType : uint8_t {
    WiFi,
    Cable
  } networkType = NetworkType::WiFi;

  char ssid[24] = "AgOpen";
  char password[24] = "wifiConnection";
  char hostname[24] = "ESP-AOG";

  //set to 1  if you want to use Steering Motor + Cytron MD30C Driver
  //set to 2  if you want to use Steering Motor + IBT 2  Driver
  //set to 3  if you want to use IBT 2  Driver + PWM 2-Coil Valve
  //set to 4  if you want to use IBT 2  Driver + Danfoss Valve PVE A/H/M
  enum class OutputType : uint8_t {
    None = 0,
    SteeringMotorCytron = 1,
    SteeringMotorIBT2,
    HydraulicPwm2Coil,
    HydraulicDanfoss,
    SteeringMotorVNH7070AS,
    HydraulicPwm2CoilVNH7070AS,
    HydraulicDanfossVNH7070AS
  } outputType = OutputType::None;

  uint16_t pwmFrequency = 12000;
  bool invertOutput = false;
  SteerConfig::Gpio gpioPwm = SteerConfig::Gpio::Esp32Gpio4;
  SteerConfig::Gpio gpioDir = SteerConfig::Gpio::None;
  SteerConfig::Gpio gpioEn = SteerConfig::Gpio::None;

  bool allowPidOverwrite = false;
  double steeringPidKp = 20;
  double steeringPidKi = 0.5;
  double steeringPidKd = 1;
  double steeringPidAutoBangOnFactor = 2;
  double steeringPidBangOn = 40;
  double steeringPidBangOff = 0.1;
//   uint16_t steeringPidDflTurnIdOff = 40;
  uint8_t steeringPidMinPwm = 20;


  enum class WorkswitchType : uint8_t {
    None = 0,
    Gpio,
    RearHitchPosition,
    FrontHitchPosition,
    RearPtoRpm,
    FrontPtoRpm,
    MotorRpm
  } workswitchType = WorkswitchType::None;
  SteerConfig::Gpio gpioWorkswitch = SteerConfig::Gpio::None;
  SteerConfig::Gpio gpioSteerswitch = SteerConfig::Gpio::None;
  uint16_t autoRecogniseSteerGpioAsSwitchOrButton = 500;
  bool workswitchActiveLow = true;
  bool steerswitchActiveLow = true;

  enum class WheelAngleSensorType : uint8_t {
    WheelAngle = 0,
    TieRodDisplacement,
    AckermannLeft,
    AckermannRight
  } wheelAngleSensorType = WheelAngleSensorType::WheelAngle;

  SteerConfig::AnalogIn wheelAngleInput = SteerConfig::AnalogIn::ADS1115A2Single;

  bool allowWheelAngleCenterAndCountsOverwrite = false;
  bool invertWheelAngleSensor = false;
  float wheelAngleCountsPerDegree = 380;
  uint16_t wheelAnglePositionZero = 13450;

  float wheelAngleOffset = 0;

  float wheelAngleFirstArmLenght = 92;
  float wheelAngleSecondArmLenght = 308;
  float wheelAngleTieRodStroke = 210;
  float wheelAngleMinimumAngle = 37;
  float wheelAngleTrackArmLenght = 165;

  bool steeringWheelEncoder = false;
  SteerConfig::Gpio gpioWheelencoderA = SteerConfig::Gpio::None;
  SteerConfig::Gpio gpioWheelencoderB = SteerConfig::Gpio::None;

  uint8_t wheelEncoderPulseCountMax = 3;

  SteerConfig::Gpio gpioSDA = SteerConfig::Gpio::Esp32Gpio32;
  SteerConfig::Gpio gpioSCL = SteerConfig::Gpio::Esp32Gpio33;
  uint32_t i2cBusSpeed = 400000;
  enum class ImuType : uint8_t {
    None = 0,
    Fxos8700Fxas21002,
    LSM9DS1
  } imuType = ImuType::LSM9DS1;

  enum class InclinoType : uint8_t {
    None = 0,
    MMA8451 = 1,
    Fxos8700Fxas21002,
    LSM9DS1
  } inclinoType = InclinoType::LSM9DS1;

  bool invertRoll = false;

  float mountCorrectionImuRoll = 0;
  float mountCorrectionImuPitch = 0;
  float mountCorrectionImuYaw = 0;

  bool canBusEnabled = false;
  SteerConfig::Gpio canBusRx = SteerConfig::Gpio::Esp32Gpio35;
  SteerConfig::Gpio canBusTx = SteerConfig::Gpio::Esp32Gpio5;
  enum class CanBusSpeed : uint16_t {
    Speed250kbs = 250,
    Speed500kbs = 500
  } canBusSpeed = CanBusSpeed::Speed500kbs;

  uint8_t canBusHitchThreshold = 50;
  uint8_t canBusHitchThresholdHysteresis = 6;

  uint16_t canBusRpmThreshold = 400;
  uint16_t canBusRpmThresholdHysteresis = 100;

  enum class RtkCorrectionType : uint8_t {
    None = 0,
    Ntrip = 1,
    udp,
    tcp
  } rtkCorrectionType = RtkCorrectionType::None;

  char rtkCorrectionServer[48] = "hostname or ip";
  uint16_t rtkCorrectionPort = 2101;
  char rtkCorrectionUsername[24] = "user";
  char rtkCorrectionPassword[24] = "password";
  char rtkCorrectionMountpoint[24] = "mountpoint";

  char rtkCorrectionNmeaToSend[120] = "";

  uint32_t rtkCorrectionBaudrate = 115200;

  uint8_t ntripPositionSendIntervall = 30;

  enum class SendNmeaDataTo : uint8_t {
    None = 0,
    UDP = 1,
    USB = 2,
    RS232 = 4,
    UDPRS232 = 5,
  } sendNmeaDataTo = SendNmeaDataTo::UDPRS232;

  uint16_t sendNmeaDataTcpPort = 0;

  uint16_t portSendFrom = 5577;
  uint16_t portListenTo = 8888;
  uint16_t portSendTo = 9999;

  bool retainWifiSettings = false;

//   char dummy[100];

};
extern SteerConfig steerConfig, steerConfigDefaults;


struct GenericImuCalibrationData {

  GenericImuCalibrationData() {
    // init with neutral element
    mag_hardiron[0] = 0.0;
    mag_hardiron[1] = 0.0;
    mag_hardiron[2] = 0.0;
    mag_softiron[0] = 1.0;
    mag_softiron[1] = 1.0;
    mag_softiron[2] = 1.0;

    gyro_zero_offsets[0] = 0;
    gyro_zero_offsets[1] = 0;
    gyro_zero_offsets[2] = 0;
  };

  // hard iron compensation
  float mag_hardiron[3];

  // Soft iron error compensation matrix
  float mag_softiron[3];

  // Offsets applied to compensate for gyro zero-drift error for x/y/z
  float gyro_zero_offsets[3];
};
extern GenericImuCalibrationData genericimucalibrationdata;

struct Initialisation {
  SteerConfig::OutputType outputType = SteerConfig::OutputType::None;
  SteerConfig::AnalogIn wheelAngleInput = SteerConfig::AnalogIn::None;
  SteerConfig::ImuType imuType = SteerConfig::ImuType::None;
  SteerConfig::InclinoType inclinoType = SteerConfig::InclinoType::None;

  uint16_t portSendFrom = 5577;
  uint16_t portListenTo = 8888;
  uint16_t portSendTo = 9999;

  String rtkCorrectionURL = "";
};
extern Initialisation initialisation;


///////////////////////////////////////////////////////////////////////////
// Global Data
///////////////////////////////////////////////////////////////////////////

enum class EepromAddresses : uint16_t {
  CRC = 0,
  Validator = 5,
  SizeOfConfig = 7,
  GenericImuCalibrationData = 9,
  SteerConfig = GenericImuCalibrationData + sizeof( GenericImuCalibrationData )
};

struct SteerSettings {
  float Ko = 0.0f;  //overall gain
  float Kp = 0.0f;  //proportional gain
  float Ki = 0.0f;//integral gain
  float Kd = 0.0f;  //derivative gain
  uint8_t minPWMValue = 15;
  int maxIntegralValue = 20; //max PWM value for integral PID component
  float wheelAngleCountsPerDegree = 118;
  uint16_t wheelAnglePositionZero = 0;

  time_t lastPacketReceived = 0;
};
extern SteerSettings steerSettings;

struct SteerSetpoints {
  uint8_t relais = 0;
  float speed = 0;
  uint16_t distanceFromLine = 32020;
  double requestedSteerAngle = 0;

  bool enabled = false;
  float receivedRoll = 0;
  double actualSteerAngle = 0;
  double wheelAngleCurrentDisplacement = 0;
  double wheelAngleRaw = 0;
  float correction = 0;

  time_t lastPacketReceived = 0;
};
extern SteerSetpoints steerSetpoints;

struct SteerMachineControl {
  uint8_t pedalControl = 0;
  float speed = 0;
  uint8_t relais = 0;
  uint8_t youTurn = 0;

  time_t lastPacketReceived = 0;
};
extern SteerMachineControl steerMachineControl;

struct SteerImuInclinometerData {
  bool sendCalibrationDataFromImu = false;

  float heading;
  float roll;
  float pitch;
};
extern SteerImuInclinometerData steerImuInclinometerData;

struct SteerCanData {
  float speed;
  uint16_t motorRpm;
  uint8_t frontHitchPosition;
  uint8_t rearHitchPosition;
  uint16_t frontPtoRpm;
  uint16_t rearPtoRpm;
};
extern SteerCanData steerCanData;

///////////////////////////////////////////////////////////////////////////
// external Libraries
///////////////////////////////////////////////////////////////////////////

extern ESPUIClass ESPUI;
extern EEPROM32_Rotate EEPROM;

// extern AsyncUDP udpLocalPort;
// extern AsyncUDP udpRemotePort;
extern AsyncUDP udpSendFrom;

extern Adafruit_MMA8451 mma;
extern Adafruit_BNO055 bno;

///////////////////////////////////////////////////////////////////////////
// Helper Classes
///////////////////////////////////////////////////////////////////////////
extern portMUX_TYPE mux;
class TCritSect {
    TCritSect() {
      portENTER_CRITICAL( &mux );
    }
    ~TCritSect() {
      portEXIT_CRITICAL( &mux );
    }
};


///////////////////////////////////////////////////////////////////////////
// Threads
///////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////
// Helper Functions
///////////////////////////////////////////////////////////////////////////

extern void writeEeprom();
extern void initIdleStats();
extern void initSensors();
extern void calculateMountingCorrection();
extern void initRtkCorrection();
extern void initCan();
extern void initAutosteer();

extern bool FXL6408_init();
extern bool FXL6408_configureAsDigitalInput(uint8_t port, bool usePullUpDown, bool pullDirectionUp);
extern bool FXL6408_configureAsDigitalOutput(uint8_t port);
extern bool FXL6408_getDigitalInput(uint8_t port);
extern void FXL6408_setDigitalOutput(uint8_t port, bool state);
extern uint8_t FXL6408_getByteI2C(int i2cregister);
extern uint8_t FXL6408_setByteI2C(int i2cregister, byte value) ;
extern uint8_t FXL6408_setBit(uint8_t byte, uint8_t position, bool value) ;

#endif
