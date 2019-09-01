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
#pragma once

#include <WiFi.h>
#include <WiFiMulti.h>

#include <HTTPClient.h>

#include <AsyncUDP.h>

#include <EEPROM32_Rotate.h>
#include <ESPUI.h>

#include <Adafruit_MMA8451.h>
#include <Adafruit_BNO055.h>

#include "average.hpp"
#include "io_access.hpp"

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

extern SemaphoreHandle_t i2cMutex;

struct ioInterface {
  IoProviders provider;
  uint8_t options;
};

///////////////////////////////////////////////////////////////////////////
// Configuration
///////////////////////////////////////////////////////////////////////////

struct SteerConfig {


  ioInterface ioInterfaces[8] = {{IoProviders::ESP32_IO,0}};

  uint8_t enterSetupTimeout = 30; // Enter setup hotspot after unsuccessfull
                                  // connecting to the configured network (seconds)
                                  // value 0 disbales timeout mechanism
  uint8_t setupInputPort = 255;   // if this input is pulled to gnd during startup, enter setup

  char ssid[24] = "AOG";
  char password[24] = "aogaogaog";
  char hostname[24] = "ESP32-AOG";

  //set to 1  if you want to use Steering Motor + Cytron MD30C Driver
  //set to 2  if you want to use Steering Motor + IBT 2  Driver
  //set to 3  if you want to use IBT 2  Driver + PWM 2-Coil Valve
  //set to 4  if you want to use IBT 2  Driver + Danfoss Valve PVE A/H/M
  enum class OutputType : uint8_t {
    None = 0,
    SteeringMotorCytron = 1,
    SteeringMotorIBT2,
    HydraulicPwm2Coil,
    HydraulicDanfoss
  } outputType = OutputType::None;

  uint16_t pwmFrequency = 1000;
  bool invertOutput = false;
  uint8_t gpioPwm = 255;
  uint8_t gpioDir = 255;
  uint8_t gpioEn = 255;

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
  uint8_t gpioWorkswitch = 255;
  uint8_t gpioSteerswitch = 255;
  uint16_t autoRecogniseSteerGpioAsSwitchOrButton = 500;
  bool workswitchActiveLow = true;
  bool steerswitchActiveLow = true;

  enum class WheelAngleSensorType : uint8_t {
    WheelAngle = 0,
    TieRodDisplacement
  } wheelAngleSensorType = WheelAngleSensorType::WheelAngle;

  uint8_t wheelAngleInput = 255;

  bool allowWheelAngleCenterAndCountsOverwrite = false;
  bool invertWheelAngleSensor = false;
  float wheelAngleCountsPerDegree = 118;
  uint16_t wheelAnglePositionZero = 5450;

  float wheelAngleOffset = 0;

  float wheelAngleFirstArmLenght = 92;
  float wheelAngleSecondArmLenght = 308;
  float wheelAngleTieRodStroke = 210;
  float wheelAngleMinimumAngle = 37;
  float wheelAngleTrackArmLenght = 165;

  bool steeringWheelEncoder = false;
  uint8_t gpioWheelencoderA = 255;
  uint8_t gpioWheelencoderB = 255;

  uint8_t wheelEncoderPulseCountMax = 3;

  uint8_t gpioSDA = 255;
  uint8_t gpioSCL = 255;
  uint32_t i2cBusSpeed = 400000;
  enum class ImuType : uint8_t {
    None = 0,
    BNO055 = 1,
    Fxos8700Fxas21002
  } imuType = ImuType::None;

  enum class InclinoType : uint8_t {
    None = 0,
    MMA8451 = 1,
    DOGS2,
    Fxos8700Fxas21002
  } inclinoType = InclinoType::None;

  bool invertRoll = false;

  float mountCorrectionImuRoll = 0;
  float mountCorrectionImuPitch = 0;
  float mountCorrectionImuYaw = 0;

  bool canBusEnabled = false;
  uint8_t canBusRx = 255;
  uint8_t canBusTx = 255;
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

  char rtkCorrectionServer[48] = "example.com";
  uint16_t rtkCorrectionPort = 2101;
  char rtkCorrectionUsername[24] = "gps";
  char rtkCorrectionPassword[24] = "gps";
  char rtkCorrectionMountpoint[24] = "STALL";
  char rtkCorrectionMountpoint2[24] = "STALL";

  char rtkCorrectionNmeaToSend[120] = "";

  uint32_t rtkCorrectionBaudrate = 115200;

  uint8_t ntripPositionSendIntervall = 30;

  enum class SendNmeaDataTo : uint8_t {
    None = 0,
    UDP = 1,
    TCP,
    Serial,
    Serial1,
    Serial2,
    Bluetooth
  } sendNmeaDataTo = SendNmeaDataTo::None;

  uint16_t sendNmeaDataTcpPort = 0;

  uint16_t portSendFrom = 5577;
  uint16_t portListenTo = 8888;
  uint16_t portSendTo = 9999;

  bool retainWifiSettings = false;

//   char dummy[100];

};
extern SteerConfig steerConfig, steerConfigDefaults;

extern adafruit_bno055_offsets_t bno055CalibrationData;

struct Fxos8700Fxas21002CalibrationData {

  Fxos8700Fxas21002CalibrationData() {
    mag_offsets[0] = -13.56F;
    mag_offsets[1] = -11.98F;
    mag_offsets[2] = -85.02F;

    mag_softiron_matrix[0][0] =  0.998;
    mag_softiron_matrix[0][1] = -0.048;
    mag_softiron_matrix[0][2] = -0.009;
    mag_softiron_matrix[1][0] = -0.048;
    mag_softiron_matrix[1][1] =  1.022;
    mag_softiron_matrix[1][2] =  0.016;
    mag_softiron_matrix[2][0] = -0.009;
    mag_softiron_matrix[2][1] =  0.016;
    mag_softiron_matrix[2][2] =  0.983;

    mag_field_strength = 53.21F;

    gyro_zero_offsets[0] = 0;
    gyro_zero_offsets[1] = 0;
    gyro_zero_offsets[2] = 0;
  };

  // Offsets applied to raw x/y/z mag values
  float mag_offsets[3];

  // Soft iron error compensation matrix
  float mag_softiron_matrix[3][3];

  float mag_field_strength;

  // Offsets applied to compensate for gyro zero-drift error for x/y/z
  float gyro_zero_offsets[3];
};
extern Fxos8700Fxas21002CalibrationData fxos8700Fxas21002CalibrationData;

struct Initialisation {
  SteerConfig::OutputType outputType = SteerConfig::OutputType::None;
  uint8_t wheelAngleInput = 255;
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
  Bno055CalibrationData = 9,
  Fxos8700Fxas21002CalibrationData = Bno055CalibrationData + sizeof( bno055CalibrationData ),
  SteerConfig = Fxos8700Fxas21002CalibrationData + sizeof( fxos8700Fxas21002CalibrationData )
};

struct SteerSettings {
  float Ko = 0.0f;  //overall gain
  float Kp = 0.0f;  //proportional gain
  float Ki = 0.0f;//integral gain
  float Kd = 0.0f;  //derivative gain
  uint8_t minPWMValue = 10;
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

#endif
