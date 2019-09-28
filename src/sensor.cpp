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

// #include <Wire.h>
// #include <WiredDevice.h>
// #include <RegisterBasedWiredDevice.h>
// #include <Accelerometer.h>
// #include <AccelerometerMMA8451.h>

#include <Adafruit_Sensor.h>

#include <Adafruit_MMA8451.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <utility/quaternion.h>

#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>

#include <Mahony.h>
#include <Madgwick.h>

#include <Adafruit_ADS1015.h>


#include <ESPUI.h>

#include "main.hpp"

#include "average.hpp"
#include "ringbuffer.hpp"

Adafruit_MMA8451 mma = Adafruit_MMA8451();
Adafruit_FXAS21002C fxas2100 = Adafruit_FXAS21002C( 0x0021002C );
Adafruit_FXOS8700 fxos8700 = Adafruit_FXOS8700( 0x8700A, 0x8700B );
Adafruit_ADS1115 ads = Adafruit_ADS1115( 0x48 );

Madgwick ahrs;
// Mahony filter;

GenericImuCalibrationData genericimucalibrationdata;

SteerImuInclinometerData steerImuInclinometerData;

volatile uint16_t samplesPerSecond;

Average<float, float, 10> wasAverage;

imu::Quaternion mountingCorrection;

//Low pass butterworth filter order=2 alpha1=0.05
class  FilterBuLp2_mma8481acc {
  public:
    FilterBuLp2_mma8481acc() {
      v[0] = 0.0;
      v[1] = 0.0;
    }
  private:
    float v[3];
  public:
    float step( float x ) { //class II
      v[0] = v[1];
      v[1] = v[2];
      v[2] = ( 2.008336556421122521e-2 * x )
             + ( -0.64135153805756306422 * v[0] )
             + ( 1.56101807580071816339 * v[1] );
      return
        ( v[0] + v[2] )
        + 2 * v[1];
    }
} mma8481accFilterX, mma8481accFilterY, mma8481accFilterZ;

// http://www.schwietering.com/jayduino/filtuino/index.php?characteristic=bu&passmode=lp&order=2&usesr=usesr&sr=100&frequencyLow=5&noteLow=&noteHigh=&pw=pw&calctype=float&run=Send
//Low pass butterworth filter order=2 alpha1=0.05
class  FilterBuLp2_3 {
  public:
    FilterBuLp2_3() {
      v[0] = 0.0;
      v[1] = 0.0;
    }
  private:
    float v[3];
  public:
    float step( float x ) { //class II
      v[0] = v[1];
      v[1] = v[2];
      v[2] = ( 2.008336556421122521e-2 * x )
             + ( -0.64135153805756306422 * v[0] )
             + ( 1.56101807580071816339 * v[1] );
      return
        ( v[0] + v[2] )
        + 2 * v[1];
    }
} wheelAngleSensorFilter;

void calculateMountingCorrection() {
  // rotate by the correction, relative to the tracot axis
  {
    mountingCorrection.fromEuler( radians( steerConfig.mountCorrectionImuRoll ), 0, 0 );
  }
  {
    imu::Quaternion correction;
    correction.fromEuler( 0, radians( steerConfig.mountCorrectionImuPitch ), 0 );
    mountingCorrection = mountingCorrection * correction;
  }
  {
    imu::Quaternion correction;
    correction.fromEuler( 0, 0, radians( steerConfig.mountCorrectionImuYaw ) );
    mountingCorrection = mountingCorrection * correction;
  }
}


void sensorWorkerImuPoller( void* z ) {
  vTaskDelay( 2000 );
  constexpr TickType_t xFrequency = 10;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for ( ;; ) {
    // where is the data stored
    float mx,my,mz; //magnetic in uTesla
    float gx,gy,gz; // gyros in uTesla
    float ax,ay,az; // acceleration in g

    if ( initialisation.inclinoType == SteerConfig::InclinoType::Fxos8700Fxas21002 ||
         initialisation.imuType == SteerConfig::ImuType::Fxos8700Fxas21002 ) {

      sensors_event_t gyro_event, accel_event, mag_event;

      // Get new data samples
      if ( xSemaphoreTake( i2cMutex, 1000 ) == pdTRUE ) {
        fxas2100.getEvent( &gyro_event );
        fxos8700.getEvent( &accel_event, &mag_event );
        xSemaphoreGive( i2cMutex );
      }
      mx = mag_event.magnetic.x;
      my = mag_event.magnetic.y;
      mz = mag_event.magnetic.z;

      gx = degrees(gyro_event.gyro.x);
      gy = degrees(gyro_event.gyro.y);
      gz = degrees(gyro_event.gyro.z);

      ax = accel_event.acceleration.x;
      ay = accel_event.acceleration.y;
      az = accel_event.acceleration.z;
    }

    // TODO: collection if calibration is enabled

    // Apply mag compensation
    mx = (mx - genericimucalibrationdata.mag_hardiron[0]) * genericimucalibrationdata.mag_softiron[0];
    my = (my - genericimucalibrationdata.mag_hardiron[1]) * genericimucalibrationdata.mag_softiron[1];
    mz = (mz - genericimucalibrationdata.mag_hardiron[2]) * genericimucalibrationdata.mag_softiron[2];

    // Apply gyro zero-rate error compensation
    gx += genericimucalibrationdata.gyro_zero_offsets[0];
    gy += genericimucalibrationdata.gyro_zero_offsets[1];
    gz += genericimucalibrationdata.gyro_zero_offsets[2];

    // input into AHRS
    ahrs.update(
      gx,
      gy,
      gz,

      ax,
      ay,
      az,

      mx,
      my,
      mz
    );

    float w, x, y, z;
    ahrs.getQuaternion( &w, &x, &y, &z );
    imu::Quaternion orientation( w, x, y, z );

    // rotate by the correction
    {
      imu::Quaternion correction;
      correction.fromEuler( radians( steerConfig.mountCorrectionImuRoll ),
                            radians( steerConfig.mountCorrectionImuPitch ),
                            radians( steerConfig.mountCorrectionImuYaw ) );

      orientation = orientation * correction;
    }

    // orientation has the corrected angles in it, extract them (and correct the refrence frame)
    {
      imu::Vector<3> euler;
      euler = orientation.toEuler();
      euler.toDegrees();
      steerImuInclinometerData.roll = euler[2];
      steerImuInclinometerData.pitch = -euler[1];

      float heading = euler[0] + 180 + 90;

      while ( heading > 360 ) {
        heading -= 360;
      }

      steerImuInclinometerData.heading = heading;

    }

    {
      static uint8_t loopCounter = 0;

      if ( loopCounter++ > 99 ) {
        loopCounter = 0;
        {
          Control* handle = ESPUI.getControl( labelOrientation );
          handle->value = "Roll: ";
          handle->value += ( float )steerImuInclinometerData.roll;
          handle->value += "°, Pitch: ";
          handle->value += ( float )steerImuInclinometerData.pitch;
          handle->value += "°, Heading: ";
          handle->value += ( float )steerImuInclinometerData.heading;

          ESPUI.updateControl( handle );
        }
      }
    }

    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void sensorWorkerSteeringPoller( void* z ) {
  vTaskDelay( 2000 );
  constexpr TickType_t xFrequency = 10;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for ( ;; ) {
    if ( steerConfig.wheelAngleInput != SteerConfig::AnalogIn::None ) {
      float wheelAngleTmp = 0;

      switch ( ( uint8_t )steerConfig.wheelAngleInput ) {
        case ( uint8_t )SteerConfig::AnalogIn::Esp32GpioA2 ...( uint8_t )SteerConfig::AnalogIn::Esp32GpioA12: {
          wheelAngleTmp = analogRead( ( uint8_t )steerConfig.wheelAngleInput );
        }
        break;

        case ( uint8_t )SteerConfig::AnalogIn::ADS1115A0Single ...( uint8_t )SteerConfig::AnalogIn::ADS1115A3Single: {
          if ( xSemaphoreTake( i2cMutex, 1000 ) == pdTRUE ) {
            wheelAngleTmp = ads.readADC_SingleEnded(
                              ( uint8_t )steerConfig.wheelAngleInput - ( uint8_t )SteerConfig::AnalogIn::ADS1115A0Single );
            xSemaphoreGive( i2cMutex );
          }
        }
        break;

        case ( uint8_t )SteerConfig::AnalogIn::ADS1115A0A1Differential: {
          if ( xSemaphoreTake( i2cMutex, 1000 ) == pdTRUE ) {
            wheelAngleTmp = ads.readADC_Differential_0_1();
            xSemaphoreGive( i2cMutex );
          }
        }
        break;

        case ( uint8_t )SteerConfig::AnalogIn::ADS1115A2A3Differential: {
          if ( xSemaphoreTake( i2cMutex, 1000 ) == pdTRUE ) {
            wheelAngleTmp = ads.readADC_Differential_2_3();
            xSemaphoreGive( i2cMutex );
          }
        }
        break;

        default:
          break;
      }

      {
        if ( steerConfig.allowWheelAngleCenterAndCountsOverwrite ) {
          wheelAngleTmp -= steerSettings.wheelAnglePositionZero;
          wheelAngleTmp /= steerSettings.wheelAngleCountsPerDegree;
        } else {
          wheelAngleTmp -= steerConfig.wheelAnglePositionZero;
          wheelAngleTmp /= steerConfig.wheelAngleCountsPerDegree;
        }

        steerSetpoints.wheelAngleRaw = wheelAngleTmp;

        if ( steerConfig.wheelAngleSensorType == SteerConfig::WheelAngleSensorType::TieRodDisplacement ) {
          if ( steerConfig.wheelAngleFirstArmLenght != 0 && steerConfig.wheelAngleSecondArmLenght != 0 &&
               steerConfig.wheelAngleTrackArmLenght != 0 && steerConfig.wheelAngleTieRodStroke != 0 ) {

            auto getDisplacementFromAngle = []( float angle ) {
              // a: 2. arm, b: 1. arm, c: abstand drehpunkt wineklsensor und anschlagpunt 2. arm an der spurstange
              // gegenwinkel: winkel zwischen 1. arm und spurstange
              double alpha = PI - radians( angle );

              // winkel zwischen spurstange und 2. arm
              double gamma = PI - alpha - ( asin( steerConfig.wheelAngleFirstArmLenght * sin( alpha ) / steerConfig.wheelAngleSecondArmLenght ) );

              // auslenkung
              return steerConfig.wheelAngleSecondArmLenght * sin( gamma ) / sin( alpha );
            };

            steerSetpoints.wheelAngleCurrentDisplacement = getDisplacementFromAngle( wheelAngleTmp );

            double relativeDisplacementToStraightAhead =
              // real displacement
              steerSetpoints.wheelAngleCurrentDisplacement -
              // calculate middle of displacement -
              ( getDisplacementFromAngle( steerConfig.wheelAngleMinimumAngle ) + ( steerConfig.wheelAngleTieRodStroke / 2 ) );

            wheelAngleTmp = degrees( asin( relativeDisplacementToStraightAhead / steerConfig.wheelAngleTrackArmLenght ) );
          }
        }

        if ( steerConfig.invertWheelAngleSensor ) {
          wheelAngleTmp *= ( float ) -1;
        }

        wheelAngleTmp -= steerConfig.wheelAngleOffset;

        wheelAngleTmp = wheelAngleSensorFilter.step( wheelAngleTmp );
        steerSetpoints.actualSteerAngle = wheelAngleTmp;
        wasAverage += wheelAngleTmp;
      }

    }

    {
      static uint8_t loopCounter = 45;

      if ( loopCounter++ > 99 ) {
        loopCounter = 0;
        {
          Control* handle = ESPUI.getControl( labelWheelAngle );
          String str;
          str.reserve( 30 );

          if ( steerConfig.wheelAngleSensorType == SteerConfig::WheelAngleSensorType::TieRodDisplacement ) {
            str += ( float )steerSetpoints.actualSteerAngle;
            str += "°, Raw ";
            str += ( float )steerSetpoints.wheelAngleRaw;
            str += "°, Displacement ";
            str += ( float )steerSetpoints.wheelAngleCurrentDisplacement;
            str += "mm";
          } else {
            str += ( float )steerSetpoints.actualSteerAngle;
            str += "°, Raw ";
            str += ( float )steerSetpoints.wheelAngleRaw;
            str += "°";
          }

          handle->value = str;
          ESPUI.updateControl( handle );
        }
      }
    }

    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}


void sensorWorker10HzPoller( void* z ) {
  constexpr TickType_t xFrequency = 100;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for ( ;; ) {
    if ( initialisation.inclinoType == SteerConfig::InclinoType::MMA8451 ) {
      sensors_event_t events[32];

      uint8_t numSamples = 0;

      if ( xSemaphoreTake( i2cMutex, 1000 ) == pdTRUE ) {
        numSamples = mma.getEventsFromFifo( events );
        xSemaphoreGive( i2cMutex );
      }

      float fXg, fYg,fZg;


      for ( uint8_t i = 0; i < numSamples; i++ ) {
        fXg = mma8481accFilterX.step( events[i].acceleration.x );
        fYg = mma8481accFilterY.step( events[i].acceleration.y );
        fZg = mma8481accFilterZ.step( events[i].acceleration.z );
      }

      float roll  = ( atan2( -fYg, fZg ) * 180.0 ) / M_PI;
      float pitch = ( atan2( fXg, sqrt( fYg * fYg + fZg * fZg ) ) * 180.0 ) / M_PI;

      imu::Quaternion orientation;
      orientation.fromEuler( roll, pitch, 0 );

      // rotate by the correction
      {
        imu::Quaternion correction;
        correction.fromEuler( radians( steerConfig.mountCorrectionImuRoll ),
                              radians( steerConfig.mountCorrectionImuPitch ),
                              radians( steerConfig.mountCorrectionImuYaw ) );

        orientation = orientation * correction;
      }

      imu::Vector<3> euler = orientation.toEuler();
      euler.toDegrees();

      steerImuInclinometerData.roll = euler[2];
      steerImuInclinometerData.pitch = euler[1];
    }

    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void initSensors() {
  calculateMountingCorrection();

  if ( steerConfig.inclinoType == SteerConfig::InclinoType::MMA8451 ) {
    Control* handle = ESPUI.getControl( labelStatusInclino );

    if ( mma.begin() ) {
      initialisation.inclinoType = SteerConfig::InclinoType::MMA8451;

      handle->value = "MMA8451 found & initialized";
      handle->color = ControlColor::Emerald;

      mma.setRange( MMA8451_RANGE_2_G );
      mma.setDataRate( MMA8451_DATARATE_200_HZ );
      mma.setFifoSettings( MMA8451_FIFO_CIRCULAR );
    } else {
      initialisation.inclinoType = SteerConfig::InclinoType::None;

      handle->value = "MMA8451 not found";
      handle->color = ControlColor::Alizarin;
    }

    ESPUI.updateControl( handle );
  }

  if ( steerConfig.inclinoType == SteerConfig::InclinoType::Fxos8700Fxas21002 ||
       steerConfig.imuType == SteerConfig::ImuType::Fxos8700Fxas21002 ) {

    if ( fxas2100.begin() && fxos8700.begin( ACCEL_RANGE_2G ) ) {

      if ( steerConfig.imuType == SteerConfig::ImuType::Fxos8700Fxas21002 ) {
        initialisation.imuType = steerConfig.imuType;

        Control* handle = ESPUI.getControl( labelStatusImu );
        handle->value = "FXAS2100/FXOS8700 found & initialized";
        handle->color = ControlColor::Emerald;
        ESPUI.updateControl( handle );
      }

      if ( steerConfig.inclinoType == SteerConfig::InclinoType::Fxos8700Fxas21002 ) {
        initialisation.inclinoType = steerConfig.inclinoType;

        Control* handle = ESPUI.getControl( labelStatusInclino );
        handle->value = "FXAS2100/FXOS8700 found & initialized";
        handle->color = ControlColor::Emerald;
        ESPUI.updateControl( handle );
      }

      ahrs.begin( 100 );
    } else {


      if ( steerConfig.imuType == SteerConfig::ImuType::Fxos8700Fxas21002 ) {
        initialisation.imuType = SteerConfig::ImuType::None;

        Control* handle = ESPUI.getControl( labelStatusImu );
        handle->value = "FXAS2100/FXOS8700 not found";
        handle->color = ControlColor::Alizarin;
        ESPUI.updateControl( handle );
      }

      if ( steerConfig.inclinoType == SteerConfig::InclinoType::Fxos8700Fxas21002 ) {
        initialisation.inclinoType = SteerConfig::InclinoType::None;

        Control* handle = ESPUI.getControl( labelStatusInclino );
        handle->value = "FXAS2100/FXOS8700 not found";
        handle->color = ControlColor::Alizarin;
        ESPUI.updateControl( handle );
      }
    }

  }

  // initialise ads1115 everytime, even if not avaible (no answer in the init -> just sending)
  {
    ads.setGain( GAIN_TWOTHIRDS );   // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
    // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
    // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
    // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
    // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
    // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

    ads.begin();
    ads.setSPS( ADS1115_DR_860SPS );

    Control* handle = ESPUI.getControl( labelStatusAdc );
    handle->value = "ADC1115 initialized";
    handle->color = ControlColor::Emerald;
    initialisation.wheelAngleInput = steerConfig.wheelAngleInput;
    ESPUI.updateControl( handle );
  }

  if ( steerConfig.inclinoType == SteerConfig::InclinoType::MMA8451 ) {
    xTaskCreate( sensorWorker10HzPoller, "sensorWorker10HzPoller", 4096, NULL, 5, NULL );
  }
  if ( steerConfig.imuType == SteerConfig::ImuType::Fxos8700Fxas21002 ||  steerConfig.imuType == SteerConfig::ImuType::LSM9DS1) {
    xTaskCreate( sensorWorkerImuPoller, "sensorWorkerImuPoller", 8192, NULL, 5, NULL );
  }

  xTaskCreate( sensorWorkerSteeringPoller, "sensorWorkerSteeringPoller", 8192 * 2, NULL, 6, NULL );
}
