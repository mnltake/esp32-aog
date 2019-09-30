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

#include <stdio.h>
#include <string.h>

#include "main.hpp"

#if defined(ESP32)
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif

#include <DNSServer.h>
#include <ESPUI.h>
#include <EEPROM32_Rotate.h>
#include <esp32-hal-adc.h>


///////////////////////////////////////////////////////////////////////////
// global data
///////////////////////////////////////////////////////////////////////////
SteerConfig steerConfig, steerConfigDefaults;
Initialisation initialisation;
SteerCanData steerCanData = {0};

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
SemaphoreHandle_t i2cMutex;

const byte DNS_PORT = 53;
IPAddress apIP( 192, 168, 1, 1 );

uint16_t labelLoad;
uint16_t labelOrientation;
uint16_t labelWheelAngle;
uint16_t buttonReset;
uint16_t textNmeaToSend;

uint16_t labelWheelAngleDisplacement;

uint16_t labelStatusOutput;
uint16_t labelStatusAdc;
uint16_t labelStatusCan;
uint16_t labelStatusImu;
uint16_t labelStatusInclino;
uint16_t labelStatusGps;
uint16_t labelStatusNtrip;

uint32_t statusLedPattern = 0b11100000111000001110000011100000;
uint8_t fxl6408_outputRegister;
bool networkConnected = false;

///////////////////////////////////////////////////////////////////////////
// external Libraries
///////////////////////////////////////////////////////////////////////////
EEPROM32_Rotate EEPROM;

ESPUIClass ESPUI( Verbosity::Quiet );
DNSServer dnsServer;


///////////////////////////////////////////////////////////////////////////
// helper functions
///////////////////////////////////////////////////////////////////////////
void setResetButtonToRed() {
  ESPUI.getControl( buttonReset )->color = ControlColor::Alizarin;
  ESPUI.updateControl( buttonReset );
}

void writeEeprom() {
  EEPROM.writeUChar( ( uint16_t )EepromAddresses::Validator, 0 );
  EEPROM.writeUShort( ( uint16_t )EepromAddresses::SizeOfConfig, ( uint16_t )sizeof( SteerConfig ) );
  EEPROM.put( ( uint16_t )EepromAddresses::GenericImuCalibrationData, genericimucalibrationdata );
  EEPROM.put( ( uint16_t )EepromAddresses::SteerConfig, steerConfig );
  EEPROM.commit();
}

void statusLedWorker( void* z ) {
  uint8_t position = 0;

  while ( 1 ) {
    // shift a bit and get first bit
    bool ledState = ( statusLedPattern >> position ) & 1;
    FXL6408_setDigitalOutput(2, ledState);

    // increase counter
    position += 1;
    if (position > 31) {
      position = 0;
    }
    // Wait
    vTaskDelay( 62 / portTICK_PERIOD_MS );
  }
}


bool FXL6408_init() {
  int returnValues = 0;
  returnValues += FXL6408_setByteI2C(0x01, 0b00000001); // reset
  delay(5);                                             // enough time for reset
  returnValues += FXL6408_setByteI2C(0x07, 0b11111111); // Output High-Z (not driven)
  returnValues += FXL6408_setByteI2C(0x03, 0b11111111); // Everything Output
  returnValues += FXL6408_setByteI2C(0x05, 0b00000000); // (Disabled) Outputs to low)
  returnValues += FXL6408_setByteI2C(0x0B, 0b00000000); // No Pullup/down
  returnValues += FXL6408_setByteI2C(0x11, 0b11111111); // No interrupts
  return returnValues == 0;
}

// configure a port, default nothing enabled
bool FXL6408_configureAsDigitalInput(uint8_t port, bool usePullUpDown, bool pullDirectionUp) {
  int returnValues = 0;
  // pullUp/Down
  returnValues += FXL6408_setByteI2C(0x0D, FXL6408_setBit(FXL6408_getByteI2C(0x0D), port, pullDirectionUp));
  returnValues += FXL6408_setByteI2C(0x0B, FXL6408_setBit(FXL6408_getByteI2C(0x0B), port, usePullUpDown));
  // direction
  returnValues += FXL6408_setByteI2C(0x03, FXL6408_setBit(FXL6408_getByteI2C(0x03), port, false));

  return returnValues == 0;
}

bool FXL6408_configureAsDigitalOutput(uint8_t port) {
  int returnValues = 0;
  // default low
  fxl6408_outputRegister = FXL6408_setBit(fxl6408_outputRegister, port, false);
  returnValues += FXL6408_setByteI2C(0x05, fxl6408_outputRegister);
  // disable High-Z
  returnValues += FXL6408_setByteI2C(0x07, FXL6408_setBit(FXL6408_getByteI2C(0x07), port, false));
  // direction
  returnValues += FXL6408_setByteI2C(0x03, FXL6408_setBit(FXL6408_getByteI2C(0x03), port, true));

  return returnValues == 0;
}

bool FXL6408_getDigitalInput(uint8_t port)  {
  uint8_t value = FXL6408_getByteI2C(0x0F);
  value = (value >> port) & 1; // shift so the port is at last bit, then mask
  return value == 1;
}

void FXL6408_setDigitalOutput(uint8_t port, bool state) {
  uint8_t old = fxl6408_outputRegister;
  fxl6408_outputRegister = FXL6408_setBit(fxl6408_outputRegister, port, state);
  if (old != fxl6408_outputRegister) {
    FXL6408_setByteI2C(0x05, fxl6408_outputRegister);
  }
};

uint8_t FXL6408_getByteI2C(int i2cregister) {
  uint8_t result = 255;
  if ( xSemaphoreTake( i2cMutex, 1000 ) == pdTRUE ) {
    Wire.beginTransmission(0x43);
    Wire.write(i2cregister);
    Wire.endTransmission(false);
    Wire.requestFrom(0x43, 1, (int)true);
    result = Wire.read();
    xSemaphoreGive( i2cMutex );
  } else {
    Serial.println( "FXL6408_getByteI2C - Failed to get i2c mutex" );
  }
  return result;
}

uint8_t FXL6408_setByteI2C(int i2cregister, byte value) {
  uint8_t result = 255;
  if ( xSemaphoreTake( i2cMutex, 1000 ) == pdTRUE ) {
    Wire.beginTransmission(0x43);
    Wire.write(i2cregister);
    Wire.write(value);
    result = Wire.endTransmission();
    xSemaphoreGive( i2cMutex );
  }else {
    Serial.println( "FXL6408_setByteI2C - Failed to get i2c mutex" );
  }
  return result;
}

uint8_t FXL6408_setBit(uint8_t byte, uint8_t position, bool value) {
uint8_t pattern = 0b00000001 << position;
if (value) {
  return byte | pattern;
}  else {
  pattern = ~pattern;
  return byte & pattern;
}
}



///////////////////////////////////////////////////////////////////////////
// Application
///////////////////////////////////////////////////////////////////////////
void setup( void ) {

//   Serial.begin( 921600 );
  Serial.begin( 115200 );
  Serial.println( "Setup()" );

  // add all the partitions for the EEPROM emulation
  EEPROM.add_by_subtype( 0x99 );
  EEPROM.begin( 4096 );

  //restore the settings from EEPROM
  if ( ( EEPROM.readUChar( ( uint16_t )EepromAddresses::Validator ) != 0xff ) &&
       ( EEPROM.readUShort( ( uint16_t )EepromAddresses::SizeOfConfig ) == sizeof( SteerConfig ) ) ) {
    Serial.println( "Read from EEPROM" );
    EEPROM.get( ( uint16_t )EepromAddresses::GenericImuCalibrationData, genericimucalibrationdata );
    EEPROM.get( ( uint16_t )EepromAddresses::SteerConfig, steerConfig );
  } else {
    Serial.println( "Not read from EEPROM" );
    writeEeprom();
  }

  // Init I2C
  //Wire.begin( ( int )steerConfig.gpioSDA, ( int )steerConfig.gpioSCL, steerConfig.i2cBusSpeed );
  Wire.begin(32, 33, 400000 );
  delay(10);
  i2cMutex = xSemaphoreCreateMutex();

  // Init FXL6408
  if (FXL6408_init() == false) {
    Serial.println( "Init FXL6408 failed" );
  }

  // start Status-led
  FXL6408_configureAsDigitalOutput(2);
  FXL6408_setDigitalOutput(2,true);

  // Init Serial Ports (normal F9P and RS232)
// try some magic
gpio_pad_select_gpio(GPIO_NUM_14);
gpio_set_direction(GPIO_NUM_14, GPIO_MODE_INPUT);
gpio_set_pull_mode(GPIO_NUM_14, GPIO_FLOATING);
gpio_pad_select_gpio(GPIO_NUM_13);
gpio_set_direction(GPIO_NUM_13, GPIO_MODE_OUTPUT);
gpio_set_pull_mode(GPIO_NUM_13, GPIO_FLOATING);
  Serial2.begin(steerConfig.rtkCorrectionBaudrate, SERIAL_8N1, 14, 13);
  Serial1.begin(57600, SERIAL_8N1, 16, 15);

  // put the PWM Port of the second VNH7070AS to high => two outputs
  digitalWrite(12, 1);

  #if defined(ESP32)
    WiFi.setHostname( steerConfig.hostname );
  #else
    WiFi.hostname( steerConfig.hostname );
  #endif

    // disable Ethernet
    FXL6408_configureAsDigitalOutput(1);
    FXL6408_setDigitalOutput(1, false);

    // set up "setup" Switch
    FXL6408_configureAsDigitalInput(0, true, true);


  // try to connect to existing network
//  WiFi.onEvent(WiFiEvent);
    WiFi.begin( steerConfig.ssid, steerConfig.password );
    Serial.print( "\n\nTry to connect to existing network " );
    Serial.print( steerConfig.ssid );
    Serial.print( " with password " );
    Serial.print( steerConfig.password );
    // TODO
//    FXL6408_setDigitalOutput(1, true);
//    ETH.begin(0, -1, 23, 18, ETH_PHY_LAN8720, ETH_CLOCK_GPIO17_OUT);
//  }

  {
    // Wait for connection,or button
    do {
      FXL6408_setDigitalOutput(2,false);
      delay( 62 );
      FXL6408_setDigitalOutput(2,true);
      delay ( 62 );
      Serial.print( "." );
//    } while ( networkConnected == false && FXL6408_getDigitalInput(0));
  } while ( FXL6408_getDigitalInput(0) && WiFi.status() != WL_CONNECTED);

    // not connected -> create hotspot
    if ( WiFi.status() != WL_CONNECTED ) {
      Serial.print( "\n\nCreating hotspot" );
      WiFi.begin( steerConfig.ssid, steerConfig.password );
      WiFi.mode( WIFI_AP );
      WiFi.softAPConfig( apIP, apIP, IPAddress( 255, 255, 255, 0 ) );
      WiFi.softAP( steerConfig.hostname );

      for (int i = 0; i <= 10; i++ ){
        FXL6408_setDigitalOutput(2,false);
        delay( 62 );
        FXL6408_setDigitalOutput(2,true);
        delay ( 62 );
      }
      statusLedPattern = 0b00000000111111110000000011111111;
    } else {
      statusLedPattern = 0b11111111111111100111111111111111;
    }
  }

  dnsServer.start( DNS_PORT, "*", apIP );

  Serial.println( "\n\nWiFi parameters:" );
  Serial.print( "Mode: " );
  Serial.println( WiFi.getMode() == WIFI_AP ? "Station" : "Client" );
  Serial.print( "IP address: " );
  Serial.println( WiFi.getMode() == WIFI_AP ? WiFi.softAPIP() : WiFi.localIP() );


  labelLoad       = ESPUI.addControl( ControlType::Label, "Load:", "", ControlColor::Turquoise );
  labelOrientation = ESPUI.addControl( ControlType::Label, "Orientation:", "", ControlColor::Emerald );
  labelWheelAngle = ESPUI.addControl( ControlType::Label, "Wheel Angle:", "0°", ControlColor::Emerald );
//   graphWheelAngle = ESPUI.addControl( ControlType::Graph, "Wheel Angle:", "", ControlColor::Emerald );

  buttonReset = ESPUI.addControl( ControlType::Button, "Store the Settings", "Apply", ControlColor::Emerald, Control::noParent,
  []( Control * control, int id ) {
    if ( id == B_UP ) {
      writeEeprom();
    }
  } );

  buttonReset = ESPUI.addControl( ControlType::Button, "If this turn red, you have to", "Apply & Reboot", ControlColor::Emerald, Control::noParent,
  []( Control * control, int id ) {
    if ( id == B_UP ) {
      writeEeprom();
      ESP.restart();
    }
  } );

  // Status Tab
  {
    uint16_t tab  = ESPUI.addControl( ControlType::Tab, "Status", "Status" );

    labelStatusOutput   = ESPUI.addControl( ControlType::Label, "Output:", "No Output configured", ControlColor::Turquoise, tab );
    labelStatusAdc      = ESPUI.addControl( ControlType::Label, "ADC:", "No ADC configured", ControlColor::Turquoise, tab );
    labelStatusCan      = ESPUI.addControl( ControlType::Label, "CAN:", "No CAN BUS configured", ControlColor::Turquoise, tab );
    labelStatusImu      = ESPUI.addControl( ControlType::Label, "IMU:", "No IMU configured", ControlColor::Turquoise, tab );
    labelStatusInclino  = ESPUI.addControl( ControlType::Label, "Inclinometer:", "No Inclinometer configured", ControlColor::Turquoise, tab );
    labelStatusGps      = ESPUI.addControl( ControlType::Label, "GPS:", "Not configured", ControlColor::Turquoise, tab );
    labelStatusNtrip    = ESPUI.addControl( ControlType::Label, "NTRIP:", "Not configured", ControlColor::Turquoise, tab );
  }

  // Info Tab
  {
    uint16_t tab  = ESPUI.addControl( ControlType::Tab, "Info/Help", "Info/Help" );
    ESPUI.addControl( ControlType::Label, "Attention:", "As this WebUI is great and looks good, it is quite taxing on the controller. So close the browser (actualy closing the tab, not minimizing) after you are finished with configuring. This helps with spurious crashes.", ControlColor::Carrot, tab );

    ESPUI.addControl( ControlType::Label, "Basics:", "This works with setting up the different options in the panels. If an option requires a reboot (indicated by the darker blue and an asterisk after the title), press on the button \"Apply & Reboot\" and refresh the page after some time, usually 5-10 seconds. Settings with the lighter shade of blue are applied immediately, but are not saved to the permanent memory. You can do this with the \"Apply\" button. If the values are complete garbage or you want a fresh start, set the config to defaults in the \"Configurations\" tab.", ControlColor::Carrot, tab );

    ESPUI.addControl( ControlType::Label, "Network:", "Here the network is configured. Leave it on defaults, only if used as roof controller (only GPS + IMU), set \"Port to send from\" to 5544.", ControlColor::Turquoise, tab );
    ESPUI.addControl( ControlType::Label, "CAN Bus/J1939:", "Enable if used. To use data from the vehicle bus as workswitch, configure it in the next tab.", ControlColor::Turquoise, tab );
    ESPUI.addControl( ControlType::Label, "Work- and Steerswitch:", "If work- and steerswitches as physical inputs are used, enable them by configuring a GPIO. If you want to use the CAN-bus (J1939), set the type to a hitch position or RPM with a threshold.", ControlColor::Turquoise, tab );
    ESPUI.addControl( ControlType::Label, "Wheel Angle Sensor:", "To enable the wheel angle sensor, configure the input first. If you use two arms connected to the tie rod, measure them exactly and configure the values. This is to calculate out the unlinearities.", ControlColor::Turquoise, tab );
    ESPUI.addControl( ControlType::Label, "Steering:", "Set up the type and the GPIOs", ControlColor::Turquoise, tab );
    ESPUI.addControl( ControlType::Label, "Steering PID:", "This controller uses its own PID-controller. No values are taken over from AOG, so everything is entered here.", ControlColor::Turquoise, tab );
    ESPUI.addControl( ControlType::Label, "Sensors:", "Here the IMU and inclinometer are set up. The Mounting Correction is entered as three angles relative to the tractor axis, so the IMU can be mounted in every position, as long as the chips are positioned relative to each other with no difference (normaly, the manufacturer of the sensor pcb does this anyway). The FXAS2100/FXOS8700-combo is recomned, as they give the most precise roll/pitch/heading with the least amount of calibration.", ControlColor::Turquoise, tab );
    ESPUI.addControl( ControlType::Label, "NTRIP/GPS:", "Here the connection to the GPS is set up, also the NTRIP-client. Usualy, you want to send the data to AOG via UDP, a serial connection via USB is also possible. The TCP-Socket enables 3rd-party GPS-Software and configuring the GPS-Receiver with u-center.", ControlColor::Turquoise, tab );
  }

  // Network Tab
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "Network", "Network" );

    ESPUI.addControl( ControlType::Text, "SSID*", String( steerConfig.ssid ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      control->value.toCharArray( steerConfig.ssid, sizeof( steerConfig.ssid ) );
      setResetButtonToRed();
    } );
    ESPUI.addControl( ControlType::Text, "Password*", String( steerConfig.password ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      control->value.toCharArray( steerConfig.password, sizeof( steerConfig.password ) );
      setResetButtonToRed();
    } );
    ESPUI.addControl( ControlType::Text, "Hostname*", String( steerConfig.hostname ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      control->value.toCharArray( steerConfig.hostname, sizeof( steerConfig.hostname ) );
      setResetButtonToRed();
    } );
    ESPUI.addControl( ControlType::Number, "Port to send from*", String( steerConfig.portSendFrom ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      steerConfig.portSendFrom = control->value.toInt();
      setResetButtonToRed();
    } );
    ESPUI.addControl( ControlType::Number, "Port to send to*", String( steerConfig.portSendTo ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      steerConfig.portSendTo = control->value.toInt();
      setResetButtonToRed();
    } );
    ESPUI.addControl( ControlType::Number, "Port to listen to*", String( steerConfig.portListenTo ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      steerConfig.portListenTo = control->value.toInt();
      setResetButtonToRed();
    } );
  }

  // CAN Bus
  {
    uint16_t tab  = ESPUI.addControl( ControlType::Tab, "CAN Bus/J1939", "CAN Bus/J1939" );

    ESPUI.addControl( ControlType::Switcher, "CAN Bus Enabled*", steerConfig.canBusEnabled ? "1" : "0", ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      steerConfig.canBusEnabled = control->value.toInt() == 1;
      setResetButtonToRed();
    } );

    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Bus Speed*", String( ( int )steerConfig.canBusSpeed ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.canBusSpeed = ( SteerConfig::CanBusSpeed )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "250kB/s", "250", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "500kB/s", "500", ControlColor::Alizarin, sel );
    }
  }

  // Switches/Buttons Tab
  {
    uint16_t tab  = ESPUI.addControl( ControlType::Tab, "Work- and Steerswitch", "Work- and Steerswitch" );

    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Workswitch Type", String( ( int )steerConfig.workswitchType ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.workswitchType = ( SteerConfig::WorkswitchType )control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Option, "None", "0", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Gpio", "1", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Rear Hitch Position (from Can Bus)", "2", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Front Hitch Position (from Can Bus)", "3", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Rear Pto Rpm (from Can Bus)", "4", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Front Pto Rpm (from Can Bus)", "5", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Motor Rpm (from Can Bus)", "6", ControlColor::Alizarin, sel );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Hitch Threshold",  String( steerConfig.canBusHitchThreshold ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.canBusHitchThreshold = control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "100", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Hitch Threshold Hysteresis",  String( steerConfig.canBusHitchThresholdHysteresis ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.canBusHitchThresholdHysteresis = control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "100", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "RPM Threshold",  String( steerConfig.canBusHitchThreshold ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.canBusHitchThreshold = control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "3500", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "RPM Threshold Hysteresis",  String( steerConfig.canBusHitchThresholdHysteresis ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.canBusHitchThresholdHysteresis = control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "1000", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }
//     {
//       uint16_t sel = ESPUI.addControl( ControlType::Select, "Workswitch LED*", String( ( int )steerConfig.gpioWorkswitch ), ControlColor::Wetasphalt, tab,
//       []( Control * control, int id ) {
//         steerConfig.gpioWorkswitch = ( SteerConfig::Gpio )control->value.toInt();
//         setResetButtonToRed();
//       } );
//       ESPUI.addControl( ControlType::Option, "None", "0", ControlColor::Alizarin, sel );
//       addGpioOutput( sel );
//     }

    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Autosteer switch/button Gpio*", String( ( int )steerConfig.gpioSteerswitch ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.gpioSteerswitch = ( SteerConfig::Gpio )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "None", "0", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "ESP32 I1", String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio36 ), ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "ESP32 I2", String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio39 ), ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "ESP32 I3", String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio34 ), ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "ADS1115 A0", String( ( uint8_t )SteerConfig::AnalogIn::ADS1115A0Single ), ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "ADS1115 A1", String( ( uint8_t )SteerConfig::AnalogIn::ADS1115A1Single ), ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "ADS1115 A2", String( ( uint8_t )SteerConfig::AnalogIn::ADS1115A2Single ), ControlColor::Alizarin, sel );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Auto recognise Autosteer GPIO as Switch [ms]",  String( steerConfig.autoRecogniseSteerGpioAsSwitchOrButton ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.autoRecogniseSteerGpioAsSwitchOrButton = control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "16000", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "100", ControlColor::Peterriver, num );
    }
    {
      ESPUI.addControl( ControlType::Switcher, "Steerswitch Active Low", steerConfig.steerswitchActiveLow ? "1" : "0", ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steerswitchActiveLow = control->value.toInt() == 1;
      } );
    }
//     {
//       uint16_t sel = ESPUI.addControl( ControlType::Select, "Autosteer LED*", String( ( int )steerConfig.gpioSteerswitch ), ControlColor::Wetasphalt, tab,
//       []( Control * control, int id ) {
//         steerConfig.gpioSteerswitch = ( SteerConfig::Gpio )control->value.toInt();
//         setResetButtonToRed();
//       } );
//       ESPUI.addControl( ControlType::Option, "None", "0", ControlColor::Alizarin, sel );
//       addGpioOutput( sel );
//     }


    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Workswitch Gpio*", String( ( int )steerConfig.gpioWorkswitch ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.gpioWorkswitch = ( SteerConfig::Gpio )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "None", "0", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "ESP32 I1", String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio36 ), ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "ESP32 I2", String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio39 ), ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "ESP32 I3", String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio34 ), ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "ADS1115 A0", String( ( uint8_t )SteerConfig::AnalogIn::ADS1115A0Single ), ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "ADS1115 A1", String( ( uint8_t )SteerConfig::AnalogIn::ADS1115A1Single ), ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "ADS1115 A2", String( ( uint8_t )SteerConfig::AnalogIn::ADS1115A2Single ), ControlColor::Alizarin, sel );
    }
    {
      ESPUI.addControl( ControlType::Switcher, "Workswitch Active Low", steerConfig.workswitchActiveLow ? "1" : "0", ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.workswitchActiveLow = control->value.toInt() == 1;
      } );
    }

  }

  // Wheel Angle Sensor Tab
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "Wheel Angle Sensor", "Wheel Angle Sensor" );

    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Wheel Angle Sensor*", String( ( int )steerConfig.wheelAngleInput ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAngleInput = ( SteerConfig::AnalogIn )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "ESP32 I1", String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio36 ), ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "ESP32 I2", String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio39 ), ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "ESP32 I3", String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio34 ), ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "ADS1115 A0", String( ( uint8_t )SteerConfig::AnalogIn::ADS1115A0Single ), ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "ADS1115 A1", String( ( uint8_t )SteerConfig::AnalogIn::ADS1115A1Single ), ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "ADS1115 A2", String( ( uint8_t )SteerConfig::AnalogIn::ADS1115A2Single ), ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "ADS1115 A0/A1 differential", String( ( uint8_t )SteerConfig::AnalogIn::ADS1115A0A1Differential ), ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "None", "0", ControlColor::Alizarin, sel );
    }

    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Wheel Angle Sensor Type", String( ( int )steerConfig.wheelAngleSensorType ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAngleSensorType = ( SteerConfig::WheelAngleSensorType )control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Option, "Direct Wheel Angle", "0", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Two Arms connected to tie rod", "1", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Ackermann (sensor left)", "2", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Ackermann (sensor right)", "3", ControlColor::Alizarin, sel );
    }

    ESPUI.addControl( ControlType::Switcher, "Allow AgOpenGPS to overwrite Counts per Degree and Steer Angle Center (not recomned)", steerConfig.allowWheelAngleCenterAndCountsOverwrite ? "1" : "0", ControlColor::Peterriver, tab,
    []( Control * control, int id ) {
      steerConfig.allowWheelAngleCenterAndCountsOverwrite = control->value.toInt() == 1;
    } );

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Wheel Angle Sensor Center", String( steerConfig.wheelAnglePositionZero ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAnglePositionZero = control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "26000", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Wheel Angle Counts per Degree", String( steerConfig.wheelAngleCountsPerDegree ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAngleCountsPerDegree = control->value.toFloat();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "250", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.1", ControlColor::Peterriver, num );
    }

    ESPUI.addControl( ControlType::Switcher, "Invert Wheel Angle Sensor", steerConfig.invertWheelAngleSensor ? "1" : "0", ControlColor::Peterriver, tab,
    []( Control * control, int id ) {
      steerConfig.invertWheelAngleSensor = control->value.toInt() == 1;
    } );

    {
      uint16_t num = ESPUI.addControl( ControlType::Slider, "Wheel Angle Offset", String( steerConfig.wheelAngleOffset ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAngleOffset = control->value.toFloat();
      } );
      ESPUI.addControl( ControlType::Min, "Roll Min", "-40", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Roll Max", "40", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Roll Step", "0.1", ControlColor::Peterriver, num );
    }

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "1. Arm connect to sensor (mm) / Wheelbase (cm)", String( steerConfig.wheelAngleFirstArmLenght ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAngleFirstArmLenght = control->value.toFloat();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "1000", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "2. Arm connect to tie rod (mm) / Track width (cm)", String( steerConfig.wheelAngleSecondArmLenght ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAngleSecondArmLenght = control->value.toFloat();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "1000", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Tie rod stroke (mm)", String( steerConfig.wheelAngleTieRodStroke ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAngleTieRodStroke = control->value.toFloat();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "500", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Minimum Angle of wheel angle sensor", String( steerConfig.wheelAngleMinimumAngle ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAngleMinimumAngle = control->value.toFloat();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "180", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Lenght of Track Arm (mm)", String( steerConfig.wheelAngleTrackArmLenght ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAngleTrackArmLenght = control->value.toFloat();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "500", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }
  }

  // Steering Tab
  {
    uint16_t tab  = ESPUI.addControl( ControlType::Tab, "Steering", "Steering" );

    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Output Type*", String( ( int )steerConfig.outputType ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.outputType = ( SteerConfig::OutputType )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "None", "0", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Motor: VNH7070AS", "5", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Hydraulic: VNH7070AS + PWM 2-Coil Valve", "6", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Hydraulic: VNH7070AS + Danfoss Valve PVE A/H/M", "7", ControlColor::Alizarin, sel );
    }

    ESPUI.addControl( ControlType::Switcher, "Invert Output", steerConfig.invertOutput ? "1" : "0", ControlColor::Peterriver, tab,
    []( Control * control, int id ) {
      steerConfig.invertOutput = control->value.toInt() == 1;
    } );

    {
      uint16_t num = ESPUI.addControl( ControlType::Slider, "PWM Frequency", String( steerConfig.pwmFrequency ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.pwmFrequency = control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "500", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "18000", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "100", ControlColor::Peterriver, num );
    }
  }

  // Steering PID Tab
  {
    uint16_t tab  = ESPUI.addControl( ControlType::Tab, "Steering PID", "Steering PID" );

//     ESPUI.addControl( ControlType::Switcher, "Allow AgOpenGPS to overwrite PID values", steerConfig.allowPidOverwrite ? "1" : "0", ControlColor::Peterriver, tab,
//     []( Control * control, int id ) {
//       steerConfig.allowPidOverwrite = control->value.toInt() == 1;
//     } );

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "PID Kp", String( steerConfig.steeringPidKp, 4 ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringPidKp = control->value.toDouble();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "50", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.1", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "PID Ki", String( steerConfig.steeringPidKi, 4 ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringPidKi = control->value.toDouble();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "50", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.01", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "PID Kd", String( steerConfig.steeringPidKd, 4 ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringPidKd = control->value.toDouble();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "50", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.01", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Automatic Bang On Factor (multiple of saturation with Kp, 0 to turn off)", String( steerConfig.steeringPidAutoBangOnFactor, 4 ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringPidAutoBangOnFactor = control->value.toDouble();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "10", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.1", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Turn Output on if error is greater (BangOn)", String( steerConfig.steeringPidBangOn, 4 ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringPidBangOn = control->value.toDouble();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "50", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.01", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Turn Output off if error is smaller (BangOff)", String( steerConfig.steeringPidBangOff, 4 ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringPidBangOff = control->value.toDouble();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "50", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.01", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Minimum PWM", String( steerConfig.steeringPidMinPwm ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringPidMinPwm = control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "255", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }

//     {
//       uint16_t num = ESPUI.addControl( ControlType::Number, "Distance from Line to turn Integral/Derivative of PID off", String( steerConfig.steeringPidDflTurnIdOff ), ControlColor::Peterriver, tab,
//       []( Control * control, int id ) {
//         steerConfig.steeringPidDflTurnIdOff = control->value.toInt();
//       } );
//       ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
//       ESPUI.addControl( ControlType::Max, "Max", "200", ControlColor::Peterriver, num );
//       ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
//     }
  }

  // Sensors Tab
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "Sensors", "Sensors" );

    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "IMU*", String( ( int )steerConfig.imuType ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.imuType = ( SteerConfig::ImuType )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "No IMU", "0", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Fxos8700Fxas21002", "2", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "LSM9DS1", "3", ControlColor::Alizarin, sel );
    }
    { // calibrate gyros
      ESPUI.addControl( ControlType::Button, "Gyro calibration (30s)", "Start", ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        if ( id == B_UP ) {
          statusLedPattern = 0b11111111111111110000000000000000;
          steerImuInclinometerData.gyroCalibration = true;
        }
      } );
    }

    {
      ESPUI.addControl( ControlType::Switcher, "MagnetometerCalibration", steerImuInclinometerData.magCalibration ? "1" : "0", ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        if (steerImuInclinometerData.magCalibration){
          statusLedPattern = 0b11111111111111100111111111111111;
          genericImuCalibrationCalcMagnetometer();
        } else if ( !steerImuInclinometerData.magCalibration )
        steerImuInclinometerData.magCalibration = control->value.toInt() == 1;
        statusLedPattern = 0b11111111111111110000000000000000;
      } );
    }

    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Inclinometer*", String( ( int )steerConfig.inclinoType ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.inclinoType = ( SteerConfig::InclinoType )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "No Inclinometer", "0", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "MMA8451", "1", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Fxos8700Fxas21002", "2", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "LSM9DS1", "3", ControlColor::Alizarin, sel );
    }

    {
      ESPUI.addControl( ControlType::Switcher, "Invert Roll Axis (enable for older versions of AgOpenGPS)", steerConfig.invertRoll ? "1" : "0", ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.invertRoll = control->value.toInt() == 1;
      } );
    }

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Mounting Correction (Roll) of Imu", String( steerConfig.mountCorrectionImuRoll ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.mountCorrectionImuRoll = control->value.toFloat();
        calculateMountingCorrection();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "-180", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "180", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.05", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Mounting Correction (Pitch) of Imu", String( steerConfig.mountCorrectionImuPitch ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.mountCorrectionImuPitch = control->value.toFloat();
        calculateMountingCorrection();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "-180", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "180", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.05", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Mounting Correction (Yaw) of Imu", String( steerConfig.mountCorrectionImuYaw ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.mountCorrectionImuYaw = control->value.toFloat();
        calculateMountingCorrection();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "-180", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "180", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.05", ControlColor::Peterriver, num );
    }
  }

//   // Steering Wheel Encoder Tab
//   {
//     uint16_t tab  = ESPUI.addControl( ControlType::Tab, "Steering Wheel Encoder", "Steering Wheel Encoder" );
//
//     ESPUI.addControl( ControlType::Switcher, "Steering Wheel Encoder*", steerConfig.steeringWheelEncoder ?"1" : "0", ControlColor::Wetasphalt, tab,
//     []( Control * control, int id ) {
//       steerConfig.steeringWheelEncoder = control->value.toInt() == 1;
//       setResetButtonToRed();
//     } );
//
//     {
//       uint16_t sel = ESPUI.addControl( ControlType::Select, "Steering Wheel Encoder Input A*", String( ( int )steerConfig.gpioWheelencoderA ), ControlColor::Wetasphalt, tab,
//       []( Control * control, int id ) {
//         steerConfig.gpioWheelencoderA = ( SteerConfig::Gpio )control->value.toInt();
//         setResetButtonToRed();
//       } );
//       ESPUI.addControl( ControlType::Option, "None", "0", ControlColor::Alizarin, sel );
//       addGpioInput( sel );
//       addGpioOutput( sel );
//     }
//     {
//       uint16_t sel = ESPUI.addControl( ControlType::Select, "Steering Wheel Encoder Input B*", String( ( int )steerConfig.gpioWheelencoderB ), ControlColor::Wetasphalt, tab,
//       []( Control * control, int id ) {
//         steerConfig.gpioWheelencoderB = ( SteerConfig::Gpio )control->value.toInt();
//         setResetButtonToRed();
//       } );
//       ESPUI.addControl( ControlType::Option, "None", "0", ControlColor::Alizarin, sel );
//       addGpioInput( sel );
//       addGpioOutput( sel );
//     }
//
//     ESPUI.addControl( ControlType::Number, "Steering Wheel Encoder max Counts", String( steerConfig.wheelEncoderPulseCountMax ), ControlColor::Peterriver, tab,
//     []( Control * control, int id ) {
//       steerConfig.wheelEncoderPulseCountMax = control->value.toInt();
//     } );
//   }

  // NTRIP/GPS Tab
  {
    uint16_t tab     = ESPUI.addControl( ControlType::Tab, "NTRIP/GPS", "NTRIP/GPS" );

    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "RTK Correction*", String( ( int )steerConfig.rtkCorrectionType ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.rtkCorrectionType = ( SteerConfig::RtkCorrectionType )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "No Correction", "0", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "NTRIP", "1", ControlColor::Alizarin, sel );
//       ESPUI.addControl( ControlType::Option, "UDP", "2", ControlColor::Alizarin, sel );
//       ESPUI.addControl( ControlType::Option, "TCP", "3", ControlColor::Alizarin, sel );
    }

    ESPUI.addControl( ControlType::Text, "Server*", String( steerConfig.rtkCorrectionServer ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      control->value.toCharArray( steerConfig.rtkCorrectionServer, sizeof( steerConfig.rtkCorrectionServer ) );
      setResetButtonToRed();
    } );
    ESPUI.addControl( ControlType::Text, "Username*", String( steerConfig.rtkCorrectionUsername ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      control->value.toCharArray( steerConfig.rtkCorrectionUsername, sizeof( steerConfig.rtkCorrectionUsername ) );
      setResetButtonToRed();
    } );
    ESPUI.addControl( ControlType::Text, "Password*", String( steerConfig.rtkCorrectionPassword ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      control->value.toCharArray( steerConfig.rtkCorrectionPassword, sizeof( steerConfig.rtkCorrectionPassword ) );
      setResetButtonToRed();
    } );
    ESPUI.addControl( ControlType::Text, "Mountpoint*", String( steerConfig.rtkCorrectionMountpoint ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      control->value.toCharArray( steerConfig.rtkCorrectionMountpoint, sizeof( steerConfig.rtkCorrectionMountpoint ) );
      setResetButtonToRed();
    } );
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Port*",  String( steerConfig.rtkCorrectionPort ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.rtkCorrectionPort = control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "1", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "65535", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }

    {
      uint16_t baudrate = ESPUI.addControl( ControlType::Select, "Baudrate", String( steerConfig.rtkCorrectionBaudrate ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        uint32_t baudrate = control->value.toInt();
        steerConfig.rtkCorrectionBaudrate = baudrate;
        Serial2.updateBaudRate( baudrate );
      } );
      ESPUI.addControl( ControlType::Option, "4800", "4800", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "9600", "9600", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "19200", "19200", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "38400", "38400", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "57600", "57600", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "115200", "115200", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "230400", "230400", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "460800", "460800", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "921600", "921600", ControlColor::Alizarin, baudrate );
    }

    ESPUI.addControl( ControlType::Number, "Intervall to send Position", String( steerConfig.ntripPositionSendIntervall ), ControlColor::Peterriver, tab,
    []( Control * control, int id ) {
      steerConfig.ntripPositionSendIntervall = control->value.toInt();
    } );

    textNmeaToSend = ESPUI.addControl( ControlType::Text, "NMEA-String to send (leave empty to send live position)", String( steerConfig.rtkCorrectionNmeaToSend ), ControlColor::Peterriver, tab,
    []( Control * control, int id ) {
      control->value.toCharArray( steerConfig.rtkCorrectionNmeaToSend, sizeof( steerConfig.rtkCorrectionNmeaToSend ) );
    } );

    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Send NMEA-data to", String( ( int )steerConfig.sendNmeaDataTo ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.sendNmeaDataTo = ( SteerConfig::SendNmeaDataTo )control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Option, "Nowhere", "0", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "UDP", "1", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "USB", "2", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "RS232", "4", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "UDP+RS232", "5", ControlColor::Alizarin, sel );
//       ESPUI.addControl( ControlType::Option, "Bluetooth", "6", ControlColor::Alizarin, sel );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "TCP-Socket for a direct connection to the GPS-Receiver (set to 0 to deactivate, can be used for configuration with u-center or with 3rd-party software)*",  String( steerConfig.sendNmeaDataTcpPort ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.sendNmeaDataTcpPort = control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "65535", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }
  }

  // Default Configurations Tab
  {
    uint16_t tab     = ESPUI.addControl( ControlType::Tab, "Configurations", "Configurations" );
    ESPUI.addControl( ControlType::Label, "Attention:", "These Buttons here reset the whole config. This affects the WIFI too, if not configured otherwise below. You have to press \"Apply & Reboot\" above to actualy store them.", ControlColor::Carrot, tab );

    {
      ESPUI.addControl( ControlType::Switcher, "Retain WIFI settings", steerConfig.retainWifiSettings ? "1" : "0", ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.retainWifiSettings = control->value.toInt() == 1;
      } );
    }
    {
      ESPUI.addControl( ControlType::Button, "Set Settings To Default*", "Defaults", ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        char ssid[24], password[24], hostname[24];

        if ( steerConfig.retainWifiSettings ) {
          memcpy( ssid, steerConfig.ssid, sizeof( ssid ) );
          memcpy( password, steerConfig.password, sizeof( password ) );
          memcpy( hostname, steerConfig.hostname, sizeof( hostname ) );
        }

        steerConfig = steerConfigDefaults;

        if ( steerConfig.retainWifiSettings ) {
          memcpy( steerConfig.ssid, ssid, sizeof( ssid ) );
          memcpy( steerConfig.password, password, sizeof( password ) );
          memcpy( steerConfig.hostname, hostname, sizeof( hostname ) );
        }

        setResetButtonToRed();
      } );
    }

  }


  /*
   * .begin loads and serves all files from PROGMEM directly.
   * If you want to serve the files from SPIFFS use ESPUI.beginSPIFFS
   * (.prepareFileSystem has to be run in an empty sketch before)
   */

  /*
   * Optionally you can use HTTP BasicAuth. Keep in mind that this is NOT a
   * SECURE way of limiting access.
   * Anyone who is able to sniff traffic will be able to intercept your password
   * since it is transmitted in cleartext. Just add a username and password,
   * for example begin("ESPUI Control", "username", "password")
   */
  static String title = "AOG Control :: ";
  title += steerConfig.hostname;
  ESPUI.begin( title.c_str() );

  initIdleStats();
  xTaskCreate( statusLedWorker, "Status-LED", 1024, NULL, 1, NULL );

  initSensors();
  initRtkCorrection();

  initCan();

  initAutosteer();
}

void loop( void ) {
  dnsServer.processNextRequest();
  vTaskDelay( 100 );
}
