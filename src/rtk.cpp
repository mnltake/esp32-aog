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

// #include <ESPAsyncTCP.h>
#include <vector>

#include <HTTPClient.h>

#include <ESPUI.h>

#include <MicroNMEA.h>

#include <NMEAGPS.h>
#include <ublox/ubxNMEA.h>

#include "main.hpp"


#if !defined( NMEAGPS_PARSE_GGA ) & !defined( NMEAGPS_PARSE_GLL ) & \
    !defined( NMEAGPS_PARSE_GSA ) & !defined( NMEAGPS_PARSE_GSV ) & \
    !defined( NMEAGPS_PARSE_RMC ) & !defined( NMEAGPS_PARSE_VTG ) & \
    !defined( NMEAGPS_PARSE_ZDA ) & !defined( NMEAGPS_PARSE_GST ) & \
    !defined( NMEAGPS_PARSE_PUBX_00 ) & !defined( NMEAGPS_PARSE_PUBX_04 )

  #error No NMEA sentences enabled: no fix data available.

#endif

#if !defined( NMEAGPS_PARSE_PUBX_00 ) & !defined( NMEAGPS_PARSE_PUBX_04 )
  #error No PUBX messages enabled!  You must enable one or more in PUBX_cfg.h!
#endif

#ifndef NMEAGPS_DERIVED_TYPES
  #error You must "#define NMEAGPS_DERIVED_TYPES" in NMEAGPS_cfg.h!
#endif

#ifndef NMEAGPS_EXPLICIT_MERGING
  #error You must define NMEAGPS_EXPLICIT_MERGING in NMEAGPS_cfg.h
#endif

#ifdef NMEAGPS_INTERRUPT_PROCESSING
  #error You must *NOT* define NMEAGPS_INTERRUPT_PROCESSING in NMEAGPS_cfg.h!
#endif
    
String lastGN;
static ubloxNMEA gps; // This parses received characters
gps_fix currentGpsFix;

constexpr size_t NmeaBufferSize = 120;
char nmeaBuffer[NmeaBufferSize];
MicroNMEA nmea( nmeaBuffer, NmeaBufferSize );

AsyncServer* server;
static std::vector<AsyncClient*> clients;

static void handleError( void* arg, AsyncClient* client, int8_t error ) {
//   Serial.printf( "\n connection error %s from client %s \n", client->errorToString( error ), client->remoteIP().toString().c_str() );
}

static void handleData( void* arg, AsyncClient* client, void* data, size_t len ) {
//   Serial.printf( "\n data received from client %s \n", client->remoteIP().toString().c_str() );
//   Serial.write( ( uint8_t* )data, len );

//  // reply to client
//  if (client->space() > 32 && client->canSend()) {
//    char reply[32];
//    sprintf(reply, "this is from %s", SERVER_HOST_NAME);
//    client->add(reply, strlen(reply));
//    client->send();
//  }
}

static void handleDisconnect( void* arg, AsyncClient* client ) {
//   Serial.printf( "\n client %s disconnected \n", client->remoteIP().toString().c_str() );

  // remove client from vector
  clients.erase( std::remove_if( clients.begin(), clients.end(), [client]( AsyncClient * itClient ) {
    return itClient == client;
  } ), clients.end() );
}

static void handleTimeOut( void* arg, AsyncClient* client, uint32_t time ) {
//   Serial.printf( "\n client ACK timeout ip: %s \n", client->remoteIP().toString().c_str() );
}
static void handleNewClient( void* arg, AsyncClient* client ) {
//   Serial.printf( "\n new client has been connected to server, ip: %s", client->remoteIP().toString().c_str() );

  // add to list
  clients.push_back( client );

  // register events
  client->onData( &handleData, NULL );
  client->onError( &handleError, NULL );
  client->onDisconnect( &handleDisconnect, NULL );
  client->onTimeout( &handleTimeOut, NULL );
}

void nmeaWorker( void* z ) {

  String sentence;
  sentence.reserve( NmeaBufferSize );
  lastGN.reserve( NmeaBufferSize );

  if ( steerConfig.sendNmeaDataTcpPort != 0 ) {
    server = new AsyncServer( steerConfig.sendNmeaDataTcpPort ); // start listening on tcp port 7050
    server->onClient( &handleNewClient, server );
    server->begin();
  }

  constexpr TickType_t xFrequency = 50;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for ( ;; ) {
    uint16_t cnt = Serial2.available();

    for ( uint16_t i = 0; i < cnt; i++ ) {
      char c = Serial2.read();

      if ( steerConfig.mergeImuWithGps ) {
        if (gps.decode( c ) == NMEAGPS::DECODE_COMPLETED) {
          currentGpsFix = gps.fix();
          currentGpsFix.calculateNorthAndEastVelocityFromSpeedAndHeading();
        }
      }

      if ( nmea.process( c ) ) {
        if ( strcmp( nmea.getMessageID(), "GGA" ) == 0 ) {
          lastGN = nmea.getSentence();
        }

        if ( steerConfig.sendNmeaDataTo != SteerConfig::SendNmeaDataTo::None ) {

          sentence = nmea.getSentence();

//           {
//             gpsData.TOW;
//             gpsData.vn;
//             gpsData.ve;
//             gpsData.vd;
//             gpsData.lat;
//             gpsData.lon;
//             gpsData.alt;
//           }


//           // snap of the checksum, if it exists
//           if ( uint8_t occurence = sentence.lastIndexOf( '*' ) > 0 ) {
//             sentence.remove( occurence );
//           }
//
//           // add the checksum
//           char checksum[3] = {'\0'};
//           nmea.generateChecksum( sentence.c_str(), checksum );
//           sentence += "*";
//           sentence += checksum;
          sentence += "\r\n";

          // send sentence to all connected clients
          for ( auto client = clients.begin() ; client != clients.end(); ++client ) {
            // reply to client
            if ( ( *client )->space() > sentence.length() && ( *client )->canSend() ) {
              ( *client )->write( sentence.c_str(), sentence.length() );
              ( *client )->send();
            }
          }

          switch ( steerConfig.sendNmeaDataTo ) {
            case SteerConfig::SendNmeaDataTo::UDP: {
              udpSendFrom.broadcastTo( ( uint8_t* )sentence.c_str(), ( uint16_t )sentence.length(), initialisation.portSendTo );
            }
            break;

            case SteerConfig::SendNmeaDataTo::Serial: {
              Serial.print( millis() );
              Serial.print( ": " );
              Serial.print( sentence );
            }
            break;

            case SteerConfig::SendNmeaDataTo::Serial1: {
              Serial1.print( sentence );
            }
            break;

            case SteerConfig::SendNmeaDataTo::Serial2: {
              Serial2.print( sentence );
            }
            break;

            default:
              break;

          }
        }
      }


//       Serial.write( );
    }

    {
      static uint8_t loopCounter = 0;

      if ( loopCounter++ >= ( 1000 / xFrequency ) ) {
        loopCounter = 0;
        Control* handle = ESPUI.getControl( labelStatusGps );

        String str;
        str.reserve( 200 );

        str = "<table style='margin:auto;'><tr><td style='text-align:left; padding: 0px 5px;'>Lat:</td><td style='text-align:left; padding: 0px 5px;'>";
        str += String( ( float )nmea.getLatitude() / 1000000, 6 );
        str += "</td></tr><tr><td style='text-align:left; padding: 0px 5px;'>Lon:</td><td style='text-align:left; padding: 0px 5px;'>";
        str += String( ( float )nmea.getLongitude() / 1000000, 6 );
        str += "</td></tr><tr><td style='text-align:left; padding: 0px 5px;'>Alt:</td><td style='text-align:left; padding: 0px 5px;'>";
        str += ( float )nmea.getAltitude() / 1000;
        str += "</td></tr><tr><td style='text-align:left; padding: 0px 5px;'>HDOP:</td><td style='text-align:left; padding: 0px 5px;'>";
        str += ( float )nmea.getHDOP() / 10;
        str += "</td></tr><tr><td style='text-align:left; padding: 0px 5px;'>Age:</td><td style='text-align:left; padding: 0px 5px;'>";
        str += ( float )nmea.getAgeOfDGPS() / 10;
        str += "</td></tr><tr><td style='text-align:left; padding: 0px 5px;'>Quality:</td><td style='text-align:left; padding: 0px 5px;'>";

        switch ( nmea.getQuality() ) {
          case 0:
            str += "No GPS Fix</td></tr></table>";
            break;

          case 1:
            str += "GPS Fix</td></tr></table>";
            break;

          case 2:
            str += "DGPS Fix</td></tr></table>";
            break;

          case 3:
            str += "PPS Fix</td></tr></table>";
            break;

          case 4:
            str += "RTK Fix</td></tr></table>";
            break;

          case 5:
            str += "RTK Float</td></tr></table>";
            break;

          default:
            str += "?</td></tr></table>";
            break;
        }

        handle->value = str;
        ESPUI.updateControl( handle );
      }
    }

    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void ntripWorker( void* z ) {
  vTaskDelay( 2000 );

  Control* labelNtripHandle = ESPUI.getControl( labelStatusNtrip );

  initialisation.rtkCorrectionURL.reserve( 200 );
  initialisation.rtkCorrectionURL = "http://";

  if ( steerConfig.rtkCorrectionUsername != '\0' ) {
    initialisation.rtkCorrectionURL += steerConfig.rtkCorrectionUsername;

    if ( steerConfig.rtkCorrectionPassword != '\0' ) {
      initialisation.rtkCorrectionURL += ":";
      initialisation.rtkCorrectionURL += steerConfig.rtkCorrectionPassword;
    }

    initialisation.rtkCorrectionURL += "@";
  }

  initialisation.rtkCorrectionURL += steerConfig.rtkCorrectionServer;

  if ( steerConfig.rtkCorrectionPort != '\0' ) {
    initialisation.rtkCorrectionURL += ":";
    initialisation.rtkCorrectionURL += steerConfig.rtkCorrectionPort;
  }

  initialisation.rtkCorrectionURL += "/";

  if ( steerConfig.rtkCorrectionMountpoint != '\0' ) {
    initialisation.rtkCorrectionURL += steerConfig.rtkCorrectionMountpoint;
  }

  if ( initialisation.rtkCorrectionURL.length() <= 8 ) {
    // update WebUI
    {
      labelNtripHandle->value = "Cannot connect to " + initialisation.rtkCorrectionURL;
      labelNtripHandle->color = ControlColor::Carrot;
      ESPUI.updateControl( labelNtripHandle );
    }

    // delete this task
    TaskHandle_t myself = xTaskGetCurrentTaskHandle();
    vTaskDelete( myself );

    return;
  }

  for ( ;; ) {
    HTTPClient http;
    http.begin( initialisation.rtkCorrectionURL );
    http.setUserAgent( "NTRIP CoffeetracNTRIPClient" );
    int httpCode = http.GET();

    if ( httpCode > 0 ) {
      // HTTP header has been send and Server response header has been handled

      // file found at server
      if ( httpCode == HTTP_CODE_OK ) {
        // update WebUI
        {
          labelNtripHandle->value = "Connected to " + initialisation.rtkCorrectionURL;
          labelNtripHandle->color = ControlColor::Emerald;
          ESPUI.updateControl( labelNtripHandle );
        }

        // create buffer for read
        constexpr uint16_t buffSize = 1436;
        uint8_t* buff = ( uint8_t* )malloc( buffSize );

        // get tcp stream
        WiFiClient* stream = http.getStreamPtr();

        time_t timeoutSendGGA = millis() + ( steerConfig.ntripPositionSendIntervall * 1000 );

        // read all data from server
        while ( http.connected() ) {
          // get available data size
          size_t size = stream->available();

          if ( size ) {
            int c = stream->readBytes( buff, ( ( size > buffSize ) ? buffSize : size ) );

            // write it to Serial
            Serial2.write( buff, c );
          }

          if ( millis() > timeoutSendGGA ) {
            String nmeaToSend;
            nmeaToSend.reserve( sizeof( SteerConfig::rtkCorrectionNmeaToSend ) );

            if ( steerConfig.rtkCorrectionNmeaToSend[0] != '\0' ) {
              nmeaToSend = steerConfig.rtkCorrectionNmeaToSend;
            } else {
              nmeaToSend = lastGN;
            }

            if ( nmeaToSend.length() ) {
              // calculate checksum if not correct
              if ( !nmea.testChecksum( nmeaToSend.c_str() ) ) {

                // snap off the checksum, if it exists
                {
                  uint8_t occurence = nmeaToSend.lastIndexOf( "*" );

                  if ( occurence > 0 ) {
                    nmeaToSend.remove( occurence );
                  }
                }

                // add the checksum
                char checksum[] = {'*', '\0', '\0', '\0'};
                nmea.generateChecksum( nmeaToSend.c_str(), &checksum[1] );
                nmeaToSend += checksum;

                // update checksum, also in the WebUI
                if ( steerConfig.rtkCorrectionNmeaToSend[0] != '\0' ) {
                  nmeaToSend.toCharArray( steerConfig.rtkCorrectionNmeaToSend, sizeof( steerConfig.rtkCorrectionNmeaToSend ) );

                  {
                    Control* handle = ESPUI.getControl( textNmeaToSend );
                    handle->value.reserve( 80 );
                    handle->value = steerConfig.rtkCorrectionNmeaToSend;
                    ESPUI.updateControl( handle );
                  }
                }
              }

              if ( nmeaToSend.lastIndexOf( '\n' ) == -1 ) {
                nmeaToSend += "\r\n";
              }

              stream->write( nmeaToSend.c_str(), nmeaToSend.length() );
            }

            timeoutSendGGA = millis() + ( steerConfig.ntripPositionSendIntervall * 1000 );
          }

          vTaskDelay( 1 );
        }

      }
    }

    // update WebUI
    {
      labelNtripHandle->value = "Cannot connect to " + initialisation.rtkCorrectionURL;
      labelNtripHandle->color = ControlColor::Carrot;
      ESPUI.updateControl( labelNtripHandle );
    }

    http.end();
    vTaskDelay( 1000 );
  }

// delete this task
  TaskHandle_t myself = xTaskGetCurrentTaskHandle();
  vTaskDelete( myself );
}


void initRtkCorrection() {
  Serial2.begin( steerConfig.rtkCorrectionBaudrate );

  if ( steerConfig.rtkCorrectionType == SteerConfig::RtkCorrectionType::Ntrip ) {
    xTaskCreate( ntripWorker, "ntripWorker", 4096, NULL, 8, NULL );
  }

  xTaskCreate( nmeaWorker, "nmeaWorker", 4096, NULL, 6, NULL );
}
