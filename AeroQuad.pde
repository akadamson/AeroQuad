/*
  AeroQuad v2.4.1 - June 2011
  www.AeroQuad.com 
  Copyright (c) 2011 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.
 
  This program is free software: you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or 
  (at your option) any later version. 

  This program is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
  GNU General Public License for more details. 

  You should have received a copy of the GNU General Public License 
  along with this program. If not, see <http://www.gnu.org/licenses/>. 
*/

/****************************************************************************
   Before flight, select the different user options for your AeroQuad below
   If you need additional assitance go to http://AeroQuad.com/forum
*****************************************************************************/

/****************************************************************************
 ************************* Hardware Configuration ***************************
 ****************************************************************************/
// Select which hardware you wish to use with the AeroQuad Flight Software
//#define AeroQuad_v1         // Arduino 2009 with AeroQuad Shield v1.7 and below
//#define AeroQuad_v1_IDG     // Arduino 2009 with AeroQuad Shield v1.7 and below using IDG yaw gyro
//#define AeroQuad_v18        // Arduino 2009 with AeroQuad Shield v1.8
//#define AeroQuad_Mini       // Arduino Pro Mini with AeroQuad Mini Shield V1.0
//#define AeroQuad_Wii        // Arduino 2009 with Wii Sensors and AeroQuad Shield v1.x
//#define AeroQuad_Paris_v3   // Define along with either AeroQuad_Wii to include specific changes for MultiWiiCopter Paris v3.0 board					
//#define AeroQuadMega_v1     // Arduino Mega with AeroQuad Shield v1.7 and below
#define AeroQuadMega_v2     // Arduino Mega with AeroQuad Shield v2.x
#define AeroQuadSeeed_v2     // Seeeduino Mega with AeroQuad Shield v1.8 (only works with AeroQuadMega_v2 defined as well
//#define AeroQuadMega_Wii    // Arduino Mega with Wii Sensors and AeroQuad Shield v2.x
//#define ArduCopter          // ArduPilot Mega (APM) with APM Sensor Board
//#define AeroQuadMega_CHR6DM // Clean Arduino Mega with CHR6DM as IMU/heading ref.
//#define APM_OP_CHR6DM       // ArduPilot Mega with CHR6DM as IMU/heading ref., Oilpan for barometer (just uncomment AltitudeHold for baro), and voltage divider

/****************************************************************************
 *********************** Define Flight Configuration ************************
 ****************************************************************************/
// Use only one of the following definitions
// quad configurations will work on 328 and mega processors
// hex configurations will work on mega processors or 328 processors with I2C speed controls, 
//   will not work on 328 processors with PWM speed controls
#define XConfig
//#define plusConfig
//#define HEXACOAXIAL
//#define HEXARADIAL

// *******************************************************************************************************************************
// Optional Sensors
// Warning:  If you enable HeadingHold or AltitudeHold and do not have the correct sensors connected, the flight software may hang
// *******************************************************************************************************************************
//#define HeadingMagHold // Enables HMC5843 Magnetometer, gets automatically selected if CHR6DM is defined
#define AltitudeHold // Enables Altitude Hold (experimental, use at your own risk) - requires one of the following to be defined
#define BMP_085 // Enable the BMP085 Baro
//#define MPX_Baro // Enalbe the MPX series Baros with Honks board
#define BattMonitor //define your personal specs in BatteryMonitor.h! Full documentation with schematic there
//#define HasGPS // define for GPS
// MUST DEFINE with GPS - unique to your location W = negative value, E = positive value
//#define MAG_VAR (-(4 + (58.0/60.0)))
//#define GPSNMEA // Use an GPS which supports the NMEA string protocol
//#define GPSBINARY // use the DIY MediaTek 3329 binary protocol
//#define RateModeOnly // Use this if you only have a gyro sensor, this will disable any attitude modes.
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// You must define *only* one of the following 2 flightAngle calculations
// if you only want DCM, then don't define either of the below
// flightAngle recommendations: use FlightAngleARG if you do not have a magnetometer, use DCM if you have a magnetometer installed
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
//#define FlightAngleDCM // Use this if you have a magnetometer installed
#define FlightAngleARG // Use this if you do not have a magnetometer installed
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
// Misc defines
#define ConfiguratorTelem // Enables telemetry interface on Serial for Configurator
//#define WirelessTelemetry  // Enables Wireless telemetry on Serial3  // Wireless telemetry enable
//#define BinaryWrite // Enables fast binary transfer of flight data to Configurator
//#define BinaryWritePID // Enables fast binary transfer of attitude PID data
//#define OpenlogBinaryWrite // Enables fast binary transfer to serial2 and openlog hardware
#define Loop_200HZ // Enable 200Hz timing loop to double sample Accel/Gyro, consume once
//#define Loop_1HZ // Enable 1Hz timing loop
#define AKA_MODS // various modifications from AKA
//#define UseLED_Library  // includes the LED library for those platforms that are built to suppor it

// *******************************************************************************************************************************
// Camera Stabilization
// Servo output goes to D11(pitch), D12(roll), D13(yaw) on AeroQuad v1.8 shield
// If using v2.0 Shield place jumper between:
// D12 to D33 for roll, connect servo to SERVO1
// D11 to D34 for pitch, connect servo to SERVO2
// D13 to D35 for yaw, connect servo to SERVO3
// Please note that you will need to have battery connected to power on servos with v2.0 shield
// *******************************************************************************************************************************
//#define CameraControl

// On screen display implementation using MAX7456 chip. See OSD.h for more info and configuration.
//#define MAX7456_OSD

/****************************************************************************
 ********************* End of User Definition Section ***********************
 ****************************************************************************/
// Debugging defines
//#define DEBUG_LOOP
//#define DEBUG_GPS

#ifdef DEBUG_GPS
#define Loop_1HZ
#endif

#if !defined(AeroQuadMega_v2)// || !defined(AeroQuad_v18) || !defined(AeroQuad_Mini)
  #if !defined(AeroQuad_v18)
    #if !defined(AeroQuad_Mini)
      #undef Loop_200HZ
    #endif
  #endif
#endif

#if defined(MAX7456_OSD) && !defined(AeroQuadMega_v2) && !defined(AeroQuadMega_Wii)
#undef MAX7456_OSD
#endif

#if defined(GPSNMEA) && defined(GPSBINARY)
#undef GPSNMEA
#endif

#ifdef AeroQuadMega_v2
#define UseLED_Library
#endif

#include <EEPROM.h>
#include <TwiMaster.h>
#ifdef HasGPS
  #include <TinyGPS.h>
#endif
#ifdef UseLED_Library
  #include <LED.h>
#endif

TwiMaster twiMaster;

#include "AeroQuad.h"
#include "PID.h"
#include "AQMath.h"
#include "Receiver.h"
#include "DataAcquisition.h"
#include "Accel.h"
#include "Gyro.h"
#include "Motors.h"

//****************************************************************************
// Create objects defined from Configuration Section above
#ifdef AeroQuad_v1
  Accel_AeroQuad_v1 accel;
  Gyro_AeroQuad_v1 gyro;
  Receiver_AeroQuad receiver;
  Motors_PWM motors;
  #include "FlightAngle.h"
  #ifdef FlightAngleARG
    FlightAngle_ARG flightAngle;
  #elif defined FlightAngleDCM
    FlightAngle_DCM flightAngle;
  #endif
  #ifdef CameraControl
    #include "Camera.h"
    Camera_AeroQuad camera;
  #endif
#endif

//****************************************************************************
#ifdef AeroQuad_v1_IDG
  Accel_AeroQuad_v1 accel;
  Gyro_AeroQuad_v1 gyro;
  Receiver_AeroQuad receiver;
  Motors_PWM motors;
  #include "FlightAngle.h"
  #ifdef FlightAngleARG
    FlightAngle_ARG flightAngle;
  #elif defined FlightAngleDCM
    FlightAngle_DCM flightAngle;
  #endif
  #ifdef CameraControl
    #include "Camera.h"
    Camera_AeroQuad camera;
  #endif
#endif

//****************************************************************************
#ifdef AeroQuad_v18
  #ifdef UseLED_Library
    LED mainLED(LEDPIN);
    BlinkLED modeLED(LED2PIN);
  #endif
  Accel_AeroQuadMega_v2 accel;
  Gyro_AeroQuadMega_v2 gyro;
  Receiver_AeroQuad receiver;
  Motors_PWMtimer motors;
  //Motors_AeroQuadI2C motors; // Use for I2C based ESC's
  #include "FlightAngle.h"
  #ifdef FlightAngleARG
    FlightAngle_ARG flightAngle;
  #elif defined FlightAngleDCM
    FlightAngle_DCM flightAngle;
  #endif
  #ifdef HeadingMagHold
    #include "Compass.h"
    Magnetometer_HMC5843 compass;
  #endif
  #ifdef AltitudeHold
    #include "Altitude.h"
    #ifdef MPX_Baro
    Altitude_MPX_SCC altitude;
    #endif
    #ifdef BMP_085
    Altitude_AeroQuad_v2 altitude;
    #endif
  #endif
  #ifdef BattMonitor
    #include "BatteryMonitor.h"
    BatteryMonitor_AeroQuad batteryMonitor;
  #endif
  #ifdef CameraControl
    #include "Camera.h"
    Camera_AeroQuad camera;
  #endif
#endif

//****************************************************************************
#ifdef AeroQuad_Mini
  Accel_AeroQuadMini accel;
  Gyro_AeroQuadMega_v2 gyro;
  Receiver_AeroQuad receiver;
  Motors_PWMtimer motors;
  //Motors_AeroQuadI2C motors; // Use for I2C based ESC's
  #include "FlightAngle.h"
  #ifdef FlightAngleARG
    FlightAngle_ARG flightAngle;
  #elif defined FlightAngleDCM
    FlightAngle_DCM flightAngle;
  #endif
  #ifdef HeadingMagHold
    #include "Compass.h"
    Magnetometer_HMC5843 compass;
  #endif
  #ifdef AltitudeHold
    #include "Altitude.h"
    #ifdef MPX_Baro
    Altitude_MPX_SCC altitude;
    #endif
    #ifdef BMP_085
    Altitude_AeroQuad_v2 altitude;
    #endif
  #endif
  #ifdef BattMonitor
    #include "BatteryMonitor.h"
    BatteryMonitor_AeroQuad batteryMonitor;
  #endif
  #ifdef CameraControl
    #include "Camera.h"
    Camera_AeroQuad camera;
  #endif
#endif

//****************************************************************************
#ifdef AeroQuadMega_v1
  // Special thanks to Wilafau for fixes for this setup
  // http://aeroquad.com/showthread.php?991-AeroQuad-Flight-Software-v2.0&p=11466&viewfull=1#post11466
  Receiver_AeroQuadMega receiver;
  Accel_AeroQuad_v1 accel;
  Gyro_AeroQuad_v1 gyro;
  Motors_PWM motors;
  #include "FlightAngle.h"
  #ifdef FlightAngleARG
    FlightAngle_ARG flightAngle;
  #elif defined FlightAngleDCM
    FlightAngle_DCM flightAngle;
  #endif
  #ifdef CameraControl
    #include "Camera.h"
    Camera_AeroQuad camera;
  #endif
#endif

//****************************************************************************
#ifdef AeroQuadMega_v2
  #ifdef UseLED_Library
    LED mainLED(LEDPIN);
    BlinkLED modeLED(LED2PIN);
    BlinkLED statusLED(LED3PIN);
  #endif
  #ifdef BattMonitor  
    #ifdef UseLED_Library
      LED buzzer(BUZZERPIN);
    #endif
  #endif    
  Receiver_AeroQuadMega receiver;
  Motors_PWMtimer motors;
  //Motors_AeroQuadI2C motors; // Use for I2C based ESC's
  Accel_AeroQuadMega_v2 accel;
  Gyro_AeroQuadMega_v2 gyro;
  #ifdef HasGPS
    #include "GPS.h"
    GPS_AeroQuad gps;
    /*
    GPSPosition wayPoints[4] =  {{34084352, -83947184, true},
                                 {34084063, -83947475, true},
                                 {34084352, -83947184, true},
                                 {34084063, -83947475, true}
                                 };
                                     
    byte wayPointIndex = 0;
    unsigned int timerCounter = 0;
    */
  #endif
  #include "FlightAngle.h"
  #ifdef FlightAngleARG
    FlightAngle_ARG flightAngle;
  #elif defined FlightAngleDCM
    FlightAngle_DCM flightAngle;
  #endif
  #ifdef HeadingMagHold
    #include "Compass.h"
    Magnetometer_HMC5843 compass;
  #endif
  #ifdef AltitudeHold
    #include "Altitude.h"
    #ifdef MPX_Baro
    Altitude_MPX_SCC altitude;
    #endif
    #ifdef BMP_085
    Altitude_AeroQuad_v2 altitude;
    #endif
  #endif
  #ifdef BattMonitor
    #include "BatteryMonitor.h"
    BatteryMonitor_AeroQuad batteryMonitor;
  #endif
  #ifdef CameraControl
    #include "Camera.h"
    Camera_AeroQuad camera;
  #endif
  #ifdef MAX7456_OSD
    #include "OSD.h"
    OSD osd;
  #endif
#endif

//****************************************************************************
#ifdef ArduCopter
  Gyro_ArduCopter gyro;
  Accel_ArduCopter accel;
  Receiver_ArduCopter receiver;
  Motors_ArduCopter motors;
  #include "FlightAngle.h"
  #ifdef FlightAngleARG
    FlightAngle_ARG flightAngle;
  #elif defined FlightAngleDCM
    FlightAngle_DCM flightAngle;
  #endif
  #ifdef HeadingMagHold
    #include "Compass.h"
    Magnetometer_HMC5843 compass;
  #endif
  #ifdef AltitudeHold
    #include "Altitude.h"
    #ifdef MPX_Baro
    Altitude_MPX_SCC altitude;
    #endif
    #ifdef BMP_085
    Altitude_AeroQuad_v2 altitude;
    #endif
  #endif
  #ifdef BattMonitor
    #include "BatteryMonitor.h"
    BatteryMonitor_APM batteryMonitor;
  #endif
#endif

//****************************************************************************
#ifdef AeroQuad_Wii
  Accel_Wii accel;
  Gyro_Wii gyro;
  Receiver_AeroQuad receiver;
  Motors_PWMtimer motors;
  #include "FlightAngle.h"
//  FlightAngle_CompFilter tempFlightAngle;
  #ifdef FlightAngleARG
    FlightAngle_ARG flightAngle;
  #elif defined FlightAngleDCM
    FlightAngle_DCM flightAngle;
  #endif
  #ifdef HeadingMagHold
    #include "Compass.h"
    Magnetometer_HMC5843 compass;
  #endif
  #ifdef AltitudeHold
    #include "Altitude.h"
    #ifdef MPX_Baro
    Altitude_MPX_SCC altitude;
    #endif
    #ifdef BMP_085
    Altitude_AeroQuad_v2 altitude;
    #endif
  #endif
  #ifdef BattMonitor
    #include "BatteryMonitor.h"
    BatteryMonitor_AeroQuad batteryMonitor;
  #endif
  #ifdef CameraControl
    #include "Camera.h"
    Camera_AeroQuad camera;
  #endif
#endif

//****************************************************************************
#ifdef AeroQuadMega_Wii
  #ifdef UseLED_Library
    LED mainLED(LEDPIN);
    BlinkLED modeLED(LED2PIN);
    BlinkLED statusLED(LED3PIN);
  #endif
  Accel_Wii accel;
  Gyro_Wii gyro;
  Receiver_AeroQuadMega receiver;
  Motors_PWMtimer motors;
  #include "FlightAngle.h"
  #ifdef FlightAngleARG
    FlightAngle_ARG flightAngle;
  #elif defined FlightAngleDCM
    FlightAngle_DCM flightAngle;
  #endif
  #ifdef HeadingMagHold
    #include "Compass.h"
    Magnetometer_HMC5843 compass;
  #endif
  #ifdef AltitudeHold
    #include "Altitude.h"
    #ifdef MPX_Baro
    Altitude_MPX_SCC altitude;
    #endif
    #ifdef BMP_085
    Altitude_AeroQuad_v2 altitude;
    #endif
  #endif
  #ifdef BattMonitor
    #include "BatteryMonitor.h"
    BatteryMonitor_AeroQuad batteryMonitor;
  #endif
  #ifdef CameraControl
    #include "Camera.h"
    Camera_AeroQuad camera;
  #endif
  #ifdef MAX7456_OSD
    #include "OSD.h"
    OSD osd;
  #endif
#endif

//****************************************************************************
#ifdef AeroQuadMega_CHR6DM
  Accel_CHR6DM accel;
  Gyro_CHR6DM gyro;
  Receiver_AeroQuadMega receiver;
  Motors_PWM motors;
  #include "FlightAngle.h"
  FlightAngle_CHR6DM flightAngle;
  #include "Compass.h"
  Compass_CHR6DM compass;
  #ifdef AltitudeHold
    #include "Altitude.h"
    #ifdef MPX_Baro
    Altitude_MPX_SCC altitude;
    #endif
    #ifdef BMP_085
    Altitude_AeroQuad_v2 altitude;
    #endif
  #endif
  #ifdef BattMonitor
    #include "BatteryMonitor.h"
    BatteryMonitor_APM batteryMonitor;
  #endif
  #ifdef CameraControl
    #include "Camera.h"
    Camera_AeroQuad camera;
  #endif
#endif

//****************************************************************************
#ifdef APM_OP_CHR6DM
  Accel_CHR6DM accel;
  Gyro_CHR6DM gyro;
  Receiver_ArduCopter receiver;
  Motors_ArduCopter motors;
  #include "FlightAngle.h"
  FlightAngle_CHR6DM flightAngle;
  #include "Compass.h"
  Compass_CHR6DM compass;
  #ifdef AltitudeHold
    #include "Altitude.h"
    #ifdef MPX_Baro
    Altitude_MPX_SCC altitude;
    #endif
    #ifdef BMP_085
    Altitude_AeroQuad_v2 altitude;
    #endif
  #endif
  #ifdef BattMonitor
    #include "BatteryMonitor.h"
    BatteryMonitor_APM batteryMonitor;
  #endif
  #ifdef CameraControl
    #include "Camera.h"
    Camera_AeroQuad camera;
  #endif
#endif

#ifdef XConfig
  void (*processFlightControl)() = &processFlightControlXMode;
#endif
#ifdef plusConfig
  void (*processFlightControl)() = &processFlightControlPlusMode;
#endif

// Include this last as it contains objects from above declarations
#include "DataStorage.h"

// ************************************************************
// ********************** Setup AeroQuad **********************
// ************************************************************
void setup() {
  SERIAL_BEGIN(SERIAL_BAUD);
  #ifndef UseLED_Library
    pinMode(LEDPIN, OUTPUT);
    digitalWrite(LEDPIN, LOW);
  #endif

  #ifdef DEBUG_LOOP
    pinMode(12, OUTPUT);
    pinMode(11, OUTPUT);
    pinMode(10, OUTPUT);
    pinMode(9, OUTPUT);
    pinMode(8, OUTPUT);
    digitalWrite(12, LOW);
    digitalWrite(11, LOW);
    digitalWrite(10, LOW);
    digitalWrite(9, LOW);
    digitalWrite(8, LOW);
  #endif    

  #if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
    Serial1.begin(BAUD);
    PORTD = B00000100;
  #endif
  #if defined(AeroQuad_v18) || defined(AeroQuad_Mini) || defined(AeroQuadMega_v2)
    #ifndef UseLED_Library)
      pinMode(LED2PIN, OUTPUT);
      digitalWrite(LED2PIN, LOW);
    #endif
  #endif
  #ifdef AeroQuadMega_v2
    // pins set to INPUT for camera stabilization so won't interfere with new camera class
    pinMode(33, INPUT); // disable SERVO 1, jumper D12 for roll
    pinMode(34, INPUT); // disable SERVO 2, jumper D11 for pitch
    pinMode(35, INPUT); // disable SERVO 3, jumper D13 for yaw
    #ifdef MikroDefine    
      pinMode(43, OUTPUT); // LED 1
      pinMode(44, OUTPUT); // LED 2
      pinMode(45, OUTPUT); // LED 3
      pinMode(46, OUTPUT); // LED 4
      digitalWrite(43, HIGH); // LED 1 on
      digitalWrite(44, HIGH); // LED 2 on
      digitalWrite(45, HIGH); // LED 3 on
      digitalWrite(46, HIGH); // LED 4 on  
    #endif
  #endif
  #if defined(APM_OP_CHR6DM) || defined(ArduCopter) 
    pinMode(LED_Red, OUTPUT);
    pinMode(LED_Yellow, OUTPUT);
    pinMode(LED_Green, OUTPUT);
  #endif
  
  #if defined(AeroQuad_v18) || defined(AeroQuadMega_v2) || defined(AeroQuad_Mini) || defined(AeroQuad_Wii) || defined(AeroQuadMega_Wii) || defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM) || defined(ArduCopter)
    twiMaster.init(false);
    // Set I2C bus time to 100khz
    TWBR = (F_CPU/100000 - 16)/2;
  #endif
  #if defined(AeroQuad_v18) || defined(AeroQuadMega_v2) || defined(AeroQuad_Mini)
    // Set I2C bus time to 400khz
    TWBR = (F_CPU/400000 - 16)/2;
  #endif

  // Read user values from EEPROM
  readEEPROM(); // defined in DataStorage.h
  
  // Configure motors
  motors.initialize(); // defined in Motors.h

  // Setup receiver pins for pin change interrupts
  receiver.initialize(); // defined in Received.h
       
  // Initialize sensors
  // If sensors have a common initialization routine
  // insert it into the gyro class because it executes first
  gyro.initialize(); // defined in Gyro.h
  accel.initialize(); // defined in Accel.h
  
  // Calibrate sensors
  gyro.autoZero(); // defined in Gyro.h
  zeroIntegralError();
  
  // Flight angle estimation
  #ifdef HeadingMagHold
    compass.initialize();
    flightAngle.initialize(compass.getHdgXY(XAXIS), compass.getHdgXY(YAXIS));
  #else
    flightAngle.initialize(1.0, 0.0);  // with no compass, DCM matrix initalizes to a heading of 0 degrees
  #endif

  // Integral Limit for attitude mode
  // This overrides default set in readEEPROM()
  // Set for 1/2 max attitude command (+/-0.75 radians)
  // Rate integral not used for now
  PID[LEVELROLL].windupGuard = 0.375;
  PID[LEVELPITCH].windupGuard = 0.375;
  
  #ifdef HasGPS
    // Velocity setup hard coded for now
    PID[XVELOCITY].P = 0.2;
    PID[XVELOCITY].I = 0;
    PID[XVELOCITY].D = 0;
    PID[XVELOCITY].windupGuard = 100.0;
    PID[XVELOCITY].integratedError = 0;
    PID[XVELOCITY].lastPosition = 0;
    PID[XVELOCITY].firstPass = true;
    PID[XVELOCITY].pidID = XVELOCITY;
    
    PID[YVELOCITY].P = 0.2;
    PID[YVELOCITY].I = 0;
    PID[YVELOCITY].D = 0;
    PID[YVELOCITY].windupGuard = 100.0;
    PID[YVELOCITY].integratedError = 0;
    PID[YVELOCITY].lastPosition = 0;
    PID[YVELOCITY].firstPass = true;
    PID[YVELOCITY].pidID = YVELOCITY;
  
    PID[XPOSITION].P = 0.2;
    PID[XPOSITION].I = 0;
    PID[XPOSITION].D = 0;
    PID[XPOSITION].windupGuard = 100.0;
    PID[XPOSITION].integratedError = 0;
    PID[XPOSITION].lastPosition = 0;
    PID[XPOSITION].firstPass = true;
    PID[XPOSITION].pidID = XPOSITION;
  
    PID[YPOSITION].P = 0.2;
    PID[YPOSITION].I = 0;
    PID[YPOSITION].D = 0;
    PID[YPOSITION].windupGuard = 100.0;
    PID[YPOSITION].integratedError = 0;
    PID[YPOSITION].lastPosition = 0;
    PID[YPOSITION].firstPass = true;
    PID[YPOSITION].pidID = YPOSITION;
  #endif
  
  // Optional Sensors
  #ifdef AltitudeHold
    altitude.initialize();
  #endif
  
  // Battery Monitor
  #ifdef BattMonitor
    batteryMonitor.initialize();
  #endif
  
  // Camera stabilization setup
  #ifdef CameraControl
    camera.initialize();
    camera.setmCameraRoll(11.11); // Need to figure out nice way to reverse servos
    camera.setCenterRoll(1500); // Need to figure out nice way to set center position
    camera.setmCameraPitch(11.11);
    camera.setCenterPitch(1300);
  #endif
  
  //initialising OSD
  #if defined(MAX7456_OSD)
    osd.initialize();
  #endif

  #if defined(BinaryWrite) || defined(BinaryWritePID)
    #ifdef OpenlogBinaryWrite
      binaryPort = &Serial2;
      binaryPort->begin(115200);
      delay(1000);
    #else
     binaryPort = &Serial;
    #endif 
  #endif
  
  // AKA use a new low pass filter called a Lag Filter uncomment only if using DCM LAG filters
  setupFourthOrder();

  #ifdef HasGPS  
    gps.initialize();
  #endif
  
  #ifdef AKA_MODS
    // AKA experiment to detect trim offset
    // detect startup trim offset
    for (byte i = 0; i < 10; i++) {
      receiver.read();
      delay(100);
    }
    // Get/set trim center value of roll/pitch/yaw channels
    receiver.setTransmitterTrim(ROLL, receiver.getData(ROLL));
    receiver.setTransmitterTrim(PITCH, receiver.getData(PITCH));
    receiver.setTransmitterTrim(YAW, receiver.getData(YAW));
  #endif
  
  #if defined(UseLED_Library) && (defined(AeroQuadMega_v2) || defined(AeroQuadMega_Wii))
    statusLED.setInterval(250000);
    statusLED.setRunState(REPEATCTDN);
    statusLED.setOff();
  #endif
  #if defined(UseLED_Library) && (defined(AeroQuadMega_v2) || defined(AeroQuad_v18) || defined(AeroQuadMega_Wii))
    modeLED.setInterval(80000);
    modeLED.setRunState(REPEAT);
    modeLED.setOff();
  #endif
  
  previousTime = micros();
  #ifdef UseLED_Library
    mainLED.on();
  #else
    digitalWrite(LEDPIN, HIGH);
  #endif

  safetyCheck = 0;
}

void loop () {
  currentTime = micros();
  deltaTime = currentTime - previousTime;
  
  // Main scheduler loop set for 100hz
  #ifdef Loop_200HZ
    if (deltaTime >= 5000) {    
  #else
    if (deltaTime >= 10000) {
  #endif

    #ifdef DEBUG_LOOP
      testSignal ^= HIGH;
      digitalWrite(LEDPIN, testSignal);
    #endif

    frameCounter++;
    
    #ifdef Loop_200HZ
      // ================================================================
      // 200hz task loop
      // ================================================================
      if (frameCounter %   1 == 0) {  //  200 Hz tasks
        #ifdef DEBUG_LOOP
          digitalWrite(12, HIGH);
        #endif
        
        G_Dt = (currentTime - twoHundredHZpreviousTime) / 1000000.0;
        twoHundredHZpreviousTime = currentTime;
        
        gyro.sample();
        accel.sample();
  
        #ifdef DEBUG_LOOP
          digitalWrite(12, LOW);
        #endif
      }
    #endif
    
    // ================================================================
    // 100hz task loop
    // ================================================================
    #ifdef Loop_200HZ
      if (frameCounter %   2 == 0) {  //  100 Hz tasks

      #ifdef DEBUG_LOOP
        digitalWrite(8, HIGH);
      #endif

      G_Dt = (currentTime - hundredHZ1previousTime) / 1000000.0;
      hundredHZ1previousTime = currentTime;

      processFlightControl();
 
      #ifdef DEBUG_LOOP
        digitalWrite(8, LOW);
      #endif
      }
    #endif

    // ================================================================
    // 100hz task loop
    // ================================================================
    #ifdef Loop_200HZ
      if (frameCounter %   2 == 1) {  //  100 Hz tasks
    #else
      if (frameCounter %   1 == 0) {  //  100 Hz tasks
    #endif
      #ifdef DEBUG_LOOP
        digitalWrite(11, HIGH);
      #endif
      
      G_Dt = (currentTime - hundredHZpreviousTime) / 1000000.0;
      hundredHZpreviousTime = currentTime;
      
      // measure critical sensors
      gyro.measure();
      accel.measure();

//      #ifdef DEBUG_LOOP
//        digitalWrite(11, HIGH);
//      #endif
      
      // AKA - need to move these inside the accel measure method at some point  
      filteredAccel[XAXIS] = computeFourthOrder(accel.getData(XAXIS), &fourthOrder[AX_FILTER]);
      filteredAccel[YAXIS] = computeFourthOrder(accel.getData(YAXIS), &fourthOrder[AY_FILTER]);
      filteredAccel[ZAXIS] = computeFourthOrder(accel.getData(ZAXIS), &fourthOrder[AZ_FILTER]);
        
//      #ifdef DEBUG_LOOP
//        digitalWrite(11, LOW);
//      #endif

      // ****************** Calculate Absolute Angle *****************
      #if defined HeadingMagHold && defined FlightAngleARG
        // ARG with compass (not used by AHRS)
        flightAngle.calculate(gyro.getData(ROLL),                       \
                               gyro.getData(PITCH),                     \
                               gyro.getData(YAW),                       \
                               filteredAccel[XAXIS],                    \
                               filteredAccel[YAXIS],                    \
                               filteredAccel[ZAXIS],                    \
//                               accel.getData(XAXIS),
//                               accel.getData(YAXIS),
//                               accel.getData(ZAXIS),
                               0.0F,                                    \
                               0.0F,                                    \
                               0.0F);
      #endif

      #if !defined HeadingMagHold && defined FlightAngleARG
        // ARG with no compass
        flightAngle.calculate(gyro.getData(ROLL),                       \
                               gyro.getData(PITCH),                     \
                               gyro.getData(YAW),                       \
                               filteredAccel[XAXIS],                    \
                               filteredAccel[YAXIS],                    \
                               filteredAccel[ZAXIS],                    \
//                               accel.getData(XAXIS),
//                               accel.getData(YAXIS),
//                               accel.getData(ZAXIS),
                               0.0F,                                    \
                               0.0F,                                    \
                               0.0F);
      #endif
    
      #if defined HeadingMagHold && defined FlightAngleDCM
        // DCM with compass
        flightAngle.calculate(gyro.getData(ROLL),                       \
                               gyro.getData(PITCH),                     \
                               gyro.getData(YAW),                       \
                               filteredAccel[XAXIS],                    \
                               filteredAccel[YAXIS],                    \
                               filteredAccel[ZAXIS],                    \
//                               accel.getData(XAXIS),
//                               accel.getData(YAXIS),
//                               accel.getData(ZAXIS),
                               ONE_G,                                   \
                               compass.getHdgXY(XAXIS),                 \
                               compass.getHdgXY(YAXIS));
      #endif
      
      #if !defined HeadingMagHold && defined FlightAngleDCM
        // DCM with no compass
        flightAngle.calculate(gyro.getData(ROLL),                       \
                              gyro.getData(PITCH),                      \
                              gyro.getData(YAW),                        \
                               filteredAccel[XAXIS],                    \
                               filteredAccel[YAXIS],                    \
                               filteredAccel[ZAXIS],                    \
//                               accel.getData(XAXIS),
//                               accel.getData(YAXIS),
//                               accel.getData(ZAXIS),
                               ONE_G,                                   \
                               0.0F,                                    \
                               0.0F);
      #endif

      // AKA added for testing      
//      fastTelemetry();
      
      #ifndef Loop_200HZ
        processFlightControl();
      #endif

      #ifdef DEBUG_LOOP
        digitalWrite(11, LOW);
      #endif
    }

    // ================================================================
    // 50hz task loop
    // ================================================================
    #ifdef Loop_200HZ
      if (frameCounter %   4 == 0) {  //  50 Hz tasks
    #else
      if (frameCounter %   2 == 0) {  //  50 Hz tasks
    #endif
      #ifdef DEBUG_LOOP
        digitalWrite(10, HIGH);
      #endif
      
      G_Dt = (currentTime - fiftyHZpreviousTime) / 1000000.0;
      fiftyHZpreviousTime = currentTime;
      
      // Reads external pilot commands and performs functions based on stick configuration
      readPilotCommands(); // defined in FlightCommand.pde

      #if defined(CameraControl)
        camera.setPitch(degrees(flightAngle.getData(PITCH)));
        camera.setRoll(degrees(flightAngle.getData(ROLL)));
        camera.setYaw(degrees(flightAngle.getData(YAW)));
        camera.move();
      #endif 
      
      #ifdef DEBUG_LOOP
        digitalWrite(10, LOW);
      #endif
    }

    // ================================================================
    // 25hz task loop
    // ================================================================
    #ifdef Loop_200HZ
      if (frameCounter %   8 == 1) {  //  25 Hz tasks offset forward by 1 frame count
    #else
      if (frameCounter %   4 == 0) {  //  25 Hz tasks
    #endif
      #ifdef DEBUG_LOOP    
        digitalWrite(9, HIGH);
      #endif
      
      G_Dt = (currentTime - twentyFiveHZpreviousTime) / 1000000.0;
      twentyFiveHZpreviousTime = currentTime;
      
      #ifdef HasGPS
        gps.run();
      #endif

      #if defined(UseLED_Library) && (defined(AeroQuadMega_v2) || defined(AeroQuadMega_Wii))
        statusLED.run(currentTime);
      #endif
      #if defined(UseLED_Library) && (defined(AeroQuadMega_v2) || defined(AeroQuad_v18) || defined(AeroQuadMega_Wii))
        modeLED.run(currentTime);
      #endif

      #ifdef DEBUG_LOOP
        digitalWrite(9, LOW);
      #endif
    }
    
    // ================================================================
    // 10hz task loop
    // ================================================================
    #ifdef Loop_200HZ
      if (frameCounter %  20 == 0) {  //   10 Hz tasks
    #else
      if (frameCounter %  10 == 0) {  //   10 Hz tasks
    #endif
      #ifdef DEBUG_LOOP
        digitalWrite(8, HIGH);
      #endif
      
      G_Dt = (currentTime - tenHZpreviousTime) / 1000000.0;
      tenHZpreviousTime = currentTime;

      #if defined(HeadingMagHold)
        compass.measure(flightAngle.getData(ROLL), flightAngle.getData(PITCH)); // defined in compass.h
      #endif
      
      #if defined(BattMonitor)
        batteryMonitor.measure(armed);
      #endif
      
      #if defined(AltitudeHold)
        altitude.measure(); // defined in altitude.h
      #endif

      #ifdef ConfiguratorTelem
        // Listen for configuration commands and reports telemetry
        readSerialCommand(); // defined in SerialCom.pde
        sendSerialTelemetry(); // defined in SerialCom.pde
      #endif

      #ifdef BinaryWrite
        if (fastTransfer == ON) {
          // write out fastTelemetry to Configurator or openLog
// AKA         fastTelemetry();
        }
      #endif      
	
      #ifdef MAX7456_OSD
        osd.update();
      #endif

      #ifdef DEBUG_LOOP
        digitalWrite(8, LOW);
      #endif
    }

    #ifdef Loop_1HZ
      // 1Hz loop timer for debugging info
      #ifdef Loop_200HZ
        if(frameCounter % 200 == 0) {
      #else
        if(frameCounter % 100 == 0) {
      #endif	    
        #ifdef HasGPS
  /*        if (gps.holdPosition.valid) {
            if (++timerCounter > 20) {
              statusLED.setRunState(PAUSED);
              statusLED.toggle();
              timerCounter = 0;
              if (wayPointIndex < 3) {
                 wayPointIndex++;
              }
            } 
          } else { */
            if (gps.fix() == 3) {
              byte sats = gps.getSats();
              #if defined(UseLED_Library) && (defined(AeroQuadMega_Wii) | defined(AeroQuadMega_v2))
                statusLED.setCountDown(sats, REPEATCTDN);
              #else
                digitalWrite(LED3PIN, HIGH);
              #endif
            } else {
              #if defined(UseLED_Library) && (defined(AeroQuadMega_Wii) | defined(AeroQuadMega_v2))
                statusLED.setOff();
              #else
                digitalWrite(LED3PIN, LOW);
              #endif
            }
  //        }
        #endif  
        #ifdef DEBUG_GPS
          static byte i = 0;
    
          if (i++ >= 4) {
            i = 0;
          #ifdef HasGPS
            gps.gpsdump();
          #endif
          }      
        #endif
      }
    #endif
   
    previousTime = currentTime;
  }
  #ifdef Loop_200HZ
    if (frameCounter >= 200) 
  #else
    if (frameCounter >= 100) 
  #endif
  if (frameCounter >= 200) 
      frameCounter = 0;
}


