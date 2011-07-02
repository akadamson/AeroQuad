/*
  AeroQuad v2.4.2 - June 2011
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

// FlightCommand.pde is responsible for decoding transmitter stick combinations
// for setting up AeroQuad modes such as motor arming and disarming

// AKA add for testing
//bool altSwitchState = false;

void readPilotCommands() {
  // read Receiver
  receiver.read();
  
  // Read quad configuration commands from transmitter when throttle down
  if (receiver.getRaw(THROTTLE) < MINCHECK) {
    zeroIntegralError();
    throttleAdjust = 0;
    //receiver.adjustThrottle(throttleAdjust);
    // Disarm motors (left stick lower left corner)
    if (receiver.getRaw(YAW) < MINCHECK && armed == ON) {
      armed = OFF;
      #ifdef HasGPS
        gps.holdPosition.valid = false;
      #endif
      motors.commandAllMotors(MINCOMMAND);
      #if defined(APM_OP_CHR6DM) || defined(ArduCopter) 
        digitalWrite(LED_Red, LOW);
      #endif
    }    
    // Zero Gyro and Accel sensors (left stick lower left, right stick lower right corner)
    if ((receiver.getRaw(YAW) < MINCHECK) && (receiver.getRaw(ROLL) > MAXCHECK) && (receiver.getRaw(PITCH) < MINCHECK)) {
      gyro.calibrate(); // defined in Gyro.h
      #if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
        flightAngle.calibrate();
      #endif
      zeroIntegralError();
      motors.pulseMotors(3);
      // ledCW() is currently a private method in BatteryMonitor.h, fix and agree on this behavior in next revision
      //#if defined(BattMonitor) && defined(ArduCopter)
      //  ledCW(); ledCW(); ledCW();
      //#endif
      #ifdef ArduCopter
        zero_ArduCopter_ADC();
      #endif
    }   
    
    // Arm motors (left stick lower right corner)
    if (receiver.getRaw(YAW) > MAXCHECK && armed == OFF && safetyCheck == ON) {
      zeroIntegralError();
      armed = ON;
      #if defined(APM_OP_CHR6DM) || defined(ArduCopter) 
      digitalWrite(LED_Red, HIGH);
      #endif
      for (byte motor = FRONT; motor < LASTMOTOR; motor++)
        motors.setMinCommand(motor, MINTHROTTLE);
    }
    // Prevents accidental arming of motor output if no transmitter command received
    if (receiver.getRaw(YAW) > MINCHECK)
      safetyCheck = ON; 
  }
  
  // Get center value of roll/pitch/yaw channels when enough throttle to lift off
  if (receiver.getRaw(THROTTLE) < 1300) {
    receiver.setTransmitterTrim(ROLL, receiver.getData(ROLL));
    receiver.setTransmitterTrim(PITCH, receiver.getData(PITCH));
    receiver.setTransmitterTrim(YAW, receiver.getData(YAW));
  }

  #ifdef RateModeOnly
    flightMode = RATE;
  #else
    // Check Mode switch for Rate, Attitude or Position
    // if greater than 1300 it's ATTITUDE
    if (receiver.getRaw(MODE) > 1250 && receiver.getRaw(MODE) < 1600) {
      #if defined(UseLED_Library) && (defined(AeroQuadMega_v2) || defined(AeroQuad_v18) || defined(AeroQuadMega_Wii))
        modeLED.setOn();
      #else
        #if defined(AeroQuad_v18) || defined(AeroQuadMega_v2) || defined(dAeroQuadMega_Wii)
          digitalWrite(LED2PIN, HIGH);
        #endif
      #endif
      if (flightMode == RATE) {
        zeroIntegralError();
      }
      flightMode = ATTITUDE;
    }
    // if greater than 1750, it's Position
    else if (receiver.getRaw(MODE) > 1750) {
      #if defined(UseLED_Library) && (defined(AeroQuadMega_v2) || defined(AeroQuad_v18) || defined(AeroQuadMega_Wii))
        modeLED.resume();
      #else
        #if defined(AeroQuad_v18) || defined(AeroQuadMega_v2) || defined(AeroQuadMega_Wii)
          digitalWrite(LED2PIN, HIGH);
        #endif
      #endif
      if (flightMode == RATE) {
        zeroIntegralError();
      }
      #ifdef HasGPS
        // latch beginning lat/lon for position hold
        // AKA need to put some error checking around this.. in the case where
        // you don't have a good gps FIX, it shouldn't capture or set the valid flag
        // then you need some way to denote this in an LED report or some such
        if(gps.holdPosition.valid == false)
          gps.capturePosition(&gps.holdPosition);
//        gps.holdPosition = wayPoints[wayPointIndex];
        flightMode = POSITION;
      #else
        flightMode = ATTITUDE;
      #endif
    } else {     // else it's RATE
      if (flightMode != RATE)
        #if defined(UseLED_Library) && (defined(AeroQuadMega_Wii) || defined(AeroQuadMega_v2) || defined(AeroQuad_v18))
          modeLED.setOff();
	      #else
	        #if defined(AeroQuad_v18) || defined(AeroQuadMega_v2) || defined(AeroQuadMega_Wii)
            digitalWrite(LED2PIN, LOW);
	        #endif
        #endif
        flightMode = RATE;
        #ifdef HasGPS
          gps.holdPosition.valid = false;
        #endif
    }
  #endif
  
   #if defined(APM_OP_CHR6DM) || defined(ArduCopter) 
      if (flightMode == RATE) {
        digitalWrite(LED_Yellow, HIGH);
        digitalWrite(LED_Green, LOW);
       }
     else if (flightMode == POSITION) {
        digitalWrite(LED_Green, HIGH);
        digitalWrite(LED_Yellow, LOW); 
     }
   #endif
  
  #ifdef AltitudeHold
   if (receiver.getRaw(AUX) < 1750) {
     if (altitudeHold != ALTPANIC ) {  // check for special condition with manditory override of Altitude hold
       if (storeAltitude == ON) {
         holdAltitude = altitude.getData();
         holdThrottle = receiver.getData(THROTTLE);
         PID[ALTITUDE].integratedError = 0;
         PID[ALTITUDE].lastPosition = holdAltitude;  // add to initialize hold position on switch turn on.
         storeAltitude = OFF;
       }
       altitudeHold = ON;
//       altSwitchState = true;
     }
     // note, Panic will stay set until Althold is toggled off/on
   } 
   else {
     storeAltitude = ON;
     altitudeHold = OFF;
/*     if (altSwitchState == true) {
       altSwitchState = false;
       wayPointIndex = 0;
       timerCounter = 0;
       // Get/set trim center value of roll/pitch/yaw channels
       receiver.setTransmitterTrim(ROLL, receiver.getData(ROLL));
       receiver.setTransmitterTrim(PITCH, receiver.getData(PITCH));
       receiver.setTransmitterTrim(YAW, receiver.getData(YAW));
     }
     */
   }
  #endif
}
