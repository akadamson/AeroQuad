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

class Accel {
public:
  float smoothFactor;
  int accelChannel[3];
  #if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
    float accelZero[3];
  #else
  #endif
  float accelData[3];
  int accelADC[3];
  #if defined(Loop_200HZ) || defined(Loop_400HZ)
    byte index;
    #ifdef Loop_200HZ
      int accelRAW[3][2];
    #else
      int accelRAW[3][4];
    #endif
  #endif    
  float accelBias[3];
  float accelScaleFactor[3];
  
  byte rollChannel, pitchChannel, zAxisChannel;
  
  Accel(void) {
  }

  void _initialize(void) {
    accelBias[XAXIS] = readFloat(XAXIS_ACCEL_BIAS_ADR);
    accelBias[YAXIS] = readFloat(YAXIS_ACCEL_BIAS_ADR);
    accelBias[ZAXIS] = readFloat(ZAXIS_ACCEL_BIAS_ADR);
    
    accelScaleFactor[XAXIS] = readFloat(XAXIS_ACCEL_SCALE_FACTOR_ADR);
    accelScaleFactor[YAXIS] = readFloat(YAXIS_ACCEL_SCALE_FACTOR_ADR);
    accelScaleFactor[ZAXIS] = readFloat(ZAXIS_ACCEL_SCALE_FACTOR_ADR);
    
    smoothFactor = readFloat(ACCSMOOTH_ADR);
    
    #if defined(Loop_200HZ) || defined (Loop_400HZ)
      #ifdef Loop_200HZ
        index = 1;  // AKA index value for flip/flop store of 2 sample average
      #else
        index = 0;
      #endif
      // init flip/flop array to zero
      for (byte axis = XAXIS; axis < LASTAXIS; axis++) {
        accelRAW[axis][0] = 0;
        accelRAW[axis][1] = 0;
        #ifdef Loop_400HZ
          accelRAW[axis][2] = 0;
          accelRAW[axis][3] = 0;
        #endif        
      }
    #endif
  }

  // return the raw ADC value from the accel, with sign change if need, not smoothed or scaled to SI units
  const int getRaw(byte axis) {
    return accelADC[axis];
  }
  
  // return the smoothed and scaled to SI units value of the accel with sign change if needed
  const float getData(byte axis) {
    return ((accelData[axis] - accelBias[axis]) * accelScaleFactor[axis]);
  }
  
  // returns the smoothfactor
  const float getSmoothFactor() {
    return smoothFactor;
  }
  
  void setSmoothFactor(float value) {
    smoothFactor = value;
  }
};

/******************************************************/
/************ AeroQuad v1 Accelerometer ***************/
/******************************************************/
#if defined(AeroQuad_v1) || defined(AeroQuad_v1_IDG) || defined(AeroQuadMega_v1)
class Accel_AeroQuad_v1 : public Accel {
private:
  
public:
  Accel_AeroQuad_v1() : Accel() {
  }
  
/******************************************************/

  void initialize(void) {
    this->_initialize();
    accelChannel[XAXIS] = 1;
    accelChannel[YAXIS] = 0;
    accelChannel[ZAXIS] = 2;
  }
  
/******************************************************/

  void measure(void) {
    accelADC[XAXIS] = analogRead(accelChannel[PITCH]);
    accelADC[YAXIS] = analogRead(accelChannel[ROLL]);
    accelADC[ZAXIS] = analogRead(accelChannel[ZAXIS]);
    for (byte axis = XAXIS; axis < LASTAXIS; axis++) {
      accelData[axis] = filterSmooth(accelADC[axis], accelData[axis], smoothFactor);
    }
  }
};
#endif

/******************************************************/
/********* AeroQuad Mega v2 Accelerometer *************/
/******************************************************/
#if defined(AeroQuad_v18) || defined(AeroQuadMega_v2)

#define ACCEL_ADDRESS 0x40 // page 54 and 61 of datasheet

class Accel_AeroQuadMega_v2 : public Accel {
private:
  
public:
  Accel_AeroQuadMega_v2() : Accel(){
  }
  
/******************************************************/

  void initialize(void) {
    byte data;

    this->_initialize();
    // Check if accel is connected
    if (readWhoI2C(ACCEL_ADDRESS) != 0x03) // page 52 of datasheet
      Serial.println("Accelerometer not found!");

    // Thanks to SwiftingSpeed for updates on these settings
    // http://aeroquad.com/showthread.php?991-AeroQuad-Flight-Software-v2.0&p=11207&viewfull=1#post11207
    updateRegisterI2C(ACCEL_ADDRESS, 0x10, 0xB6); //reset device
    delay(10);  //sleep 10 ms after reset (page 25)

    // In datasheet, summary register map is page 21
    // Low pass filter settings is page 27
    // Range settings is page 28
    updateRegisterI2C(ACCEL_ADDRESS, 0x0D, 0x10); //enable writing to control registers
    sendByteI2C(ACCEL_ADDRESS, 0x20); // register bw_tcs (bits 4-7)
    data = readByteI2C(ACCEL_ADDRESS); // get current register value
    #if defined(Loop_400HZ)
      updateRegisterI2C(ACCEL_ADDRESS, 0x20, data & 0x7F); // set low pass filter to 1200Hz (value = 0111xxxx)
    #else
      updateRegisterI2C(ACCEL_ADDRESS, 0x20, data & 0x0F); // set low pass filter to 10Hz (value = 0000xxxx)
    #endif

    // From page 27 of BMA180 Datasheet
    //  1.0g = 0.13 mg/LSB
    //  1.5g = 0.19 mg/LSB
    //  2.0g = 0.25 mg/LSB
    //  3.0g = 0.38 mg/LSB
    //  4.0g = 0.50 mg/LSB
    //  8.0g = 0.99 mg/LSB
    // 16.0g = 1.98 mg/LSB
    sendByteI2C(ACCEL_ADDRESS, 0x35); // register offset_lsb1 (bits 1-3)
    data = readByteI2C(ACCEL_ADDRESS);
    data &= 0xF1;
    data |= 0x08; // Set range select bits for +/-4g
    updateRegisterI2C(ACCEL_ADDRESS, 0x35, data);
  }
  
/******************************************************/

  void sample(void) {
    Wire.beginTransmission(ACCEL_ADDRESS);
    Wire.send(0x02);
    Wire.endTransmission();
    Wire.requestFrom(ACCEL_ADDRESS, 6);
    #ifdef Loop_400HZ
      index++;
    #endif
    for (byte axis = XAXIS; axis < LASTAXIS; axis++) {
      #if defined(Loop_200HZ) || defined(Loop_400HZ)
        #ifdef Loop_200HZ
          accelRAW[axis][index ^ 1] = ((Wire.receive() | (Wire.receive() << 8)) >> 2);
        #else
          accelRAW[axis][index % 4] = ((Wire.receive() | (Wire.receive() << 8)) >> 2);
        #endif
      #else
        accelADC[axis] = ((Wire.receive() | (Wire.receive() << 8)) >> 2);
      #endif
        Serial.println(accelADC[axis]);
    }
    #ifdef Loop_200HZ
      index ^= 1;
    #endif
  }

/******************************************************/

  void measure(void) {
    #if defined(Loop_400HZ) || defined(Loop_200HZ)
      for (byte axis = XAXIS; axis < LASTAXIS; axis++) {
        #ifdef Loop_400HZ
          long tempLong = 0;
          for (byte i = 0; i <= LASTAXIS; i++) {
            tempLong += accelRAW[axis][i];
          }
          accelADC[axis] = (int)((tempLong + 2L) / 4L);
        #else
          accelADC[axis] = ((int)((long)((long)accelRAW[axis][0] + (long)accelRAW[axis][1] - 1L) / 2L)) + 1; // average the 2 samples with integer rounding
        #endif
        accelData[axis] = filterSmooth(accelADC[axis], accelData[axis], smoothFactor);
      }
    #else
      sample();
      for (byte axis = XAXIS; axis < LASTAXIS; axis++)
        accelData[axis] = filterSmooth(accelADC[axis], accelData[axis], smoothFactor);
    #endif
  }
};
#endif

/******************************************************/
/********* AeroQuad Mini v1 Accelerometer *************/
/******************************************************/
#if defined(AeroQuad_Mini)

#define ACCEL_ADDRESS 0x53 // page 10 of datasheet

class Accel_AeroQuadMini : public Accel {
private:
  
public:
  Accel_AeroQuadMini() : Accel(){
  }
  
/******************************************************/

  void initialize(void) {
    this->_initialize();
    // Check if accel is connected
    
    if (readWhoI2C(ACCEL_ADDRESS) !=  0xE5) // page 14 of datasheet
      Serial.println("Accelerometer not found!");

    updateRegisterI2C(ACCEL_ADDRESS, 0x2D, 1<<3); // set device to *measure*
    updateRegisterI2C(ACCEL_ADDRESS, 0x31, 0x09); // set full range and +/- 2G
    updateRegisterI2C(ACCEL_ADDRESS, 0x2C, 8+2+1);    // 200hz sampling
    delay(10); 
  }
  
/******************************************************/

  void sample(void) {
    sendByteI2C(ACCEL_ADDRESS, 0x32);
    Wire.requestFrom(ACCEL_ADDRESS, 6);
    #ifdef Loop_400HZ
      index++;
    #endif
    for (byte axis = XAXIS; axis < LASTAXIS; axis++) {
      #if defined(Loop_200HZ) || defined(Loop_400HZ)
        #ifdef Loop_200HZ
          accelRAW[axis][index ^ 1] = ((Wire.receive() | (Wire.receive() << 8)));
        #else
          accelRAW[axis][index % 4] = ((Wire.receive() | (Wire.receive() << 8)));
        #endif
      #else
        accelADC[axis] = ((Wire.receive() | (Wire.receive() << 8)));
      #endif
    }
    #ifdef Loop_200HZ
      index ^= 1;
    #endif
  }

/******************************************************/

  void measure(void) {
    #if defined(Loop_400HZ) || defined(Loop_200HZ)
      for (byte axis = XAXIS; axis < LASTAXIS; axis++) {
        #ifdef Loop_400HZ
          long tempLong = 0;
          for (byte i = 0; i <= LASTAXIS; i++) {
            tempLong += accelRAW[axis][i];
          }
          accelADC[axis] = (int)((tempLong + 2L) / 4L);
        #else
          accelADC[axis] = ((int)((long)((long)accelRAW[axis][0] + (long)accelRAW[axis][1] - 1L) / 2L)) + 1; // average the 2 samples with integer rounding
        #endif
        accelData[axis] = filterSmooth(accelADC[axis], accelData[axis], smoothFactor);
      }
    #else
      sample();
      for (byte axis = XAXIS; axis < LASTAXIS; axis++)
        accelData[axis] = filterSmooth(accelADC[axis], accelData[axis], smoothFactor);
    #endif
  }
};
#endif

/******************************************************/
/*********** ArduCopter ADC Accelerometer *************/
/******************************************************/
#ifdef ArduCopter
class Accel_ArduCopter : public Accel {
private:
  int rawADC;

public:
  Accel_ArduCopter() : Accel(){
  }
  
/******************************************************/

  void initialize(void) {
    this->_initialize();
    // old AQ way
    // rollChannel = 5
    // pitchChannel = 4
    // zAxisChannel = 6
    // new way in 2.3
    // rollChannel = 3
    // pitchChannel = 4
    // zAxisChannel = 5
    accelChannel[XAXIS] = 3;
    accelChannel[YAXIS] = 4;
    accelChannel[ZAXIS] = 5;
  }
  
  void measure(void) {
    for (byte axis = XAXIS; axis < LASTAXIS; axis++) {
      rawADC = analogRead_ArduCopter_ADC(accelChannel[axis]);
      if (rawADC > 500) { // Check if measurement good
          accelADC[axis] = rawADC;
        accelData[axis] = filterSmooth(accelADC[axis], accelData[axis], smoothFactor);
      }
    }
  }
};
#endif

/******************************************************/
/****************** Wii Accelerometer *****************/
/******************************************************/
#if defined(AeroQuad_Wii) || defined(AeroQuadMega_Wii)
class Accel_Wii : public Accel {
public:
  Accel_Wii() : Accel(){
  }
  
/******************************************************/

  void initialize(void) {
    this->_initialize();
  }
  
/******************************************************/

  void measure(void) {
    // Actual measurement performed in gyro class
    // We just update the appropriate variables here
  
    // Original Wii sensor orientation
    //accelADC[XAXIS] = NWMP_acc[YAXIS];
    //accelADC[YAXIS] = NWMP_acc[XAXIS];
    //accelADC[ZAXIS] = NWMP_acc[ZAXIS];
  
    accelADC[XAXIS] =  NWMP_acc[XAXIS];  // Configured for Paris MultiWii Board
    accelADC[YAXIS] =  NWMP_acc[YAXIS];  // Configured for Paris MultiWii Board
    accelADC[ZAXIS] =  NWMP_acc[ZAXIS];  // Configured for Paris MultiWii Board
    
    for (byte axis = XAXIS; axis < LASTAXIS; axis++) {
      accelData[axis] = filterSmooth(accelADC[axis], accelData[axis], smoothFactor);
    }
  }
};
#endif

/******************************************************/
/****************** CHR6DM Accelerometer **************/
/******************************************************/
#if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
class Accel_CHR6DM : public Accel {
public:
  Accel_CHR6DM() : Accel() {
    accelScaleFactor = 0;
  }

  void initialize(void) {
    smoothFactor = readFloat(ACCSMOOTH_ADR);
    accelZero[ROLL] = readFloat(LEVELROLLCAL_ADR);
    accelZero[PITCH] = readFloat(LEVELPITCHCAL_ADR);
    accelZero[ZAXIS] = readFloat(LEVELZCAL_ADR);
    accelOneG = readFloat(ACCEL1G_ADR);
    calibrate();
  }

  void measure(void) {
      accelADC[XAXIS] = chr6dm.data.ax - accelZero[XAXIS];
      accelADC[YAXIS] = chr6dm.data.ay - accelZero[YAXIS];
      accelADC[ZAXIS] = chr6dm.data.az - accelOneG;

      accelData[XAXIS] = filterSmooth(accelADC[XAXIS], accelData[XAXIS], smoothFactor); //to get around 1
      accelData[YAXIS] = filterSmooth(accelADC[YAXIS], accelData[YAXIS], smoothFactor);
      accelData[ZAXIS] = filterSmooth(accelADC[ZAXIS], accelData[ZAXIS], smoothFactor);

    //previousTime = currentTime; // AKA removed as a result of Honks original work, not needed further
  }    

  const int getFlightData(byte axis) {
    return getRaw(axis);
  }

  // Allows user to zero accelerometers on command
  void calibrate(void) {

   float zeroXreads[FINDZERO];
   float zeroYreads[FINDZERO];
   float zeroZreads[FINDZERO];


    for (int i=0; i<FINDZERO; i++) {
        chr6dm.requestAndReadPacket();
        zeroXreads[i] = chr6dm.data.ax;
        zeroYreads[i] = chr6dm.data.ay;
        zeroZreads[i] = chr6dm.data.az;
    }


    accelZero[XAXIS] = findMedian(zeroXreads, FINDZERO);
    accelZero[YAXIS] = findMedian(zeroYreads, FINDZERO);
    accelZero[ZAXIS] = findMedian(zeroZreads, FINDZERO);
   
    // store accel value that represents 1g
    accelOneG = accelZero[ZAXIS];
    // replace with estimated Z axis 0g value
    //accelZero[ZAXIS] = (accelZero[ROLL] + accelZero[PITCH]) / 2;

    writeFloat(accelOneG, ACCEL1G_ADR);
    writeFloat(accelZero[XAXIS], LEVELROLLCAL_ADR);
    writeFloat(accelZero[YAXIS], LEVELPITCHCAL_ADR);
    writeFloat(accelZero[ZAXIS], LEVELZCAL_ADR);
  }
/*  AKA - NOT USED
  void calculateAltitude() {
    currentTime = micros();
    if ((abs(CHR_RollAngle) < 5) && (abs(CHR_PitchAngle) < 5)) 
      rawAltitude += (getZaxis()) * ((currentTime - previousTime) / 1000000.0);
    previousTime = currentTime;
  } 
*/  
};
#endif


