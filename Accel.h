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
  #ifdef Loop_200HZ
    int accelRAW[3][2];
    byte index;
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
    
    #ifdef Loop_200HZ
      index = 1;  // AKA index value for flip/flop store of 2 sample average
      // init flip/flop array to zero
      for (byte axis = XAXIS; axis < LASTAXIS; axis++) {
        accelRAW[axis][0] = 0;
        accelRAW[axis][1] = 0;
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
class Accel_AeroQuadMega_v2 : public Accel {
private:
//  int accelAddress;
  
public:
  Accel_AeroQuadMega_v2() : Accel(){
    #define ACCEL_ADDRESS 0x80  // page 54 and 61 of datasheet
  }
  
/******************************************************/

  void initialize(void) {
    byte data;

    this->_initialize();
    // Check if accel is connected
    twiMaster.start(ACCEL_ADDRESS | I2C_WRITE);
    twiMaster.write(0x00);
    delay(100);
    twiMaster.start(ACCEL_ADDRESS | I2C_READ);
    if (twiMaster.read(1) != 0x03)                
      Serial.println("Accelerometer not found!");
    else {
      twiMaster.start(ACCEL_ADDRESS | I2C_WRITE);
      twiMaster.write(0x10);
      twiMaster.write(0xB6);  // Reset device
      
      delay(10); // sleep 10 ms after reset (page 25)
      
      twiMaster.start(ACCEL_ADDRESS | I2C_WRITE);
      twiMaster.write(0x0D);
      twiMaster.write(0x10);  // Enable writting to control registers
      
      twiMaster.start(ACCEL_ADDRESS | I2C_WRITE);
      twiMaster.write(0x20);  // Register bw_tcs (bits 4-7)
      
      twiMaster.start(ACCEL_ADDRESS | I2C_READ);
      data = twiMaster.read(1);
      
      twiMaster.start(ACCEL_ADDRESS | I2C_WRITE);
      twiMaster.write(0x20);
      twiMaster.write(data & 0x0F);  // Set low pass filter to 10 Hz (value = 0000xxxx)
      
      twiMaster.start(ACCEL_ADDRESS | I2C_WRITE);
      twiMaster.write(0x35);
      
      twiMaster.start(ACCEL_ADDRESS | I2C_READ);
      data = twiMaster.read(1);
      data &= 0xF1;
      data |= 0x08;
      
      twiMaster.start(ACCEL_ADDRESS | I2C_WRITE);
      twiMaster.write(0x35);
      twiMaster.write(data);  // Set range select bits for +/- 4g
    }
    twiMaster.stop();
  }
  
/******************************************************/

  void sample(void) {
    twiMaster.start(ACCEL_ADDRESS | I2C_WRITE);
    twiMaster.write(0x02);
    twiMaster.start(ACCEL_ADDRESS | I2C_READ);
    for (byte axis = XAXIS; axis < LASTAXIS; axis++) {
      #ifdef Loop_200HZ
        accelRAW[axis][index ^= 1] = ((twiMaster.read(0)|(twiMaster.read((axis*2+1) == 5) << 8)));
      #else
        accelADC[axis] = ((twiMaster.read(0)|(twiMaster.read((axis*2+1) == 5) << 8)));
      #endif
    }
    twiMaster.stop();
  }

/******************************************************/

  void measure(void) {
    for (byte axis = XAXIS; axis < LASTAXIS; axis++) {
      #ifdef Loop_200HZ
        accelADC[axis] = ((int)((long)((long)accelRAW[axis][0] + (long)accelRAW[axis][1] - 1L) / 2L)) + 1; // average the 2 samples with integer rounding
      #else
        sample();
      #endif

      accelData[axis] = filterSmooth(accelADC[axis], accelData[axis], smoothFactor);
    }
  }
};
#endif

/******************************************************/
/********* AeroQuad Mini v1 Accelerometer *************/
/******************************************************/
#if defined(AeroQuad_Mini)
class Accel_AeroQuadMini : public Accel {
private:
  
public:
  Accel_AeroQuadMini() : Accel(){
    #define ACCEL_ADDRESS 0xA6  // page 10 of datasheet
  }
  
/******************************************************/

  void initialize(void) {
    this->_initialize();
    twiMaster.start(ACCEL_ADDRESS | I2C_WRITE);
    twiMaster.write(0x00);
    delay(100);
    twiMaster.start(ACCEL_ADDRESS | I2C_READ);
    if (twiMaster.read(1) != 0xE5)
      Serial.println("Accelerometer not found!");
    else {
      twiMaster.start(ACCEL_ADDRESS | I2C_WRITE);  // set device to *measure*
      twiMaster.write(0x2D);
      twiMaster.write(0x08);
  
      twiMaster.start(ACCEL_ADDRESS | I2C_WRITE);  // set full resolution and +/- 4G
      twiMaster.write(0x31);
      twiMaster.write(0x09);
  
      twiMaster.start(ACCEL_ADDRESS | I2C_WRITE);  // 200hz sampling
      twiMaster.write(0x2C);
      twiMaster.write(0x0B);
    }
    twiMaster.stop();
    delay(10); 
  }
  
/******************************************************/

  void sample(void) {
    twiMaster.start(ACCEL_ADDRESS | I2C_WRITE);
    twiMaster.write(0x32);
    twiMaster.start(ACCEL_ADDRESS | I2C_READ);
    for (byte axis = XAXIS; axis < LASTAXIS; axis++) {
      #ifdef Loop_200HZ
        accelRAW[axis][index ^= 1] = ((twiMaster.read(0)|(twiMaster.read((axis*2+1) == 5) << 8)));
      #else
        accelADC[axis] = ((twiMaster.read(0)|(twiMaster.read((axis*2+1) == 5) << 8)));
      #endif
    }
    twiMaster.stop();
  }
/******************************************************/

  void measure(void) {
    for (byte axis = XAXIS; axis < LASTAXIS; axis++) {
      #ifdef Loop_200HZ
        accelADC[axis] = ((accelRAW[axis][0] + accelRAW[axis][1] - 1) / 2 ) + 1; // average the 2 samples with integer rounding
      #else
        sample();
      #endif

      accelData[axis] = filterSmooth(accelADC[axis], accelData[axis], smoothFactor);
    }
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


