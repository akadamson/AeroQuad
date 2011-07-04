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

class Gyro {
public:
  float gyroScaleFactor;
  float smoothFactor;
  int gyroChannel[3];
  float gyroData[3];
  #if defined(Loop_200HZ) || defined(Loop_400HZ)
    byte index;
    #ifdef Loop_200HZ
      int gyroRAW[3][2];
    #else
      int gyroRAW[3][4];
    #endif    
  #endif
  #if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
    float gyroZero[3];
  #else
    int gyroZero[3];
  #endif
  int gyroADC[3];
  float gyroHeading;
  long previousGyroTime;

  Gyro(void) {
  }

  void _initialize(void) {  
    gyroZero[ROLL] = readFloat(GYRO_ROLL_ZERO_ADR);
    gyroZero[PITCH] = readFloat(GYRO_PITCH_ZERO_ADR);
    gyroZero[YAW] = readFloat(GYRO_YAW_ZERO_ADR);
    smoothFactor = readFloat(GYROSMOOTH_ADR);

    #if defined(Loop_200HZ) || defined(Loop_400HZ)
      #ifdef Loop_200HZ
        index = 1; // AKA index value for flip/flop store of 2 sample average
      #else
        index = 0;
      #endif
      // init flip/flop array to zero
      for (byte axis = ROLL; axis < LASTAXIS; axis++) {
        gyroRAW[axis][0] = 0;
        gyroRAW[axis][1] = 0;
        #ifdef Loop_400HZ
          gyroRAW[axis][2] = 0;
          gyroRAW[axis][3] = 0;
        #endif        
      }
    #endif
  }
  
  // returns the raw ADC value from the gyro 
  const int getRaw(byte axis) {
    return gyroADC[axis];
  }
  
  // returns the smoothed and scaled to SI units value of the Gyro
  const float getData(byte axis) {
    return gyroData[axis];
  }

  // returns the smooth factor used on the gyro
  const float getSmoothFactor(void) {
    return smoothFactor;
  }
  
  void setSmoothFactor(float value) {
    smoothFactor = value;
  }

  // returns gyro based heading as +/- PI in radians
  const float getHeading(void) {
    return gyroHeading;
  }
};

/******************************************************/
/****************** AeroQuad_v1 Gyro ******************/
/******************************************************/
#if defined(AeroQuad_v1) || defined(AeroQuad_v1_IDG) || defined(AeroQuadMega_v1)
class Gyro_AeroQuad_v1 : public Gyro {
public:
  Gyro_AeroQuad_v1() : Gyro() {
    gyroScaleFactor = radians((aref/1024.0) / 0.002);  // IDG/IXZ500 sensitivity = 2mV/(deg/sec)
    previousGyroTime = micros();
  }
  
/******************************************************/

  void initialize(void) {
  
    this->_initialize();
    analogReference(EXTERNAL);
    // Configure gyro auto zero pins
    pinMode (AZPIN, OUTPUT);
    digitalWrite(AZPIN, LOW);
    delay(1);

    // rollChannel = 4
    // pitchChannel = 3
    // yawChannel = 5
    gyroChannel[ROLL] = 4;
    gyroChannel[PITCH] = 3;
    gyroChannel[YAW]  = 5;
  }
  
/******************************************************/

  void measure(void) {
    for (byte axis = ROLL; axis < LASTAXIS; axis++) {
      if (axis == PITCH)
        gyroADC[axis] = analogRead(gyroChannel[axis]) - gyroZero[axis];
      else
        gyroADC[axis] = gyroZero[axis] - analogRead(gyroChannel[axis]);
      gyroData[axis] = filterSmooth(gyroADC[axis] * gyroScaleFactor, gyroData[axis], smoothFactor);
    }
    
    //  Calculate gyro based heading
    long currentGyroTime = micros();
    if (gyroData[YAW] > radians(1.0) || gyroData[YAW] < radians(-1.0))
      gyroHeading += gyroData[YAW] * ((currentGyroTime - previousGyroTime) / 1000000.0);
      previousGyroTime = currentGyroTime;
    if (gyroHeading > PI)
      gyroHeading -= (2*PI);
    if (gyroHeading < -PI)
      gyroHeading += (2*PI);
  }

/******************************************************/

  void calibrate() {
    autoZero();
    writeFloat(gyroZero[ROLL],  GYRO_ROLL_ZERO_ADR);
    writeFloat(gyroZero[PITCH], GYRO_PITCH_ZERO_ADR);
    writeFloat(gyroZero[YAW],   GYRO_YAW_ZERO_ADR);
  }
  
/******************************************************/

  void autoZero() {
    int findZero[FINDZERO];
    digitalWrite(AZPIN, HIGH);
    delayMicroseconds(750);
    digitalWrite(AZPIN, LOW);
    delay(8);

    for (byte calAxis = ROLL; calAxis < LASTAXIS; calAxis++) {
      for (int i=0; i<FINDZERO; i++)
        findZero[i] = analogRead(gyroChannel[calAxis]);
      gyroZero[calAxis] = findMedian(findZero, FINDZERO);
    }
  }
};
#endif

/******************************************************/
/****************** AeroQuad_v2 Gyro ******************/
/******************************************************/
#if defined(AeroQuad_v18) || defined(AeroQuadMega_v2) || defined(AeroQuad_Mini)
class Gyro_AeroQuadMega_v2 : public Gyro {
private:
  
public:
  Gyro_AeroQuadMega_v2() : Gyro() {
  #ifdef AeroQuad_Mini
    #define GYRO_ADDRESS 0xD0
  #else        
    #define GYRO_ADDRESS 0xD2
  #endif    
    gyroScaleFactor = radians(1.0 / 14.375);  //  ITG3200 14.375 LSBs per °/sec
    previousGyroTime = micros();
  }
  
/******************************************************/

  void initialize(void) {
    this->_initialize();
    // Check if gyro is connected
    twiMaster.start(GYRO_ADDRESS | I2C_WRITE);
    twiMaster.write(0x00);
    delay(100);
    twiMaster.start(GYRO_ADDRESS | I2C_READ);

    #ifdef AeroQuad_Mini    
      if (twiMaster.read(1) != GYRO_ADDRESS / 2 + 1)
    #else    
      if (twiMaster.read(1) != GYRO_ADDRESS / 2)
    #endif    
        SERIAL_PRINTLN("Gyro not found!");
    else {
      twiMaster.start(GYRO_ADDRESS | I2C_WRITE);
      twiMaster.write(0x3E);
      twiMaster.write(0x80);  // send a reset to the device
      
      delay(5);
  
      twiMaster.start(GYRO_ADDRESS | I2C_WRITE);
      twiMaster.write(0x16);
      #if defined(Loop_400HZ)
        twiMaster.write(0x18);  // 256hz filter 8k internal sample
      #else
        twiMaster.write(0x1D);  // 10Hz low pass filter 1k internal sample
      #endif
  
      delay(5);
      
      twiMaster.start(GYRO_ADDRESS | I2C_WRITE);
      twiMaster.write(0x3E);
      twiMaster.write(0x01);  // use PLL X axis gyro as oscilator
      
      delay(10);
    }
    twiMaster.stop();
  }
/******************************************************/

  void sample(void) {
    twiMaster.start(GYRO_ADDRESS | I2C_WRITE);
    twiMaster.write(0x1D);
    twiMaster.start(GYRO_ADDRESS | I2C_READ);
    #ifdef Loop_400HZ
      index++;
    #endif
    #if defined(Loop_200HZ) || defined(Loop_400HZ)
      #ifdef Loop_200HZ
      gyroRAW[ROLL][index ^ 1] =  ((twiMaster.read(0) << 8) | twiMaster.read(0))  - gyroZero[ROLL];
      gyroRAW[PITCH][index ^ 1] = gyroZero[PITCH] - ((twiMaster.read(0) << 8) | twiMaster.read(0));
      gyroRAW[YAW][index ^ 1]   = gyroZero[YAW]   - ((twiMaster.read(0) << 8) | twiMaster.read(1));
      index ^= 1;
      #else
      gyroRAW[ROLL][index % 4] =  ((twiMaster.read(0) << 8) | twiMaster.read(0))  - gyroZero[ROLL];
      gyroRAW[PITCH][index % 4] = gyroZero[PITCH] - ((twiMaster.read(0) << 8) | twiMaster.read(0));
      gyroRAW[YAW][index % 4]   = gyroZero[YAW]   - ((twiMaster.read(0) << 8) | twiMaster.read(1));
      #endif      
    #else
      gyroADC[ROLL] =  ((twiMaster.read(0) << 8) | twiMaster.read(0))  - gyroZero[ROLL];
      gyroADC[PITCH] = gyroZero[PITCH] - ((twiMaster.read(0) << 8) | twiMaster.read(0));
      gyroADC[YAW] =   gyroZero[YAW]   - ((twiMaster.read(0) << 8) | twiMaster.read(1));
    #endif

    twiMaster.stop();
  }
  
/******************************************************/

  void measure(void) {
    #if defined(Loop_400HZ) || defined(Loop_200HZ)
      for (byte axis = ROLL; axis < LASTAXIS; axis++) {
        #ifdef Loop_400HZ
          long tempLong = 0;
          for (byte i = 0; i <= LASTAXIS; i++) {
            tempLong += gyroRAW[axis][i];
          }
          gyroADC[axis] = (int)((tempLong + 2L) / 4L);
        #else
          gyroADC[axis] = ((int)((long)((long)gyroRAW[axis][0] + (long)gyroRAW[axis][1] - 1L) / 2L)) + 1; // average the 2 samples with integer rounding
        #endif
        gyroData[axis] = filterSmooth((float)gyroADC[axis] * gyroScaleFactor, gyroData[axis], smoothFactor);
      }
    #else
      sample();
      for (byte axis = ROLL; axis < LASTAXIS; axis++)
        gyroData[axis] = filterSmooth((float)gyroADC[axis] * gyroScaleFactor, gyroData[axis], smoothFactor);
    #endif
          
    //  Calculate gyro based heading
    long currentGyroTime = micros();
    if (gyroData[YAW] > radians(1.0) || gyroData[YAW] < radians(-1.0))
      gyroHeading += gyroData[YAW] * ((currentGyroTime - previousGyroTime) / 1000000.0);
    previousGyroTime = currentGyroTime;
    if (gyroHeading > PI)
      gyroHeading -= (2*PI);
    if (gyroHeading < -PI)
      gyroHeading += (2*PI);
  }
  
/******************************************************/

void calibrate() {
    autoZero();
    writeFloat(gyroZero[ROLL],  GYRO_ROLL_ZERO_ADR);
    writeFloat(gyroZero[PITCH], GYRO_PITCH_ZERO_ADR);
    writeFloat(gyroZero[YAW],   GYRO_YAW_ZERO_ADR);
  }
  
/******************************************************/

void autoZero() {
    int findZero[FINDZERO];
    for (byte calAxis = ROLL; calAxis < LASTAXIS; calAxis++) {
      for (int i=0; i<FINDZERO; i++) {
        twiMaster.start(GYRO_ADDRESS | I2C_WRITE);
        twiMaster.write((calAxis * 2) + 0x1D);
        twiMaster.start(GYRO_ADDRESS | I2C_READ);
        findZero[i] = ((twiMaster.read(0) << 8) | twiMaster.read(1));
        delay(10);
      }
      twiMaster.stop();
      gyroZero[calAxis] = findMedian(findZero, FINDZERO);
    }
  }
};
#endif

/******************************************************/
/**************** ArduCopter Gyro *********************/
/******************************************************/
#ifdef ArduCopter
class Gyro_ArduCopter : public Gyro {
private:
  int rawADC;

public:
  Gyro_ArduCopter() : Gyro() {
    gyroScaleFactor = radians((3.3/4096) / 0.002);  // IDG/IXZ500 sensitivity = 2mV/(deg/sec)
    previousGyroTime = micros();
  }
  
/******************************************************/

  void initialize(void) {
    this->_initialize();
    // old AQ way
    // rollChannel = 1
    // pitchChannel = 2
    // yawChannel = 0
    // revised in 2.3 way
    // rollChannel = 0
    // pitchChannel = 1
    // yawChannel = 2
    gyroChannel[ROLL] = 0;
    gyroChannel[PITCH] = 1;
    gyroChannel[YAW] = 2;
    this->_initialize();
    initialize_ArduCopter_ADC(); // this is needed for both gyros and accels, done once in this class
  }
  
/******************************************************/

  void measure(void) {
    for (byte axis = ROLL; axis < LASTAXIS; axis++) {
      rawADC = analogRead_ArduCopter_ADC(gyroChannel[axis]);
      if (rawADC > 500) // Check if good measurement
        if (axis == ROLL)
          gyroADC[axis] =  rawADC - gyroZero[axis];
        else
          gyroADC[axis] =  gyroZero[axis] - rawADC;
      gyroData[axis] = filterSmooth(gyroADC[axis] * gyroScaleFactor, gyroData[axis], smoothFactor);
    }
    
    //  Calculate gyro based heading
    long currentGyroTime = micros();
    if (gyroData[YAW] > radians(1.0) || gyroData[YAW] < radians(-1.0))
      gyroHeading += gyroData[YAW] * ((currentGyroTime - previousGyroTime) / 1000000.0);
      previousGyroTime = currentGyroTime;
    if (gyroHeading > PI)
      gyroHeading -= (2*PI);
    if (gyroHeading < -PI)
      gyroHeading += (2*PI);
  }

/******************************************************/

  void calibrate() {
    autoZero();
    writeFloat(gyroZero[ROLL],  GYRO_ROLL_ZERO_ADR);
    writeFloat(gyroZero[PITCH], GYRO_PITCH_ZERO_ADR);
    writeFloat(gyroZero[YAW],   GYRO_YAW_ZERO_ADR);
  }
  
/******************************************************/

  void autoZero() {
    int findZero[FINDZERO];
    for (byte calAxis = ROLL; calAxis < LASTAXIS; calAxis++) {
      for (int i=0; i<FINDZERO; i++) {
        findZero[i] = analogRead_ArduCopter_ADC(gyroChannel[calAxis]);
        delay(10);
      }
      gyroZero[calAxis] = findMedian(findZero, FINDZERO);
    }
  }
};
#endif

/******************************************************/
/********************** Wii Gyro **********************/
/******************************************************/
#if defined(AeroQuad_Wii) || defined(AeroQuadMega_Wii)
class Gyro_Wii : public Gyro {
private:
  #if defined(AeroQuad_Wii) || defined(AeroQuadMega_Wii)
    float wmpLowRangeToRadPerSec;
    float wmpHighRangeToRadPerSec;
  #endif

public:
  Gyro_Wii() : Gyro() {
    // Wii Motion+ has a low range and high range. Scaling is thought to be as follows:
    //
    // Vref = 1.35 volts
    // At 0 rate, reading is approximately 8063 bits
    // Scaling is then 1.35/8063, or 0.00016743 volts/bit
    //
    // Low Range
    //    440 degrees per second at 2.7 millivolts/degree (from datasheet)
    //    degrees per bit = 0.00016743 / 2.7 mVolts = 0.06201166 degrees per second per bit
    //                                              = 0.00108231 radians per second per bit
    // High Range
    //   2000 degrees per second at 0.5 millivolts/degree (from datasheet)
    //    degrees per bit = 0.00016743 / 0.5 mVolts = 0.33486295 degrees per second per bit
    //                                              = 0.00584446 radians per second per bit
    wmpLowRangeToRadPerSec  = 0.001082308;
    wmpHighRangeToRadPerSec = 0.005844461;

    previousGyroTime = micros();
  }
  
/******************************************************/

  void initialize(void) {
    this->_initialize();
    Init_Gyro_Acc(); // defined in DataAquisition.h
  }
  
/******************************************************/

  void measure(void) {
    updateControls(); // defined in DataAcquisition.h
    
    // Original Wii sensor orientation
    //gyroADC[ROLL]  = NWMP_gyro[1]  - gyroZero[1];
    //gyroADC[PITCH] = NWMP_gyro[0]  - gyroZero[0];
    //gyroADC[YAW]   = gyroZero[YAW] - NWMP_gyro[YAW];

    gyroADC[ROLL]  = gyroZero[ROLL]   - NWMP_gyro[ROLL];  // Configured for Paris MultiWii Board
    gyroADC[PITCH] = NWMP_gyro[PITCH] - gyroZero[PITCH];  // Configured for Paris MultiWii Board
    gyroADC[YAW]   = gyroZero[YAW]    - NWMP_gyro[YAW];   // Configured for Paris MultiWii Board
    
    for (byte axis = ROLL; axis < LASTAXIS; axis++) { 
      gyroScaleFactor = wmpSlow[axis] ? wmpLowRangeToRadPerSec : wmpHighRangeToRadPerSec ;  // if wmpSlow == 1, use low range conversion,
      gyroData[axis] = filterSmooth(gyroADC[axis] * gyroScaleFactor, gyroData[axis], smoothFactor); 
    }
    
    //  Calculate gyro based heading
    long currentGyroTime = micros();
    if (gyroData[YAW] > radians(1.0) || gyroData[YAW] < radians(-1.0))
      gyroHeading += gyroData[YAW] * ((currentGyroTime - previousGyroTime) / 1000000.0);
      previousGyroTime = currentGyroTime;
    if (gyroHeading > PI)
      gyroHeading -= (2*PI);
    if (gyroHeading < -PI)
      gyroHeading += (2*PI);
  }

/******************************************************/

  void calibrate() {
    autoZero();
    writeFloat(gyroZero[ROLL],  GYRO_ROLL_ZERO_ADR);
    writeFloat(gyroZero[PITCH], GYRO_PITCH_ZERO_ADR);
    writeFloat(gyroZero[YAW],   GYRO_YAW_ZERO_ADR);
  }

/******************************************************/

  void autoZero() {
    int findZero[FINDZERO];
  
    for (byte calAxis = ROLL; calAxis < LASTAXIS; calAxis++) {
      for (int i=0; i<FINDZERO; i++) {
        updateControls();
        findZero[i] = NWMP_gyro[calAxis];
      }
      gyroZero[calAxis] = findMedian(findZero, FINDZERO);
    }
  }
};
#endif

/******************************************************/
/********************** CHR6DM Gyro **********************/
/******************************************************/
#if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
class Gyro_CHR6DM : public Gyro {

public:
  Gyro_CHR6DM() : Gyro() {
   gyroScaleFactor = 0;
   previousGyroTime = micros();
  }

 /******************************************************/

 void initialize(void) {
    smoothFactor = readFloat(GYROSMOOTH_ADR);
    gyroZero[ROLL] = readFloat(GYRO_ROLL_ZERO_ADR);
    gyroZero[PITCH] = readFloat(GYRO_PITCH_ZERO_ADR);
    gyroZero[ZAXIS] = readFloat(GYRO_YAW_ZERO_ADR);
    initCHR6DM();
  }

/******************************************************/

  void measure(void) {
    readCHR6DM();
    gyroADC[ROLL] =  chr6dm.data.rollRate - gyroZero[ROLL];
    gyroADC[PITCH] = gyroZero[PITCH]      - chr6dm.data.pitchRate;
    gyroADC[YAW] =   chr6dm.data.yawRate  - gyroZero[ZAXIS];

    gyroData[ROLL] =  filterSmooth(gyroADC[ROLL],  gyroData[ROLL],  smoothFactor);
    gyroData[PITCH] = filterSmooth(gyroADC[PITCH], gyroData[PITCH], smoothFactor);
    gyroData[YAW] =   filterSmooth(gyroADC[YAW],   gyroData[YAW],   smoothFactor);
    
    //  Calculate gyro based heading
    long currentGyroTime = micros();
    if (gyroData[YAW] > radians(1.0) || gyroData[YAW] < radians(-1.0))
      gyroHeading += gyroData[YAW] * ((currentGyroTime - previousGyroTime) / 1000000.0);
      previousGyroTime = currentGyroTime;
    if (gyroHeading > PI)
      gyroHeading -= (2*PI);
    if (gyroHeading < -PI)
      gyroHeading += (2*PI);
  }

/******************************************************/

  void calibrate() {
    autoZero();
    writeFloat(gyroZero[ROLL],  GYRO_ROLL_ZERO_ADR);
    writeFloat(gyroZero[PITCH], GYRO_PITCH_ZERO_ADR);
    writeFloat(gyroZero[YAW],   GYRO_YAW_ZERO_ADR);
  }

/******************************************************/

  void autoZero() {
    float zeroXreads[FINDZERO];
    float zeroYreads[FINDZERO];
    float zeroZreads[FINDZERO];

    for (int i=0; i<FINDZERO; i++) {
        readCHR6DM();
        zeroXreads[i] = chr6dm.data.rollRate;
        zeroYreads[i] = chr6dm.data.pitchRate;
        zeroZreads[i] = chr6dm.data.yawRate;
    }

    gyroZero[XAXIS] = findMedian(zeroXreads, FINDZERO);
    gyroZero[YAXIS] = findMedian(zeroYreads, FINDZERO);
    gyroZero[ZAXIS] = findMedian(zeroZreads, FINDZERO);


  }
};
#endif
