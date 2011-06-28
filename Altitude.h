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

// Class to define sensors that can determine altitude

// ***********************************************************************
// ************************** Altitude Class *****************************
// ***********************************************************************

class Altitude {
public:
  double altitude, rawAltitude;
  float groundTemperature; // remove later
  float groundPressure; // remove later
  float groundAltitude;
  float smoothFactor;
  
  Altitude (void) { 
    altitude = 0;
    smoothFactor = 0.02;
  }

  // **********************************************************************
  // The following function calls must be defined inside any new subclasses
  // **********************************************************************
  virtual void initialize(void); 
  virtual void measure(void);
  
  // *********************************************************
  // The following functions are common between all subclasses
  // *********************************************************
  const float getData(void) {
    return altitude - groundAltitude;
  }
  
  const float getRaw(void) {
    return rawAltitude;
  }
  
  void setStartAltitude(float value) {
    altitude = value;
  }
  
  void measureGround(void) {
    // measure initial ground pressure (multiple samples)
    groundAltitude = 0;
    for (int i=0; i < 25; i++) {
      measure();
      delay(26);
      groundAltitude += rawAltitude;
    }
    groundAltitude = groundAltitude / 25.0;
  }
  
  void setGroundAltitude(float value) {
    groundAltitude = value;
  }
  
  const float getGroundAltitude(void) {
    return groundAltitude;
  }
  
  void setSmoothFactor(float value) {
    smoothFactor = value;
  }
  
  const float getSmoothFactor(void) {
    return smoothFactor;
  }
};

// ***********************************************************************
// ************************* MPX_SCC Subclass ****************************
// ***********************************************************************

class Altitude_MPX_SCC : public Altitude {
private:
  uint16_t rawADC;
  float baroScaleFactor;

public: 
  Altitude_MPX_SCC() : Altitude() {
    #define ALTITUDE_ADDRESS 0x90 // 0x48 is 7bit address
    groundAltitude = 0;
    baroScaleFactor = 0.018; // experimentally found
  }

  // ***********************************************************
  // Define all the virtual functions declared in the main class
  // ***********************************************************
  void initialize(void) {     
    twiMaster.start(ALTITUDE_ADDRESS | I2C_WRITE);
    twiMaster.write(0x01); // config register
    twiMaster.write(0xC2); // msb
    twiMaster.write(0xE3); // lsb
    twiMaster.start(ALTITUDE_ADDRESS | I2C_WRITE);
    twiMaster.write(0x00); // conversion register
    twiMaster.stop();
   
    measureGround();
    setStartAltitude(getGroundAltitude());
  }
  
  void measure(void) {
    twiMaster.start(ALTITUDE_ADDRESS | I2C_READ);
    rawADC = ((twiMaster.read(0) << 8) | twiMaster.read(1));
      
    if (rawADC > 500 && rawADC < 32000) {
      // 16-1 bit since not in differential mode = 32768 steps
      #ifdef UseLED_Library
        mainLED.on();
      #else
        digitalWrite(LEDPIN, HIGH);
      #endif
    } else {
      #ifdef UseLED_Library
        mainLED.off();
      #else 
        digitalWrite(LEDPIN, LOW); // out of range
      #endif
      altitudeHold = ALTPANIC;  // force manual control
    }
 
    twiMaster.stop();
         
    rawAltitude = rawADC * baroScaleFactor;
    altitude = rawAltitude;
  }
};

// ***********************************************************************
// ************************* BMP085 Subclass *****************************
// ***********************************************************************
class Altitude_AeroQuad_v2 : public Altitude {
// This sets up the BMP085 from Sparkfun
// Code from http://wiring.org.co/learning/libraries/bmp085.html
// Also made bug fixes based on BMP085 library from Jordi Munoz and Jose Julio
private:
  byte overSamplingSetting;
  int ac1, ac2, ac3;
  unsigned int ac4, ac5, ac6;
  int b1, b2, mb, mc, md;
  long pressure;
  long temperature;
  long rawPressure, rawTemperature;
  byte select, pressureCount;
  float pressureFactor;
  #define ALTITUDE_ADDRESS 0xEE
  
  void requestRawPressure(void) {
    twiMaster.start(ALTITUDE_ADDRESS | I2C_WRITE);
    twiMaster.write(0xF4);
    twiMaster.write(0x34 + (overSamplingSetting << 6));
    twiMaster.stop();
  }
  
  long readRawPressure(void) {
    unsigned char msb, lsb, xlsb;
    twiMaster.start(ALTITUDE_ADDRESS | I2C_WRITE);
    twiMaster.write(0xF6);
    twiMaster.start(ALTITUDE_ADDRESS | I2C_READ);
    msb = twiMaster.read(0);
    lsb = twiMaster.read(0);
    xlsb = twiMaster.read(1);
    twiMaster.stop();
    return (((long)msb<<16) | ((long)lsb<<8) | ((long)xlsb)) >>(8-overSamplingSetting);
  }

  void requestRawTemperature(void) {
    twiMaster.start(ALTITUDE_ADDRESS | I2C_WRITE);
    twiMaster.write(0xF4);
    twiMaster.write(0x2E);
    twiMaster.stop();
  }
  
  unsigned int readRegister(byte r) {
    unsigned char msb, lsb;
    twiMaster.start(ALTITUDE_ADDRESS | I2C_WRITE);
    twiMaster.write(r);
    twiMaster.start(ALTITUDE_ADDRESS | I2C_READ);
    msb = twiMaster.read(0);
    lsb = twiMaster.read(1);
    twiMaster.stop();
    return(((int)msb<<8) | ((int)lsb));
  }

public: 
  Altitude_AeroQuad_v2() : Altitude(){
    // oversampling setting
    // 0 = ultra low power
    // 1 = standard
    // 2 = high
    // 3 = ultra high resolution
    overSamplingSetting = 3;
    pressure = 0;
    groundPressure = 0;
    temperature = 0;
    groundTemperature = 0;
    groundAltitude = 0;
    pressureFactor = 1/5.255;
  }

  // ***********************************************************
  // Define all the virtual functions declared in the main class
  // ***********************************************************
  void initialize(void) {
    ac1 = readRegister(0xAA);
    ac2 = readRegister(0xAC);
    ac3 = readRegister(0xAE);
    ac4 = readRegister(0xB0);
    ac5 = readRegister(0xB2);
    ac6 = readRegister(0xB4);
    b1 = readRegister(0xB6);
    b2 = readRegister(0xB8);
    mb = readRegister(0xBA);
    mc = readRegister(0xBC);
    md = readRegister(0xBE);

    requestRawTemperature(); // setup up next measure() for temperature
    select = TEMPERATURE;
    pressureCount = 0;
    measure();
    delay(5); // delay for temperature
    measure();
    delay(26); // delay for pressure
    measureGround();
    // check if measured ground altitude is valid
    while (abs(getRaw() - getGroundAltitude()) > 10) {
      delay(26);
      measureGround();
    }
    setStartAltitude(getGroundAltitude());
  }
  
  void measure(void) {
    long x1, x2, x3, b3, b5, b6, p;
    unsigned long b4, b7;
    int32_t tmp;

    // switch between pressure and tempature measurements
    // each loop, since it's slow to measure pressure
    if (select == PRESSURE) {
      rawPressure = readRawPressure();
      if (pressureCount == 4) {
        requestRawTemperature();
        pressureCount = 0;
       select = TEMPERATURE;
      }
      else
        requestRawPressure();
      pressureCount++;
    }
    else { // select must equal TEMPERATURE
      rawTemperature = (long)readRegister(0xF6);
      requestRawPressure();
      select = PRESSURE;
    }
    
    //calculate true temperature
    x1 = ((long)rawTemperature - ac6) * ac5 >> 15;
    x2 = ((long) mc << 11) / (x1 + md);
    b5 = x1 + x2;
    temperature = ((b5 + 8) >> 4);
  
    //calculate true pressure
    b6 = b5 - 4000;
    x1 = (b2 * (b6 * b6 >> 12)) >> 11; 
    x2 = ac2 * b6 >> 11;
    x3 = x1 + x2;
 
    // Real Bosch formula - b3 = ((((int32_t)ac1 * 4 + x3) << overSamplingSetting) + 2) >> 2;
    // The version below is the same, but takes less program space
    tmp = ac1;
    tmp = (tmp * 4 + x3) << overSamplingSetting;
    b3 = (tmp + 2) >> 2;
 
    x1 = ac3 * b6 >> 13;
    x2 = (b1 * (b6 * b6 >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (ac4 * (uint32_t) (x3 + 32768)) >> 15;
    b7 = ((uint32_t) rawPressure - b3) * (50000 >> overSamplingSetting);
    p = b7 < 0x80000000 ? (b7 << 1) / b4 : (b7 / b4) << 1;
    
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    pressure = (p + ((x1 + x2 + 3791) >> 4));
    
    rawAltitude = 44330 * (1 - pow(pressure/101325.0, pressureFactor)); // returns absolute altitude in meters
    // save the line below it will remove approx 700 bytes of code for the 328p platform, but it's unknown how accurate the result will be.
    //rawAltitude = (101325.0-pressure)/4096*346;
    altitude = filterSmooth(rawAltitude, altitude, smoothFactor);
  }
};


