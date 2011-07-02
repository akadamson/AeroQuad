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

/**************************************************************/
/************************** GPS *******************************/
/**************************************************************/
// This is experimental, it is not yet functional

class GPS_AeroQuad : public TinyGPS {
    
  private:
    HardwareSerial *gpsPort;
  
  public:
    #define EARTHK 6.371

    typedef struct {
      float x;
      float y;
    } XYOffset;

    GPSPosition holdPosition;
    GPSPosition currentPosition;
    XYOffset positionOffset;

    GPS_AeroQuad() : TinyGPS () {
    }

    void initialize(void) {
      gpsPort = &Serial1;
      gpsPort->begin(38400);
      gpsPort->println("$PMTK251,115200*1F"); // set to 115200
      delay(200);
      gpsPort->begin(115200);
      gpsPort->println("$PMTK220,200*2C"); // set to 5hz
      gpsPort->println("$PMTK301,2*2E"); // set to use WAAS
//      gpsPort->println("$PMTK300,200,0,0,0,0*2F");
      delay(200);
      #ifdef GPSBINARY
        gpsPort->println("$PGCMD,16,0,0,0,0,0*6A"); // turn off all strings, start binary transfer
      #endif
      #ifdef GPSNMEA
        gpsPort->println("$PGCMD,16,1,0,0,0,1*6A"); // turn only NMEA strings needed
//        gpsPort->println("$PMTK397,0*23"); // disable 1.0mps reporting threashold
      #endif
      holdPosition.valid = false;
      currentPosition.valid = false;
    }
    
    // main GPS process loop, may swtich to case statement and state machine
    void run(void) {
//      Serial.print(currentPosition.lat); Serial.print(","); Serial.print(currentPosition.lon); Serial.print(",");
//      Serial.print(holdPosition.lat); Serial.print(","); Serial.print(holdPosition.lon); Serial.println();
      
      this->getNewInput();
      if (this->fix() > 2)
        this->capturePosition(&this->currentPosition);
      // this next line probabably needs to be moved to the gps.run routine and executed as some internval in the executive
      if (this->currentPosition.valid && this->holdPosition.valid)
        calculateXYDistance();

    }
    
    // return the X or Y position distance for a given Axis
    float getAxisDistance(byte axis) {
      if (axis == PITCH)
        return (this->positionOffset.x);
      if (axis == ROLL)
        return (this->positionOffset.y);
    }
    
    // capture a GPS position in a GPSPosition structure
    void capturePosition(GPSPosition *position) {
      this->get_position(&position->lat, &position->lon);
      position->valid = true;
    }
    
    // return the velocity for a given axis
    // based upon gps direction oriented correctly for
    // vehicle heading
    float getAxisVelocity(byte axis, float trueHDG) {
      float course = radians(this->f_course());  // convert gps course to radians
      
      if (course >= PI) course -= (2*PI);       // convert to +/- PI
      if (course < -PI) course += (2*PI);       // convert to +/- PI
     
      if (axis == XAXIS)
        return(this->f_speed_mps() * cos(course - trueHDG));
      if (axis == YAXIS)
        return(this->f_speed_mps() * sin(course - trueHDG));
      else
        return(0);
    }

    // return the distance offset between 2 lat/lon gps positions.  
    // current/hold for example
    void calculateXYDistance(void) {
//      currentPosition.lat = 34084352;
//      currentPosition.lon = -83947184;
//      holdPosition.lat = 34084352;
//      holdPosition.lon = -83947184;

//      Serial.print(currentPosition.lat); Serial.print(","); Serial.print(currentPosition.lon); Serial.print(",");
      // subtract the lat/lon's
      long x =  currentPosition.lat - holdPosition.lat;
      long y =  currentPosition.lon - holdPosition.lon;

      // convert the lat/lon deltas to meters
      this->positionOffset.x = radians(x) * EARTHK;
      this->positionOffset.y = radians(y) * EARTHK * cos(radians(currentPosition.lat / 1000000));

//      Serial.print(holdPosition.lat); Serial.print(","); Serial.print(holdPosition.lon); Serial.print(",");
//      Serial.print(x); Serial.print(",");Serial.print(this->positionOffset.x,6); Serial.print(",");
//      Serial.print(y); Serial.print(",");Serial.println(this->positionOffset.y,6);  
    }
    
    // get new data from the GPS, decode it and store in the various structures
    bool getNewInput(void)
    {
      while (gpsPort->available())
      {
        #ifdef GPSBINARY
          if (this->binaryEncode(gpsPort->read()))
        #endif
        #ifdef GPSNMEA
          if (this->encode(gpsPort->read()))
        #endif
          return true;
      }
      return false;
    }
    
    #ifdef DEBUG_GPS
      void printFloat(double number, int digits = 2)
      {
        // Handle negative numbers
        if (number < 0.0)
        {
           SERIAL_PRINT('-');
           number = -number;
        }
      
        // Round correctly so that print(1.999, 2) prints as "2.00"
        double rounding = 0.5;
        for (uint8_t i=0; i<digits; ++i)
          rounding /= 10.0;
        
        number += rounding;
      
        // Extract the integer part of the number and print it
        unsigned long int_part = (unsigned long)number;
        double remainder = number - (double)int_part;
        SERIAL_PRINT(int_part);
      
        // Print the decimal point, but only if there are digits beyond
        if (digits > 0)
          SERIAL_PRINT("."); 
      
        // Extract digits from the remainder one at a time
        while (digits-- > 0)
        {
          remainder *= 10.0;
          int toPrint = int(remainder);
          SERIAL_PRINT(toPrint);
          remainder -= toPrint; 
        } 
      }
      
      void gpsdump(void)
      {
        long lat, lon;
        float flat, flon;
        unsigned long age, date, time, chars;
        int year;
        byte month, day, hour, minute, second, hundredths;
        unsigned short sentences, failed;
      
        this->get_position(&lat, &lon, &age);
        SERIAL_PRINT("Lat/Long(10^-6 deg): "); SERIAL_PRINT(lat); SERIAL_PRINT(", "); SERIAL_PRINT(lon); 
        SERIAL_PRINT(" Fix age: "); SERIAL_PRINT(age); SERIAL_PRINTLN("ms.");
        
        getNewInput(); // If we don't feed the gps during this long routine, we may drop characters and get checksum errors
      
        this->f_get_position(&flat, &flon, &age);
        SERIAL_PRINT("Lat/Long(float): "); printFloat(flat, 6); SERIAL_PRINT(", "); printFloat(flon, 6);
        SERIAL_PRINT(" Fix age: "); SERIAL_PRINT(age); SERIAL_PRINTLN("ms.");
        SERIAL_PRINT("Number of Sats: "); SERIAL_PRINT(this->getSats(), DEC); SERIAL_PRINT(" GPS Fix type: "); SERIAL_PRINTLN(this->fix(), DEC);
      
        getNewInput();
      
        this->get_datetime(&date, &time, &age);
        SERIAL_PRINT("Date(ddmmyy): "); SERIAL_PRINT(date); SERIAL_PRINT(" Time(hhmmsscc): "); SERIAL_PRINT(time);
        SERIAL_PRINT(" Fix age: "); SERIAL_PRINT(age); SERIAL_PRINTLN("ms.");
      
        getNewInput();
      
        this->crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
        SERIAL_PRINT("Date: "); SERIAL_PRINT(static_cast<int>(month)); SERIAL_PRINT("/"); SERIAL_PRINT(static_cast<int>(day)); SERIAL_PRINT("/"); SERIAL_PRINT(year);
        SERIAL_PRINT("  Time: "); SERIAL_PRINT(static_cast<int>(hour)); SERIAL_PRINT(":"); SERIAL_PRINT(static_cast<int>(minute)); SERIAL_PRINT(":"); SERIAL_PRINT(static_cast<int>(second)); SERIAL_PRINT("."); SERIAL_PRINT(static_cast<int>(hundredths));
        SERIAL_PRINT("  Fix age: ");  SERIAL_PRINT(age); SERIAL_PRINTLN("ms.");
        
        getNewInput();
      
        SERIAL_PRINT("Alt(cm): "); SERIAL_PRINT(this->altitude()); SERIAL_PRINT(" Course(10^-2 deg): "); SERIAL_PRINT(this->course()); SERIAL_PRINT(" Speed(10^-2 knots): "); SERIAL_PRINTLN(this->speed());
        SERIAL_PRINT("Alt(float): "); printFloat(this->f_altitude()); SERIAL_PRINT(" Course(float): "); printFloat(this->f_course()); SERIAL_PRINTLN();
        SERIAL_PRINT("Speed(knots): "); printFloat(this->f_speed_knots()); SERIAL_PRINT(" (mph): ");  printFloat(this->f_speed_mph());
        SERIAL_PRINT(" (mps): "); printFloat(this->f_speed_mps()); SERIAL_PRINT(" (kmph): "); printFloat(this->f_speed_kmph()); SERIAL_PRINTLN();
      
        getNewInput();
#ifndef _GPS_NO_STATS      
        this->stats(&chars, &sentences, &failed);
        SERIAL_PRINT("Stats: characters: "); SERIAL_PRINT(chars); SERIAL_PRINT(" sentences: "); SERIAL_PRINT(sentences); SERIAL_PRINT(" failed checksum: "); SERIAL_PRINTLN(failed);
#endif
        SERIAL_PRINTLN();
      }
    #endif  
};


