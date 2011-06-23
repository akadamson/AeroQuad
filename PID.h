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

// Modified from http://www.arduino.cc/playground/Main/BarebonesPIDForEspresso
float updatePID(float targetPosition, float currentPosition, struct PIDdata *PIDparameters) {
  float error;
  float pTerm;
  float dTerm;
  float iTerm;
  float pidOut;
  
  // AKA PID experiments
  float deltaPIDTime = (currentTime - PIDparameters->previousPIDTime) / 1000000.0;

  PIDparameters->previousPIDTime = currentTime;  // AKA PID experiments
  
  error = targetPosition - currentPosition;

// AKA PID experiments
// special case of +/- PI
/*
  if (PIDparameters->pidID == HEADING) {
    if (error >= PI) error -= (2*PI);
    if (error < -PI) error += (2*PI);
  }
*/    
  
  if (PIDparameters->firstPass) { // AKA PID experiments
    PIDparameters->firstPass = false;
    return (constrain(error, -PIDparameters->windupGuard, PIDparameters->windupGuard));
  }

  PIDparameters->integratedError += error * deltaPIDTime;
  PIDparameters->integratedError = constrain(PIDparameters->integratedError, -PIDparameters->windupGuard, PIDparameters->windupGuard);
  
  pTerm = PIDparameters->P * error;
  iTerm = PIDparameters->I * PIDparameters->integratedError;
  dTerm = PIDparameters->D * (currentPosition - PIDparameters->lastPosition) / (deltaPIDTime * 100); // dT fix from Honk
  pidOut = pTerm + iTerm + dTerm;

  PIDparameters->lastPosition = currentPosition;
  
  return pidOut;
}

void zeroIntegralError() __attribute__ ((noinline));
void zeroIntegralError() {
  for (byte axis = ROLL; axis < LASTLEVELAXIS; axis++) {
    PID[axis].integratedError = 0;
    PID[axis].previousPIDTime = currentTime;
  }
}



