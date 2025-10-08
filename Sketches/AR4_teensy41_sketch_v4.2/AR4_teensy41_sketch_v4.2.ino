//VERSION 4.2

/*  AR4 - Stepper motor robot control software
    Copyright (c) 2023, Chris Annin
    All rights reserved.

    You are free to share, copy and redistribute in any medium
    or format.  You are free to remix, transform and build upon
    this material.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

          Redistributions of source code must retain the above copyright
          notice, this list of conditions and the following disclaimer.
          Redistribution of this software in source or binary forms shall be free
          of all charges or fees to the recipient of this software.
          Redistributions in binary form must reproduce the above copyright
          notice, this list of conditions and the following disclaimer in the
          documentation and/or other materials provided with the distribution.
          you must give appropriate credit and indicate if changes were made. You may do
          so in any reasonable manner, but not in any way that suggests the
          licensor endorses you or your use.
          Selling Annin Robotics software, robots, robot parts, or any versions of robots or software based on this
          work is strictly prohibited.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL CHRIS ANNIN BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    chris.annin@gmail.com

*/


// VERSION LOG
// 1.0 - 2/6/21 - initial release
// 1.1 - 2/20/21 - bug fix, calibration offset on negative axis calibration direction axis 2,4,5
// 2.0 - 10/1/22 - added lookahead and spline functionality
// 2.2 - 11/6/22 - added Move V for open cv integrated vision
// 3.0 - 2/3/23 - open loop bypass moved to teensy board / add external axis 8 & 9 / bug fix live jog drift
// 3.1 - 5/10/23 - gcode initial
// 3.2 - 5/12/23 - remove RoboDK kinematics
// 3.3 - 6/4/23 - update geometric kinematics
// 4.0 - 11/5/23 - .txt .ar4 extension, gcode tab, kinematics tab. Initial MK2 release.
// 4.1 - 11/23/23 - bug fix added - R06_neg_matrix[2][3] = -DHparams[5][2]; added to UPdate CMD & GCC diagnostic
// 4.2 - 1/12/24 - bug fix - step dir delay


#include <math.h>
#include <avr/pgmspace.h>
#include <Encoder.h>
#pragma GCC diagnostic ignored "-Warray-bounds"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wsequence-point"
#pragma GCC diagnostic ignored "-Wunused-value"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#pragma GCC diagnostic ignored "-Waddress"

String cmdBuffer1;
String cmdBuffer2;
String cmdBuffer3;
String inData;
String recData;
String checkData;
String function;
volatile byte state = LOW;


const int J1stepPin = 0;
const int J1dirPin = 1;
const int J2stepPin = 2;
const int J2dirPin = 3;
const int J3stepPin = 4;
const int J3dirPin = 5;
const int J4stepPin = 6;
const int J4dirPin = 7;
const int J5stepPin = 8;
const int J5dirPin = 9;
const int J6stepPin = 10;
const int J6dirPin = 11;
const int J7stepPin = 12;
const int J7dirPin = 13;
const int J8stepPin = 32;
const int J8dirPin = 33;
const int J9stepPin = 34;
const int J9dirPin = 35;

const int J1calPin = 26;
const int J2calPin = 27;
const int J3calPin = 28;
const int J4calPin = 29;
const int J5calPin = 30;
const int J6calPin = 31;
const int J7calPin = 36;
const int J8calPin = 37;
const int J9calPin = 38;


const int Input39 = 39;

const int Output40 = 40;
const int Output41 = 41;


//set encoder multiplier
float J1encMult = 10;
float J2encMult = 10;
float J3encMult = 10;
float J4encMult = 10;
float J5encMult = 5;
float J6encMult = 10;

//set encoder pins
Encoder J1encPos(14, 15);
Encoder J2encPos(17, 16);
Encoder J3encPos(19, 18);
Encoder J4encPos(20, 21);
Encoder J5encPos(23, 22);
Encoder J6encPos(24, 25);


// GLOBAL VARS //

//define axis limits in degrees
float J1axisLimPos = 170;
float J1axisLimNeg = 170;
float J2axisLimPos = 90;
float J2axisLimNeg = 42;
float J3axisLimPos = 52;
float J3axisLimNeg = 89;
float J4axisLimPos = 165;
float J4axisLimNeg = 165;
float J5axisLimPos = 105;
float J5axisLimNeg = 105;
float J6axisLimPos = 155;
float J6axisLimNeg = 155;
float J7axisLimPos = 3450;
float J7axisLimNeg = 0;
float J8axisLimPos = 3450;
float J8axisLimNeg = 0;
float J9axisLimPos = 3450;
float J9axisLimNeg = 0;

int J1MotDir = 0;
int J2MotDir = 1;
int J3MotDir = 1;
int J4MotDir = 1;
int J5MotDir = 1;
int J6MotDir = 1;
int J7MotDir = 1;
int J8MotDir = 1;
int J9MotDir = 1;

int J1CalDir = 1;
int J2CalDir = 0;
int J3CalDir = 1;
int J4CalDir = 0;
int J5CalDir = 0;
int J6CalDir = 1;
int J7CalDir = 0;
int J8CalDir = 0;
int J9CalDir = 0;

//define total axis travel
float J1axisLim = J1axisLimPos + J1axisLimNeg;
float J2axisLim = J2axisLimPos + J2axisLimNeg;
float J3axisLim = J3axisLimPos + J3axisLimNeg;
float J4axisLim = J4axisLimPos + J4axisLimNeg;
float J5axisLim = J5axisLimPos + J5axisLimNeg;
float J6axisLim = J6axisLimPos + J6axisLimNeg;
float J7axisLim = J7axisLimPos + J7axisLimNeg;
float J8axisLim = J8axisLimPos + J8axisLimNeg;
float J9axisLim = J9axisLimPos + J9axisLimNeg;

//motor steps per degree
float J1StepDeg = 44.4444;
float J2StepDeg = 55.5555;
float J3StepDeg = 55.5555;
float J4StepDeg = 49.7777;
float J5StepDeg = 21.8602;
float J6StepDeg = 22.2222;
float J7StepDeg = 14.2857;
float J8StepDeg = 14.2857;
float J9StepDeg = 14.2857;

//steps full movement of each axis
int J1StepLim = J1axisLim * J1StepDeg;
int J2StepLim = J2axisLim * J2StepDeg;
int J3StepLim = J3axisLim * J3StepDeg;
int J4StepLim = J4axisLim * J4StepDeg;
int J5StepLim = J5axisLim * J5StepDeg;
int J6StepLim = J6axisLim * J6StepDeg;
int J7StepLim = J7axisLim * J7StepDeg;
int J8StepLim = J8axisLim * J8StepDeg;
int J9StepLim = J9axisLim * J9StepDeg;

//step and axis zero
int J1zeroStep = J1axisLimNeg * J1StepDeg;
int J2zeroStep = J2axisLimNeg * J2StepDeg;
int J3zeroStep = J3axisLimNeg * J3StepDeg;
int J4zeroStep = J4axisLimNeg * J4StepDeg;
int J5zeroStep = J5axisLimNeg * J5StepDeg;
int J6zeroStep = J6axisLimNeg * J6StepDeg;
int J7zeroStep = J7axisLimNeg * J7StepDeg;
int J8zeroStep = J8axisLimNeg * J8StepDeg;
int J9zeroStep = J9axisLimNeg * J9StepDeg;

//start master step count at Jzerostep
int J1StepM = J1zeroStep;
int J2StepM = J2zeroStep;
int J3StepM = J3zeroStep;
int J4StepM = J4zeroStep;
int J5StepM = J5zeroStep;
int J6StepM = J6zeroStep;
int J7StepM = J7zeroStep;
int J8StepM = J8zeroStep;
int J9StepM = J9zeroStep;



//degrees from limit switch to offset calibration
float J1calBaseOff = -1;
float J2calBaseOff = 1.5;
float J3calBaseOff = 4.1;
float J4calBaseOff = -2;
float J5calBaseOff = 10.1;
float J6calBaseOff = -.5;
float J7calBaseOff = 0;
float J8calBaseOff = 0;
float J9calBaseOff = 0;

//reset collision indicators
int J1collisionTrue = 0;
int J2collisionTrue = 0;
int J3collisionTrue = 0;
int J4collisionTrue = 0;
int J5collisionTrue = 0;
int J6collisionTrue = 0;
int TotalCollision = 0;
int KinematicError = 0;

float J7length;
float J7rot;
float J7steps;

float J8length;
float J8rot;
float J8steps;

float J9length;
float J9rot;
float J9steps;

float lineDist;

String WristCon;
int Quadrant;

unsigned long J1DebounceTime = 0;
unsigned long J2DebounceTime = 0;
unsigned long J3DebounceTime = 0;
unsigned long J4DebounceTime = 0;
unsigned long J5DebounceTime = 0;
unsigned long J6DebounceTime = 0;
unsigned long debounceDelay = 50;

String Alarm = "0";
String speedViolation = "0";
float minSpeedDelay = 500;
float linWayDistSP = 1;
String debug = "";
String flag = "";
const int TRACKrotdir = 0;
float JogStepInc = .5;

int J1EncSteps;
int J2EncSteps;
int J3EncSteps;
int J4EncSteps;
int J5EncSteps;
int J6EncSteps;

int J1LoopMode;
int J2LoopMode;
int J3LoopMode;
int J4LoopMode;
int J5LoopMode;
int J6LoopMode;

#define ROBOT_nDOFs 6

//declare in out vars
float xyzuvw_Out[ROBOT_nDOFs];
float xyzuvw_In[ROBOT_nDOFs];
float xyzuvw_Temp[ROBOT_nDOFs];

float JangleOut[ROBOT_nDOFs];
float JangleIn[ROBOT_nDOFs];

//external axis
float J7_pos;
float J8_pos;
float J9_pos;

float J7_In;
float J8_In;
float J9_In;

#define Table_Size 6
typedef float Matrix4x4[16];
typedef float tRobot[66];

float pose[16];

String moveSequence;

//define rounding vars
float rndArcStart[6];
float rndArcMid[6];
float rndArcEnd[6];
float rndCalcCen[6];
String rndData;
bool rndTrue;
float rndSpeed;
bool splineTrue;
bool splineEndReceived;

float Xtool = 0;
float Ytool = 0;
float Ztool = 0;
float RZtool = 0;
float RYtool = 0;
float RXtool = 0;



//DENAVIT HARTENBERG PARAMETERS

float DHparams[6][4] = {
  {    0,    -90,  169.77,   64.2  },
  {  -90,      0,       0,    305  },
  {  180,     90,       0, -.0001  },
  {    0,    -90,  222.63,      0  },
  {    0,     90,       0,      0  },
  {    0,      0,   36.25,      0  }
};


//DECLARE TOOL FRAME
float toolFrame[4][4];
float toolFrameRev[4][4];


//DECLARE R06 NEG FRAME

float R06_neg_matrix[4][4] = {
  {1,  0,  0,  0},
  {0,  1,  0,  0},
  {0,  0,  1,  -DHparams[5][2]},
  {0,  0,  0,  1}
};


//DECLARE JOINT MATRICES

float J1matrix [4][4];
float J2matrix [4][4];
float J3matrix [4][4];
float J4matrix [4][4];
float J5matrix [4][4];
float J6matrix [4][4];

float J1matrix_rev [4][4];
float J2matrix_rev [4][4];
float J3matrix_rev [4][4];

float R02matrix [4][4];
float R03matrix [4][4];
float R04matrix [4][4];
float R05matrix [4][4];
float R06matrix [4][4];
float R0Tmatrix [4][4];

float R02matrix_rev [4][4];
float R03matrix_rev [4][4];

float R0T_rev_matrix [4][4];
float InvtoolFrame [4][4];
float R06_rev_matrix [4][4];
float R05_rev_matrix [4][4];
float InvR03matrix_rev [4][4];
float R03_6matrix [4][4];


float blank [4][4];

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//CONSTRUCT TOOL MATRIX
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Tool_Matrix(float Xval, float Yval, float Zval, float RZval, float RYval, float RXval) {
  toolFrame [0][0] = cos(radians(RZval)) * cos(radians(RYval));
  toolFrame [0][1] = cos(radians(RZval)) * sin(radians(RYval)) * sin(radians(RXval)) - sin(radians(RZval)) * cos(radians(RXval));
  toolFrame [0][2] = cos(radians(RZval)) * sin(radians(RYval)) * cos(radians(RXval)) + sin(radians(RZval)) * sin(radians(RXval));
  toolFrame [0][3] = (Xval);
  toolFrame [1][0] = sin(radians(RZval)) * cos(radians(RYval));
  toolFrame [1][1] = sin(radians(RZval)) * sin(radians(RYval)) * sin(radians(RXval)) + cos(radians(RZval)) * cos(radians(RXval));
  toolFrame [1][2] = sin(radians(RZval)) * sin(radians(RYval)) * cos(radians(RXval)) - cos(radians(RZval)) * sin(radians(RXval));
  toolFrame [1][3] = (Yval);
  toolFrame [2][0] = -sin(radians(RYval));
  toolFrame [2][1] = cos(radians(RYval)) * sin(radians(RXval));
  toolFrame [2][2] = cos(radians(RYval)) * cos(radians(RXval));
  toolFrame [2][3] = (Zval);
  toolFrame [3][0] = 0;
  toolFrame [3][1] = 0;
  toolFrame [3][2] = 0;
  toolFrame [3][3] = 1;

}

void Tool_MatrixRev(float Xval, float Yval, float Zval, float RZval, float RYval, float RXval) {
  toolFrameRev [0][0] = cos(radians(RZval)) * cos(radians(RYval));
  toolFrameRev [0][1] = cos(radians(RZval)) * sin(radians(RYval)) * sin(radians(RXval)) - sin(radians(RZval)) * cos(radians(RXval));
  toolFrameRev [0][2] = cos(radians(RZval)) * sin(radians(RYval)) * cos(radians(RXval)) + sin(radians(RZval)) * sin(radians(RXval));
  toolFrameRev [0][3] = -(Xval);
  toolFrameRev [1][0] = sin(radians(RZval)) * cos(radians(RYval));
  toolFrameRev [1][1] = sin(radians(RZval)) * sin(radians(RYval)) * sin(radians(RXval)) + cos(radians(RZval)) * cos(radians(RXval));
  toolFrameRev [1][2] = sin(radians(RZval)) * sin(radians(RYval)) * cos(radians(RXval)) - cos(radians(RZval)) * sin(radians(RXval));
  toolFrameRev [1][3] = -(Yval);
  toolFrameRev [2][0] = -sin(radians(RYval));
  toolFrameRev [2][1] = cos(radians(RYval)) * sin(radians(RXval));
  toolFrameRev [2][2] = cos(radians(RYval)) * cos(radians(RXval));
  toolFrameRev [2][3] = -(Zval);
  toolFrameRev [3][0] = 0;
  toolFrameRev [3][1] = 0;
  toolFrameRev [3][2] = 0;
  toolFrameRev [3][3] = 1;

}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//CONSTRUCT DH MATRICES FORWARD KINEMATICS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void DH_Matrices() {

  if (JangleIn[0] == 0) {
    JangleIn[0] = .0001;
  }
  if (JangleIn[1] == 0) {
    JangleIn[1] = .0001;
  }
  if (JangleIn[2] == 0) {
    JangleIn[2] = .0001;
  }
  if (JangleIn[3] == 0) {
    JangleIn[3] = .0001;
  }
  if (JangleIn[4] == 0) {
    JangleIn[4] = .0001;
  }
  if (JangleIn[5] == 0) {
    JangleIn[5] = .0001;
  }

  J1matrix [0][0] = cos(radians(JangleIn[0] + DHparams[0][0]));
  J1matrix [0][1] = -sin(radians(JangleIn[0] + DHparams[0][0])) * cos(radians(DHparams[0][1]));
  J1matrix [0][2] = sin(radians(JangleIn[0] + DHparams[0][0])) * sin(radians(DHparams[0][1]));
  J1matrix [0][3] = DHparams[0][3] * cos(radians(JangleIn[0] + DHparams[0][0]));
  J1matrix [1][0] = sin(radians(JangleIn[0] + DHparams[0][0]));
  J1matrix [1][1] = cos(radians(JangleIn[0] + DHparams[0][0])) * cos(radians(DHparams[0][1]));
  J1matrix [1][2] = -cos(radians(JangleIn[0] + DHparams[0][0])) * sin(radians(DHparams[0][1]));
  J1matrix [1][3] = DHparams[0][3] * sin(radians(JangleIn[0] + DHparams[0][0]));
  J1matrix [2][0] = 0;
  J1matrix [2][1] = sin(radians(DHparams[0][1]));
  J1matrix [2][2] = cos(radians(DHparams[0][1]));
  J1matrix [2][3] = DHparams[0][2];
  J1matrix [3][0] = 0;
  J1matrix [3][1] = 0;
  J1matrix [3][2] = 0;
  J1matrix [3][3] = 1;

  J2matrix [0][0] = cos(radians(JangleIn[1] + DHparams[1][0]));
  J2matrix [0][1] = -sin(radians(JangleIn[1] + DHparams[1][0])) * cos(radians(DHparams[1][1]));
  J2matrix [0][2] = sin(radians(JangleIn[1] + DHparams[1][0])) * sin(radians(DHparams[1][1]));
  J2matrix [0][3] = DHparams[1][3] * cos(radians(JangleIn[1] + DHparams[1][0]));
  J2matrix [1][0] = sin(radians(JangleIn[1] + DHparams[1][0]));
  J2matrix [1][1] = cos(radians(JangleIn[1] + DHparams[1][0])) * cos(radians(DHparams[1][1]));
  J2matrix [1][2] = -cos(radians(JangleIn[1] + DHparams[1][0])) * sin(radians(DHparams[1][1]));
  J2matrix [1][3] = DHparams[1][3] * sin(radians(JangleIn[1] + DHparams[1][0]));
  J2matrix [2][0] = 0;
  J2matrix [2][1] = sin(radians(DHparams[1][1]));
  J2matrix [2][2] = cos(radians(DHparams[1][1]));
  J2matrix [2][3] = DHparams[1][2];
  J2matrix [3][0] = 0;
  J2matrix [3][1] = 0;
  J2matrix [3][2] = 0;
  J2matrix [3][3] = 1;

  J3matrix [0][0] = cos(radians(JangleIn[2] + DHparams[2][0]));
  J3matrix [0][1] = -sin(radians(JangleIn[2] + DHparams[2][0])) * cos(radians(DHparams[2][1]));
  J3matrix [0][2] = sin(radians(JangleIn[2] + DHparams[2][0])) * sin(radians(DHparams[2][1]));
  J3matrix [0][3] = DHparams[2][3] * cos(radians(JangleIn[2] + DHparams[2][0]));
  J3matrix [1][0] = sin(radians(JangleIn[2] + DHparams[2][0]));
  J3matrix [1][1] = cos(radians(JangleIn[2] + DHparams[2][0])) * cos(radians(DHparams[2][1]));
  J3matrix [1][2] = -cos(radians(JangleIn[2] + DHparams[2][0])) * sin(radians(DHparams[2][1]));
  J3matrix [1][3] = DHparams[2][3] * sin(radians(JangleIn[2] + DHparams[2][0]));
  J3matrix [2][0] = 0;
  J3matrix [2][1] = sin(radians(DHparams[2][1]));
  J3matrix [2][2] = cos(radians(DHparams[2][1]));
  J3matrix [2][3] = DHparams[2][2];
  J3matrix [3][0] = 0;
  J3matrix [3][1] = 0;
  J3matrix [3][2] = 0;
  J3matrix [3][3] = 1;

  J4matrix [0][0] = cos(radians(JangleIn[3] + DHparams[3][0]));
  J4matrix [0][1] = -sin(radians(JangleIn[3] + DHparams[3][0])) * cos(radians(DHparams[3][1]));
  J4matrix [0][2] = sin(radians(JangleIn[3] + DHparams[3][0])) * sin(radians(DHparams[3][1]));
  J4matrix [0][3] = DHparams[3][3] * cos(radians(JangleIn[3] + DHparams[3][0]));
  J4matrix [1][0] = sin(radians(JangleIn[3] + DHparams[3][0]));
  J4matrix [1][1] = cos(radians(JangleIn[3] + DHparams[3][0])) * cos(radians(DHparams[3][1]));
  J4matrix [1][2] = -cos(radians(JangleIn[3] + DHparams[3][0])) * sin(radians(DHparams[3][1]));
  J4matrix [1][3] = DHparams[3][3] * sin(radians(JangleIn[3] + DHparams[3][0]));
  J4matrix [2][0] = 0;
  J4matrix [2][1] = sin(radians(DHparams[3][1]));
  J4matrix [2][2] = cos(radians(DHparams[3][1]));
  J4matrix [2][3] = DHparams[3][2];
  J4matrix [3][0] = 0;
  J4matrix [3][1] = 0;
  J4matrix [3][2] = 0;
  J4matrix [3][3] = 1;

  J5matrix [0][0] = cos(radians(JangleIn[4] + DHparams[4][0]));
  J5matrix [0][1] = -sin(radians(JangleIn[4] + DHparams[4][0])) * cos(radians(DHparams[4][1]));
  J5matrix [0][2] = sin(radians(JangleIn[4] + DHparams[4][0])) * sin(radians(DHparams[4][1]));
  J5matrix [0][3] = DHparams[4][3] * cos(radians(JangleIn[4] + DHparams[4][0]));
  J5matrix [1][0] = sin(radians(JangleIn[4] + DHparams[4][0]));
  J5matrix [1][1] = cos(radians(JangleIn[4] + DHparams[4][0])) * cos(radians(DHparams[4][1]));
  J5matrix [1][2] = -cos(radians(JangleIn[4] + DHparams[4][0])) * sin(radians(DHparams[4][1]));
  J5matrix [1][3] = DHparams[4][3] * sin(radians(JangleIn[4] + DHparams[4][0]));
  J5matrix [2][0] = 0;
  J5matrix [2][1] = sin(radians(DHparams[4][1]));
  J5matrix [2][2] = cos(radians(DHparams[4][1]));
  J5matrix [2][3] = DHparams[4][2];
  J5matrix [3][0] = 0;
  J5matrix [3][1] = 0;
  J5matrix [3][2] = 0;
  J5matrix [3][3] = 1;

  J6matrix [0][0] = cos(radians(JangleIn[5] + DHparams[5][0]));
  J6matrix [0][1] = -sin(radians(JangleIn[5] + DHparams[5][0])) * cos(radians(DHparams[5][1]));
  J6matrix [0][2] = sin(radians(JangleIn[5] + DHparams[5][0])) * sin(radians(DHparams[5][1]));
  J6matrix [0][3] = DHparams[5][3] * cos(radians(JangleIn[5] + DHparams[5][0]));
  J6matrix [1][0] = sin(radians(JangleIn[5] + DHparams[5][0]));
  J6matrix [1][1] = cos(radians(JangleIn[5] + DHparams[5][0])) * cos(radians(DHparams[5][1]));
  J6matrix [1][2] = -cos(radians(JangleIn[5] + DHparams[5][0])) * sin(radians(DHparams[5][1]));
  J6matrix [1][3] = DHparams[5][3] * sin(radians(JangleIn[5] + DHparams[5][0]));
  J6matrix [2][0] = 0;
  J6matrix [2][1] = sin(radians(DHparams[5][1]));
  J6matrix [2][2] = cos(radians(DHparams[5][1]));
  J6matrix [2][3] = DHparams[5][2];
  J6matrix [3][0] = 0;
  J6matrix [3][1] = 0;
  J6matrix [3][2] = 0;
  J6matrix [3][3] = 1;

}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//CONSTRUCT ROTATION MATRICES FORWARD KINEMATICS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void FwdMatrices() {

  int i, j, k = 0;

  for (int j = 0; j < 4; j++) {
    for (int i = 0; i < 4; i++) {
      R02matrix[j][i] = 0;
    }
  }
  for (int j = 0; j < 4; j++) {
    for (int i = 0; i < 4; i++) {
      R03matrix[j][i] = 0;
    }
  }
  for (int j = 0; j < 4; j++) {
    for (int i = 0; i < 4; i++) {
      R04matrix[j][i] = 0;
    }
  }
  for (int j = 0; j < 4; j++) {
    for (int i = 0; i < 4; i++) {
      R05matrix[j][i] = 0;
    }
  }
  for (int j = 0; j < 4; j++) {
    for (int i = 0; i < 4; i++) {
      R06matrix[j][i] = 0;
    }
  }
  for (int j = 0; j < 4; j++) {
    for (int i = 0; i < 4; i++) {
      R0Tmatrix[j][i] = 0;
    }
  }

  for (int k = 0; k < 4; k++) {
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        R02matrix[k][i] = R02matrix[k][i] + (J1matrix [k][j] * J2matrix [j][i]);
      }
    }
  }
  for (int k = 0; k < 4; k++) {
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        R03matrix[k][i] = R03matrix[k][i] + (R02matrix [k][j] * J3matrix [j][i]);
      }
    }
  }
  for (int k = 0; k < 4; k++) {
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        R04matrix[k][i] = R04matrix[k][i] + (R03matrix [k][j] * J4matrix [j][i]);
      }
    }
  }
  for (int k = 0; k < 4; k++) {
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        R05matrix[k][i] = R05matrix[k][i] + (R04matrix [k][j] * J5matrix [j][i]);
      }
    }
  }
  for (int k = 0; k < 4; k++) {
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        R06matrix[k][i] = R06matrix[k][i] + (R05matrix [k][j] * J6matrix [j][i]);
      }
    }
  }
  for (int k = 0; k < 4; k++) {
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        R0Tmatrix[k][i] = R0Tmatrix[k][i] + (R06matrix [k][j] * toolFrame [j][i]);
      }
    }
  }
}



void SolveFowardKinematic() {
  DH_Matrices();
  FwdMatrices();
  xyzuvw_Out[0] = R0Tmatrix[0][3];
  xyzuvw_Out[1] = R0Tmatrix[1][3];
  xyzuvw_Out[2] = R0Tmatrix[2][3];
  xyzuvw_Out[4] = atan2(-R0Tmatrix[2][0], pow((pow(R0Tmatrix[2][1], 2) + pow(R0Tmatrix[2][2], 2)), 0.5));
  xyzuvw_Out[3] = degrees(atan2(R0Tmatrix[1][0] / cos(xyzuvw_Out[4]), R0Tmatrix[0][0] / cos(xyzuvw_Out[4])));
  xyzuvw_Out[5] = degrees(atan2(R0Tmatrix[2][1] / cos(xyzuvw_Out[4]), R0Tmatrix[2][2] / cos(xyzuvw_Out[4])));
  xyzuvw_Out[4] = degrees(xyzuvw_Out[4]);
}





/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//REVERSE KINEMATICS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void SolveInverseKinematic() {

  float pX;
  float pY;
  float pX_a1_fwd;
  float pX_a1_mid;
  float pa2H_fwd;
  float pa2H_mid;
  float pa3H;
  float thetaA_fwd;
  float thetaA_mid;
  float thetaB_fwd;
  float thetaB_mid;
  float thetaC_fwd;
  float thetaC_mid;
  float thetaD;
  float thetaE;

  float XatJ1zero;
  float YatJ1zero;
  float Length_1;
  float Length_2;
  float Length_3;
  float Length_4;
  float Theta_A;
  float Theta_B;
  float Theta_C;
  float Theta_D;
  float Theta_E;


  int i, j, k = 0;
  KinematicError = 0;

  //generate matrices for center of spherical wrist location

  R0T_rev_matrix [0][0] = cos(radians(xyzuvw_In[3])) * cos(radians(xyzuvw_In[4]));
  R0T_rev_matrix [0][1] = cos(radians(xyzuvw_In[3])) * sin(radians(xyzuvw_In[4])) * sin(radians(xyzuvw_In[5])) - sin(radians(xyzuvw_In[3])) * cos(radians(xyzuvw_In[5]));
  R0T_rev_matrix [0][2] = cos(radians(xyzuvw_In[3])) * sin(radians(xyzuvw_In[4])) * cos(radians(xyzuvw_In[5])) + sin(radians(xyzuvw_In[3])) * sin(radians(xyzuvw_In[5]));
  R0T_rev_matrix [0][3] = xyzuvw_In[0];
  R0T_rev_matrix [1][0] = sin(radians(xyzuvw_In[3])) * cos(radians(xyzuvw_In[4]));
  R0T_rev_matrix [1][1] = sin(radians(xyzuvw_In[3])) * sin(radians(xyzuvw_In[4])) * sin(radians(xyzuvw_In[5])) + cos(radians(xyzuvw_In[3])) * cos(radians(xyzuvw_In[5]));
  R0T_rev_matrix [1][2] = sin(radians(xyzuvw_In[3])) * sin(radians(xyzuvw_In[4])) * cos(radians(xyzuvw_In[5])) - cos(radians(xyzuvw_In[3])) * sin(radians(xyzuvw_In[5]));
  R0T_rev_matrix [1][3] = xyzuvw_In[1];
  R0T_rev_matrix [2][0] = -sin(radians(xyzuvw_In[4]));
  R0T_rev_matrix [2][1] = cos(radians(xyzuvw_In[4])) * sin(radians(xyzuvw_In[5]));
  R0T_rev_matrix [2][2] = cos(radians(xyzuvw_In[4])) * cos(radians(xyzuvw_In[5]));
  R0T_rev_matrix [2][3] = xyzuvw_In[2];
  R0T_rev_matrix [3][0] = 0;
  R0T_rev_matrix [3][1] = 0;
  R0T_rev_matrix [3][2] = 0;
  R0T_rev_matrix [3][3] = 1;

  InvtoolFrame [0][0] = toolFrameRev [0][0];
  InvtoolFrame [0][1] = toolFrameRev [1][0];
  InvtoolFrame [0][2] = toolFrameRev [2][0];
  InvtoolFrame [0][3] = (InvtoolFrame [0][0] * toolFrameRev [0][3]) + (InvtoolFrame [0][1] * toolFrameRev [1][3]) + (InvtoolFrame [0][2] * toolFrameRev [2][3]);
  InvtoolFrame [1][0] = toolFrameRev [0][1];
  InvtoolFrame [1][1] = toolFrameRev [1][1];
  InvtoolFrame [1][2] = toolFrameRev [2][1];
  InvtoolFrame [1][3] = (InvtoolFrame [1][0] * toolFrameRev [0][3]) + (InvtoolFrame [1][1] * toolFrameRev [1][3]) + (InvtoolFrame [1][2] * toolFrameRev [2][3]);
  InvtoolFrame [2][0] = toolFrameRev [0][2];
  InvtoolFrame [2][1] = toolFrameRev [1][2];
  InvtoolFrame [2][2] = toolFrameRev [2][2];
  InvtoolFrame [2][3] = (InvtoolFrame [2][0] * toolFrameRev [0][3]) + (InvtoolFrame [2][1] * toolFrameRev [1][3]) + (InvtoolFrame [2][2] * toolFrameRev [2][3]);
  InvtoolFrame [3][0] = 0;
  InvtoolFrame [3][1] = 0;
  InvtoolFrame [3][2] = 0;
  InvtoolFrame [3][3] = 1;

  for (int j = 0; j < 4; j++) {
    for (int i = 0; i < 4; i++) {
      R06_rev_matrix[j][i] = 0;
    }
  }

  for (int j = 0; j < 4; j++) {
    for (int i = 0; i < 4; i++) {
      R05_rev_matrix[j][i] = 0;
    }
  }

  for (int j = 0; j < 4; j++) {
    for (int i = 0; i < 4; i++) {
      R02matrix_rev[j][i] = 0;
    }
  }

  for (int j = 0; j < 4; j++) {
    for (int i = 0; i < 4; i++) {
      R03matrix_rev[j][i] = 0;
    }
  }

  for (int j = 0; j < 4; j++) {
    for (int i = 0; i < 4; i++) {
      R03_6matrix[j][i] = 0;
    }
  }

  for (int k = 0; k < 4; k++) {
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        R06_rev_matrix[k][i] = R06_rev_matrix[k][i] + (R0T_rev_matrix [k][j] * InvtoolFrame [j][i]);
      }
    }
  }

  for (int k = 0; k < 4; k++) {
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        R05_rev_matrix[k][i] = R05_rev_matrix[k][i] + (R06_rev_matrix [k][j] * R06_neg_matrix [j][i]);
      }
    }
  }

  //calc J1 angle

  if (R05_rev_matrix[0][3] >= 0 and R05_rev_matrix[1][3] > 0) {
    JangleOut[0] = degrees(atan(R05_rev_matrix[1][3] / R05_rev_matrix[0][3]));
  }
  else if (R05_rev_matrix[0][3] >= 0 and R05_rev_matrix[1][3] < 0) {
    JangleOut[0] = degrees(atan(R05_rev_matrix[1][3] / R05_rev_matrix[0][3]));
  }
  else if (R05_rev_matrix[0][3] < 0 and R05_rev_matrix[1][3] <= 0) {
    JangleOut[0] = -180 + degrees(atan(R05_rev_matrix[1][3] / R05_rev_matrix[0][3]));
  }
  else if (R05_rev_matrix[0][3] <= 0  and R05_rev_matrix[1][3] > 0) {
    JangleOut[0] = 180 + degrees(atan(R05_rev_matrix[1][3] / R05_rev_matrix[0][3]));
  }

  //calculate J2 & J3 geometry

  XatJ1zero = (R05_rev_matrix[0][3] * cos(radians(-JangleOut[0]))) - (R05_rev_matrix[1][3] * sin(radians(-JangleOut[0])));
  YatJ1zero = 0;

  Length_1 = abs(XatJ1zero - DHparams[0][3]);
  Length_2 = sqrt(pow((XatJ1zero - DHparams[0][3]), 2) + pow((YatJ1zero - YatJ1zero), 2) + pow((R05_rev_matrix[2][3] - DHparams[0][2]), 2));
  Length_3 = sqrt(pow(DHparams[3][2], 2) + pow(DHparams[2][3], 2));
  Length_4 = R05_rev_matrix[2][3] - DHparams[0][2];
  Theta_B = degrees(atan(Length_1 / Length_4));
  Theta_C = degrees(acos((pow(DHparams[1][3], 2) + pow(Length_2, 2) - pow(Length_3, 2)) / (2 * DHparams[1][3] * Length_2)));
  Theta_D = degrees(acos((pow(Length_3, 2) + pow(DHparams[1][3], 2) - pow(Length_2, 2)) / (2 * Length_3 * DHparams[1][3])));
  Theta_E = degrees(atan(DHparams[2][3] / DHparams[3][2]));

  // calc J2 angle

  if (XatJ1zero > DHparams[0][3]) {
    if (Length_4 > 0) {
      JangleOut[1] = Theta_B - Theta_C;
    }
    else {
      JangleOut[1] = Theta_B - Theta_C + 180;
    }
  }
  else {
    JangleOut[1] = -(Theta_B + Theta_C);
  }

  // calc J3 angle

  JangleOut[2] = -(Theta_D + Theta_E) + 90;


  // generate reverse matrices for wrist orientaion

  J1matrix_rev [0][0] = cos(radians(JangleOut[0] + DHparams[0][0]));
  J1matrix_rev [0][1] = -sin(radians(JangleOut[0] + DHparams[0][0])) * cos(radians(DHparams[0][1]));
  J1matrix_rev [0][2] = sin(radians(JangleOut[0] + DHparams[0][0])) * sin(radians(DHparams[0][1]));
  J1matrix_rev [0][3] = DHparams[0][3] * cos(radians(JangleOut[0] + DHparams[0][0]));
  J1matrix_rev [1][0] = sin(radians(JangleOut[0] + DHparams[0][0]));
  J1matrix_rev [1][1] = cos(radians(JangleOut[0] + DHparams[0][0])) * cos(radians(DHparams[0][1]));
  J1matrix_rev [1][2] = -cos(radians(JangleOut[0] + DHparams[0][0])) * sin(radians(DHparams[0][1]));
  J1matrix_rev [1][3] = DHparams[0][3] * sin(radians(JangleOut[0] + DHparams[0][0]));
  J1matrix_rev [2][0] = 0;
  J1matrix_rev [2][1] = sin(radians(DHparams[0][1]));
  J1matrix_rev [2][2] = cos(radians(DHparams[0][1]));
  J1matrix_rev [2][3] = DHparams[0][2];
  J1matrix_rev [3][0] = 0;
  J1matrix_rev [3][1] = 0;
  J1matrix_rev [3][2] = 0;
  J1matrix_rev [3][3] = 1;

  J2matrix_rev [0][0] = cos(radians(JangleOut[1] + DHparams[1][0]));
  J2matrix_rev [0][1] = -sin(radians(JangleOut[1] + DHparams[1][0])) * cos(radians(DHparams[1][1]));
  J2matrix_rev [0][2] = sin(radians(JangleOut[1] + DHparams[1][0])) * sin(radians(DHparams[1][1]));
  J2matrix_rev [0][3] = DHparams[1][3] * cos(radians(JangleOut[1] + DHparams[1][0]));
  J2matrix_rev [1][0] = sin(radians(JangleOut[1] + DHparams[1][0]));
  J2matrix_rev [1][1] = cos(radians(JangleOut[1] + DHparams[1][0])) * cos(radians(DHparams[1][1]));
  J2matrix_rev [1][2] = -cos(radians(JangleOut[1] + DHparams[1][0])) * sin(radians(DHparams[1][1]));
  J2matrix_rev [1][3] = DHparams[1][3] * sin(radians(JangleOut[1] + DHparams[1][0]));
  J2matrix_rev [2][0] = 0;
  J2matrix_rev [2][1] = sin(radians(DHparams[1][1]));
  J2matrix_rev [2][2] = cos(radians(DHparams[1][1]));
  J2matrix_rev [2][3] = DHparams[1][2];
  J2matrix_rev [3][0] = 0;
  J2matrix_rev [3][1] = 0;
  J2matrix_rev [3][2] = 0;
  J2matrix_rev [3][3] = 1;

  J3matrix_rev [0][0] = cos(radians(JangleOut[2] + DHparams[2][0]));
  J3matrix_rev [0][1] = -sin(radians(JangleOut[2] + DHparams[2][0])) * cos(radians(DHparams[2][1]));
  J3matrix_rev [0][2] = sin(radians(JangleOut[2] + DHparams[2][0])) * sin(radians(DHparams[2][1]));
  J3matrix_rev [0][3] = DHparams[2][3] * cos(radians(JangleOut[2] + DHparams[2][0]));
  J3matrix_rev [1][0] = sin(radians(JangleOut[2] + DHparams[2][0]));
  J3matrix_rev [1][1] = cos(radians(JangleOut[2] + DHparams[2][0])) * cos(radians(DHparams[2][1]));
  J3matrix_rev [1][2] = -cos(radians(JangleOut[2] + DHparams[2][0])) * sin(radians(DHparams[2][1]));
  J3matrix_rev [1][3] = DHparams[2][3] * sin(radians(JangleOut[2] + DHparams[2][0]));
  J3matrix_rev [2][0] = 0;
  J3matrix_rev [2][1] = sin(radians(DHparams[2][1]));
  J3matrix_rev [2][2] = cos(radians(DHparams[2][1]));
  J3matrix_rev [2][3] = DHparams[2][2];
  J3matrix_rev [3][0] = 0;
  J3matrix_rev [3][1] = 0;
  J3matrix_rev [3][2] = 0;
  J3matrix_rev [3][3] = 1;

  for (int k = 0; k < 4; k++) {
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        R02matrix_rev[k][i] = R02matrix_rev[k][i] + (J1matrix_rev [k][j] * J2matrix_rev [j][i]);
      }
    }
  }
  for (int k = 0; k < 4; k++) {
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        R03matrix_rev[k][i] = R03matrix_rev[k][i] + (R02matrix_rev [k][j] * J3matrix_rev [j][i]);
      }
    }
  }

  InvR03matrix_rev [0][0] = R03matrix_rev [0][0];
  InvR03matrix_rev [0][1] = R03matrix_rev [1][0];
  InvR03matrix_rev [0][2] = R03matrix_rev [2][0];
  InvR03matrix_rev [1][0] = R03matrix_rev [0][1];
  InvR03matrix_rev [1][1] = R03matrix_rev [1][1];
  InvR03matrix_rev [1][2] = R03matrix_rev [2][1];
  InvR03matrix_rev [2][0] = R03matrix_rev [0][2];
  InvR03matrix_rev [2][1] = R03matrix_rev [1][2];
  InvR03matrix_rev [2][2] = R03matrix_rev [2][2];

  for (int k = 0; k < 4; k++) {
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        R03_6matrix[k][i] = R03_6matrix[k][i] + (InvR03matrix_rev [k][j] * R05_rev_matrix [j][i]);
      }
    }
  }

  //calculate J5 angle

  if (WristCon == "F") {
    JangleOut[4] = degrees(atan2(sqrt(1 - pow(R03_6matrix[2][2], 2)), R03_6matrix[2][2]));
  }
  else {
    JangleOut[4] = degrees(atan2(-sqrt(1 - pow(R03_6matrix[2][2], 2)), R03_6matrix[2][2]));
  }

  //calculate J4 angle

  if (JangleOut[4] < 0) {
    JangleOut[3] = -degrees(atan2(R03_6matrix[1][2], -R03_6matrix[0][2]));
  }
  else {
    JangleOut[3] = -degrees(atan2(-R03_6matrix[1][2], R03_6matrix[0][2]));
  }

  //calculate J6 angle

  if (JangleOut[4] < 0) {
    JangleOut[5] = degrees(atan2(-R03_6matrix[2][1], R03_6matrix[2][0]));
  }
  else  {
    JangleOut[5] = degrees(atan2(R03_6matrix[2][1], -R03_6matrix[2][0]));
  }

}






/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//CALCULATE POSITIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void sendRobotPos() {

  updatePos();

  String sendPos = "A" + String(JangleIn[0], 3) + "B" + String(JangleIn[1], 3) + "C" + String(JangleIn[2], 3) + "D" + String(JangleIn[3], 3) + "E" + String(JangleIn[4], 3) + "F" + String(JangleIn[5], 3) + "G" + String(xyzuvw_Out[0], 3) + "H" + String(xyzuvw_Out[1], 3) + "I" + String(xyzuvw_Out[2], 3) + "J" + String(xyzuvw_Out[3], 3) + "K" + String(xyzuvw_Out[4], 3) + "L" + String(xyzuvw_Out[5], 3) + "M" + speedViolation + "N" + debug + "O" + flag + "P" + J7_pos + "Q" + J8_pos + "R" + J9_pos;
  delay(5);
  Serial.println(sendPos);
  speedViolation = "0";
  flag = "";

}

void sendRobotPosSpline() {

  updatePos();

  String sendPos = "A" + String(JangleIn[0], 3) + "B" + String(JangleIn[1], 3) + "C" + String(JangleIn[2], 3) + "D" + String(JangleIn[3], 3) + "E" + String(JangleIn[4], 3) + "F" + String(JangleIn[5], 3) + "G" + String(xyzuvw_Out[0], 3) + "H" + String(xyzuvw_Out[1], 3) + "I" + String(xyzuvw_Out[2], 3) + "J" + String(xyzuvw_Out[3], 3) + "K" + String(xyzuvw_Out[4], 3) + "L" + String(xyzuvw_Out[5], 3) + "M" + speedViolation + "N" + debug + "O" + flag + "P" + J7_pos + "Q" + J8_pos + "R" + J9_pos;
  delay(5);
  Serial.println(sendPos);
  speedViolation = "0";

}

void updatePos() {

  JangleIn[0] = (J1StepM - J1zeroStep) / J1StepDeg;
  JangleIn[1] = (J2StepM - J2zeroStep) / J2StepDeg;
  JangleIn[2] = (J3StepM - J3zeroStep) / J3StepDeg;
  JangleIn[3] = (J4StepM - J4zeroStep) / J4StepDeg;
  JangleIn[4] = (J5StepM - J5zeroStep) / J5StepDeg;
  JangleIn[5] = (J6StepM - J6zeroStep) / J6StepDeg;

  J7_pos = (J7StepM - J7zeroStep) / J7StepDeg;
  J8_pos = (J8StepM - J8zeroStep) / J8StepDeg;
  J9_pos = (J9StepM - J9zeroStep) / J9StepDeg;

  SolveFowardKinematic();
}



void correctRobotPos () {

  J1StepM = J1encPos.read() / J1encMult;
  J2StepM = J2encPos.read() / J2encMult;
  J3StepM = J3encPos.read() / J3encMult;
  J4StepM = J4encPos.read() / J4encMult;
  J5StepM = J5encPos.read() / J5encMult;
  J6StepM = J6encPos.read() / J6encMult;

  JangleIn[0] = (J1StepM - J1zeroStep) / J1StepDeg;
  JangleIn[1] = (J2StepM - J2zeroStep) / J2StepDeg;
  JangleIn[2] = (J3StepM - J3zeroStep) / J3StepDeg;
  JangleIn[3] = (J4StepM - J4zeroStep) / J4StepDeg;
  JangleIn[4] = (J5StepM - J5zeroStep) / J5StepDeg;
  JangleIn[5] = (J6StepM - J6zeroStep) / J6StepDeg;


  SolveFowardKinematic();

  String sendPos = "A" + String(JangleIn[0], 3) + "B" + String(JangleIn[1], 3) + "C" + String(JangleIn[2], 3) + "D" + String(JangleIn[3], 3) + "E" + String(JangleIn[4], 3) + "F" + String(JangleIn[5], 3) + "G" + String(xyzuvw_Out[0], 3) + "H" + String(xyzuvw_Out[1], 3) + "I" + String(xyzuvw_Out[2], 3) + "J" + String(xyzuvw_Out[3], 3) + "K" + String(xyzuvw_Out[4], 3) + "L" + String(xyzuvw_Out[5], 3) + "M" + speedViolation + "N" + debug + "O" + flag + "P" + J7_pos + "Q" + J8_pos + "R" + J9_pos;
  delay(5);
  Serial.println(sendPos);
  speedViolation = "0";
  flag = "";



}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//DRIVE LIMIT
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void driveLimit(int J1Step, int J2Step, int J3Step, int J4Step, int J5Step, int J6Step, int J7Step, int J8Step, int J9Step, float SpeedVal) {

  //RESET COUNTERS
  int J1done = 0;
  int J2done = 0;
  int J3done = 0;
  int J4done = 0;
  int J5done = 0;
  int J6done = 0;
  int J7done = 0;
  int J8done = 0;
  int J9done = 0;

  int J1complete = 0;
  int J2complete = 0;
  int J3complete = 0;
  int J4complete = 0;
  int J5complete = 0;
  int J6complete = 0;
  int J7complete = 0;
  int J8complete = 0;
  int J9complete = 0;

  int calcStepGap = minSpeedDelay / (SpeedVal / 100);

  //SET DIRECTIONS
  /// J1 ///
  if (J1CalDir == 1 && J1MotDir == 1) {
    digitalWrite(J1dirPin, HIGH);
  }
  if (J1CalDir == 1 && J1MotDir == 0) {
    digitalWrite(J1dirPin, LOW);
  }
  if (J1CalDir == 0 && J1MotDir == 1) {
    digitalWrite(J1dirPin, LOW);
  }
  if (J1CalDir == 0 && J1MotDir == 0) {
    digitalWrite(J1dirPin, HIGH);
  }
  /// J2 ///
  if (J2CalDir == 1 && J2MotDir == 1) {
    digitalWrite(J2dirPin, HIGH);
  }
  if (J2CalDir == 1 && J2MotDir == 0) {
    digitalWrite(J2dirPin, LOW);
  }
  if (J2CalDir == 0 && J2MotDir == 1) {
    digitalWrite(J2dirPin, LOW);
  }
  if (J2CalDir == 0 && J2MotDir == 0) {
    digitalWrite(J2dirPin, HIGH);
  }
  /// J3 ///
  if (J3CalDir == 1 && J3MotDir == 1) {
    digitalWrite(J3dirPin, HIGH);
  }
  if (J3CalDir == 1 && J3MotDir == 0) {
    digitalWrite(J3dirPin, LOW);
  }
  if (J3CalDir == 0 && J3MotDir == 1) {
    digitalWrite(J3dirPin, LOW);
  }
  if (J3CalDir == 0 && J3MotDir == 0) {
    digitalWrite(J3dirPin, HIGH);
  }
  /// J4 ///
  if (J4CalDir == 1 && J4MotDir == 1) {
    digitalWrite(J4dirPin, HIGH);
  }
  if (J4CalDir == 1 && J4MotDir == 0) {
    digitalWrite(J4dirPin, LOW);
  }
  if (J4CalDir == 0 && J4MotDir == 1) {
    digitalWrite(J4dirPin, LOW);
  }
  if (J4CalDir == 0 && J4MotDir == 0) {
    digitalWrite(J4dirPin, HIGH);
  }
  /// J5 ///
  if (J5CalDir == 1 && J5MotDir == 1) {
    digitalWrite(J5dirPin, HIGH);
  }
  if (J5CalDir == 1 && J5MotDir == 0) {
    digitalWrite(J5dirPin, LOW);
  }
  if (J5CalDir == 0 && J5MotDir == 1) {
    digitalWrite(J5dirPin, LOW);
  }
  if (J5CalDir == 0 && J5MotDir == 0) {
    digitalWrite(J5dirPin, HIGH);
  }
  /// J6 ///
  if (J6CalDir == 1 && J6MotDir == 1) {
    digitalWrite(J6dirPin, HIGH);
  }
  if (J6CalDir == 1 && J6MotDir == 0) {
    digitalWrite(J6dirPin, LOW);
  }
  if (J6CalDir == 0 && J6MotDir == 1) {
    digitalWrite(J6dirPin, LOW);
  }
  if (J6CalDir == 0 && J6MotDir == 0) {
    digitalWrite(J6dirPin, HIGH);
  }
  /// J7 ///
  if (J7CalDir == 1 && J7MotDir == 1) {
    digitalWrite(J7dirPin, HIGH);
  }
  if (J7CalDir == 1 && J7MotDir == 0) {
    digitalWrite(J7dirPin, LOW);
  }
  if (J7CalDir == 0 && J7MotDir == 1) {
    digitalWrite(J7dirPin, LOW);
  }
  if (J7CalDir == 0 && J7MotDir == 0) {
    digitalWrite(J7dirPin, HIGH);
  }
  /// J8 ///
  if (J8CalDir == 1 && J8MotDir == 1) {
    digitalWrite(J8dirPin, HIGH);
  }
  if (J8CalDir == 1 && J8MotDir == 0) {
    digitalWrite(J8dirPin, LOW);
  }
  if (J8CalDir == 0 && J8MotDir == 1) {
    digitalWrite(J8dirPin, LOW);
  }
  if (J8CalDir == 0 && J8MotDir == 0) {
    digitalWrite(J8dirPin, HIGH);
  }
  /// J9 ///
  if (J9CalDir == 1 && J9MotDir == 1) {
    digitalWrite(J9dirPin, HIGH);
  }
  if (J9CalDir == 1 && J9MotDir == 0) {
    digitalWrite(J9dirPin, LOW);
  }
  if (J9CalDir == 0 && J9MotDir == 1) {
    digitalWrite(J9dirPin, LOW);
  }
  if (J9CalDir == 0 && J9MotDir == 0) {
    digitalWrite(J9dirPin, HIGH);
  }



  //DRIVE MOTORS FOR CALIBRATION

  int curRead;
  int J1CurState;
  int J2CurState;
  int J3CurState;
  int J4CurState;
  int J5CurState;
  int J6CurState;
  int J7CurState;
  int J8CurState;
  int J9CurState;
  int DriveLimInProc = 1;

  if (J1Step <= 0 ) {
    J1complete = 1;
  }
  if (J2Step <= 0) {
    J2complete = 1;
  }
  if (J3Step <= 0) {
    J3complete = 1;
  }
  if (J4Step <= 0) {
    J4complete = 1;
  }
  if (J5Step <= 0) {
    J5complete = 1;
  }
  if (J6Step <= 0) {
    J6complete = 1;
  }
  if (J7Step <= 0) {
    J7complete = 1;
  }
  if (J8Step <= 0) {
    J8complete = 1;
  }
  if (J9Step <= 0) {
    J9complete = 1;
  }


  while (DriveLimInProc == 1) {

    //EVAL J1
    if (digitalRead(J1calPin) == LOW) {
      J1CurState = LOW;
    }
    else {
      delayMicroseconds(10);
      if (digitalRead(J1calPin) == LOW) {
        J1CurState = LOW;
      }
      else {
        delayMicroseconds(10);
        if (digitalRead(J1calPin) == LOW) {
          J1CurState = LOW;
        }
        else {
          delayMicroseconds(10);
          if (digitalRead(J1calPin) == LOW) {
            J1CurState = LOW;
          }
          else {
            J1CurState = digitalRead(J1calPin);
          }
        }
      }
    }

    //EVAL J2
    if (digitalRead(J2calPin) == LOW) {
      J2CurState = LOW;
    }
    else {
      delayMicroseconds(10);
      if (digitalRead(J2calPin) == LOW) {
        J2CurState = LOW;
      }
      else {
        delayMicroseconds(10);
        if (digitalRead(J2calPin) == LOW) {
          J2CurState = LOW;
        }
        else {
          delayMicroseconds(10);
          if (digitalRead(J2calPin) == LOW) {
            J2CurState = LOW;
          }
          else {
            J2CurState = digitalRead(J2calPin);
          }
        }
      }
    }

    //EVAL J3
    if (digitalRead(J3calPin) == LOW) {
      J3CurState = LOW;
    }
    else {
      delayMicroseconds(10);
      if (digitalRead(J3calPin) == LOW) {
        J3CurState = LOW;
      }
      else {
        delayMicroseconds(10);
        if (digitalRead(J3calPin) == LOW) {
          J3CurState = LOW;
        }
        else {
          delayMicroseconds(10);
          if (digitalRead(J3calPin) == LOW) {
            J3CurState = LOW;
          }
          else {
            J3CurState = digitalRead(J3calPin);
          }
        }
      }
    }

    //EVAL J4
    if (digitalRead(J4calPin) == LOW) {
      J4CurState = LOW;
    }
    else {
      delayMicroseconds(10);
      if (digitalRead(J4calPin) == LOW) {
        J4CurState = LOW;
      }
      else {
        delayMicroseconds(10);
        if (digitalRead(J4calPin) == LOW) {
          J4CurState = LOW;
        }
        else {
          delayMicroseconds(10);
          if (digitalRead(J4calPin) == LOW) {
            J4CurState = LOW;
          }
          else {
            J4CurState = digitalRead(J4calPin);
          }
        }
      }
    }

    //EVAL J5
    if (digitalRead(J5calPin) == LOW) {
      J5CurState = LOW;
    }
    else {
      delayMicroseconds(10);
      if (digitalRead(J5calPin) == LOW) {
        J5CurState = LOW;
      }
      else {
        delayMicroseconds(10);
        if (digitalRead(J5calPin) == LOW) {
          J5CurState = LOW;
        }
        else {
          delayMicroseconds(10);
          if (digitalRead(J5calPin) == LOW) {
            J5CurState = LOW;
          }
          else {
            J5CurState = digitalRead(J5calPin);
          }
        }
      }
    }

    //EVAL J6
    if (digitalRead(J6calPin) == LOW) {
      J6CurState = LOW;
    }
    else {
      delayMicroseconds(10);
      if (digitalRead(J6calPin) == LOW) {
        J6CurState = LOW;
      }
      else {
        delayMicroseconds(10);
        if (digitalRead(J6calPin) == LOW) {
          J6CurState = LOW;
        }
        else {
          delayMicroseconds(10);
          if (digitalRead(J6calPin) == LOW) {
            J6CurState = LOW;
          }
          else {
            J6CurState = digitalRead(J6calPin);
          }
        }
      }
    }

    //EVAL J7
    if (digitalRead(J7calPin) == LOW) {
      J7CurState = LOW;
    }
    else {
      delayMicroseconds(10);
      if (digitalRead(J7calPin) == LOW) {
        J7CurState = LOW;
      }
      else {
        delayMicroseconds(10);
        if (digitalRead(J7calPin) == LOW) {
          J7CurState = LOW;
        }
        else {
          delayMicroseconds(10);
          if (digitalRead(J7calPin) == LOW) {
            J7CurState = LOW;
          }
          else {
            J7CurState = digitalRead(J7calPin);
          }
        }
      }
    }

    //EVAL J8
    if (digitalRead(J8calPin) == LOW) {
      J8CurState = LOW;
    }
    else {
      delayMicroseconds(10);
      if (digitalRead(J8calPin) == LOW) {
        J8CurState = LOW;
      }
      else {
        delayMicroseconds(10);
        if (digitalRead(J8calPin) == LOW) {
          J8CurState = LOW;
        }
        else {
          delayMicroseconds(10);
          if (digitalRead(J8calPin) == LOW) {
            J8CurState = LOW;
          }
          else {
            J8CurState = digitalRead(J8calPin);
          }
        }
      }
    }

    //EVAL J9
    if (digitalRead(J9calPin) == LOW) {
      J9CurState = LOW;
    }
    else {
      delayMicroseconds(10);
      if (digitalRead(J9calPin) == LOW) {
        J9CurState = LOW;
      }
      else {
        delayMicroseconds(10);
        if (digitalRead(J9calPin) == LOW) {
          J9CurState = LOW;
        }
        else {
          delayMicroseconds(10);
          if (digitalRead(J9calPin) == LOW) {
            J9CurState = LOW;
          }
          else {
            J9CurState = digitalRead(J9calPin);
          }
        }
      }
    }



    if (J1done < J1Step && J1CurState == LOW) {
      digitalWrite(J1stepPin, LOW);
      delayMicroseconds(50);
      digitalWrite(J1stepPin, HIGH);
      J1done = ++J1done;
    }
    else {
      J1complete = 1;
    }
    if (J2done < J2Step && J2CurState == LOW) {
      digitalWrite(J2stepPin, LOW);
      delayMicroseconds(50);
      digitalWrite(J2stepPin, HIGH);
      J2done = ++J2done;
    }
    else {
      J2complete = 1;
    }
    if (J3done < J3Step && J3CurState == LOW) {
      digitalWrite(J3stepPin, LOW);
      delayMicroseconds(50);
      digitalWrite(J3stepPin, HIGH);
      J3done = ++J3done;
    }
    else {
      J3complete = 1;
    }
    if (J4done < J4Step && J4CurState == LOW) {
      digitalWrite(J4stepPin, LOW);
      delayMicroseconds(50);
      digitalWrite(J4stepPin, HIGH);
      J4done = ++J4done;
    }
    else {
      J4complete = 1;
    }
    if (J5done < J5Step && J5CurState == LOW) {
      digitalWrite(J5stepPin, LOW);
      delayMicroseconds(50);
      digitalWrite(J5stepPin, HIGH);
      J5done = ++J5done;
    }
    else {
      J5complete = 1;
    }
    if (J6done < J6Step && J6CurState == LOW) {
      digitalWrite(J6stepPin, LOW);
      delayMicroseconds(50);
      digitalWrite(J6stepPin, HIGH);
      J6done = ++J6done;
    }
    else {
      J6complete = 1;
    }
    if (J7done < J7Step && J7CurState == LOW) {
      digitalWrite(J7stepPin, LOW);
      delayMicroseconds(50);
      digitalWrite(J7stepPin, HIGH);
      J7done = ++J7done;
    }
    else {
      J7complete = 1;
    }
    if (J8done < J8Step && J8CurState == LOW) {
      digitalWrite(J8stepPin, LOW);
      delayMicroseconds(50);
      digitalWrite(J8stepPin, HIGH);
      J8done = ++J8done;
    }
    else {
      J8complete = 1;
    }
    if (J9done < J9Step && J9CurState == LOW) {
      digitalWrite(J9stepPin, LOW);
      delayMicroseconds(50);
      digitalWrite(J9stepPin, HIGH);
      J9done = ++J9done;
    }
    else {
      J9complete = 1;
    }
    //jump out if complete
    if (J1complete + J2complete + J3complete + J4complete + J5complete + J6complete + J7complete + J8complete + J9complete == 9) {
      DriveLimInProc = 0;
    }
    ///////////////DELAY BEFORE RESTARTING LOOP
    delayMicroseconds(calcStepGap);
  }

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//CHECK ENCODERS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void resetEncoders() {

  J1collisionTrue = 0;
  J2collisionTrue = 0;
  J3collisionTrue = 0;
  J4collisionTrue = 0;
  J5collisionTrue = 0;
  J6collisionTrue = 0;

  //set encoders to current position
  J1encPos.write(J1StepM * J1encMult);
  J2encPos.write(J2StepM * J2encMult);
  J3encPos.write(J3StepM * J3encMult);
  J4encPos.write(J4StepM * J4encMult);
  J5encPos.write(J5StepM * J5encMult);
  J6encPos.write(J6StepM * J6encMult);
  //delayMicroseconds(5);

}

void checkEncoders() {
  //read encoders
  J1EncSteps = J1encPos.read() / J1encMult;
  J2EncSteps = J2encPos.read() / J2encMult;
  J3EncSteps = J3encPos.read() / J3encMult;
  J4EncSteps = J4encPos.read() / J4encMult;
  J5EncSteps = J5encPos.read() / J5encMult;
  J6EncSteps = J6encPos.read() / J6encMult;

  if (abs((J1EncSteps - J1StepM)) >= 15) {
    if (J1LoopMode == 0) {
      J1collisionTrue = 1;
      J1StepM = J1encPos.read() / J1encMult;
    }
  }
  if (abs((J2EncSteps - J2StepM)) >= 15) {
    if (J2LoopMode == 0) {
      J2collisionTrue = 1;
      J2StepM = J2encPos.read() / J2encMult;
    }
  }
  if (abs((J3EncSteps - J3StepM)) >= 15) {
    if (J3LoopMode == 0) {
      J3collisionTrue = 1;
      J3StepM = J3encPos.read() / J3encMult;
    }
  }
  if (abs((J4EncSteps - J4StepM)) >= 15) {
    if (J4LoopMode == 0) {
      J4collisionTrue = 1;
      J4StepM = J4encPos.read() / J4encMult;
    }
  }
  if (abs((J5EncSteps - J5StepM)) >= 15) {
    if (J5LoopMode == 0) {
      J5collisionTrue = 1;
      J5StepM = J5encPos.read() / J5encMult;
    }
  }
  if (abs((J6EncSteps - J6StepM)) >= 15) {
    if (J6LoopMode == 0) {
      J6collisionTrue = 1;
      J6StepM = J6encPos.read() / J6encMult;
    }
  }

  TotalCollision = J1collisionTrue + J2collisionTrue + J3collisionTrue + J4collisionTrue + J5collisionTrue + J6collisionTrue;
  if (TotalCollision > 0) {
    flag = "EC" + String(J1collisionTrue) + String(J2collisionTrue) + String(J3collisionTrue) + String(J4collisionTrue) + String(J5collisionTrue) + String(J6collisionTrue);
  }
}







/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//DRIVE MOTORS J
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void driveMotorsJ(int J1step, int J2step, int J3step, int J4step, int J5step, int J6step, int J7step, int J8step, int J9step, int J1dir, int J2dir, int J3dir, int J4dir, int J5dir, int J6dir, int J7dir, int J8dir, int J9dir, String SpeedType, float SpeedVal, float ACCspd, float DCCspd, float ACCramp) {

  //FIND HIGHEST STEP
  int HighStep = J1step;
  if (J2step > HighStep)
  {
    HighStep = J2step;
  }
  if (J3step > HighStep)
  {
    HighStep = J3step;
  }
  if (J4step > HighStep)
  {
    HighStep = J4step;
  }
  if (J5step > HighStep)
  {
    HighStep = J5step;
  }
  if (J6step > HighStep)
  {
    HighStep = J6step;
  }
  if (J7step > HighStep)
  {
    HighStep = J7step;
  }
  if (J8step > HighStep)
  {
    HighStep = J8step;
  }
  if (J9step > HighStep)
  {
    HighStep = J9step;
  }

  //FIND ACTIVE JOINTS
  int J1active = 0;
  int J2active = 0;
  int J3active = 0;
  int J4active = 0;
  int J5active = 0;
  int J6active = 0;
  int J7active = 0;
  int J8active = 0;
  int J9active = 0;
  int Jactive = 0;

  if (J1step >= 1)
  {
    J1active = 1;
  }
  if (J2step >= 1)
  {
    J2active = 1;
  }
  if (J3step >= 1)
  {
    J3active = 1;
  }
  if (J4step >= 1)
  {
    J4active = 1;
  }
  if (J5step >= 1)
  {
    J5active = 1;
  }
  if (J6step >= 1)
  {
    J6active = 1;
  }
  if (J7step >= 1)
  {
    J7active = 1;
  }
  if (J8step >= 1)
  {
    J8active = 1;
  }
  if (J9step >= 1)
  {
    J9active = 1;
  }
  Jactive = (J1active + J2active + J3active + J4active + J5active + J6active + J7active + J8active + J9active);

  int J1_PE = 0;
  int J2_PE = 0;
  int J3_PE = 0;
  int J4_PE = 0;
  int J5_PE = 0;
  int J6_PE = 0;
  int J7_PE = 0;
  int J8_PE = 0;
  int J9_PE = 0;

  int J1_SE_1 = 0;
  int J2_SE_1 = 0;
  int J3_SE_1 = 0;
  int J4_SE_1 = 0;
  int J5_SE_1 = 0;
  int J6_SE_1 = 0;
  int J7_SE_1 = 0;
  int J8_SE_1 = 0;
  int J9_SE_1 = 0;

  int J1_SE_2 = 0;
  int J2_SE_2 = 0;
  int J3_SE_2 = 0;
  int J4_SE_2 = 0;
  int J5_SE_2 = 0;
  int J6_SE_2 = 0;
  int J7_SE_2 = 0;
  int J8_SE_2 = 0;
  int J9_SE_2 = 0;

  int J1_LO_1 = 0;
  int J2_LO_1 = 0;
  int J3_LO_1 = 0;
  int J4_LO_1 = 0;
  int J5_LO_1 = 0;
  int J6_LO_1 = 0;
  int J7_LO_1 = 0;
  int J8_LO_1 = 0;
  int J9_LO_1 = 0;

  int J1_LO_2 = 0;
  int J2_LO_2 = 0;
  int J3_LO_2 = 0;
  int J4_LO_2 = 0;
  int J5_LO_2 = 0;
  int J6_LO_2 = 0;
  int J7_LO_2 = 0;
  int J8_LO_2 = 0;
  int J9_LO_2 = 0;

  //reset
  int J1cur = 0;
  int J2cur = 0;
  int J3cur = 0;
  int J4cur = 0;
  int J5cur = 0;
  int J6cur = 0;
  int J7cur = 0;
  int J8cur = 0;
  int J9cur = 0;

  int J1_PEcur = 0;
  int J2_PEcur = 0;
  int J3_PEcur = 0;
  int J4_PEcur = 0;
  int J5_PEcur = 0;
  int J6_PEcur = 0;
  int J7_PEcur = 0;
  int J8_PEcur = 0;
  int J9_PEcur = 0;

  int J1_SE_1cur = 0;
  int J2_SE_1cur = 0;
  int J3_SE_1cur = 0;
  int J4_SE_1cur = 0;
  int J5_SE_1cur = 0;
  int J6_SE_1cur = 0;
  int J7_SE_1cur = 0;
  int J8_SE_1cur = 0;
  int J9_SE_1cur = 0;

  int J1_SE_2cur = 0;
  int J2_SE_2cur = 0;
  int J3_SE_2cur = 0;
  int J4_SE_2cur = 0;
  int J5_SE_2cur = 0;
  int J6_SE_2cur = 0;
  int J7_SE_2cur = 0;
  int J8_SE_2cur = 0;
  int J9_SE_2cur = 0;

  int highStepCur = 0;
  float curDelay = 0;

  float speedSP;
  float moveDist;

  //int J4EncSteps;

  //SET DIRECTIONS

  /// J1 ///
  if (J1dir == 1 && J1MotDir == 1) {
    digitalWrite(J1dirPin, HIGH);
  }
  if (J1dir == 1 && J1MotDir == 0) {
    digitalWrite(J1dirPin, LOW);
  }
  if (J1dir == 0 && J1MotDir == 1) {
    digitalWrite(J1dirPin, LOW);
  }
  if (J1dir == 0 && J1MotDir == 0) {
    digitalWrite(J1dirPin, HIGH);
  }
  /// J2 ///
  if (J2dir == 1 && J2MotDir == 1) {
    digitalWrite(J2dirPin, HIGH);
  }
  if (J2dir == 1 && J2MotDir == 0) {
    digitalWrite(J2dirPin, LOW);
  }
  if (J2dir == 0 && J2MotDir == 1) {
    digitalWrite(J2dirPin, LOW);
  }
  if (J2dir == 0 && J2MotDir == 0) {
    digitalWrite(J2dirPin, HIGH);
  }
  /// J3 ///
  if (J3dir == 1 && J3MotDir == 1) {
    digitalWrite(J3dirPin, HIGH);
  }
  if (J3dir == 1 && J3MotDir == 0) {
    digitalWrite(J3dirPin, LOW);
  }
  if (J3dir == 0 && J3MotDir == 1) {
    digitalWrite(J3dirPin, LOW);
  }
  if (J3dir == 0 && J3MotDir == 0) {
    digitalWrite(J3dirPin, HIGH);
  }
  /// J4 ///
  if (J4dir == 1 && J4MotDir == 1) {
    digitalWrite(J4dirPin, HIGH);
  }
  if (J4dir == 1 && J4MotDir == 0) {
    digitalWrite(J4dirPin, LOW);
  }
  if (J4dir == 0 && J4MotDir == 1) {
    digitalWrite(J4dirPin, LOW);
  }
  if (J4dir == 0 && J4MotDir == 0) {
    digitalWrite(J4dirPin, HIGH);
  }
  /// J5 ///
  if (J5dir == 1 && J5MotDir == 1) {
    digitalWrite(J5dirPin, HIGH);
  }
  if (J5dir == 1 && J5MotDir == 0) {
    digitalWrite(J5dirPin, LOW);
  }
  if (J5dir == 0 && J5MotDir == 1) {
    digitalWrite(J5dirPin, LOW);
  }
  if (J5dir == 0 && J5MotDir == 0) {
    digitalWrite(J5dirPin, HIGH);
  }
  /// J6 ///
  if (J6dir == 1 && J6MotDir == 1) {
    digitalWrite(J6dirPin, HIGH);
  }
  if (J6dir == 1 && J6MotDir == 0) {
    digitalWrite(J6dirPin, LOW);
  }
  if (J6dir == 0 && J6MotDir == 1) {
    digitalWrite(J6dirPin, LOW);
  }
  if (J6dir == 0 && J6MotDir == 0) {
    digitalWrite(J6dirPin, HIGH);
  }
  /// J7 ///
  if (J7dir == 1 && J7MotDir == 1) {
    digitalWrite(J7dirPin, HIGH);
  }
  if (J7dir == 1 && J7MotDir == 0) {
    digitalWrite(J7dirPin, LOW);
  }
  if (J7dir == 0 && J7MotDir == 1) {
    digitalWrite(J7dirPin, LOW);
  }
  if (J7dir == 0 && J7MotDir == 0) {
    digitalWrite(J7dirPin, HIGH);
  }
  /// J8 ///
  if (J8dir == 1 && J8MotDir == 1) {
    digitalWrite(J8dirPin, HIGH);
  }
  if (J8dir == 1 && J8MotDir == 0) {
    digitalWrite(J8dirPin, LOW);
  }
  if (J8dir == 0 && J8MotDir == 1) {
    digitalWrite(J8dirPin, LOW);
  }
  if (J8dir == 0 && J8MotDir == 0) {
    digitalWrite(J8dirPin, HIGH);
  }
  /// J9 ///
  if (J9dir == 1 && J9MotDir == 1) {
    digitalWrite(J9dirPin, HIGH);
  }
  if (J9dir == 1 && J9MotDir == 0) {
    digitalWrite(J9dirPin, LOW);
  }
  if (J9dir == 0 && J9MotDir == 1) {
    digitalWrite(J9dirPin, LOW);
  }
  if (J9dir == 0 && J9MotDir == 0) {
    digitalWrite(J9dirPin, HIGH);
  }

  delayMicroseconds(15);

  /////CALC SPEEDS//////
  float calcStepGap;

  //determine steps
  float ACCStep = HighStep * (ACCspd / 100);
  float NORStep = HighStep * ((100 - ACCspd - DCCspd) / 100);
  float DCCStep = HighStep * (DCCspd / 100);

  //set speed for seconds or mm per sec
  if (SpeedType == "s") {
    speedSP = (SpeedVal * 1000000) * 1.2;
  }
  else if (SpeedType == "m") {
    lineDist = pow((pow((xyzuvw_In[0] - xyzuvw_Out[0]), 2) + pow((xyzuvw_In[1] - xyzuvw_Out[1]), 2) + pow((xyzuvw_In[2] - xyzuvw_Out[2]), 2)), .5);
    speedSP = ((lineDist / SpeedVal) * 1000000) * 1.2;
  }

  //calc step gap for seconds or mm per sec
  if (SpeedType == "s" or SpeedType == "m" ) {
    float zeroStepGap = speedSP / HighStep;
    float zeroACCstepInc = (zeroStepGap * (100 / ACCramp)) / ACCStep;
    float zeroACCtime = ((ACCStep) * zeroStepGap) + ((ACCStep - 9) * (((ACCStep) * (zeroACCstepInc / 2))));
    float zeroNORtime = NORStep * zeroStepGap;
    float zeroDCCstepInc = (zeroStepGap * (100 / ACCramp)) / DCCStep;
    float zeroDCCtime = ((DCCStep) * zeroStepGap) + ((DCCStep - 9) * (((DCCStep) * (zeroDCCstepInc / 2))));
    float zeroTOTtime = zeroACCtime + zeroNORtime + zeroDCCtime;
    float overclockPerc = speedSP / zeroTOTtime;
    calcStepGap = zeroStepGap * overclockPerc;
    if (calcStepGap <= minSpeedDelay) {
      calcStepGap = minSpeedDelay;
      speedViolation = "1";
    }
  }

  //calc step gap for percentage
  else if (SpeedType == "p") {
    //calcStepGap = (maxSpeedDelay - ((SpeedVal / 100) * (maxSpeedDelay - calcStepGap =)));
    calcStepGap = minSpeedDelay / (SpeedVal / 100);
  }

  //calculate final step increments
  float calcACCstepInc = (calcStepGap * (100 / ACCramp)) / ACCStep;
  float calcDCCstepInc = (calcStepGap * (100 / ACCramp)) / DCCStep;
  float calcACCstartDel = (calcACCstepInc * ACCStep) * 2;

  //set starting delay
  if (rndTrue == true) {
    curDelay = rndSpeed;
    rndTrue = false;
  }
  else {
    curDelay = calcACCstartDel;
  }

  ///// DRIVE MOTORS /////
  while (J1cur < J1step || J2cur < J2step || J3cur < J3step || J4cur < J4step || J5cur < J5step || J6cur < J6step || J7cur < J7step || J8cur < J8step || J9cur < J9step)
  {

    ////DELAY CALC/////
    if (highStepCur <= ACCStep) {
      curDelay = curDelay - (calcACCstepInc);
    }
    else if (highStepCur >= (HighStep - DCCStep)) {
      curDelay = curDelay + (calcDCCstepInc);
    }
    else {
      curDelay = calcStepGap;
    }



    float distDelay = 60;
    float disDelayCur = 0;


    /////// J1 ////////////////////////////////
    ///find pulse every
    if (J1cur < J1step)
    {
      J1_PE = (HighStep / J1step);
      ///find left over 1
      J1_LO_1 = (HighStep - (J1step * J1_PE));
      ///find skip 1
      if (J1_LO_1 > 0)
      {
        J1_SE_1 = (HighStep / J1_LO_1);
      }
      else
      {
        J1_SE_1 = 0;
      }
      ///find left over 2
      if (J1_SE_1 > 0)
      {
        J1_LO_2 = HighStep - ((J1step * J1_PE) + ((J1step * J1_PE) / J1_SE_1));
      }
      else
      {
        J1_LO_2 = 0;
      }
      ///find skip 2
      if (J1_LO_2 > 0)
      {
        J1_SE_2 = (HighStep / J1_LO_2);
      }
      else
      {
        J1_SE_2 = 0;
      }
      /////////  J1  ///////////////
      if (J1_SE_2 == 0)
      {
        J1_SE_2cur = (J1_SE_2 + 1);
      }
      if (J1_SE_2cur != J1_SE_2)
      {
        J1_SE_2cur = ++J1_SE_2cur;
        if (J1_SE_1 == 0)
        {
          J1_SE_1cur = (J1_SE_1 + 1);
        }
        if (J1_SE_1cur != J1_SE_1)
        {
          J1_SE_1cur = ++J1_SE_1cur;
          J1_PEcur = ++J1_PEcur;
          if (J1_PEcur == J1_PE)
          {
            J1cur = ++J1cur;
            J1_PEcur = 0;
            digitalWrite(J1stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J1dir == 0) {
              J1StepM == --J1StepM;
            }
            else {
              J1StepM == ++J1StepM;
            }
          }
        }
        else
        {
          J1_SE_1cur = 0;
        }
      }
      else
      {
        J1_SE_2cur = 0;
      }
    }


    /////// J2 ////////////////////////////////
    ///find pulse every
    if (J2cur < J2step)
    {
      J2_PE = (HighStep / J2step);
      ///find left over 1
      J2_LO_1 = (HighStep - (J2step * J2_PE));
      ///find skip 1
      if (J2_LO_1 > 0)
      {
        J2_SE_1 = (HighStep / J2_LO_1);
      }
      else
      {
        J2_SE_1 = 0;
      }
      ///find left over 2
      if (J2_SE_1 > 0)
      {
        J2_LO_2 = HighStep - ((J2step * J2_PE) + ((J2step * J2_PE) / J2_SE_1));
      }
      else
      {
        J2_LO_2 = 0;
      }
      ///find skip 2
      if (J2_LO_2 > 0)
      {
        J2_SE_2 = (HighStep / J2_LO_2);
      }
      else
      {
        J2_SE_2 = 0;
      }
      /////////  J2  ///////////////
      if (J2_SE_2 == 0)
      {
        J2_SE_2cur = (J2_SE_2 + 1);
      }
      if (J2_SE_2cur != J2_SE_2)
      {
        J2_SE_2cur = ++J2_SE_2cur;
        if (J2_SE_1 == 0)
        {
          J2_SE_1cur = (J2_SE_1 + 1);
        }
        if (J2_SE_1cur != J2_SE_1)
        {
          J2_SE_1cur = ++J2_SE_1cur;
          J2_PEcur = ++J2_PEcur;
          if (J2_PEcur == J2_PE)
          {
            J2cur = ++J2cur;
            J2_PEcur = 0;
            digitalWrite(J2stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J2dir == 0) {
              J2StepM == --J2StepM;
            }
            else {
              J2StepM == ++J2StepM;
            }
          }
        }
        else
        {
          J2_SE_1cur = 0;
        }
      }
      else
      {
        J2_SE_2cur = 0;
      }
    }

    /////// J3 ////////////////////////////////
    ///find pulse every
    if (J3cur < J3step)
    {
      J3_PE = (HighStep / J3step);
      ///find left over 1
      J3_LO_1 = (HighStep - (J3step * J3_PE));
      ///find skip 1
      if (J3_LO_1 > 0)
      {
        J3_SE_1 = (HighStep / J3_LO_1);
      }
      else
      {
        J3_SE_1 = 0;
      }
      ///find left over 2
      if (J3_SE_1 > 0)
      {
        J3_LO_2 = HighStep - ((J3step * J3_PE) + ((J3step * J3_PE) / J3_SE_1));
      }
      else
      {
        J3_LO_2 = 0;
      }
      ///find skip 2
      if (J3_LO_2 > 0)
      {
        J3_SE_2 = (HighStep / J3_LO_2);
      }
      else
      {
        J3_SE_2 = 0;
      }
      /////////  J3  ///////////////
      if (J3_SE_2 == 0)
      {
        J3_SE_2cur = (J3_SE_2 + 1);
      }
      if (J3_SE_2cur != J3_SE_2)
      {
        J3_SE_2cur = ++J3_SE_2cur;
        if (J3_SE_1 == 0)
        {
          J3_SE_1cur = (J3_SE_1 + 1);
        }
        if (J3_SE_1cur != J3_SE_1)
        {
          J3_SE_1cur = ++J3_SE_1cur;
          J3_PEcur = ++J3_PEcur;
          if (J3_PEcur == J3_PE)
          {
            J3cur = ++J3cur;
            J3_PEcur = 0;
            digitalWrite(J3stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J3dir == 0) {
              J3StepM == --J3StepM;
            }
            else {
              J3StepM == ++J3StepM;
            }
          }
        }
        else
        {
          J3_SE_1cur = 0;
        }
      }
      else
      {
        J3_SE_2cur = 0;
      }
    }


    /////// J4 ////////////////////////////////
    ///find pulse every
    if (J4cur < J4step)
    {
      J4_PE = (HighStep / J4step);
      ///find left over 1
      J4_LO_1 = (HighStep - (J4step * J4_PE));
      ///find skip 1
      if (J4_LO_1 > 0)
      {
        J4_SE_1 = (HighStep / J4_LO_1);
      }
      else
      {
        J4_SE_1 = 0;
      }
      ///find left over 2
      if (J4_SE_1 > 0)
      {
        J4_LO_2 = HighStep - ((J4step * J4_PE) + ((J4step * J4_PE) / J4_SE_1));
      }
      else
      {
        J4_LO_2 = 0;
      }
      ///find skip 2
      if (J4_LO_2 > 0)
      {
        J4_SE_2 = (HighStep / J4_LO_2);
      }
      else
      {
        J4_SE_2 = 0;
      }
      /////////  J4  ///////////////
      if (J4_SE_2 == 0)
      {
        J4_SE_2cur = (J4_SE_2 + 1);
      }
      if (J4_SE_2cur != J4_SE_2)
      {
        J4_SE_2cur = ++J4_SE_2cur;
        if (J4_SE_1 == 0)
        {
          J4_SE_1cur = (J4_SE_1 + 1);
        }
        if (J4_SE_1cur != J4_SE_1)
        {
          J4_SE_1cur = ++J4_SE_1cur;
          J4_PEcur = ++J4_PEcur;
          if (J4_PEcur == J4_PE)
          {
            J4cur = ++J4cur;
            J4_PEcur = 0;
            digitalWrite(J4stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J4dir == 0) {
              J4StepM == --J4StepM;
            }
            else {
              J4StepM == ++J4StepM;
            }
          }
        }
        else
        {
          J4_SE_1cur = 0;
        }
      }
      else
      {
        J4_SE_2cur = 0;
      }
    }


    /////// J5 ////////////////////////////////
    ///find pulse every
    if (J5cur < J5step)
    {
      J5_PE = (HighStep / J5step);
      ///find left over 1
      J5_LO_1 = (HighStep - (J5step * J5_PE));
      ///find skip 1
      if (J5_LO_1 > 0)
      {
        J5_SE_1 = (HighStep / J5_LO_1);
      }
      else
      {
        J5_SE_1 = 0;
      }
      ///find left over 2
      if (J5_SE_1 > 0)
      {
        J5_LO_2 = HighStep - ((J5step * J5_PE) + ((J5step * J5_PE) / J5_SE_1));
      }
      else
      {
        J5_LO_2 = 0;
      }
      ///find skip 2
      if (J5_LO_2 > 0)
      {
        J5_SE_2 = (HighStep / J5_LO_2);
      }
      else
      {
        J5_SE_2 = 0;
      }
      /////////  J5  ///////////////
      if (J5_SE_2 == 0)
      {
        J5_SE_2cur = (J5_SE_2 + 1);
      }
      if (J5_SE_2cur != J5_SE_2)
      {
        J5_SE_2cur = ++J5_SE_2cur;
        if (J5_SE_1 == 0)
        {
          J5_SE_1cur = (J5_SE_1 + 1);
        }
        if (J5_SE_1cur != J5_SE_1)
        {
          J5_SE_1cur = ++J5_SE_1cur;
          J5_PEcur = ++J5_PEcur;
          if (J5_PEcur == J5_PE)
          {
            J5cur = ++J5cur;
            J5_PEcur = 0;
            digitalWrite(J5stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J5dir == 0) {
              J5StepM == --J5StepM;
            }
            else {
              J5StepM == ++J5StepM;
            }
          }
        }
        else
        {
          J5_SE_1cur = 0;
        }
      }
      else
      {
        J5_SE_2cur = 0;
      }
    }


    /////// J6 ////////////////////////////////
    ///find pulse every
    if (J6cur < J6step)
    {
      J6_PE = (HighStep / J6step);
      ///find left over 1
      J6_LO_1 = (HighStep - (J6step * J6_PE));
      ///find skip 1
      if (J6_LO_1 > 0)
      {
        J6_SE_1 = (HighStep / J6_LO_1);
      }
      else
      {
        J6_SE_1 = 0;
      }
      ///find left over 2
      if (J6_SE_1 > 0)
      {
        J6_LO_2 = HighStep - ((J6step * J6_PE) + ((J6step * J6_PE) / J6_SE_1));
      }
      else
      {
        J6_LO_2 = 0;
      }
      ///find skip 2
      if (J6_LO_2 > 0)
      {
        J6_SE_2 = (HighStep / J6_LO_2);
      }
      else
      {
        J6_SE_2 = 0;
      }
      /////////  J6  ///////////////
      if (J6_SE_2 == 0)
      {
        J6_SE_2cur = (J6_SE_2 + 1);
      }
      if (J6_SE_2cur != J6_SE_2)
      {
        J6_SE_2cur = ++J6_SE_2cur;
        if (J6_SE_1 == 0)
        {
          J6_SE_1cur = (J6_SE_1 + 1);
        }
        if (J6_SE_1cur != J6_SE_1)
        {
          J6_SE_1cur = ++J6_SE_1cur;
          J6_PEcur = ++J6_PEcur;
          if (J6_PEcur == J6_PE)
          {
            J6cur = ++J6cur;
            J6_PEcur = 0;
            digitalWrite(J6stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J6dir == 0) {
              J6StepM == --J6StepM;
            }
            else {
              J6StepM == ++J6StepM;
            }
          }
        }
        else
        {
          J6_SE_1cur = 0;
        }
      }
      else
      {
        J6_SE_2cur = 0;
      }
    }


    /////// J7 ////////////////////////////////
    ///find pulse every
    if (J7cur < J7step)
    {
      J7_PE = (HighStep / J7step);
      ///find left over 1
      J7_LO_1 = (HighStep - (J7step * J7_PE));
      ///find skip 1
      if (J7_LO_1 > 0)
      {
        J7_SE_1 = (HighStep / J7_LO_1);
      }
      else
      {
        J7_SE_1 = 0;
      }
      ///find left over 2
      if (J7_SE_1 > 0)
      {
        J7_LO_2 = HighStep - ((J7step * J7_PE) + ((J7step * J7_PE) / J7_SE_1));
      }
      else
      {
        J7_LO_2 = 0;
      }
      ///find skip 2
      if (J7_LO_2 > 0)
      {
        J7_SE_2 = (HighStep / J7_LO_2);
      }
      else
      {
        J7_SE_2 = 0;
      }
      /////////  J7  ///////////////
      if (J7_SE_2 == 0)
      {
        J7_SE_2cur = (J7_SE_2 + 1);
      }
      if (J7_SE_2cur != J7_SE_2)
      {
        J7_SE_2cur = ++J7_SE_2cur;
        if (J7_SE_1 == 0)
        {
          J7_SE_1cur = (J7_SE_1 + 1);
        }
        if (J7_SE_1cur != J7_SE_1)
        {
          J7_SE_1cur = ++J7_SE_1cur;
          J7_PEcur = ++J7_PEcur;
          if (J7_PEcur == J7_PE)
          {
            J7cur = ++J7cur;
            J7_PEcur = 0;
            digitalWrite(J7stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J7dir == 0) {
              J7StepM == --J7StepM;
            }
            else {
              J7StepM == ++J7StepM;
            }
          }
        }
        else
        {
          J7_SE_1cur = 0;
        }
      }
      else
      {
        J7_SE_2cur = 0;
      }
    }




    /////// J8 ////////////////////////////////
    ///find pulse every
    if (J8cur < J8step)
    {
      J8_PE = (HighStep / J8step);
      ///find left over 1
      J8_LO_1 = (HighStep - (J8step * J8_PE));
      ///find skip 1
      if (J8_LO_1 > 0)
      {
        J8_SE_1 = (HighStep / J8_LO_1);
      }
      else
      {
        J8_SE_1 = 0;
      }
      ///find left over 2
      if (J8_SE_1 > 0)
      {
        J8_LO_2 = HighStep - ((J8step * J8_PE) + ((J8step * J8_PE) / J8_SE_1));
      }
      else
      {
        J8_LO_2 = 0;
      }
      ///find skip 2
      if (J8_LO_2 > 0)
      {
        J8_SE_2 = (HighStep / J8_LO_2);
      }
      else
      {
        J8_SE_2 = 0;
      }
      /////////  J8  ///////////////
      if (J8_SE_2 == 0)
      {
        J8_SE_2cur = (J8_SE_2 + 1);
      }
      if (J8_SE_2cur != J8_SE_2)
      {
        J8_SE_2cur = ++J8_SE_2cur;
        if (J8_SE_1 == 0)
        {
          J8_SE_1cur = (J8_SE_1 + 1);
        }
        if (J8_SE_1cur != J8_SE_1)
        {
          J8_SE_1cur = ++J8_SE_1cur;
          J8_PEcur = ++J8_PEcur;
          if (J8_PEcur == J8_PE)
          {
            J8cur = ++J8cur;
            J8_PEcur = 0;
            digitalWrite(J8stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J8dir == 0) {
              J8StepM == --J8StepM;
            }
            else {
              J8StepM == ++J8StepM;
            }
          }
        }
        else
        {
          J8_SE_1cur = 0;
        }
      }
      else
      {
        J8_SE_2cur = 0;
      }
    }


    /////// J9 ////////////////////////////////
    ///find pulse every
    if (J9cur < J9step)
    {
      J9_PE = (HighStep / J9step);
      ///find left over 1
      J9_LO_1 = (HighStep - (J9step * J9_PE));
      ///find skip 1
      if (J9_LO_1 > 0)
      {
        J9_SE_1 = (HighStep / J9_LO_1);
      }
      else
      {
        J9_SE_1 = 0;
      }
      ///find left over 2
      if (J9_SE_1 > 0)
      {
        J9_LO_2 = HighStep - ((J9step * J9_PE) + ((J9step * J9_PE) / J9_SE_1));
      }
      else
      {
        J9_LO_2 = 0;
      }
      ///find skip 2
      if (J9_LO_2 > 0)
      {
        J9_SE_2 = (HighStep / J9_LO_2);
      }
      else
      {
        J9_SE_2 = 0;
      }
      /////////  J9  ///////////////
      if (J9_SE_2 == 0)
      {
        J9_SE_2cur = (J9_SE_2 + 1);
      }
      if (J9_SE_2cur != J9_SE_2)
      {
        J9_SE_2cur = ++J9_SE_2cur;
        if (J9_SE_1 == 0)
        {
          J9_SE_1cur = (J9_SE_1 + 1);
        }
        if (J9_SE_1cur != J9_SE_1)
        {
          J9_SE_1cur = ++J9_SE_1cur;
          J9_PEcur = ++J9_PEcur;
          if (J9_PEcur == J9_PE)
          {
            J9cur = ++J9cur;
            J9_PEcur = 0;
            digitalWrite(J9stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J9dir == 0) {
              J9StepM == --J9StepM;
            }
            else {
              J9StepM == ++J9StepM;
            }
          }
        }
        else
        {
          J9_SE_1cur = 0;
        }
      }
      else
      {
        J9_SE_2cur = 0;
      }
    }


    // inc cur step
    highStepCur = ++highStepCur;
    digitalWrite(J1stepPin, HIGH);
    digitalWrite(J2stepPin, HIGH);
    digitalWrite(J3stepPin, HIGH);
    digitalWrite(J4stepPin, HIGH);
    digitalWrite(J5stepPin, HIGH);
    digitalWrite(J6stepPin, HIGH);
    digitalWrite(J7stepPin, HIGH);
    digitalWrite(J8stepPin, HIGH);
    digitalWrite(J9stepPin, HIGH);
    delayMicroseconds(curDelay - disDelayCur);


  }
  //set rounding speed to last move speed
  rndSpeed = curDelay;
}




/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//DRIVE MOTORS G
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void driveMotorsG(int J1step, int J2step, int J3step, int J4step, int J5step, int J6step, int J7step, int J8step, int J9step, int J1dir, int J2dir, int J3dir, int J4dir, int J5dir, int J6dir, int J7dir, int J8dir, int J9dir, String SpeedType, float SpeedVal, float ACCspd, float DCCspd, float ACCramp) {

  //FIND HIGHEST STEP
  int HighStep = J1step;
  if (J2step > HighStep)
  {
    HighStep = J2step;
  }
  if (J3step > HighStep)
  {
    HighStep = J3step;
  }
  if (J4step > HighStep)
  {
    HighStep = J4step;
  }
  if (J5step > HighStep)
  {
    HighStep = J5step;
  }
  if (J6step > HighStep)
  {
    HighStep = J6step;
  }
  if (J7step > HighStep)
  {
    HighStep = J7step;
  }
  if (J8step > HighStep)
  {
    HighStep = J8step;
  }
  if (J9step > HighStep)
  {
    HighStep = J9step;
  }

  //FIND ACTIVE JOINTS
  int J1active = 0;
  int J2active = 0;
  int J3active = 0;
  int J4active = 0;
  int J5active = 0;
  int J6active = 0;
  int J7active = 0;
  int J8active = 0;
  int J9active = 0;
  int Jactive = 0;

  if (J1step >= 1)
  {
    J1active = 1;
  }
  if (J2step >= 1)
  {
    J2active = 1;
  }
  if (J3step >= 1)
  {
    J3active = 1;
  }
  if (J4step >= 1)
  {
    J4active = 1;
  }
  if (J5step >= 1)
  {
    J5active = 1;
  }
  if (J6step >= 1)
  {
    J6active = 1;
  }
  if (J7step >= 1)
  {
    J7active = 1;
  }
  if (J8step >= 1)
  {
    J8active = 1;
  }
  if (J9step >= 1)
  {
    J9active = 1;
  }
  Jactive = (J1active + J2active + J3active + J4active + J5active + J6active + J7active + J8active + J9active);

  int J1_PE = 0;
  int J2_PE = 0;
  int J3_PE = 0;
  int J4_PE = 0;
  int J5_PE = 0;
  int J6_PE = 0;
  int J7_PE = 0;
  int J8_PE = 0;
  int J9_PE = 0;

  int J1_SE_1 = 0;
  int J2_SE_1 = 0;
  int J3_SE_1 = 0;
  int J4_SE_1 = 0;
  int J5_SE_1 = 0;
  int J6_SE_1 = 0;
  int J7_SE_1 = 0;
  int J8_SE_1 = 0;
  int J9_SE_1 = 0;

  int J1_SE_2 = 0;
  int J2_SE_2 = 0;
  int J3_SE_2 = 0;
  int J4_SE_2 = 0;
  int J5_SE_2 = 0;
  int J6_SE_2 = 0;
  int J7_SE_2 = 0;
  int J8_SE_2 = 0;
  int J9_SE_2 = 0;

  int J1_LO_1 = 0;
  int J2_LO_1 = 0;
  int J3_LO_1 = 0;
  int J4_LO_1 = 0;
  int J5_LO_1 = 0;
  int J6_LO_1 = 0;
  int J7_LO_1 = 0;
  int J8_LO_1 = 0;
  int J9_LO_1 = 0;

  int J1_LO_2 = 0;
  int J2_LO_2 = 0;
  int J3_LO_2 = 0;
  int J4_LO_2 = 0;
  int J5_LO_2 = 0;
  int J6_LO_2 = 0;
  int J7_LO_2 = 0;
  int J8_LO_2 = 0;
  int J9_LO_2 = 0;

  //reset
  int J1cur = 0;
  int J2cur = 0;
  int J3cur = 0;
  int J4cur = 0;
  int J5cur = 0;
  int J6cur = 0;
  int J7cur = 0;
  int J8cur = 0;
  int J9cur = 0;

  int J1_PEcur = 0;
  int J2_PEcur = 0;
  int J3_PEcur = 0;
  int J4_PEcur = 0;
  int J5_PEcur = 0;
  int J6_PEcur = 0;
  int J7_PEcur = 0;
  int J8_PEcur = 0;
  int J9_PEcur = 0;

  int J1_SE_1cur = 0;
  int J2_SE_1cur = 0;
  int J3_SE_1cur = 0;
  int J4_SE_1cur = 0;
  int J5_SE_1cur = 0;
  int J6_SE_1cur = 0;
  int J7_SE_1cur = 0;
  int J8_SE_1cur = 0;
  int J9_SE_1cur = 0;

  int J1_SE_2cur = 0;
  int J2_SE_2cur = 0;
  int J3_SE_2cur = 0;
  int J4_SE_2cur = 0;
  int J5_SE_2cur = 0;
  int J6_SE_2cur = 0;
  int J7_SE_2cur = 0;
  int J8_SE_2cur = 0;
  int J9_SE_2cur = 0;

  int highStepCur = 0;
  float curDelay = 0;

  float speedSP;
  float moveDist;

  //int J4EncSteps;

  //SET DIRECTIONS
  /// J1 ///
  if (J1dir == 1 && J1MotDir == 1) {
    digitalWrite(J1dirPin, HIGH);
  }
  if (J1dir == 1 && J1MotDir == 0) {
    digitalWrite(J1dirPin, LOW);
  }
  if (J1dir == 0 && J1MotDir == 1) {
    digitalWrite(J1dirPin, LOW);
  }
  if (J1dir == 0 && J1MotDir == 0) {
    digitalWrite(J1dirPin, HIGH);
  }
  /// J2 ///
  if (J2dir == 1 && J2MotDir == 1) {
    digitalWrite(J2dirPin, HIGH);
  }
  if (J2dir == 1 && J2MotDir == 0) {
    digitalWrite(J2dirPin, LOW);
  }
  if (J2dir == 0 && J2MotDir == 1) {
    digitalWrite(J2dirPin, LOW);
  }
  if (J2dir == 0 && J2MotDir == 0) {
    digitalWrite(J2dirPin, HIGH);
  }
  /// J3 ///
  if (J3dir == 1 && J3MotDir == 1) {
    digitalWrite(J3dirPin, HIGH);
  }
  if (J3dir == 1 && J3MotDir == 0) {
    digitalWrite(J3dirPin, LOW);
  }
  if (J3dir == 0 && J3MotDir == 1) {
    digitalWrite(J3dirPin, LOW);
  }
  if (J3dir == 0 && J3MotDir == 0) {
    digitalWrite(J3dirPin, HIGH);
  }
  /// J4 ///
  if (J4dir == 1 && J4MotDir == 1) {
    digitalWrite(J4dirPin, HIGH);
  }
  if (J4dir == 1 && J4MotDir == 0) {
    digitalWrite(J4dirPin, LOW);
  }
  if (J4dir == 0 && J4MotDir == 1) {
    digitalWrite(J4dirPin, LOW);
  }
  if (J4dir == 0 && J4MotDir == 0) {
    digitalWrite(J4dirPin, HIGH);
  }
  /// J5 ///
  if (J5dir == 1 && J5MotDir == 1) {
    digitalWrite(J5dirPin, HIGH);
  }
  if (J5dir == 1 && J5MotDir == 0) {
    digitalWrite(J5dirPin, LOW);
  }
  if (J5dir == 0 && J5MotDir == 1) {
    digitalWrite(J5dirPin, LOW);
  }
  if (J5dir == 0 && J5MotDir == 0) {
    digitalWrite(J5dirPin, HIGH);
  }
  /// J6 ///
  if (J6dir == 1 && J6MotDir == 1) {
    digitalWrite(J6dirPin, HIGH);
  }
  if (J6dir == 1 && J6MotDir == 0) {
    digitalWrite(J6dirPin, LOW);
  }
  if (J6dir == 0 && J6MotDir == 1) {
    digitalWrite(J6dirPin, LOW);
  }
  if (J6dir == 0 && J6MotDir == 0) {
    digitalWrite(J6dirPin, HIGH);
  }
  /// J7 ///
  if (J7dir == 1 && J7MotDir == 1) {
    digitalWrite(J7dirPin, HIGH);
  }
  if (J7dir == 1 && J7MotDir == 0) {
    digitalWrite(J7dirPin, LOW);
  }
  if (J7dir == 0 && J7MotDir == 1) {
    digitalWrite(J7dirPin, LOW);
  }
  if (J7dir == 0 && J7MotDir == 0) {
    digitalWrite(J7dirPin, HIGH);
  }
  /// J8 ///
  if (J8dir == 1 && J8MotDir == 1) {
    digitalWrite(J8dirPin, HIGH);
  }
  if (J8dir == 1 && J8MotDir == 0) {
    digitalWrite(J8dirPin, LOW);
  }
  if (J8dir == 0 && J8MotDir == 1) {
    digitalWrite(J8dirPin, LOW);
  }
  if (J8dir == 0 && J8MotDir == 0) {
    digitalWrite(J8dirPin, HIGH);
  }
  /// J9 ///
  if (J9dir == 1 && J9MotDir == 1) {
    digitalWrite(J9dirPin, HIGH);
  }
  if (J9dir == 1 && J9MotDir == 0) {
    digitalWrite(J9dirPin, LOW);
  }
  if (J9dir == 0 && J9MotDir == 1) {
    digitalWrite(J9dirPin, LOW);
  }
  if (J9dir == 0 && J9MotDir == 0) {
    digitalWrite(J9dirPin, HIGH);
  }

  delayMicroseconds(15);

  /////CALC SPEEDS//////
  float calcStepGap;

  //set speed for seconds or mm per sec
  if (SpeedType == "s") {
    speedSP = (SpeedVal * 1000000) * 1.2;
  }
  else if (SpeedType == "m") {
    lineDist = pow((pow((xyzuvw_In[0] - xyzuvw_Out[0]), 2) + pow((xyzuvw_In[1] - xyzuvw_Out[1]), 2) + pow((xyzuvw_In[2] - xyzuvw_Out[2]), 2)), .5);
    speedSP = ((lineDist / SpeedVal) * 1000000) * 1.2;
  }

  //calc step gap for seconds or mm per sec
  if (SpeedType == "s" or SpeedType == "m" ) {
    calcStepGap = speedSP / HighStep;
    if (calcStepGap <= minSpeedDelay) {
      calcStepGap = minSpeedDelay;
      speedViolation = "1";
    }
  }

  //calc step gap for percentage
  else if (SpeedType == "p") {
    calcStepGap = minSpeedDelay / (SpeedVal / 100);
  }


  ///// DRIVE MOTORS /////
  while (J1cur < J1step || J2cur < J2step || J3cur < J3step || J4cur < J4step || J5cur < J5step || J6cur < J6step || J7cur < J7step || J8cur < J8step || J9cur < J9step)
  {

    curDelay = calcStepGap;

    float distDelay = 60;
    float disDelayCur = 0;


    /////// J1 ////////////////////////////////
    ///find pulse every
    if (J1cur < J1step)
    {
      J1_PE = (HighStep / J1step);
      ///find left over 1
      J1_LO_1 = (HighStep - (J1step * J1_PE));
      ///find skip 1
      if (J1_LO_1 > 0)
      {
        J1_SE_1 = (HighStep / J1_LO_1);
      }
      else
      {
        J1_SE_1 = 0;
      }
      ///find left over 2
      if (J1_SE_1 > 0)
      {
        J1_LO_2 = HighStep - ((J1step * J1_PE) + ((J1step * J1_PE) / J1_SE_1));
      }
      else
      {
        J1_LO_2 = 0;
      }
      ///find skip 2
      if (J1_LO_2 > 0)
      {
        J1_SE_2 = (HighStep / J1_LO_2);
      }
      else
      {
        J1_SE_2 = 0;
      }
      /////////  J1  ///////////////
      if (J1_SE_2 == 0)
      {
        J1_SE_2cur = (J1_SE_2 + 1);
      }
      if (J1_SE_2cur != J1_SE_2)
      {
        J1_SE_2cur = ++J1_SE_2cur;
        if (J1_SE_1 == 0)
        {
          J1_SE_1cur = (J1_SE_1 + 1);
        }
        if (J1_SE_1cur != J1_SE_1)
        {
          J1_SE_1cur = ++J1_SE_1cur;
          J1_PEcur = ++J1_PEcur;
          if (J1_PEcur == J1_PE)
          {
            J1cur = ++J1cur;
            J1_PEcur = 0;
            digitalWrite(J1stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J1dir == 0) {
              J1StepM == --J1StepM;
            }
            else {
              J1StepM == ++J1StepM;
            }
          }
        }
        else
        {
          J1_SE_1cur = 0;
        }
      }
      else
      {
        J1_SE_2cur = 0;
      }
    }


    /////// J2 ////////////////////////////////
    ///find pulse every
    if (J2cur < J2step)
    {
      J2_PE = (HighStep / J2step);
      ///find left over 1
      J2_LO_1 = (HighStep - (J2step * J2_PE));
      ///find skip 1
      if (J2_LO_1 > 0)
      {
        J2_SE_1 = (HighStep / J2_LO_1);
      }
      else
      {
        J2_SE_1 = 0;
      }
      ///find left over 2
      if (J2_SE_1 > 0)
      {
        J2_LO_2 = HighStep - ((J2step * J2_PE) + ((J2step * J2_PE) / J2_SE_1));
      }
      else
      {
        J2_LO_2 = 0;
      }
      ///find skip 2
      if (J2_LO_2 > 0)
      {
        J2_SE_2 = (HighStep / J2_LO_2);
      }
      else
      {
        J2_SE_2 = 0;
      }
      /////////  J2  ///////////////
      if (J2_SE_2 == 0)
      {
        J2_SE_2cur = (J2_SE_2 + 1);
      }
      if (J2_SE_2cur != J2_SE_2)
      {
        J2_SE_2cur = ++J2_SE_2cur;
        if (J2_SE_1 == 0)
        {
          J2_SE_1cur = (J2_SE_1 + 1);
        }
        if (J2_SE_1cur != J2_SE_1)
        {
          J2_SE_1cur = ++J2_SE_1cur;
          J2_PEcur = ++J2_PEcur;
          if (J2_PEcur == J2_PE)
          {
            J2cur = ++J2cur;
            J2_PEcur = 0;
            digitalWrite(J2stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J2dir == 0) {
              J2StepM == --J2StepM;
            }
            else {
              J2StepM == ++J2StepM;
            }
          }
        }
        else
        {
          J2_SE_1cur = 0;
        }
      }
      else
      {
        J2_SE_2cur = 0;
      }
    }

    /////// J3 ////////////////////////////////
    ///find pulse every
    if (J3cur < J3step)
    {
      J3_PE = (HighStep / J3step);
      ///find left over 1
      J3_LO_1 = (HighStep - (J3step * J3_PE));
      ///find skip 1
      if (J3_LO_1 > 0)
      {
        J3_SE_1 = (HighStep / J3_LO_1);
      }
      else
      {
        J3_SE_1 = 0;
      }
      ///find left over 2
      if (J3_SE_1 > 0)
      {
        J3_LO_2 = HighStep - ((J3step * J3_PE) + ((J3step * J3_PE) / J3_SE_1));
      }
      else
      {
        J3_LO_2 = 0;
      }
      ///find skip 2
      if (J3_LO_2 > 0)
      {
        J3_SE_2 = (HighStep / J3_LO_2);
      }
      else
      {
        J3_SE_2 = 0;
      }
      /////////  J3  ///////////////
      if (J3_SE_2 == 0)
      {
        J3_SE_2cur = (J3_SE_2 + 1);
      }
      if (J3_SE_2cur != J3_SE_2)
      {
        J3_SE_2cur = ++J3_SE_2cur;
        if (J3_SE_1 == 0)
        {
          J3_SE_1cur = (J3_SE_1 + 1);
        }
        if (J3_SE_1cur != J3_SE_1)
        {
          J3_SE_1cur = ++J3_SE_1cur;
          J3_PEcur = ++J3_PEcur;
          if (J3_PEcur == J3_PE)
          {
            J3cur = ++J3cur;
            J3_PEcur = 0;
            digitalWrite(J3stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J3dir == 0) {
              J3StepM == --J3StepM;
            }
            else {
              J3StepM == ++J3StepM;
            }
          }
        }
        else
        {
          J3_SE_1cur = 0;
        }
      }
      else
      {
        J3_SE_2cur = 0;
      }
    }


    /////// J4 ////////////////////////////////
    ///find pulse every
    if (J4cur < J4step)
    {
      J4_PE = (HighStep / J4step);
      ///find left over 1
      J4_LO_1 = (HighStep - (J4step * J4_PE));
      ///find skip 1
      if (J4_LO_1 > 0)
      {
        J4_SE_1 = (HighStep / J4_LO_1);
      }
      else
      {
        J4_SE_1 = 0;
      }
      ///find left over 2
      if (J4_SE_1 > 0)
      {
        J4_LO_2 = HighStep - ((J4step * J4_PE) + ((J4step * J4_PE) / J4_SE_1));
      }
      else
      {
        J4_LO_2 = 0;
      }
      ///find skip 2
      if (J4_LO_2 > 0)
      {
        J4_SE_2 = (HighStep / J4_LO_2);
      }
      else
      {
        J4_SE_2 = 0;
      }
      /////////  J4  ///////////////
      if (J4_SE_2 == 0)
      {
        J4_SE_2cur = (J4_SE_2 + 1);
      }
      if (J4_SE_2cur != J4_SE_2)
      {
        J4_SE_2cur = ++J4_SE_2cur;
        if (J4_SE_1 == 0)
        {
          J4_SE_1cur = (J4_SE_1 + 1);
        }
        if (J4_SE_1cur != J4_SE_1)
        {
          J4_SE_1cur = ++J4_SE_1cur;
          J4_PEcur = ++J4_PEcur;
          if (J4_PEcur == J4_PE)
          {
            J4cur = ++J4cur;
            J4_PEcur = 0;
            digitalWrite(J4stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J4dir == 0) {
              J4StepM == --J4StepM;
            }
            else {
              J4StepM == ++J4StepM;
            }
          }
        }
        else
        {
          J4_SE_1cur = 0;
        }
      }
      else
      {
        J4_SE_2cur = 0;
      }
    }


    /////// J5 ////////////////////////////////
    ///find pulse every
    if (J5cur < J5step)
    {
      J5_PE = (HighStep / J5step);
      ///find left over 1
      J5_LO_1 = (HighStep - (J5step * J5_PE));
      ///find skip 1
      if (J5_LO_1 > 0)
      {
        J5_SE_1 = (HighStep / J5_LO_1);
      }
      else
      {
        J5_SE_1 = 0;
      }
      ///find left over 2
      if (J5_SE_1 > 0)
      {
        J5_LO_2 = HighStep - ((J5step * J5_PE) + ((J5step * J5_PE) / J5_SE_1));
      }
      else
      {
        J5_LO_2 = 0;
      }
      ///find skip 2
      if (J5_LO_2 > 0)
      {
        J5_SE_2 = (HighStep / J5_LO_2);
      }
      else
      {
        J5_SE_2 = 0;
      }
      /////////  J5  ///////////////
      if (J5_SE_2 == 0)
      {
        J5_SE_2cur = (J5_SE_2 + 1);
      }
      if (J5_SE_2cur != J5_SE_2)
      {
        J5_SE_2cur = ++J5_SE_2cur;
        if (J5_SE_1 == 0)
        {
          J5_SE_1cur = (J5_SE_1 + 1);
        }
        if (J5_SE_1cur != J5_SE_1)
        {
          J5_SE_1cur = ++J5_SE_1cur;
          J5_PEcur = ++J5_PEcur;
          if (J5_PEcur == J5_PE)
          {
            J5cur = ++J5cur;
            J5_PEcur = 0;
            digitalWrite(J5stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J5dir == 0) {
              J5StepM == --J5StepM;
            }
            else {
              J5StepM == ++J5StepM;
            }
          }
        }
        else
        {
          J5_SE_1cur = 0;
        }
      }
      else
      {
        J5_SE_2cur = 0;
      }
    }


    /////// J6 ////////////////////////////////
    ///find pulse every
    if (J6cur < J6step)
    {
      J6_PE = (HighStep / J6step);
      ///find left over 1
      J6_LO_1 = (HighStep - (J6step * J6_PE));
      ///find skip 1
      if (J6_LO_1 > 0)
      {
        J6_SE_1 = (HighStep / J6_LO_1);
      }
      else
      {
        J6_SE_1 = 0;
      }
      ///find left over 2
      if (J6_SE_1 > 0)
      {
        J6_LO_2 = HighStep - ((J6step * J6_PE) + ((J6step * J6_PE) / J6_SE_1));
      }
      else
      {
        J6_LO_2 = 0;
      }
      ///find skip 2
      if (J6_LO_2 > 0)
      {
        J6_SE_2 = (HighStep / J6_LO_2);
      }
      else
      {
        J6_SE_2 = 0;
      }
      /////////  J6  ///////////////
      if (J6_SE_2 == 0)
      {
        J6_SE_2cur = (J6_SE_2 + 1);
      }
      if (J6_SE_2cur != J6_SE_2)
      {
        J6_SE_2cur = ++J6_SE_2cur;
        if (J6_SE_1 == 0)
        {
          J6_SE_1cur = (J6_SE_1 + 1);
        }
        if (J6_SE_1cur != J6_SE_1)
        {
          J6_SE_1cur = ++J6_SE_1cur;
          J6_PEcur = ++J6_PEcur;
          if (J6_PEcur == J6_PE)
          {
            J6cur = ++J6cur;
            J6_PEcur = 0;
            digitalWrite(J6stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J6dir == 0) {
              J6StepM == --J6StepM;
            }
            else {
              J6StepM == ++J6StepM;
            }
          }
        }
        else
        {
          J6_SE_1cur = 0;
        }
      }
      else
      {
        J6_SE_2cur = 0;
      }
    }


    /////// J7 ////////////////////////////////
    ///find pulse every
    if (J7cur < J7step)
    {
      J7_PE = (HighStep / J7step);
      ///find left over 1
      J7_LO_1 = (HighStep - (J7step * J7_PE));
      ///find skip 1
      if (J7_LO_1 > 0)
      {
        J7_SE_1 = (HighStep / J7_LO_1);
      }
      else
      {
        J7_SE_1 = 0;
      }
      ///find left over 2
      if (J7_SE_1 > 0)
      {
        J7_LO_2 = HighStep - ((J7step * J7_PE) + ((J7step * J7_PE) / J7_SE_1));
      }
      else
      {
        J7_LO_2 = 0;
      }
      ///find skip 2
      if (J7_LO_2 > 0)
      {
        J7_SE_2 = (HighStep / J7_LO_2);
      }
      else
      {
        J7_SE_2 = 0;
      }
      /////////  J7  ///////////////
      if (J7_SE_2 == 0)
      {
        J7_SE_2cur = (J7_SE_2 + 1);
      }
      if (J7_SE_2cur != J7_SE_2)
      {
        J7_SE_2cur = ++J7_SE_2cur;
        if (J7_SE_1 == 0)
        {
          J7_SE_1cur = (J7_SE_1 + 1);
        }
        if (J7_SE_1cur != J7_SE_1)
        {
          J7_SE_1cur = ++J7_SE_1cur;
          J7_PEcur = ++J7_PEcur;
          if (J7_PEcur == J7_PE)
          {
            J7cur = ++J7cur;
            J7_PEcur = 0;
            digitalWrite(J7stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J7dir == 0) {
              J7StepM == --J7StepM;
            }
            else {
              J7StepM == ++J7StepM;
            }
          }
        }
        else
        {
          J7_SE_1cur = 0;
        }
      }
      else
      {
        J7_SE_2cur = 0;
      }
    }




    /////// J8 ////////////////////////////////
    ///find pulse every
    if (J8cur < J8step)
    {
      J8_PE = (HighStep / J8step);
      ///find left over 1
      J8_LO_1 = (HighStep - (J8step * J8_PE));
      ///find skip 1
      if (J8_LO_1 > 0)
      {
        J8_SE_1 = (HighStep / J8_LO_1);
      }
      else
      {
        J8_SE_1 = 0;
      }
      ///find left over 2
      if (J8_SE_1 > 0)
      {
        J8_LO_2 = HighStep - ((J8step * J8_PE) + ((J8step * J8_PE) / J8_SE_1));
      }
      else
      {
        J8_LO_2 = 0;
      }
      ///find skip 2
      if (J8_LO_2 > 0)
      {
        J8_SE_2 = (HighStep / J8_LO_2);
      }
      else
      {
        J8_SE_2 = 0;
      }
      /////////  J8  ///////////////
      if (J8_SE_2 == 0)
      {
        J8_SE_2cur = (J8_SE_2 + 1);
      }
      if (J8_SE_2cur != J8_SE_2)
      {
        J8_SE_2cur = ++J8_SE_2cur;
        if (J8_SE_1 == 0)
        {
          J8_SE_1cur = (J8_SE_1 + 1);
        }
        if (J8_SE_1cur != J8_SE_1)
        {
          J8_SE_1cur = ++J8_SE_1cur;
          J8_PEcur = ++J8_PEcur;
          if (J8_PEcur == J8_PE)
          {
            J8cur = ++J8cur;
            J8_PEcur = 0;
            digitalWrite(J8stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J8dir == 0) {
              J8StepM == --J8StepM;
            }
            else {
              J8StepM == ++J8StepM;
            }
          }
        }
        else
        {
          J8_SE_1cur = 0;
        }
      }
      else
      {
        J8_SE_2cur = 0;
      }
    }


    /////// J9 ////////////////////////////////
    ///find pulse every
    if (J9cur < J9step)
    {
      J9_PE = (HighStep / J9step);
      ///find left over 1
      J9_LO_1 = (HighStep - (J9step * J9_PE));
      ///find skip 1
      if (J9_LO_1 > 0)
      {
        J9_SE_1 = (HighStep / J9_LO_1);
      }
      else
      {
        J9_SE_1 = 0;
      }
      ///find left over 2
      if (J9_SE_1 > 0)
      {
        J9_LO_2 = HighStep - ((J9step * J9_PE) + ((J9step * J9_PE) / J9_SE_1));
      }
      else
      {
        J9_LO_2 = 0;
      }
      ///find skip 2
      if (J9_LO_2 > 0)
      {
        J9_SE_2 = (HighStep / J9_LO_2);
      }
      else
      {
        J9_SE_2 = 0;
      }
      /////////  J9  ///////////////
      if (J9_SE_2 == 0)
      {
        J9_SE_2cur = (J9_SE_2 + 1);
      }
      if (J9_SE_2cur != J9_SE_2)
      {
        J9_SE_2cur = ++J9_SE_2cur;
        if (J9_SE_1 == 0)
        {
          J9_SE_1cur = (J9_SE_1 + 1);
        }
        if (J9_SE_1cur != J9_SE_1)
        {
          J9_SE_1cur = ++J9_SE_1cur;
          J9_PEcur = ++J9_PEcur;
          if (J9_PEcur == J9_PE)
          {
            J9cur = ++J9cur;
            J9_PEcur = 0;
            digitalWrite(J9stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J9dir == 0) {
              J9StepM == --J9StepM;
            }
            else {
              J9StepM == ++J9StepM;
            }
          }
        }
        else
        {
          J9_SE_1cur = 0;
        }
      }
      else
      {
        J9_SE_2cur = 0;
      }
    }


    // inc cur step
    highStepCur = ++highStepCur;
    digitalWrite(J1stepPin, HIGH);
    digitalWrite(J2stepPin, HIGH);
    digitalWrite(J3stepPin, HIGH);
    digitalWrite(J4stepPin, HIGH);
    digitalWrite(J5stepPin, HIGH);
    digitalWrite(J6stepPin, HIGH);
    digitalWrite(J7stepPin, HIGH);
    digitalWrite(J8stepPin, HIGH);
    digitalWrite(J9stepPin, HIGH);
    delayMicroseconds(curDelay - disDelayCur);


  }
  //set rounding speed to last move speed
  rndSpeed = curDelay;
}





/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//DRIVE MOTORS L
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void driveMotorsL(int J1step, int J2step, int J3step, int J4step, int J5step, int J6step, int J7step, int J8step, int J9step, int J1dir, int J2dir, int J3dir, int J4dir, int J5dir, int J6dir, int J7dir, int J8dir, int J9dir, float curDelay) {

  //FIND HIGHEST STEP
  int HighStep = J1step;
  if (J2step > HighStep)
  {
    HighStep = J2step;
  }
  if (J3step > HighStep)
  {
    HighStep = J3step;
  }
  if (J4step > HighStep)
  {
    HighStep = J4step;
  }
  if (J5step > HighStep)
  {
    HighStep = J5step;
  }
  if (J6step > HighStep)
  {
    HighStep = J6step;
  }
  if (J7step > HighStep)
  {
    HighStep = J7step;
  }
  if (J8step > HighStep)
  {
    HighStep = J8step;
  }
  if (J9step > HighStep)
  {
    HighStep = J9step;
  }

  //FIND ACTIVE JOINTS
  int J1active = 0;
  int J2active = 0;
  int J3active = 0;
  int J4active = 0;
  int J5active = 0;
  int J6active = 0;
  int J7active = 0;
  int J8active = 0;
  int J9active = 0;
  int Jactive = 0;

  if (J1step >= 1)
  {
    J1active = 1;
  }
  if (J2step >= 1)
  {
    J2active = 1;
  }
  if (J3step >= 1)
  {
    J3active = 1;
  }
  if (J4step >= 1)
  {
    J4active = 1;
  }
  if (J5step >= 1)
  {
    J5active = 1;
  }
  if (J6step >= 1)
  {
    J6active = 1;
  }
  if (J7step >= 1)
  {
    J7active = 1;
  }
  if (J8step >= 1)
  {
    J8active = 1;
  }
  if (J9step >= 1)
  {
    J9active = 1;
  }
  Jactive = (J1active + J2active + J3active + J4active + J5active + J6active + J7active + J8active + J9active);

  int J1_PE = 0;
  int J2_PE = 0;
  int J3_PE = 0;
  int J4_PE = 0;
  int J5_PE = 0;
  int J6_PE = 0;
  int J7_PE = 0;
  int J8_PE = 0;
  int J9_PE = 0;

  int J1_SE_1 = 0;
  int J2_SE_1 = 0;
  int J3_SE_1 = 0;
  int J4_SE_1 = 0;
  int J5_SE_1 = 0;
  int J6_SE_1 = 0;
  int J7_SE_1 = 0;
  int J8_SE_1 = 0;
  int J9_SE_1 = 0;

  int J1_SE_2 = 0;
  int J2_SE_2 = 0;
  int J3_SE_2 = 0;
  int J4_SE_2 = 0;
  int J5_SE_2 = 0;
  int J6_SE_2 = 0;
  int J7_SE_2 = 0;
  int J8_SE_2 = 0;
  int J9_SE_2 = 0;

  int J1_LO_1 = 0;
  int J2_LO_1 = 0;
  int J3_LO_1 = 0;
  int J4_LO_1 = 0;
  int J5_LO_1 = 0;
  int J6_LO_1 = 0;
  int J7_LO_1 = 0;
  int J8_LO_1 = 0;
  int J9_LO_1 = 0;

  int J1_LO_2 = 0;
  int J2_LO_2 = 0;
  int J3_LO_2 = 0;
  int J4_LO_2 = 0;
  int J5_LO_2 = 0;
  int J6_LO_2 = 0;
  int J7_LO_2 = 0;
  int J8_LO_2 = 0;
  int J9_LO_2 = 0;

  //reset
  int J1cur = 0;
  int J2cur = 0;
  int J3cur = 0;
  int J4cur = 0;
  int J5cur = 0;
  int J6cur = 0;
  int J7cur = 0;
  int J8cur = 0;
  int J9cur = 0;

  int J1_PEcur = 0;
  int J2_PEcur = 0;
  int J3_PEcur = 0;
  int J4_PEcur = 0;
  int J5_PEcur = 0;
  int J6_PEcur = 0;
  int J7_PEcur = 0;
  int J8_PEcur = 0;
  int J9_PEcur = 0;

  int J1_SE_1cur = 0;
  int J2_SE_1cur = 0;
  int J3_SE_1cur = 0;
  int J4_SE_1cur = 0;
  int J5_SE_1cur = 0;
  int J6_SE_1cur = 0;
  int J7_SE_1cur = 0;
  int J8_SE_1cur = 0;
  int J9_SE_1cur = 0;

  int J1_SE_2cur = 0;
  int J2_SE_2cur = 0;
  int J3_SE_2cur = 0;
  int J4_SE_2cur = 0;
  int J5_SE_2cur = 0;
  int J6_SE_2cur = 0;
  int J7_SE_2cur = 0;
  int J8_SE_2cur = 0;
  int J9_SE_2cur = 0;

  int highStepCur = 0;

  float speedSP;
  float moveDist;

  //process lookahead
  if (splineTrue == true) {
    processSerial();
  }

  //SET DIRECTIONS

  /// J1 ///
  if (J1dir == 1 && J1MotDir == 1) {
    digitalWrite(J1dirPin, HIGH);
  }
  if (J1dir == 1 && J1MotDir == 0) {
    digitalWrite(J1dirPin, LOW);
  }
  if (J1dir == 0 && J1MotDir == 1) {
    digitalWrite(J1dirPin, LOW);
  }
  if (J1dir == 0 && J1MotDir == 0) {
    digitalWrite(J1dirPin, HIGH);
  }
  /// J2 ///
  if (J2dir == 1 && J2MotDir == 1) {
    digitalWrite(J2dirPin, HIGH);
  }
  if (J2dir == 1 && J2MotDir == 0) {
    digitalWrite(J2dirPin, LOW);
  }
  if (J2dir == 0 && J2MotDir == 1) {
    digitalWrite(J2dirPin, LOW);
  }
  if (J2dir == 0 && J2MotDir == 0) {
    digitalWrite(J2dirPin, HIGH);
  }
  /// J3 ///
  if (J3dir == 1 && J3MotDir == 1) {
    digitalWrite(J3dirPin, HIGH);
  }
  if (J3dir == 1 && J3MotDir == 0) {
    digitalWrite(J3dirPin, LOW);
  }
  if (J3dir == 0 && J3MotDir == 1) {
    digitalWrite(J3dirPin, LOW);
  }
  if (J3dir == 0 && J3MotDir == 0) {
    digitalWrite(J3dirPin, HIGH);
  }
  /// J4 ///
  if (J4dir == 1 && J4MotDir == 1) {
    digitalWrite(J4dirPin, HIGH);
  }
  if (J4dir == 1 && J4MotDir == 0) {
    digitalWrite(J4dirPin, LOW);
  }
  if (J4dir == 0 && J4MotDir == 1) {
    digitalWrite(J4dirPin, LOW);
  }
  if (J4dir == 0 && J4MotDir == 0) {
    digitalWrite(J4dirPin, HIGH);
  }
  /// J5 ///
  if (J5dir == 1 && J5MotDir == 1) {
    digitalWrite(J5dirPin, HIGH);
  }
  if (J5dir == 1 && J5MotDir == 0) {
    digitalWrite(J5dirPin, LOW);
  }
  if (J5dir == 0 && J5MotDir == 1) {
    digitalWrite(J5dirPin, LOW);
  }
  if (J5dir == 0 && J5MotDir == 0) {
    digitalWrite(J5dirPin, HIGH);
  }
  /// J6 ///
  if (J6dir == 1 && J6MotDir == 1) {
    digitalWrite(J6dirPin, HIGH);
  }
  if (J6dir == 1 && J6MotDir == 0) {
    digitalWrite(J6dirPin, LOW);
  }
  if (J6dir == 0 && J6MotDir == 1) {
    digitalWrite(J6dirPin, LOW);
  }
  if (J6dir == 0 && J6MotDir == 0) {
    digitalWrite(J6dirPin, HIGH);
  }
  /// J7 ///
  if (J7dir == 1 && J7MotDir == 1) {
    digitalWrite(J7dirPin, HIGH);
  }
  if (J7dir == 1 && J7MotDir == 0) {
    digitalWrite(J7dirPin, LOW);
  }
  if (J7dir == 0 && J7MotDir == 1) {
    digitalWrite(J7dirPin, LOW);
  }
  if (J7dir == 0 && J7MotDir == 0) {
    digitalWrite(J7dirPin, HIGH);
  }
  /// J8 ///
  if (J8dir == 1 && J8MotDir == 1) {
    digitalWrite(J8dirPin, HIGH);
  }
  if (J8dir == 1 && J8MotDir == 0) {
    digitalWrite(J8dirPin, LOW);
  }
  if (J8dir == 0 && J8MotDir == 1) {
    digitalWrite(J8dirPin, LOW);
  }
  if (J8dir == 0 && J8MotDir == 0) {
    digitalWrite(J8dirPin, HIGH);
  }
  /// J9 ///
  if (J9dir == 1 && J9MotDir == 1) {
    digitalWrite(J9dirPin, HIGH);
  }
  if (J9dir == 1 && J9MotDir == 0) {
    digitalWrite(J9dirPin, LOW);
  }
  if (J9dir == 0 && J9MotDir == 1) {
    digitalWrite(J9dirPin, LOW);
  }
  if (J9dir == 0 && J9MotDir == 0) {
    digitalWrite(J9dirPin, HIGH);
  }

  delayMicroseconds(15);


  J1collisionTrue = 0;
  J2collisionTrue = 0;
  J3collisionTrue = 0;
  J4collisionTrue = 0;
  J5collisionTrue = 0;
  J6collisionTrue = 0;

  ///// DRIVE MOTORS /////
  while (J1cur < J1step || J2cur < J2step || J3cur < J3step || J4cur < J4step || J5cur < J5step || J6cur < J6step || J7cur < J7step || J8cur < J8step || J9cur < J9step)
  {

    float distDelay = 60;
    float disDelayCur = 0;

    //process lookahead
    if (splineTrue == true) {
      processSerial();
    }

    /////// J1 ////////////////////////////////
    ///find pulse every
    if (J1cur < J1step)
    {
      J1_PE = (HighStep / J1step);
      ///find left over 1
      J1_LO_1 = (HighStep - (J1step * J1_PE));
      ///find skip 1
      if (J1_LO_1 > 0)
      {
        J1_SE_1 = (HighStep / J1_LO_1);
      }
      else
      {
        J1_SE_1 = 0;
      }
      ///find left over 2
      if (J1_SE_1 > 0)
      {
        J1_LO_2 = HighStep - ((J1step * J1_PE) + ((J1step * J1_PE) / J1_SE_1));
      }
      else
      {
        J1_LO_2 = 0;
      }
      ///find skip 2
      if (J1_LO_2 > 0)
      {
        J1_SE_2 = (HighStep / J1_LO_2);
      }
      else
      {
        J1_SE_2 = 0;
      }
      /////////  J1  ///////////////
      if (J1_SE_2 == 0)
      {
        J1_SE_2cur = (J1_SE_2 + 1);
      }
      if (J1_SE_2cur != J1_SE_2)
      {
        J1_SE_2cur = ++J1_SE_2cur;
        if (J1_SE_1 == 0)
        {
          J1_SE_1cur = (J1_SE_1 + 1);
        }
        if (J1_SE_1cur != J1_SE_1)
        {
          J1_SE_1cur = ++J1_SE_1cur;
          J1_PEcur = ++J1_PEcur;
          if (J1_PEcur == J1_PE)
          {
            J1cur = ++J1cur;
            J1_PEcur = 0;
            digitalWrite(J1stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J1dir == 0) {
              J1StepM == --J1StepM;
            }
            else {
              J1StepM == ++J1StepM;
            }
          }
        }
        else
        {
          J1_SE_1cur = 0;
        }
      }
      else
      {
        J1_SE_2cur = 0;
      }
    }


    /////// J2 ////////////////////////////////

    ///find pulse every
    if (J2cur < J2step)
    {
      J2_PE = (HighStep / J2step);
      ///find left over 1
      J2_LO_1 = (HighStep - (J2step * J2_PE));
      ///find skip 1
      if (J2_LO_1 > 0)
      {
        J2_SE_1 = (HighStep / J2_LO_1);
      }
      else
      {
        J2_SE_1 = 0;
      }
      ///find left over 2
      if (J2_SE_1 > 0)
      {
        J2_LO_2 = HighStep - ((J2step * J2_PE) + ((J2step * J2_PE) / J2_SE_1));
      }
      else
      {
        J2_LO_2 = 0;
      }
      ///find skip 2
      if (J2_LO_2 > 0)
      {
        J2_SE_2 = (HighStep / J2_LO_2);
      }
      else
      {
        J2_SE_2 = 0;
      }
      /////////  J2  ///////////////
      if (J2_SE_2 == 0)
      {
        J2_SE_2cur = (J2_SE_2 + 1);
      }
      if (J2_SE_2cur != J2_SE_2)
      {
        J2_SE_2cur = ++J2_SE_2cur;
        if (J2_SE_1 == 0)
        {
          J2_SE_1cur = (J2_SE_1 + 1);
        }
        if (J2_SE_1cur != J2_SE_1)
        {
          J2_SE_1cur = ++J2_SE_1cur;
          J2_PEcur = ++J2_PEcur;
          if (J2_PEcur == J2_PE)
          {
            J2cur = ++J2cur;
            J2_PEcur = 0;
            digitalWrite(J2stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J2dir == 0) {
              J2StepM == --J2StepM;
            }
            else {
              J2StepM == ++J2StepM;
            }
          }
        }
        else
        {
          J2_SE_1cur = 0;
        }
      }
      else
      {
        J2_SE_2cur = 0;
      }
    }

    /////// J3 ////////////////////////////////
    ///find pulse every
    if (J3cur < J3step)
    {
      J3_PE = (HighStep / J3step);
      ///find left over 1
      J3_LO_1 = (HighStep - (J3step * J3_PE));
      ///find skip 1
      if (J3_LO_1 > 0)
      {
        J3_SE_1 = (HighStep / J3_LO_1);
      }
      else
      {
        J3_SE_1 = 0;
      }
      ///find left over 2
      if (J3_SE_1 > 0)
      {
        J3_LO_2 = HighStep - ((J3step * J3_PE) + ((J3step * J3_PE) / J3_SE_1));
      }
      else
      {
        J3_LO_2 = 0;
      }
      ///find skip 2
      if (J3_LO_2 > 0)
      {
        J3_SE_2 = (HighStep / J3_LO_2);
      }
      else
      {
        J3_SE_2 = 0;
      }
      /////////  J3  ///////////////
      if (J3_SE_2 == 0)
      {
        J3_SE_2cur = (J3_SE_2 + 1);
      }
      if (J3_SE_2cur != J3_SE_2)
      {
        J3_SE_2cur = ++J3_SE_2cur;
        if (J3_SE_1 == 0)
        {
          J3_SE_1cur = (J3_SE_1 + 1);
        }
        if (J3_SE_1cur != J3_SE_1)
        {
          J3_SE_1cur = ++J3_SE_1cur;
          J3_PEcur = ++J3_PEcur;
          if (J3_PEcur == J3_PE)
          {
            J3cur = ++J3cur;
            J3_PEcur = 0;
            digitalWrite(J3stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J3dir == 0) {
              J3StepM == --J3StepM;
            }
            else {
              J3StepM == ++J3StepM;
            }
          }
        }
        else
        {
          J3_SE_1cur = 0;
        }
      }
      else
      {
        J3_SE_2cur = 0;
      }
    }


    /////// J4 ////////////////////////////////
    ///find pulse every
    if (J4cur < J4step)
    {
      J4_PE = (HighStep / J4step);
      ///find left over 1
      J4_LO_1 = (HighStep - (J4step * J4_PE));
      ///find skip 1
      if (J4_LO_1 > 0)
      {
        J4_SE_1 = (HighStep / J4_LO_1);
      }
      else
      {
        J4_SE_1 = 0;
      }
      ///find left over 2
      if (J4_SE_1 > 0)
      {
        J4_LO_2 = HighStep - ((J4step * J4_PE) + ((J4step * J4_PE) / J4_SE_1));
      }
      else
      {
        J4_LO_2 = 0;
      }
      ///find skip 2
      if (J4_LO_2 > 0)
      {
        J4_SE_2 = (HighStep / J4_LO_2);
      }
      else
      {
        J4_SE_2 = 0;
      }
      /////////  J4  ///////////////
      if (J4_SE_2 == 0)
      {
        J4_SE_2cur = (J4_SE_2 + 1);
      }
      if (J4_SE_2cur != J4_SE_2)
      {
        J4_SE_2cur = ++J4_SE_2cur;
        if (J4_SE_1 == 0)
        {
          J4_SE_1cur = (J4_SE_1 + 1);
        }
        if (J4_SE_1cur != J4_SE_1)
        {
          J4_SE_1cur = ++J4_SE_1cur;
          J4_PEcur = ++J4_PEcur;
          if (J4_PEcur == J4_PE)
          {
            J4cur = ++J4cur;
            J4_PEcur = 0;
            digitalWrite(J4stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J4dir == 0) {
              J4StepM == --J4StepM;
            }
            else {
              J4StepM == ++J4StepM;
            }
          }
        }
        else
        {
          J4_SE_1cur = 0;
        }
      }
      else
      {
        J4_SE_2cur = 0;
      }
    }


    /////// J5 ////////////////////////////////
    ///find pulse every
    if (J5cur < J5step)
    {
      J5_PE = (HighStep / J5step);
      ///find left over 1
      J5_LO_1 = (HighStep - (J5step * J5_PE));
      ///find skip 1
      if (J5_LO_1 > 0)
      {
        J5_SE_1 = (HighStep / J5_LO_1);
      }
      else
      {
        J5_SE_1 = 0;
      }
      ///find left over 2
      if (J5_SE_1 > 0)
      {
        J5_LO_2 = HighStep - ((J5step * J5_PE) + ((J5step * J5_PE) / J5_SE_1));
      }
      else
      {
        J5_LO_2 = 0;
      }
      ///find skip 2
      if (J5_LO_2 > 0)
      {
        J5_SE_2 = (HighStep / J5_LO_2);
      }
      else
      {
        J5_SE_2 = 0;
      }
      /////////  J5  ///////////////
      if (J5_SE_2 == 0)
      {
        J5_SE_2cur = (J5_SE_2 + 1);
      }
      if (J5_SE_2cur != J5_SE_2)
      {
        J5_SE_2cur = ++J5_SE_2cur;
        if (J5_SE_1 == 0)
        {
          J5_SE_1cur = (J5_SE_1 + 1);
        }
        if (J5_SE_1cur != J5_SE_1)
        {
          J5_SE_1cur = ++J5_SE_1cur;
          J5_PEcur = ++J5_PEcur;
          if (J5_PEcur == J5_PE)
          {
            J5cur = ++J5cur;
            J5_PEcur = 0;
            digitalWrite(J5stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J5dir == 0) {
              J5StepM == --J5StepM;
            }
            else {
              J5StepM == ++J5StepM;
            }
          }
        }
        else
        {
          J5_SE_1cur = 0;
        }
      }
      else
      {
        J5_SE_2cur = 0;
      }
    }


    /////// J6 ////////////////////////////////
    ///find pulse every
    if (J6cur < J6step)
    {
      J6_PE = (HighStep / J6step);
      ///find left over 1
      J6_LO_1 = (HighStep - (J6step * J6_PE));
      ///find skip 1
      if (J6_LO_1 > 0)
      {
        J6_SE_1 = (HighStep / J6_LO_1);
      }
      else
      {
        J6_SE_1 = 0;
      }
      ///find left over 2
      if (J6_SE_1 > 0)
      {
        J6_LO_2 = HighStep - ((J6step * J6_PE) + ((J6step * J6_PE) / J6_SE_1));
      }
      else
      {
        J6_LO_2 = 0;
      }
      ///find skip 2
      if (J6_LO_2 > 0)
      {
        J6_SE_2 = (HighStep / J6_LO_2);
      }
      else
      {
        J6_SE_2 = 0;
      }
      /////////  J6  ///////////////
      if (J6_SE_2 == 0)
      {
        J6_SE_2cur = (J6_SE_2 + 1);
      }
      if (J6_SE_2cur != J6_SE_2)
      {
        J6_SE_2cur = ++J6_SE_2cur;
        if (J6_SE_1 == 0)
        {
          J6_SE_1cur = (J6_SE_1 + 1);
        }
        if (J6_SE_1cur != J6_SE_1)
        {
          J6_SE_1cur = ++J6_SE_1cur;
          J6_PEcur = ++J6_PEcur;
          if (J6_PEcur == J6_PE)
          {
            J6cur = ++J6cur;
            J6_PEcur = 0;
            digitalWrite(J6stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J6dir == 0) {
              J6StepM == --J6StepM;
            }
            else {
              J6StepM == ++J6StepM;
            }
          }
        }
        else
        {
          J6_SE_1cur = 0;
        }
      }
      else
      {
        J6_SE_2cur = 0;
      }
    }


    /////// J7 ////////////////////////////////
    ///find pulse every
    if (J7cur < J7step)
    {
      J7_PE = (HighStep / J7step);
      ///find left over 1
      J7_LO_1 = (HighStep - (J7step * J7_PE));
      ///find skip 1
      if (J7_LO_1 > 0)
      {
        J7_SE_1 = (HighStep / J7_LO_1);
      }
      else
      {
        J7_SE_1 = 0;
      }
      ///find left over 2
      if (J7_SE_1 > 0)
      {
        J7_LO_2 = HighStep - ((J7step * J7_PE) + ((J7step * J7_PE) / J7_SE_1));
      }
      else
      {
        J7_LO_2 = 0;
      }
      ///find skip 2
      if (J7_LO_2 > 0)
      {
        J7_SE_2 = (HighStep / J7_LO_2);
      }
      else
      {
        J7_SE_2 = 0;
      }
      /////////  J7  ///////////////
      if (J7_SE_2 == 0)
      {
        J7_SE_2cur = (J7_SE_2 + 1);
      }
      if (J7_SE_2cur != J7_SE_2)
      {
        J7_SE_2cur = ++J7_SE_2cur;
        if (J7_SE_1 == 0)
        {
          J7_SE_1cur = (J7_SE_1 + 1);
        }
        if (J7_SE_1cur != J7_SE_1)
        {
          J7_SE_1cur = ++J7_SE_1cur;
          J7_PEcur = ++J7_PEcur;
          if (J7_PEcur == J7_PE)
          {
            J7cur = ++J7cur;
            J7_PEcur = 0;
            digitalWrite(J7stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J7dir == 0) {
              J7StepM == --J7StepM;
            }
            else {
              J7StepM == ++J7StepM;
            }
          }
        }
        else
        {
          J7_SE_1cur = 0;
        }
      }
      else
      {
        J7_SE_2cur = 0;
      }
    }


    /////// J8 ////////////////////////////////
    ///find pulse every
    if (J8cur < J8step)
    {
      J8_PE = (HighStep / J8step);
      ///find left over 1
      J8_LO_1 = (HighStep - (J8step * J8_PE));
      ///find skip 1
      if (J8_LO_1 > 0)
      {
        J8_SE_1 = (HighStep / J8_LO_1);
      }
      else
      {
        J8_SE_1 = 0;
      }
      ///find left over 2
      if (J8_SE_1 > 0)
      {
        J8_LO_2 = HighStep - ((J8step * J8_PE) + ((J8step * J8_PE) / J8_SE_1));
      }
      else
      {
        J8_LO_2 = 0;
      }
      ///find skip 2
      if (J8_LO_2 > 0)
      {
        J8_SE_2 = (HighStep / J8_LO_2);
      }
      else
      {
        J8_SE_2 = 0;
      }
      /////////  J8  ///////////////
      if (J8_SE_2 == 0)
      {
        J8_SE_2cur = (J8_SE_2 + 1);
      }
      if (J8_SE_2cur != J8_SE_2)
      {
        J8_SE_2cur = ++J8_SE_2cur;
        if (J8_SE_1 == 0)
        {
          J8_SE_1cur = (J8_SE_1 + 1);
        }
        if (J8_SE_1cur != J8_SE_1)
        {
          J8_SE_1cur = ++J8_SE_1cur;
          J8_PEcur = ++J8_PEcur;
          if (J8_PEcur == J8_PE)
          {
            J8cur = ++J8cur;
            J8_PEcur = 0;
            digitalWrite(J8stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J8dir == 0) {
              J8StepM == --J8StepM;
            }
            else {
              J8StepM == ++J8StepM;
            }
          }
        }
        else
        {
          J8_SE_1cur = 0;
        }
      }
      else
      {
        J8_SE_2cur = 0;
      }
    }


    /////// J9 ////////////////////////////////
    ///find pulse every
    if (J9cur < J9step)
    {
      J9_PE = (HighStep / J9step);
      ///find left over 1
      J9_LO_1 = (HighStep - (J9step * J9_PE));
      ///find skip 1
      if (J9_LO_1 > 0)
      {
        J9_SE_1 = (HighStep / J9_LO_1);
      }
      else
      {
        J9_SE_1 = 0;
      }
      ///find left over 2
      if (J9_SE_1 > 0)
      {
        J9_LO_2 = HighStep - ((J9step * J9_PE) + ((J9step * J9_PE) / J9_SE_1));
      }
      else
      {
        J9_LO_2 = 0;
      }
      ///find skip 2
      if (J9_LO_2 > 0)
      {
        J9_SE_2 = (HighStep / J9_LO_2);
      }
      else
      {
        J9_SE_2 = 0;
      }
      /////////  J9  ///////////////
      if (J9_SE_2 == 0)
      {
        J9_SE_2cur = (J9_SE_2 + 1);
      }
      if (J9_SE_2cur != J9_SE_2)
      {
        J9_SE_2cur = ++J9_SE_2cur;
        if (J9_SE_1 == 0)
        {
          J9_SE_1cur = (J9_SE_1 + 1);
        }
        if (J9_SE_1cur != J9_SE_1)
        {
          J9_SE_1cur = ++J9_SE_1cur;
          J9_PEcur = ++J9_PEcur;
          if (J9_PEcur == J9_PE)
          {
            J9cur = ++J9cur;
            J9_PEcur = 0;
            digitalWrite(J9stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J9dir == 0) {
              J9StepM == --J9StepM;
            }
            else {
              J9StepM == ++J9StepM;
            }
          }
        }
        else
        {
          J9_SE_1cur = 0;
        }
      }
      else
      {
        J9_SE_2cur = 0;
      }
    }




    // inc cur step
    highStepCur = ++highStepCur;
    digitalWrite(J1stepPin, HIGH);
    digitalWrite(J2stepPin, HIGH);
    digitalWrite(J3stepPin, HIGH);
    digitalWrite(J4stepPin, HIGH);
    digitalWrite(J5stepPin, HIGH);
    digitalWrite(J6stepPin, HIGH);
    digitalWrite(J7stepPin, HIGH);
    digitalWrite(J8stepPin, HIGH);
    digitalWrite(J9stepPin, HIGH);
    delayMicroseconds(curDelay - disDelayCur);



  }
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//READ DATA
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void processSerial() {
  if (Serial.available() > 0 and cmdBuffer3 == "") {
    char recieved = Serial.read();
    recData += recieved;
    // Process message when new line character is recieved
    if (recieved == '\n') {
      //place data in last position
      cmdBuffer3 = recData;
      //determine if move command
      recData.trim();
      String procCMDtype = recData.substring(0, 2);
      if (procCMDtype == "SS") {
        splineTrue = false;
        splineEndReceived = true;
      }
      if (splineTrue == true) {
        if (moveSequence == "") {
          moveSequence = "firsMoveActive";
        }
        //close serial so next command can be read in
        if (Alarm == "0") {
          sendRobotPosSpline();
        }
        else {
          Serial.println(Alarm);
          Alarm = "0";
        }
      }

      recData = ""; // Clear recieved buffer

      shiftCMDarray();


      //if second position is empty and first move command read in process second move ahead of time
      if (procCMDtype == "MS" and moveSequence == "firsMoveActive" and cmdBuffer2 == "" and cmdBuffer1 != "" and splineTrue == true) {
        moveSequence = "secondMoveProcessed";
        while (cmdBuffer2 == "") {
          if (Serial.available() > 0) {
            char recieved = Serial.read();
            recData += recieved;
            if (recieved == '\n') {
              cmdBuffer2 = recData;
              recData.trim();
              procCMDtype = recData.substring(0, 2);
              if (procCMDtype == "MS") {
                //close serial so next command can be read in
                delay(5);
                if (Alarm == "0") {
                  sendRobotPosSpline();
                }
                else {
                  Serial.println(Alarm);
                  Alarm = "0";
                }
              }
              recData = ""; // Clear recieved buffer
            }
          }
        }
      }
    }
  }
}


void shiftCMDarray() {
  if (cmdBuffer1 == "") {
    //shift 2 to 1
    cmdBuffer1 = cmdBuffer2;
    cmdBuffer2 = "";
  }
  if (cmdBuffer2 == "") {
    //shift 3 to 2
    cmdBuffer2 = cmdBuffer3;
    cmdBuffer3 = "";
  }
  if (cmdBuffer1 == "") {
    //shift 2 to 1
    cmdBuffer1 = cmdBuffer2;
    cmdBuffer2 = "";
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MAIN
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  // run once:
  Serial.begin(9600);


  pinMode(J1stepPin, OUTPUT);
  pinMode(J1dirPin, OUTPUT);
  pinMode(J2stepPin, OUTPUT);
  pinMode(J2dirPin, OUTPUT);
  pinMode(J3stepPin, OUTPUT);
  pinMode(J3dirPin, OUTPUT);
  pinMode(J4stepPin, OUTPUT);
  pinMode(J4dirPin, OUTPUT);
  pinMode(J5stepPin, OUTPUT);
  pinMode(J5dirPin, OUTPUT);
  pinMode(J6stepPin, OUTPUT);
  pinMode(J6dirPin, OUTPUT);
  pinMode(J7stepPin, OUTPUT);
  pinMode(J7dirPin, OUTPUT);
  pinMode(J8stepPin, OUTPUT);
  pinMode(J8dirPin, OUTPUT);
  pinMode(J9stepPin, OUTPUT);
  pinMode(J9dirPin, OUTPUT);

  pinMode(J1calPin, INPUT);
  pinMode(J2calPin, INPUT);
  pinMode(J3calPin, INPUT);
  pinMode(J4calPin, INPUT);
  pinMode(J5calPin, INPUT);
  pinMode(J6calPin, INPUT);
  pinMode(J7calPin, INPUT);
  pinMode(J8calPin, INPUT);
  pinMode(J9calPin, INPUT);


  pinMode(Input39, INPUT_PULLUP);


  pinMode(Output40, OUTPUT);
  pinMode(Output41, OUTPUT);



  digitalWrite(J1stepPin, HIGH);
  digitalWrite(J2stepPin, HIGH);
  digitalWrite(J3stepPin, HIGH);
  digitalWrite(J4stepPin, HIGH);
  digitalWrite(J5stepPin, HIGH);
  digitalWrite(J6stepPin, HIGH);
  digitalWrite(J7stepPin, HIGH);
  digitalWrite(J8stepPin, HIGH);
  digitalWrite(J9stepPin, HIGH);

  //clear command buffer array
  cmdBuffer1 = "";
  cmdBuffer2 = "";
  cmdBuffer3 = "";
  //reset move command flag
  moveSequence = "";
  flag = "";
  rndTrue = false;
  splineTrue = false;
  splineEndReceived = false;
}


void loop() {

  ////////////////////////////////////
  ///////////start loop///////////////

  if (splineEndReceived == false) {
    processSerial();
  }
  //dont start unless at least one command has been read in
  if (cmdBuffer1 != "") {
    //process data
    inData = cmdBuffer1;
    inData.trim();
    String function = inData.substring(0, 2);
    inData = inData.substring(2);
    KinematicError = 0;
    debug = "";

    //-----SPLINE START------------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "SL") {
      splineTrue = true;
      delay(5);
      Serial.print("SL");
      moveSequence = "";
      flag = "";
      rndTrue = false;
      splineEndReceived = false;
    }

    //----- SPLINE STOP  ----------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "SS") {
      delay(5);
      sendRobotPos();
      splineTrue = false;
      splineEndReceived = false;
    }

    //-----COMMAND TO CLOSE---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "CL")
    {
      delay(5);
      Serial.end();
    }

    //-----COMMAND TEST LIMIT SWITCHES---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "TL") {

      String J1calTest = "0";
      String J2calTest = "0";
      String J3calTest = "0";
      String J4calTest = "0";
      String J5calTest = "0";
      String J6calTest = "0";

      if (digitalRead(J1calPin) == HIGH) {
        J1calTest = "1";
      }
      if (digitalRead(J2calPin) == HIGH) {
        J2calTest = "1";
      }
      if (digitalRead(J3calPin) == HIGH) {
        J3calTest = "1";
      }
      if (digitalRead(J4calPin) == HIGH) {
        J4calTest = "1";
      }
      if (digitalRead(J5calPin) == HIGH) {
        J5calTest = "1";
      }
      if (digitalRead(J6calPin) == HIGH) {
        J6calTest = "1";
      }
      String TestLim = " J1 = " + J1calTest + "   J2 = " + J2calTest + "   J3 = " + J3calTest + "   J4 = " + J4calTest + "   J5 = " + J5calTest + "   J6 = " + J6calTest;
      delay(5);
      Serial.println(TestLim);
    }


    //-----COMMAND SET ENCODERS TO 1000---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "SE")
    {
      J1encPos.write(1000);
      J2encPos.write(1000);
      J3encPos.write(1000);
      J4encPos.write(1000);
      J5encPos.write(1000);
      J6encPos.write(1000);
      delay(5);
      Serial.print("Done");
    }

    //-----COMMAND READ ENCODERS---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "RE")
    {
      J1EncSteps = J1encPos.read();
      J2EncSteps = J2encPos.read();
      J3EncSteps = J3encPos.read();
      J4EncSteps = J4encPos.read();
      J5EncSteps = J5encPos.read();
      J6EncSteps = J6encPos.read();
      String Read = " J1 = " + String(J1EncSteps) + "   J2 = " + String(J2EncSteps) + "   J3 = " + String(J3EncSteps) + "   J4 = " + String(J4EncSteps) + "   J5 = " + String(J5EncSteps) + "   J6 = " + String(J6EncSteps);
      delay(5);
      Serial.println(Read);
    }

    //-----COMMAND REQUEST POSITION---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "RP")
    {
      //close serial so next command can be read in
      delay(5);
      if (Alarm == "0") {
        sendRobotPos();
      }
      else {
        Serial.println(Alarm);
        Alarm = "0";
      }
    }



    //-----COMMAND HOME POSITION---------------------------------------------------
    //-----------------------------------------------------------------------

    //For debugging
    if (function == "HM")
    {

      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;


      String SpeedType = "p";
      float SpeedVal = 25.0;
      float ACCspd = 10.0;
      float DCCspd = 10.0;
      float ACCramp = 20.0;

      JangleIn[0] = 0.00;
      JangleIn[1] = 0.00;
      JangleIn[2] = 0.00;
      JangleIn[3] = 0.00;
      JangleIn[4] = 0.00;
      JangleIn[5] = 0.00;


      //calc destination motor steps
      int J1futStepM = J1axisLimNeg * J1StepDeg;
      int J2futStepM = J2axisLimNeg * J2StepDeg;
      int J3futStepM = J3axisLimNeg * J3StepDeg;
      int J4futStepM = J4axisLimNeg * J4StepDeg;
      int J5futStepM = J5axisLimNeg * J5StepDeg;
      int J6futStepM = J6axisLimNeg * J6StepDeg;

      //calc delta from current to destination
      int J1stepDif = J1StepM - J1futStepM;
      int J2stepDif = J2StepM - J2futStepM;
      int J3stepDif = J3StepM - J3futStepM;
      int J4stepDif = J4StepM - J4futStepM;
      int J5stepDif = J5StepM - J5futStepM;
      int J6stepDif = J6StepM - J6futStepM;
      int J7stepDif = 0;
      int J8stepDif = 0;
      int J9stepDif = 0;

      //determine motor directions
      if (J1stepDif <= 0) {
        J1dir = 1;
      }
      else {
        J1dir = 0;
      }

      if (J2stepDif <= 0) {
        J2dir = 1;
      }
      else {
        J2dir = 0;
      }

      if (J3stepDif <= 0) {
        J3dir = 1;
      }
      else {
        J3dir = 0;
      }

      if (J4stepDif <= 0) {
        J4dir = 1;
      }
      else {
        J4dir = 0;
      }

      if (J5stepDif <= 0) {
        J5dir = 1;
      }
      else {
        J5dir = 0;
      }

      if (J6stepDif <= 0) {
        J6dir = 1;
      }
      else {
        J6dir = 0;
      }

      J7dir = 0;
      J8dir = 0;
      J9dir = 0;



      resetEncoders();
      driveMotorsJ(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(J7stepDif), abs(J8stepDif), abs(J9stepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
      checkEncoders();
      sendRobotPos();
      delay(5);
      Serial.println("Done");
    }


    //-----COMMAND CORRECT POSITION---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "CP")
    {
      correctRobotPos();
    }

    //-----COMMAND UPDATE PARAMS---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "UP")
    {
      int TFxStart = inData.indexOf('A');
      int TFyStart = inData.indexOf('B');
      int TFzStart = inData.indexOf('C');
      int TFrzStart = inData.indexOf('D');
      int TFryStart = inData.indexOf('E');
      int TFrxStart = inData.indexOf('F');

      int J1motDirStart = inData.indexOf('G');
      int J2motDirStart = inData.indexOf('H');
      int J3motDirStart = inData.indexOf('I');
      int J4motDirStart = inData.indexOf('J');
      int J5motDirStart = inData.indexOf('K');
      int J6motDirStart = inData.indexOf('L');
      int J7motDirStart = inData.indexOf('M');
      int J8motDirStart = inData.indexOf('N');
      int J9motDirStart = inData.indexOf('O');

      int J1calDirStart = inData.indexOf('P');
      int J2calDirStart = inData.indexOf('Q');
      int J3calDirStart = inData.indexOf('R');
      int J4calDirStart = inData.indexOf('S');
      int J5calDirStart = inData.indexOf('T');
      int J6calDirStart = inData.indexOf('U');
      int J7calDirStart = inData.indexOf('V');
      int J8calDirStart = inData.indexOf('W');
      int J9calDirStart = inData.indexOf('X');

      int J1PosLimStart = inData.indexOf('Y');
      int J1NegLimStart = inData.indexOf('Z');
      int J2PosLimStart = inData.indexOf('a');
      int J2NegLimStart = inData.indexOf('b');
      int J3PosLimStart = inData.indexOf('c');
      int J3NegLimStart = inData.indexOf('d');
      int J4PosLimStart = inData.indexOf('e');
      int J4NegLimStart = inData.indexOf('f');
      int J5PosLimStart = inData.indexOf('g');
      int J5NegLimStart = inData.indexOf('h');
      int J6PosLimStart = inData.indexOf('i');
      int J6NegLimStart = inData.indexOf('j');

      int J1StepDegStart = inData.indexOf('k');
      int J2StepDegStart = inData.indexOf('l');
      int J3StepDegStart = inData.indexOf('m');
      int J4StepDegStart = inData.indexOf('n');
      int J5StepDegStart = inData.indexOf('o');
      int J6StepDegStart = inData.indexOf('p');

      int J1EncMultStart = inData.indexOf('q');
      int J2EncMultStart = inData.indexOf('r');
      int J3EncMultStart = inData.indexOf('s');
      int J4EncMultStart = inData.indexOf('t');
      int J5EncMultStart = inData.indexOf('u');
      int J6EncMultStart = inData.indexOf('v');

      int J1tDHparStart = inData.indexOf('w');
      int J2tDHparStart = inData.indexOf('x');
      int J3tDHparStart = inData.indexOf('y');
      int J4tDHparStart = inData.indexOf('z');
      int J5tDHparStart = inData.indexOf('!');
      int J6tDHparStart = inData.indexOf('@');

      int J1uDHparStart = inData.indexOf('#');
      int J2uDHparStart = inData.indexOf('$');
      int J3uDHparStart = inData.indexOf('%');
      int J4uDHparStart = inData.indexOf('^');
      int J5uDHparStart = inData.indexOf('&');
      int J6uDHparStart = inData.indexOf('*');

      int J1dDHparStart = inData.indexOf('(');
      int J2dDHparStart = inData.indexOf(')');
      int J3dDHparStart = inData.indexOf('+');
      int J4dDHparStart = inData.indexOf('=');
      int J5dDHparStart = inData.indexOf(',');
      int J6dDHparStart = inData.indexOf('_');

      int J1aDHparStart = inData.indexOf('<');
      int J2aDHparStart = inData.indexOf('>');
      int J3aDHparStart = inData.indexOf('?');
      int J4aDHparStart = inData.indexOf('{');
      int J5aDHparStart = inData.indexOf('}');
      int J6aDHparStart = inData.indexOf('~');

      Xtool = inData.substring(TFxStart + 1, TFyStart).toFloat();
      Ytool = inData.substring(TFyStart + 1, TFzStart).toFloat();
      Ztool = inData.substring(TFzStart + 1, TFrzStart).toFloat();
      RZtool = inData.substring(TFrzStart + 1, TFryStart).toFloat();
      RYtool = inData.substring(TFryStart + 1, TFrxStart).toFloat();
      RXtool = inData.substring(TFrxStart + 1, J1motDirStart).toFloat();
      J1MotDir = inData.substring(J1motDirStart + 1, J2motDirStart).toInt();
      J2MotDir = inData.substring(J2motDirStart + 1, J3motDirStart).toInt();
      J3MotDir = inData.substring(J3motDirStart + 1, J4motDirStart).toInt();
      J4MotDir = inData.substring(J4motDirStart + 1, J5motDirStart).toInt();
      J5MotDir = inData.substring(J5motDirStart + 1, J6motDirStart).toInt();
      J6MotDir = inData.substring(J6motDirStart + 1, J7motDirStart).toInt();
      J7MotDir = inData.substring(J7motDirStart + 1, J8motDirStart).toInt();
      J8MotDir = inData.substring(J8motDirStart + 1, J9motDirStart).toInt();
      J9MotDir = inData.substring(J9motDirStart + 1, J1calDirStart).toInt();
      J1CalDir = inData.substring(J1calDirStart + 1, J2calDirStart).toInt();
      J2CalDir = inData.substring(J2calDirStart + 1, J3calDirStart).toInt();
      J3CalDir = inData.substring(J3calDirStart + 1, J4calDirStart).toInt();
      J4CalDir = inData.substring(J4calDirStart + 1, J5calDirStart).toInt();
      J5CalDir = inData.substring(J5calDirStart + 1, J6calDirStart).toInt();
      J6CalDir = inData.substring(J6calDirStart + 1, J7calDirStart).toInt();
      J7CalDir = inData.substring(J7calDirStart + 1, J8calDirStart).toInt();
      J8CalDir = inData.substring(J8calDirStart + 1, J9calDirStart).toInt();
      J9CalDir = inData.substring(J9calDirStart + 1, J1PosLimStart).toInt();
      J1axisLimPos = inData.substring(J1PosLimStart + 1, J1NegLimStart).toFloat();
      J1axisLimNeg = inData.substring(J1NegLimStart + 1, J2PosLimStart).toFloat();
      J2axisLimPos = inData.substring(J2PosLimStart + 1, J2NegLimStart).toFloat();
      J2axisLimNeg = inData.substring(J2NegLimStart + 1, J3PosLimStart).toFloat();
      J3axisLimPos = inData.substring(J3PosLimStart + 1, J3NegLimStart).toFloat();
      J3axisLimNeg = inData.substring(J3NegLimStart + 1, J4PosLimStart).toFloat();
      J4axisLimPos = inData.substring(J4PosLimStart + 1, J4NegLimStart).toFloat();
      J4axisLimNeg = inData.substring(J4NegLimStart + 1, J5PosLimStart).toFloat();
      J5axisLimPos = inData.substring(J5PosLimStart + 1, J5NegLimStart).toFloat();
      J5axisLimNeg = inData.substring(J5NegLimStart + 1, J6PosLimStart).toFloat();
      J6axisLimPos = inData.substring(J6PosLimStart + 1, J6NegLimStart).toFloat();
      J6axisLimNeg = inData.substring(J6NegLimStart + 1, J1StepDegStart).toFloat();

      J1StepDeg = inData.substring(J1StepDegStart + 1, J2StepDegStart).toFloat();
      J2StepDeg = inData.substring(J2StepDegStart + 1, J3StepDegStart).toFloat();
      J3StepDeg = inData.substring(J3StepDegStart + 1, J4StepDegStart).toFloat();
      J4StepDeg = inData.substring(J4StepDegStart + 1, J5StepDegStart).toFloat();
      J5StepDeg = inData.substring(J5StepDegStart + 1, J6StepDegStart).toFloat();
      J6StepDeg = inData.substring(J6StepDegStart + 1, J1EncMultStart).toFloat();

      J1encMult = inData.substring(J1EncMultStart + 1, J2EncMultStart).toFloat();
      J2encMult = inData.substring(J2EncMultStart + 1, J3EncMultStart).toFloat();
      J3encMult = inData.substring(J3EncMultStart + 1, J4EncMultStart).toFloat();
      J4encMult = inData.substring(J4EncMultStart + 1, J5EncMultStart).toFloat();
      J5encMult = inData.substring(J5EncMultStart + 1, J6EncMultStart).toFloat();
      J6encMult = inData.substring(J6EncMultStart + 1, J1tDHparStart).toFloat();

      DHparams[0][0] = inData.substring(J1tDHparStart + 1, J2tDHparStart).toFloat();
      DHparams[1][0] = inData.substring(J2tDHparStart + 1, J3tDHparStart).toFloat();
      DHparams[2][0] = inData.substring(J3tDHparStart + 1, J4tDHparStart).toFloat();
      DHparams[3][0] = inData.substring(J4tDHparStart + 1, J5tDHparStart).toFloat();
      DHparams[4][0] = inData.substring(J5tDHparStart + 1, J6tDHparStart).toFloat();
      DHparams[5][0] = inData.substring(J6tDHparStart + 1, J1uDHparStart).toFloat();

      DHparams[0][1] = inData.substring(J1uDHparStart + 1, J2uDHparStart).toFloat();
      DHparams[1][1] = inData.substring(J2uDHparStart + 1, J3uDHparStart).toFloat();
      DHparams[2][1] = inData.substring(J3uDHparStart + 1, J4uDHparStart).toFloat();
      DHparams[3][1] = inData.substring(J4uDHparStart + 1, J5uDHparStart).toFloat();
      DHparams[4][1] = inData.substring(J5uDHparStart + 1, J6uDHparStart).toFloat();
      DHparams[5][1] = inData.substring(J6uDHparStart + 1, J1dDHparStart).toFloat();

      DHparams[0][2] = inData.substring(J1dDHparStart + 1, J2dDHparStart).toFloat();
      DHparams[1][2] = inData.substring(J2dDHparStart + 1, J3dDHparStart).toFloat();
      DHparams[2][2] = inData.substring(J3dDHparStart + 1, J4dDHparStart).toFloat();
      DHparams[3][2] = inData.substring(J4dDHparStart + 1, J5dDHparStart).toFloat();
      DHparams[4][2] = inData.substring(J5dDHparStart + 1, J6dDHparStart).toFloat();
      DHparams[5][2] = inData.substring(J6dDHparStart + 1, J1aDHparStart).toFloat();

      DHparams[0][3] = inData.substring(J1aDHparStart + 1, J2aDHparStart).toFloat();
      DHparams[1][3] = inData.substring(J2aDHparStart + 1, J3aDHparStart).toFloat();
      DHparams[2][3] = inData.substring(J3aDHparStart + 1, J4aDHparStart).toFloat();
      DHparams[3][3] = inData.substring(J4aDHparStart + 1, J5aDHparStart).toFloat();
      DHparams[4][3] = inData.substring(J5aDHparStart + 1, J6aDHparStart).toFloat();
      DHparams[5][3] = inData.substring(J6aDHparStart + 1).toFloat();

      R06_neg_matrix[2][3] = -DHparams[5][2];

      //define total axis travel
      J1axisLim = J1axisLimPos + J1axisLimNeg;
      J2axisLim = J2axisLimPos + J2axisLimNeg;
      J3axisLim = J3axisLimPos + J3axisLimNeg;
      J4axisLim = J4axisLimPos + J4axisLimNeg;
      J5axisLim = J5axisLimPos + J5axisLimNeg;
      J6axisLim = J6axisLimPos + J6axisLimNeg;

      //steps full movement of each axis
      J1StepLim = J1axisLim * J1StepDeg;
      J2StepLim = J2axisLim * J2StepDeg;
      J3StepLim = J3axisLim * J3StepDeg;
      J4StepLim = J4axisLim * J4StepDeg;
      J5StepLim = J5axisLim * J5StepDeg;
      J6StepLim = J6axisLim * J6StepDeg;

      //step and axis zero
      J1zeroStep = J1axisLimNeg * J1StepDeg;
      J2zeroStep = J2axisLimNeg * J2StepDeg;
      J3zeroStep = J3axisLimNeg * J3StepDeg;
      J4zeroStep = J4axisLimNeg * J4StepDeg;
      J5zeroStep = J5axisLimNeg * J5StepDeg;
      J6zeroStep = J6axisLimNeg * J6StepDeg;

      Tool_Matrix(Xtool, Ytool, Ztool, RZtool, RYtool, RXtool);
      Tool_MatrixRev(Xtool, Ytool, Ztool, RZtool, RYtool, RXtool);

      
      Serial.print("Done");
    }

    //-----COMMAND CALIBRATE EXTERNAL AXIS---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "CE")
    {
      int J7lengthStart = inData.indexOf('A');
      int J7rotStart = inData.indexOf('B');
      int J7stepsStart = inData.indexOf('C');
      int J8lengthStart = inData.indexOf('D');
      int J8rotStart = inData.indexOf('E');
      int J8stepsStart = inData.indexOf('F');
      int J9lengthStart = inData.indexOf('G');
      int J9rotStart = inData.indexOf('H');
      int J9stepsStart = inData.indexOf('I');

      J7length = inData.substring(J7lengthStart + 1, J7rotStart).toFloat();
      J7rot = inData.substring(J7rotStart + 1, J7stepsStart).toFloat();
      J7steps = inData.substring(J7stepsStart + 1, J8lengthStart).toFloat();

      J8length = inData.substring(J8lengthStart + 1, J8rotStart).toFloat();
      J8rot = inData.substring(J8rotStart + 1, J8stepsStart).toFloat();
      J8steps = inData.substring(J8stepsStart + 1, J9lengthStart).toFloat();

      J9length = inData.substring(J9lengthStart + 1, J9rotStart).toFloat();
      J9rot = inData.substring(J9rotStart + 1, J9stepsStart).toFloat();
      J9steps = inData.substring(J9stepsStart + 1).toFloat();

      J7axisLimNeg = 0;
      J7axisLimPos = J7length;
      J7axisLim = J7axisLimPos + J7axisLimNeg;
      J7StepDeg = J7steps / J7rot;
      J7StepLim = J7axisLim * J7StepDeg;

      J8axisLimNeg = 0;
      J8axisLimPos = J8length;
      J8axisLim = J8axisLimPos + J8axisLimNeg;
      J8StepDeg = J8steps / J8rot;
      J8StepLim = J8axisLim * J8StepDeg;

      J9axisLimNeg = 0;
      J9axisLimPos = J9length;
      J9axisLim = J9axisLimPos + J9axisLimNeg;
      J9StepDeg = J9steps / J9rot;
      J9StepLim = J9axisLim * J9StepDeg;

      delay(5);
      Serial.print("Done");
    }

    //-----COMMAND ZERO J7---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "Z7")
    {
      J7StepM = 0;
      sendRobotPos();
    }

    //-----COMMAND ZERO J8---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "Z8")
    {
      J8StepM = 0;
      sendRobotPos();
    }

    //-----COMMAND ZERO J9---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "Z9")
    {
      J9StepM = 0;
      sendRobotPos();
    }





    //-----COMMAND TO WAIT TIME---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "WT")
    {
      int WTstart = inData.indexOf('S');
      float WaitTime = inData.substring(WTstart + 1).toFloat();
      int WaitTimeMS = WaitTime * 1000;
      delay(WaitTimeMS);
      Serial.println("WTdone");
    }

    //-----COMMAND IF INPUT THEN JUMP---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "JF")
    {
      int IJstart = inData.indexOf('X');
      int IJTabstart = inData.indexOf('T');
      int IJInputNum = inData.substring(IJstart + 1, IJTabstart).toInt();
      if (digitalRead(IJInputNum) == HIGH)
      {
        delay(5);
        Serial.println("T");
      }
      if (digitalRead(IJInputNum) == LOW)
      {
        delay(5);
        Serial.println("F");
      }
    }
    //-----COMMAND SET OUTPUT ON---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "ON")
    {
      int ONstart = inData.indexOf('X');
      int outputNum = inData.substring(ONstart + 1).toInt();
      digitalWrite(outputNum, HIGH);
      delay(5);
      Serial.println("Done");
    }
    //-----COMMAND SET OUTPUT OFF---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "OF")
    {
      int ONstart = inData.indexOf('X');
      int outputNum = inData.substring(ONstart + 1).toInt();
      digitalWrite(outputNum, LOW);
      delay(5);
      Serial.println("Done");
    }
    //-----COMMAND TO WAIT INPUT ON---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "WI")
    {
      int WIstart = inData.indexOf('N');
      int InputNum = inData.substring(WIstart + 1).toInt();
      while (digitalRead(InputNum) == LOW) {
        delay(100);
      }
      delay(5);
      Serial.println("Done");
    }
    //-----COMMAND TO WAIT INPUT OFF---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "WO")
    {
      int WIstart = inData.indexOf('N');
      int InputNum = inData.substring(WIstart + 1).toInt();
      while (digitalRead(InputNum) == HIGH) {
        delay(100);
      }
      delay(5);
      Serial.println("Done");
    }

    //-----COMMAND SEND POSITION---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "SP")
    {
      int J1angStart = inData.indexOf('A');
      int J2angStart = inData.indexOf('B');
      int J3angStart = inData.indexOf('C');
      int J4angStart = inData.indexOf('D');
      int J5angStart = inData.indexOf('E');
      int J6angStart = inData.indexOf('F');
      int J7angStart = inData.indexOf('G');
      int J8angStart = inData.indexOf('H');
      int J9angStart = inData.indexOf('I');
      J1StepM = ((inData.substring(J1angStart + 1, J2angStart).toFloat()) + J1axisLimNeg) * J1StepDeg;
      J2StepM = ((inData.substring(J2angStart + 1, J3angStart).toFloat()) + J2axisLimNeg) * J2StepDeg;
      J3StepM = ((inData.substring(J3angStart + 1, J4angStart).toFloat()) + J3axisLimNeg) * J3StepDeg;
      J4StepM = ((inData.substring(J4angStart + 1, J5angStart).toFloat()) + J4axisLimNeg) * J4StepDeg;
      J5StepM = ((inData.substring(J5angStart + 1, J6angStart).toFloat()) + J5axisLimNeg) * J5StepDeg;
      J6StepM = ((inData.substring(J6angStart + 1, J7angStart).toFloat()) + J6axisLimNeg) * J6StepDeg;
      J7StepM = ((inData.substring(J7angStart + 1, J8angStart).toFloat()) + J7axisLimNeg) * J7StepDeg;
      J8StepM = ((inData.substring(J8angStart + 1, J9angStart).toFloat()) + J8axisLimNeg) * J8StepDeg;
      J9StepM = ((inData.substring(J9angStart + 1).toFloat()) + J9axisLimNeg) * J9StepDeg;
      delay(5);
      Serial.println("Done");
    }


    //-----COMMAND ECHO TEST MESSAGE---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "TM")
    {
      int J1start = inData.indexOf('A');
      int J2start = inData.indexOf('B');
      int J3start = inData.indexOf('C');
      int J4start = inData.indexOf('D');
      int J5start = inData.indexOf('E');
      int J6start = inData.indexOf('F');
      int WristConStart = inData.indexOf('W');
      JangleIn[0] = inData.substring(J1start + 1, J2start).toFloat();
      JangleIn[1] = inData.substring(J2start + 1, J3start).toFloat();
      JangleIn[2] = inData.substring(J3start + 1, J4start).toFloat();
      JangleIn[3] = inData.substring(J4start + 1, J5start).toFloat();
      JangleIn[4] = inData.substring(J5start + 1, J6start).toFloat();
      JangleIn[5] = inData.substring(J6start + 1, WristConStart).toFloat();
      WristCon = inData.substring(WristConStart + 1);
      WristCon.trim();

      SolveInverseKinematic();

      String echo = "";
      delay(5);
      Serial.println(inData);


    }
    //-----COMMAND TO CALIBRATE---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "LL")
    {
      int J1start = inData.indexOf('A');
      int J2start = inData.indexOf('B');
      int J3start = inData.indexOf('C');
      int J4start = inData.indexOf('D');
      int J5start = inData.indexOf('E');
      int J6start = inData.indexOf('F');
      int J7start = inData.indexOf('G');
      int J8start = inData.indexOf('H');
      int J9start = inData.indexOf('I');

      int J1calstart = inData.indexOf('J');
      int J2calstart = inData.indexOf('K');
      int J3calstart = inData.indexOf('L');
      int J4calstart = inData.indexOf('M');
      int J5calstart = inData.indexOf('N');
      int J6calstart = inData.indexOf('O');
      int J7calstart = inData.indexOf('P');
      int J8calstart = inData.indexOf('Q');
      int J9calstart = inData.indexOf('R');



      ///
      int J1req = inData.substring(J1start + 1, J2start).toInt();
      int J2req = inData.substring(J2start + 1, J3start).toInt();
      int J3req = inData.substring(J3start + 1, J4start).toInt();
      int J4req = inData.substring(J4start + 1, J5start).toInt();
      int J5req = inData.substring(J5start + 1, J6start).toInt();
      int J6req = inData.substring(J6start + 1, J7start).toInt();
      int J7req = inData.substring(J7start + 1, J8start).toInt();
      int J8req = inData.substring(J8start + 1, J9start).toInt();
      int J9req = inData.substring(J9start + 1, J1calstart).toInt();



      float J1calOff = inData.substring(J1calstart + 1, J2calstart).toFloat();
      float J2calOff = inData.substring(J2calstart + 1, J3calstart).toFloat();
      float J3calOff = inData.substring(J3calstart + 1, J4calstart).toFloat();
      float J4calOff = inData.substring(J4calstart + 1, J5calstart).toFloat();
      float J5calOff = inData.substring(J5calstart + 1, J6calstart).toFloat();
      float J6calOff = inData.substring(J6calstart + 1, J7calstart).toFloat();
      float J7calOff = inData.substring(J7calstart + 1, J8calstart).toFloat();
      float J8calOff = inData.substring(J8calstart + 1, J9calstart).toFloat();
      float J9calOff = inData.substring(J9calstart + 1).toFloat();
      ///
      float SpeedIn;
      ///
      int J1Step = 0;
      int J2Step = 0;
      int J3Step = 0;
      int J4Step = 0;
      int J5Step = 0;
      int J6Step = 0;
      int J7Step = 0;
      int J8Step = 0;
      int J9Step = 0;
      ///
      int J1stepCen = 0;
      int J2stepCen = 0;
      int J3stepCen = 0;
      int J4stepCen = 0;
      int J5stepCen = 0;
      int J5step90 = 0;
      int J6stepCen = 0;
      int J7stepCen = 0;
      int J8stepCen = 0;
      int J9stepCen = 0;
      ///
      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;

      Alarm = "0";

      //--IF JOINT IS CALLED FOR CALIBRATION PASS ITS STEP LIMIT OTHERWISE PASS 0---
      if (J1req == 1) {
        J1Step = J1StepLim;
      }
      if (J2req == 1) {
        J2Step = J2StepLim;
      }
      if (J3req == 1) {
        J3Step = J3StepLim;
      }
      if (J4req == 1) {
        J4Step = J4StepLim;
      }
      if (J5req == 1) {
        J5Step = J5StepLim;
      }
      if (J6req == 1) {
        J6Step = J6StepLim;
      }
      if (J7req == 1) {
        J7Step = J7StepLim;
      }
      if (J8req == 1) {
        J8Step = J8StepLim;
      }
      if (J9req == 1) {
        J9Step = J9StepLim;
      }

      //--CALL FUNCT TO DRIVE TO LIMITS--
      SpeedIn = 50;
      driveLimit(J1Step, J2Step, J3Step, J4Step, J5Step, J6Step, J7Step, J8Step, J9Step, SpeedIn);
      delay(500);

      //BACKOFF
      //SET DIRECTIONS
      /// J1 ///
      if (J1CalDir == 1 && J1MotDir == 1) {
        digitalWrite(J1dirPin, LOW);
      }
      if (J1CalDir == 1 && J1MotDir == 0) {
        digitalWrite(J1dirPin, HIGH);
      }
      if (J1CalDir == 0 && J1MotDir == 1) {
        digitalWrite(J1dirPin, HIGH);
      }
      if (J1CalDir == 0 && J1MotDir == 0) {
        digitalWrite(J1dirPin, LOW);
      }
      /// J2 ///
      if (J2CalDir == 1 && J2MotDir == 1) {
        digitalWrite(J2dirPin, LOW);
      }
      if (J2CalDir == 1 && J2MotDir == 0) {
        digitalWrite(J2dirPin, HIGH);
      }
      if (J2CalDir == 0 && J2MotDir == 1) {
        digitalWrite(J2dirPin, HIGH);
      }
      if (J2CalDir == 0 && J2MotDir == 0) {
        digitalWrite(J2dirPin, LOW);
      }
      /// J3 ///
      if (J3CalDir == 1 && J3MotDir == 1) {
        digitalWrite(J3dirPin, LOW);
      }
      if (J3CalDir == 1 && J3MotDir == 0) {
        digitalWrite(J3dirPin, HIGH);
      }
      if (J3CalDir == 0 && J3MotDir == 1) {
        digitalWrite(J3dirPin, HIGH);
      }
      if (J3CalDir == 0 && J3MotDir == 0) {
        digitalWrite(J3dirPin, LOW);
      }
      /// J4 ///
      if (J4CalDir == 1 && J4MotDir == 1) {
        digitalWrite(J4dirPin, LOW);
      }
      if (J4CalDir == 1 && J4MotDir == 0) {
        digitalWrite(J4dirPin, HIGH);
      }
      if (J4CalDir == 0 && J4MotDir == 1) {
        digitalWrite(J4dirPin, HIGH);
      }
      if (J4CalDir == 0 && J4MotDir == 0) {
        digitalWrite(J4dirPin, LOW);
      }
      /// J5 ///
      if (J5CalDir == 1 && J5MotDir == 1) {
        digitalWrite(J5dirPin, LOW);
      }
      if (J5CalDir == 1 && J5MotDir == 0) {
        digitalWrite(J5dirPin, HIGH);
      }
      if (J5CalDir == 0 && J5MotDir == 1) {
        digitalWrite(J5dirPin, HIGH);
      }
      if (J5CalDir == 0 && J5MotDir == 0) {
        digitalWrite(J5dirPin, LOW);
      }
      /// J6 ///
      if (J6CalDir == 1 && J6MotDir == 1) {
        digitalWrite(J6dirPin, LOW);
      }
      if (J6CalDir == 1 && J6MotDir == 0) {
        digitalWrite(J6dirPin, HIGH);
      }
      if (J6CalDir == 0 && J6MotDir == 1) {
        digitalWrite(J6dirPin, HIGH);
      }
      if (J6CalDir == 0 && J6MotDir == 0) {
        digitalWrite(J6dirPin, LOW);
      }
      /// J7 ///
      if (J7CalDir == 1 && J7MotDir == 1) {
        digitalWrite(J7dirPin, LOW);
      }
      if (J7CalDir == 1 && J7MotDir == 0) {
        digitalWrite(J7dirPin, HIGH);
      }
      if (J7CalDir == 0 && J7MotDir == 1) {
        digitalWrite(J7dirPin, HIGH);
      }
      if (J7CalDir == 0 && J7MotDir == 0) {
        digitalWrite(J7dirPin, LOW);
      }
      /// J8 ///
      if (J8CalDir == 1 && J8MotDir == 1) {
        digitalWrite(J8dirPin, LOW);
      }
      if (J8CalDir == 1 && J8MotDir == 0) {
        digitalWrite(J8dirPin, HIGH);
      }
      if (J8CalDir == 0 && J8MotDir == 1) {
        digitalWrite(J8dirPin, HIGH);
      }
      if (J8CalDir == 0 && J8MotDir == 0) {
        digitalWrite(J8dirPin, LOW);
      }
      /// J9 ///
      if (J9CalDir == 1 && J9MotDir == 1) {
        digitalWrite(J9dirPin, LOW);
      }
      if (J9CalDir == 1 && J9MotDir == 0) {
        digitalWrite(J9dirPin, HIGH);
      }
      if (J9CalDir == 0 && J9MotDir == 1) {
        digitalWrite(J9dirPin, HIGH);
      }
      if (J9CalDir == 0 && J9MotDir == 0) {
        digitalWrite(J9dirPin, LOW);
      }
      int BacOff = 0;
      while (BacOff <= 250)
      {
        if (J1req == 1) {
          digitalWrite(J1stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J1stepPin, HIGH);
          delayMicroseconds(5);
        }
        if (J2req == 1) {
          digitalWrite(J2stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J2stepPin, HIGH);
          delayMicroseconds(5);
        }
        if (J3req == 1) {
          digitalWrite(J3stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J3stepPin, HIGH);
          delayMicroseconds(5);
        }
        if (J4req == 1) {
          digitalWrite(J4stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J4stepPin, HIGH);
          delayMicroseconds(5);
        }
        if (J5req == 1) {
          digitalWrite(J5stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J5stepPin, HIGH);
          delayMicroseconds(5);
        }
        if (J6req == 1) {
          digitalWrite(J6stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J6stepPin, HIGH);
          delayMicroseconds(5);
        }
        if (J7req == 1) {
          digitalWrite(J7stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J7stepPin, HIGH);
          delayMicroseconds(5);
        }
        if (J8req == 1) {
          digitalWrite(J8stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J8stepPin, HIGH);
          delayMicroseconds(5);
        }
        if (J9req == 1) {
          digitalWrite(J9stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J9stepPin, HIGH);
          delayMicroseconds(5);
        }
        BacOff = ++BacOff;
        delayMicroseconds(7000);
      }

      //--CALL FUNCT TO DRIVE BACK TO LIMITS SLOWLY--
      SpeedIn = 2;
      driveLimit(J1Step, J2Step, J3Step, J4Step, J5Step, J6Step, J7Step, J8Step, J9Step, SpeedIn);

      //OVERDRIVE - MAKE SURE LIMIT SWITCH STAYS MADE
      //SET DIRECTIONS
      /// J1 ///
      if (J1CalDir == 1 && J1MotDir == 1) {
        digitalWrite(J1dirPin, HIGH);
      }
      if (J1CalDir == 1 && J1MotDir == 0) {
        digitalWrite(J1dirPin, LOW);
      }
      if (J1CalDir == 0 && J1MotDir == 1) {
        digitalWrite(J1dirPin, LOW);
      }
      if (J1CalDir == 0 && J1MotDir == 0) {
        digitalWrite(J1dirPin, HIGH);
      }
      /// J2 ///
      if (J2CalDir == 1 && J2MotDir == 1) {
        digitalWrite(J2dirPin, HIGH);
      }
      if (J2CalDir == 1 && J2MotDir == 0) {
        digitalWrite(J2dirPin, LOW);
      }
      if (J2CalDir == 0 && J2MotDir == 1) {
        digitalWrite(J2dirPin, LOW);
      }
      if (J2CalDir == 0 && J2MotDir == 0) {
        digitalWrite(J2dirPin, HIGH);
      }
      /// J3 ///
      if (J3CalDir == 1 && J3MotDir == 1) {
        digitalWrite(J3dirPin, HIGH);
      }
      if (J3CalDir == 1 && J3MotDir == 0) {
        digitalWrite(J3dirPin, LOW);
      }
      if (J3CalDir == 0 && J3MotDir == 1) {
        digitalWrite(J3dirPin, LOW);
      }
      if (J3CalDir == 0 && J3MotDir == 0) {
        digitalWrite(J3dirPin, HIGH);
      }
      /// J4 ///
      if (J4CalDir == 1 && J4MotDir == 1) {
        digitalWrite(J4dirPin, HIGH);
      }
      if (J4CalDir == 1 && J4MotDir == 0) {
        digitalWrite(J4dirPin, LOW);
      }
      if (J4CalDir == 0 && J4MotDir == 1) {
        digitalWrite(J4dirPin, LOW);
      }
      if (J4CalDir == 0 && J4MotDir == 0) {
        digitalWrite(J4dirPin, HIGH);
      }
      /// J5 ///
      if (J5CalDir == 1 && J5MotDir == 1) {
        digitalWrite(J5dirPin, HIGH);
      }
      if (J5CalDir == 1 && J5MotDir == 0) {
        digitalWrite(J5dirPin, LOW);
      }
      if (J5CalDir == 0 && J5MotDir == 1) {
        digitalWrite(J5dirPin, LOW);
      }
      if (J5CalDir == 0 && J5MotDir == 0) {
        digitalWrite(J5dirPin, HIGH);
      }
      /// J6 ///
      if (J6CalDir == 1 && J6MotDir == 1) {
        digitalWrite(J6dirPin, HIGH);
      }
      if (J6CalDir == 1 && J6MotDir == 0) {
        digitalWrite(J6dirPin, LOW);
      }
      if (J6CalDir == 0 && J6MotDir == 1) {
        digitalWrite(J6dirPin, LOW);
      }
      if (J6CalDir == 0 && J6MotDir == 0) {
        digitalWrite(J6dirPin, HIGH);
      }
      /// J7 ///
      if (J7CalDir == 1 && J7MotDir == 1) {
        digitalWrite(J7dirPin, HIGH);
      }
      if (J7CalDir == 1 && J7MotDir == 0) {
        digitalWrite(J7dirPin, LOW);
      }
      if (J7CalDir == 0 && J7MotDir == 1) {
        digitalWrite(J7dirPin, LOW);
      }
      if (J7CalDir == 0 && J7MotDir == 0) {
        digitalWrite(J7dirPin, HIGH);
      }
      /// J8 ///
      if (J8CalDir == 1 && J8MotDir == 1) {
        digitalWrite(J8dirPin, HIGH);
      }
      if (J8CalDir == 1 && J8MotDir == 0) {
        digitalWrite(J8dirPin, LOW);
      }
      if (J8CalDir == 0 && J8MotDir == 1) {
        digitalWrite(J8dirPin, LOW);
      }
      if (J8CalDir == 0 && J8MotDir == 0) {
        digitalWrite(J8dirPin, HIGH);
      }
      /// J9 ///
      if (J9CalDir == 1 && J9MotDir == 1) {
        digitalWrite(J9dirPin, HIGH);
      }
      if (J9CalDir == 1 && J9MotDir == 0) {
        digitalWrite(J9dirPin, LOW);
      }
      if (J9CalDir == 0 && J9MotDir == 1) {
        digitalWrite(J9dirPin, LOW);
      }
      if (J9CalDir == 0 && J9MotDir == 0) {
        digitalWrite(J9dirPin, HIGH);
      }

      int OvrDrv = 0;
      while (OvrDrv <= 50)
      {
        if (J1req == 1) {
          digitalWrite(J1stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J1stepPin, HIGH);
          delayMicroseconds(5);
        }
        if (J2req == 1) {
          digitalWrite(J2stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J2stepPin, HIGH);
          delayMicroseconds(5);
        }
        if (J3req == 1) {
          digitalWrite(J3stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J3stepPin, HIGH);
          delayMicroseconds(5);
        }
        if (J4req == 1) {
          digitalWrite(J4stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J4stepPin, HIGH);
          delayMicroseconds(5);
        }
        if (J5req == 1) {
          digitalWrite(J5stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J5stepPin, HIGH);
          delayMicroseconds(5);
        }
        if (J6req == 1) {
          digitalWrite(J6stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J6stepPin, HIGH);
          delayMicroseconds(5);
        }
        if (J7req == 1) {
          digitalWrite(J7stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J7stepPin, HIGH);
          delayMicroseconds(5);
        }
        if (J8req == 1) {
          digitalWrite(J8stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J8stepPin, HIGH);
          delayMicroseconds(5);
        }
        if (J9req == 1) {
          digitalWrite(J9stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J9stepPin, HIGH);
          delayMicroseconds(5);
        }
        OvrDrv = ++OvrDrv;
        delayMicroseconds(7000);
      }

      //SEE IF ANY SWITCHES NOT MADE
      delay(500);
      ///
      if (J1req == 1) {
        if (digitalRead(J1calPin) == LOW) {
          Alarm = "EA1";
        }
      }
      if (J2req == 1) {
        if (digitalRead(J2calPin) == LOW) {
          Alarm = "EA2";
        }
      }
      if (J3req == 1) {
        if (digitalRead(J3calPin) == LOW) {
          Alarm = "EA3";
        }
      }
      if (J4req == 1) {
        if (digitalRead(J4calPin) == LOW) {
          Alarm = "EA4";
        }
      }
      if (J5req == 1) {
        if (digitalRead(J5calPin) == LOW) {
          Alarm = "EA5";
        }
      }
      if (J6req == 1) {
        if (digitalRead(J6calPin) == LOW) {
          Alarm = "EA6";
        }
      }
      if (J7req == 1) {
        if (digitalRead(J7calPin) == LOW) {
          Alarm = "EA7";
        }
      }
      if (J8req == 1) {
        if (digitalRead(J8calPin) == LOW) {
          Alarm = "EA8";
        }
      }
      if (J9req == 1) {
        if (digitalRead(J9calPin) == LOW) {
          Alarm = "EA9";
        }
      }
      ///
      if (Alarm == "0") {

        //set master steps and center step

        if (J1req == 1) {
          if (J1CalDir == 1) {
            J1StepM = ((J1axisLim) + J1calBaseOff + J1calOff) * J1StepDeg;
            J1stepCen = ((J1axisLimPos) + J1calBaseOff + J1calOff) * J1StepDeg;
          }
          else {
            J1StepM = (0 + J1calBaseOff + J1calOff) * J1StepDeg;
            J1stepCen = ((J1axisLimNeg) - J1calBaseOff - J1calOff) * J1StepDeg;
          }
        }
        if (J2req == 1) {
          if (J2CalDir == 1) {
            J2StepM = ((J2axisLim) + J2calBaseOff + J2calOff) * J2StepDeg;
            J2stepCen = ((J2axisLimPos) + J2calBaseOff + J2calOff) * J2StepDeg;
          }
          else {
            J2StepM = (0 + J2calBaseOff + J2calOff) * J2StepDeg;
            J2stepCen = ((J2axisLimNeg) - J2calBaseOff - J2calOff) * J2StepDeg;
          }
        }
        if (J3req == 1) {
          if (J3CalDir == 1) {
            J3StepM = ((J3axisLim) + J3calBaseOff + J3calOff) * J3StepDeg;
            J3stepCen = ((J3axisLimPos) + J3calBaseOff + J3calOff) * J3StepDeg;
          }
          else {
            J3StepM = (0 + J3calBaseOff + J3calOff) * J3StepDeg;
            J3stepCen = ((J3axisLimNeg) - J3calBaseOff - J3calOff) * J3StepDeg;
          }
        }
        if (J4req == 1) {
          if (J4CalDir == 1) {
            J4StepM = ((J4axisLim) + J4calBaseOff + J4calOff) * J4StepDeg;
            J4stepCen = ((J4axisLimPos) + J4calBaseOff + J4calOff) * J4StepDeg;
          }
          else {
            J4StepM = (0 + J4calBaseOff + J4calOff) * J4StepDeg;
            J4stepCen = ((J4axisLimNeg) - J4calBaseOff - J4calOff) * J4StepDeg;
          }
        }
        if (J5req == 1) {
          if (J5CalDir == 1) {
            J5StepM = ((J5axisLim) + J5calBaseOff + J5calOff) * J5StepDeg;
            J5stepCen = ((J5axisLimPos) + J5calBaseOff + J5calOff) * J5StepDeg;
            J5step90 = (((J5axisLimNeg) + J5calBaseOff + J5calOff) - 90) * J5StepDeg;
          }
          else {
            J5StepM = (0 + J5calBaseOff + J5calOff) * J5StepDeg;
            J5stepCen = ((J5axisLimNeg) - J5calBaseOff - J5calOff) * J5StepDeg;
            J5step90 = (((J5axisLimNeg) - J5calBaseOff - J5calOff) + 90) * J5StepDeg;
          }
        }
        if (J6req == 1) {
          if (J6CalDir == 1) {
            J6StepM = ((J6axisLim) + J6calBaseOff + J6calOff) * J6StepDeg;
            J6stepCen = ((J6axisLimPos) + J6calBaseOff + J6calOff) * J6StepDeg;
          }
          else {
            J6StepM = (0 + J6calBaseOff + J6calOff) * J6StepDeg;
            J6stepCen = ((J6axisLimNeg) - J6calBaseOff - J6calOff) * J6StepDeg;
          }
        }
        if (J7req == 1) {
          if (J7CalDir == 1) {
            J7StepM = ((J7axisLim) + J7calBaseOff + J7calOff) * J7StepDeg;
            J7stepCen = ((J7axisLimPos) + J7calBaseOff + J7calOff) * J7StepDeg;
          }
          else {
            J7StepM = (0 + J7calBaseOff + J7calOff) * J7StepDeg;
            J7stepCen = ((J7axisLimNeg) - J7calBaseOff - J7calOff) * J7StepDeg;
          }
        }
        if (J8req == 1) {
          if (J8CalDir == 1) {
            J8StepM = ((J8axisLim) + J8calBaseOff + J8calOff) * J8StepDeg;
            J8stepCen = ((J8axisLimPos) + J8calBaseOff + J8calOff) * J8StepDeg;
          }
          else {
            J8StepM = (0 + J8calBaseOff + J8calOff) * J8StepDeg;
            J8stepCen = ((J8axisLimNeg) - J8calBaseOff - J8calOff) * J8StepDeg;
          }
        }
        if (J9req == 1) {
          if (J9CalDir == 1) {
            J9StepM = ((J9axisLim) + J9calBaseOff + J9calOff) * J9StepDeg;
            J9stepCen = ((J9axisLimPos) + J9calBaseOff + J9calOff) * J9StepDeg;
          }
          else {
            J9StepM = (0 + J9calBaseOff + J9calOff) * J9StepDeg;
            J9stepCen = ((J9axisLimNeg) - J9calBaseOff - J9calOff) * J9StepDeg;
          }
        }

        //move to center
        /// J1 ///
        if (J1CalDir) {
          J1dir = 0;
        }
        else {
          J1dir = 1;
        }
        /// J2 ///
        if (J2CalDir) {
          J2dir = 0;
        }
        else {
          J2dir = 1;
        }
        /// J3 ///
        if (J3CalDir) {
          J3dir = 0;
        }
        else {
          J3dir = 1;
        }
        /// J4 ///
        if (J4CalDir) {
          J4dir = 0;
        }
        else {
          J4dir = 1;
        }
        /// J5 ///
        if (J5CalDir) {
          J5dir = 0;
        }
        else {
          J5dir = 1;
        }
        /// J6 ///
        if (J6CalDir) {
          J6dir = 0;
        }
        else {
          J6dir = 1;
        }
        /// J7 ///
        if (J7CalDir) {
          J7dir = 0;
        }
        else {
          J7dir = 1;
        }
        /// J8 ///
        if (J8CalDir) {
          J8dir = 0;
        }
        else {
          J8dir = 1;
        }
        /// J9 ///
        if (J9CalDir) {
          J9dir = 0;
        }
        else {
          J9dir = 1;
        }

        float ACCspd = 10;
        float DCCspd = 10;
        String SpeedType = "p";
        float SpeedVal = 50;
        float ACCramp = 50;

        driveMotorsJ(J1stepCen, J2stepCen, J3stepCen, J4stepCen, J5step90, J6stepCen, J7stepCen, J8stepCen, J9stepCen, J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
        sendRobotPos();

      }
      else {
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      }

      inData = ""; // Clear recieved buffer
    }








    //----- LIVE CARTESIAN JOG  ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "LC")
    {
      delay(5);
      Serial.println();


      updatePos();

      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;

      int J1axisFault = 0;
      int J2axisFault = 0;
      int J3axisFault = 0;
      int J4axisFault = 0;
      int J5axisFault = 0;
      int J6axisFault = 0;
      int TotalAxisFault = 0;

      bool JogInPoc = true;
      Alarm = "0";


      int VStart = inData.indexOf("V");
      int SPstart = inData.indexOf("S");
      int AcStart = inData.indexOf("Ac");
      int DcStart = inData.indexOf("Dc");
      int RmStart = inData.indexOf("Rm");
      int WristConStart = inData.indexOf("W");
      int LoopModeStart = inData.indexOf("Lm");


      float Vector = inData.substring(VStart + 1, SPstart).toFloat();
      String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
      float SpeedVal = inData.substring(SPstart + 2, AcStart).toFloat();
      float ACCspd = 100;
      float DCCspd = 100;
      float ACCramp = 100;
      WristCon = inData.substring(WristConStart + 1, LoopModeStart);
      String LoopMode = inData.substring(LoopModeStart + 2);
      LoopMode.trim();
      J1LoopMode = LoopMode.substring(0, 1).toInt();
      J2LoopMode = LoopMode.substring(1, 2).toInt();
      J3LoopMode = LoopMode.substring(2, 3).toInt();
      J4LoopMode = LoopMode.substring(3, 4).toInt();
      J5LoopMode = LoopMode.substring(4, 5).toInt();
      J6LoopMode = LoopMode.substring(5).toInt();

      inData = ""; // Clear recieved buffer

      xyzuvw_In[0] = xyzuvw_Out[0];
      xyzuvw_In[1] = xyzuvw_Out[1];
      xyzuvw_In[2] = xyzuvw_Out[2];
      xyzuvw_In[3] = xyzuvw_Out[3];
      xyzuvw_In[4] = xyzuvw_Out[4];
      xyzuvw_In[5] = xyzuvw_Out[5];


      while (JogInPoc == true) {

        if (Vector == 10) {
          xyzuvw_In[0] = xyzuvw_Out[0] - JogStepInc;
        }
        if (Vector == 11) {
          xyzuvw_In[0] = xyzuvw_Out[0] + JogStepInc;
        }

        if (Vector == 20) {
          xyzuvw_In[1] = xyzuvw_Out[1] - JogStepInc;
        }
        if (Vector == 21) {
          xyzuvw_In[1] = xyzuvw_Out[1] + JogStepInc;
        }

        if (Vector == 30) {
          xyzuvw_In[2] = xyzuvw_Out[2] - JogStepInc;
        }
        if (Vector == 31) {
          xyzuvw_In[2] = xyzuvw_Out[2] + JogStepInc;
        }

        if (Vector == 40) {
          xyzuvw_In[3] = xyzuvw_Out[3] - JogStepInc;
        }
        if (Vector == 41) {
          xyzuvw_In[3] = xyzuvw_Out[3] + JogStepInc;
        }

        if (Vector == 50) {
          xyzuvw_In[4] = xyzuvw_Out[4] - JogStepInc;
        }
        if (Vector == 51) {
          xyzuvw_In[4] = xyzuvw_Out[4] + JogStepInc;
        }

        if (Vector == 60) {
          xyzuvw_In[5] = xyzuvw_Out[5] - JogStepInc;
        }
        if (Vector == 61) {
          xyzuvw_In[5] = xyzuvw_Out[5] + JogStepInc;
        }

        SolveInverseKinematic();

        //calc destination motor steps
        int J1futStepM = (JangleOut[0] + J1axisLimNeg) * J1StepDeg;
        int J2futStepM = (JangleOut[1] + J2axisLimNeg) * J2StepDeg;
        int J3futStepM = (JangleOut[2] + J3axisLimNeg) * J3StepDeg;
        int J4futStepM = (JangleOut[3] + J4axisLimNeg) * J4StepDeg;
        int J5futStepM = (JangleOut[4] + J5axisLimNeg) * J5StepDeg;
        int J6futStepM = (JangleOut[5] + J6axisLimNeg) * J6StepDeg;

        //calc delta from current to destination
        int J1stepDif = J1StepM - J1futStepM;
        int J2stepDif = J2StepM - J2futStepM;
        int J3stepDif = J3StepM - J3futStepM;
        int J4stepDif = J4StepM - J4futStepM;
        int J5stepDif = J5StepM - J5futStepM;
        int J6stepDif = J6StepM - J6futStepM;
        int J7stepDif = 0;
        int J8stepDif = 0;
        int J9stepDif = 0;

        //determine motor directions
        if (J1stepDif <= 0) {
          J1dir = 1;
        }
        else {
          J1dir = 0;
        }

        if (J2stepDif <= 0) {
          J2dir = 1;
        }
        else {
          J2dir = 0;
        }

        if (J3stepDif <= 0) {
          J3dir = 1;
        }
        else {
          J3dir = 0;
        }

        if (J4stepDif <= 0) {
          J4dir = 1;
        }
        else {
          J4dir = 0;
        }

        if (J5stepDif <= 0) {
          J5dir = 1;
        }
        else {
          J5dir = 0;
        }

        if (J6stepDif <= 0) {
          J6dir = 1;
        }
        else {
          J6dir = 0;
        }
        J7dir = 0;
        J8dir = 0;
        J9dir = 0;


        //determine if requested position is within axis limits
        if ((J1dir == 1 and (J1StepM + J1stepDif > J1StepLim)) or (J1dir == 0 and (J1StepM - J1stepDif < 0))) {
          J1axisFault = 1;
        }
        if ((J2dir == 1 and (J2StepM + J2stepDif > J2StepLim)) or (J2dir == 0 and (J2StepM - J2stepDif < 0))) {
          J2axisFault = 1;
        }
        if ((J3dir == 1 and (J3StepM + J3stepDif > J3StepLim)) or (J3dir == 0 and (J3StepM - J3stepDif < 0))) {
          J3axisFault = 1;
        }
        if ((J4dir == 1 and (J4StepM + J4stepDif > J4StepLim)) or (J4dir == 0 and (J4StepM - J4stepDif < 0))) {
          J4axisFault = 1;
        }
        if ((J5dir == 1 and (J5StepM + J5stepDif > J5StepLim)) or (J5dir == 0 and (J5StepM - J5stepDif < 0))) {
          J5axisFault = 1;
        }
        if ((J6dir == 1 and (J6StepM + J6stepDif > J6StepLim)) or (J6dir == 0 and (J6StepM - J6stepDif < 0))) {
          J6axisFault = 1;
        }
        TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault;


        //send move command if no axis limit error
        if (TotalAxisFault == 0 && KinematicError == 0) {
          resetEncoders();
          driveMotorsJ(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(J7stepDif), abs(J8stepDif), abs(J9stepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
          //checkEncoders();
          J1EncSteps = J1encPos.read() / J1encMult;
          J2EncSteps = J2encPos.read() / J2encMult;
          J3EncSteps = J3encPos.read() / J3encMult;
          J4EncSteps = J4encPos.read() / J4encMult;
          J5EncSteps = J5encPos.read() / J5encMult;
          J6EncSteps = J6encPos.read() / J6encMult;

          if (J1LoopMode == 0) {
            if (abs((J1EncSteps - J1StepM)) >= 5) {
              J1collisionTrue = 1;
              J1StepM = J1encPos.read() / J1encMult;
            }
          }
          if (J2LoopMode == 0) {
            if (abs((J2EncSteps - J2StepM)) >= 5) {
              J2collisionTrue = 1;
              J2StepM = J2encPos.read() / J2encMult;
            }
          }
          if (J3LoopMode == 0) {
            if (abs((J3EncSteps - J3StepM)) >= 5) {
              J3collisionTrue = 1;
              J3StepM = J3encPos.read() / J3encMult;
            }
          }
          if (J4LoopMode == 0) {
            if (abs((J4EncSteps - J4StepM)) >= 5) {
              J4collisionTrue = 1;
              J4StepM = J4encPos.read() / J4encMult;
            }
          }
          if (J5LoopMode == 0) {
            if (abs((J5EncSteps - J5StepM)) >= 5) {
              J5collisionTrue = 1;
              J5StepM = J5encPos.read() / J5encMult;
            }
          }
          if (J6LoopMode == 0) {
            if (abs((J6EncSteps - J6StepM)) >= 5) {
              J6collisionTrue = 1;
              J6StepM = J6encPos.read() / J6encMult;
            }
          }
          updatePos();
        }

        //stop loop if any serial command is recieved - but the expected command is "S" to stop the loop.

        char recieved = Serial.read();
        inData += recieved;
        if (recieved == '\n') {
          break;
        }

        //end loop
      }

      TotalCollision = J1collisionTrue + J2collisionTrue + J3collisionTrue + J4collisionTrue + J5collisionTrue + J6collisionTrue;
      if (TotalCollision > 0) {
        flag = "EC" + String(J1collisionTrue) + String(J2collisionTrue) + String(J3collisionTrue) + String(J4collisionTrue) + String(J5collisionTrue) + String(J6collisionTrue);
      }

      //send move command if no axis limit error
      if (TotalAxisFault == 0 && KinematicError == 0) {
        sendRobotPos();
      }
      else if (KinematicError == 1) {
        Alarm = "ER";
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      }
      else {
        Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault);
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      }

      inData = ""; // Clear recieved buffer
      ////////MOVE COMPLETE///////////
    }







    //----- LIVE JOINT JOG  ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "LJ")
    {

      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;

      int J1axisFault = 0;
      int J2axisFault = 0;
      int J3axisFault = 0;
      int J4axisFault = 0;
      int J5axisFault = 0;
      int J6axisFault = 0;
      int J7axisFault = 0;
      int J8axisFault = 0;
      int J9axisFault = 0;
      int TotalAxisFault = 0;

      bool JogInPoc = true;
      Alarm = "0";


      int VStart = inData.indexOf("V");
      int SPstart = inData.indexOf("S");
      int AcStart = inData.indexOf("Ac");
      int DcStart = inData.indexOf("Dc");
      int RmStart = inData.indexOf("Rm");
      int WristConStart = inData.indexOf("W");
      int LoopModeStart = inData.indexOf("Lm");


      float Vector = inData.substring(VStart + 1, SPstart).toFloat();
      String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
      float SpeedVal = inData.substring(SPstart + 2, AcStart).toFloat();
      float ACCspd = 100;
      float DCCspd = 100;
      float ACCramp = 100;
      WristCon = inData.substring(WristConStart + 1, LoopModeStart);
      String LoopMode = inData.substring(LoopModeStart + 2);
      LoopMode.trim();
      J1LoopMode = LoopMode.substring(0, 1).toInt();
      J2LoopMode = LoopMode.substring(1, 2).toInt();
      J3LoopMode = LoopMode.substring(2, 3).toInt();
      J4LoopMode = LoopMode.substring(3, 4).toInt();
      J5LoopMode = LoopMode.substring(4, 5).toInt();
      J6LoopMode = LoopMode.substring(5).toInt();

      inData = ""; // Clear recieved buffer

      //clear serial
      delay(5);
      Serial.println();
      updatePos();

      float J1Angle = JangleIn[0];
      float J2Angle = JangleIn[1];
      float J3Angle = JangleIn[2];
      float J4Angle = JangleIn[3];
      float J5Angle = JangleIn[4];
      float J6Angle = JangleIn[5];
      float J7Angle = J7_pos;
      float J8Angle = J8_pos;
      float J9Angle = J9_pos;
      float xyzuvw_In[6];

      while (JogInPoc == true) {

        if (Vector == 10) {
          J1Angle = JangleIn[0] - .25;
        }
        if (Vector == 11) {
          J1Angle = JangleIn[0] + .25;
        }

        if (Vector == 20) {
          J2Angle = JangleIn[1] - .25;
        }
        if (Vector == 21) {
          J2Angle = JangleIn[1] + .25;
        }

        if (Vector == 30) {
          J3Angle = JangleIn[2] - .25;
        }
        if (Vector == 31) {
          J3Angle = JangleIn[2] + .25;
        }

        if (Vector == 40) {
          J4Angle = JangleIn[3] - .25;
        }
        if (Vector == 41) {
          J4Angle = JangleIn[3] + .25;
        }

        if (Vector == 50) {
          J5Angle = JangleIn[4] - .25;
        }
        if (Vector == 51) {
          J5Angle = JangleIn[4] + .25;
        }

        if (Vector == 60) {
          J6Angle = JangleIn[5] - .25;
        }
        if (Vector == 61) {
          J6Angle = JangleIn[5] + .25;
        }
        if (Vector == 70) {
          J7Angle = J7_pos - .25;
        }
        if (Vector == 71) {
          J7Angle = J7_pos + .25;
        }
        if (Vector == 80) {
          J8Angle = J8_pos - .25;
        }
        if (Vector == 81) {
          J8Angle = J8_pos + .25;
        }
        if (Vector == 90) {
          J9Angle = J9_pos - .25;
        }
        if (Vector == 91) {
          J9Angle = J9_pos + .25;
        }

        //calc destination motor steps
        int J1futStepM = (J1Angle + J1axisLimNeg) * J1StepDeg;
        int J2futStepM = (J2Angle + J2axisLimNeg) * J2StepDeg;
        int J3futStepM = (J3Angle + J3axisLimNeg) * J3StepDeg;
        int J4futStepM = (J4Angle + J4axisLimNeg) * J4StepDeg;
        int J5futStepM = (J5Angle + J5axisLimNeg) * J5StepDeg;
        int J6futStepM = (J6Angle + J6axisLimNeg) * J6StepDeg;
        int J7futStepM = (J7Angle + J7axisLimNeg) * J7StepDeg;
        int J8futStepM = (J8Angle + J8axisLimNeg) * J8StepDeg;
        int J9futStepM = (J9Angle + J9axisLimNeg) * J9StepDeg;

        //calc delta from current to destination
        int J1stepDif = J1StepM - J1futStepM;
        int J2stepDif = J2StepM - J2futStepM;
        int J3stepDif = J3StepM - J3futStepM;
        int J4stepDif = J4StepM - J4futStepM;
        int J5stepDif = J5StepM - J5futStepM;
        int J6stepDif = J6StepM - J6futStepM;
        int J7stepDif = J7StepM - J7futStepM;
        int J8stepDif = J8StepM - J8futStepM;
        int J9stepDif = J9StepM - J9futStepM;

        //determine motor directions
        if (J1stepDif <= 0) {
          J1dir = 1;
        }
        else {
          J1dir = 0;
        }

        if (J2stepDif <= 0) {
          J2dir = 1;
        }
        else {
          J2dir = 0;
        }

        if (J3stepDif <= 0) {
          J3dir = 1;
        }
        else {
          J3dir = 0;
        }

        if (J4stepDif <= 0) {
          J4dir = 1;
        }
        else {
          J4dir = 0;
        }

        if (J5stepDif <= 0) {
          J5dir = 1;
        }
        else {
          J5dir = 0;
        }

        if (J6stepDif <= 0) {
          J6dir = 1;
        }
        else {
          J6dir = 0;
        }

        if (J7stepDif <= 0) {
          J7dir = 1;
        }
        else {
          J7dir = 0;
        }

        if (J8stepDif <= 0) {
          J8dir = 1;
        }
        else {
          J8dir = 0;
        }

        if (J9stepDif <= 0) {
          J9dir = 1;
        }
        else {
          J9dir = 0;
        }

        //determine if requested position is within axis limits
        if ((J1dir == 1 and (J1StepM + J1stepDif > J1StepLim)) or (J1dir == 0 and (J1StepM - J1stepDif < 0))) {
          J1axisFault = 1;
        }
        if ((J2dir == 1 and (J2StepM + J2stepDif > J2StepLim)) or (J2dir == 0 and (J2StepM - J2stepDif < 0))) {
          J2axisFault = 1;
        }
        if ((J3dir == 1 and (J3StepM + J3stepDif > J3StepLim)) or (J3dir == 0 and (J3StepM - J3stepDif < 0))) {
          J3axisFault = 1;
        }
        if ((J4dir == 1 and (J4StepM + J4stepDif > J4StepLim)) or (J4dir == 0 and (J4StepM - J4stepDif < 0))) {
          J4axisFault = 1;
        }
        if ((J5dir == 1 and (J5StepM + J5stepDif > J5StepLim)) or (J5dir == 0 and (J5StepM - J5stepDif < 0))) {
          J5axisFault = 1;
        }
        if ((J6dir == 1 and (J6StepM + J6stepDif > J6StepLim)) or (J6dir == 0 and (J6StepM - J6stepDif < 0))) {
          J6axisFault = 1;
        }
        if ((J7dir == 1 and (J7StepM + J7stepDif > J7StepLim)) or (J7dir == 0 and (J7StepM - J7stepDif < 0))) {
          J7axisFault = 1;
        }
        if ((J8dir == 1 and (J8StepM + J8stepDif > J8StepLim)) or (J8dir == 0 and (J8StepM - J8stepDif < 0))) {
          J8axisFault = 1;
        }
        if ((J9dir == 1 and (J9StepM + J9stepDif > J9StepLim)) or (J9dir == 0 and (J9StepM - J9stepDif < 0))) {
          J9axisFault = 1;
        }
        TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault + J7axisFault + J8axisFault + J9axisFault;

        //send move command if no axis limit error
        if (TotalAxisFault == 0 && KinematicError == 0) {
          resetEncoders();
          driveMotorsJ(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(J7stepDif), abs(J8stepDif), abs(J9stepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
          //checkEncoders();
          J1EncSteps = J1encPos.read() / J1encMult;
          J2EncSteps = J2encPos.read() / J2encMult;
          J3EncSteps = J3encPos.read() / J3encMult;
          J4EncSteps = J4encPos.read() / J4encMult;
          J5EncSteps = J5encPos.read() / J5encMult;
          J6EncSteps = J6encPos.read() / J6encMult;

          if (Vector == 10 or Vector == 11) {
            if (J1LoopMode == 0) {
              if (abs((J1EncSteps - J1StepM)) >= 5) {
                J1collisionTrue = 1;
                J1StepM = J1encPos.read() / J1encMult;
              }
            }
          }

          if (Vector == 20 or Vector == 21) {
            if (J2LoopMode == 0) {
              if (abs((J2EncSteps - J2StepM)) >= 5) {
                J2collisionTrue = 1;
                J2StepM = J2encPos.read() / J2encMult;
              }
            }
          }

          if (Vector == 30 or Vector == 31) {
            if (J3LoopMode == 0) {
              if (abs((J3EncSteps - J3StepM)) >= 5) {
                J3collisionTrue = 1;
                J3StepM = J3encPos.read() / J3encMult;
              }
            }
          }

          if (Vector == 40 or Vector == 41) {
            if (J4LoopMode == 0) {
              if (abs((J4EncSteps - J4StepM)) >= 5) {
                J4collisionTrue = 1;
                J4StepM = J4encPos.read() / J4encMult;
              }
            }
          }

          if (Vector == 50 or Vector == 51) {
            if (J5LoopMode == 0) {
              if (abs((J5EncSteps - J5StepM)) >= 5) {
                J5collisionTrue = 1;
                J5StepM = J5encPos.read() / J5encMult;
              }
            }
          }

          if (Vector == 60 or Vector == 61) {
            if (J6LoopMode == 0) {
              if (abs((J6EncSteps - J6StepM)) >= 5) {
                J6collisionTrue = 1;
                J6StepM = J6encPos.read() / J6encMult;
              }
            }
          }

          updatePos();
        }

        //stop loop if any serial command is recieved - but the expected command is "S" to stop the loop.

        char recieved = Serial.read();
        inData += recieved;
        if (recieved == '\n') {
          break;
        }

        //end loop
      }

      TotalCollision = J1collisionTrue + J2collisionTrue + J3collisionTrue + J4collisionTrue + J5collisionTrue + J6collisionTrue;
      if (TotalCollision > 0) {
        flag = "EC" + String(J1collisionTrue) + String(J2collisionTrue) + String(J3collisionTrue) + String(J4collisionTrue) + String(J5collisionTrue) + String(J6collisionTrue);
      }

      //send move command if no axis limit error
      if (TotalAxisFault == 0 && KinematicError == 0) {
        sendRobotPos();
      }
      else if (KinematicError == 1) {
        Alarm = "ER";
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      }
      else {
        Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault) + String(J7axisFault) + String(J8axisFault) + String(J9axisFault);
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      }

      inData = ""; // Clear recieved buffer
      ////////MOVE COMPLETE///////////
    }








    //----- LIVE TOOL JOG  ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "LT")
    {
      delay(5);
      Serial.println();

      updatePos();

      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;

      int J1axisFault = 0;
      int J2axisFault = 0;
      int J3axisFault = 0;
      int J4axisFault = 0;
      int J5axisFault = 0;
      int J6axisFault = 0;
      int TRaxisFault = 0;
      int TotalAxisFault = 0;

      bool JogInPoc = true;
      Alarm = "0";

      int VStart = inData.indexOf("V");
      int SPstart = inData.indexOf("S");
      int AcStart = inData.indexOf("Ac");
      int DcStart = inData.indexOf("Dc");
      int RmStart = inData.indexOf("Rm");
      int WristConStart = inData.indexOf("W");
      int LoopModeStart = inData.indexOf("Lm");

      float Vector = inData.substring(VStart + 1, SPstart).toFloat();
      String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
      float SpeedVal = inData.substring(SPstart + 2, AcStart).toFloat();
      float ACCspd = 100;
      float DCCspd = 100;
      float ACCramp = 100;
      WristCon = inData.substring(WristConStart + 1, LoopModeStart);
      String LoopMode = inData.substring(LoopModeStart + 2);
      LoopMode.trim();
      J1LoopMode = LoopMode.substring(0, 1).toInt();
      J2LoopMode = LoopMode.substring(1, 2).toInt();
      J3LoopMode = LoopMode.substring(2, 3).toInt();
      J4LoopMode = LoopMode.substring(3, 4).toInt();
      J5LoopMode = LoopMode.substring(4, 5).toInt();
      J6LoopMode = LoopMode.substring(5).toInt();

      inData = ""; // Clear recieved buffer

      JangleIn[0] = (J1StepM - J1zeroStep) / J1StepDeg;
      JangleIn[1] = (J2StepM - J2zeroStep) / J2StepDeg;
      JangleIn[2] = (J3StepM - J3zeroStep) / J3StepDeg;
      JangleIn[3] = (J4StepM - J4zeroStep) / J4StepDeg;
      JangleIn[4] = (J5StepM - J5zeroStep) / J5StepDeg;
      JangleIn[5] = (J6StepM - J6zeroStep) / J6StepDeg;

      while (JogInPoc == true) {

        if (Vector == 10) {
          Tool_MatrixRev(Xtool - JogStepInc, Ytool, Ztool, RZtool, RYtool, RXtool);
        }
        else if (Vector == 11) {
          Tool_MatrixRev(Xtool + JogStepInc, Ytool, Ztool, RZtool, RYtool, RXtool);
        }
        else if (Vector == 20) {
          Tool_MatrixRev(Xtool, Ytool - JogStepInc, Ztool, RZtool, RYtool, RXtool);
        }
        else if (Vector == 21) {
          Tool_MatrixRev(Xtool, Ytool + JogStepInc, Ztool, RZtool, RYtool, RXtool);
        }
        else if (Vector == 30) {
          Tool_MatrixRev(Xtool, Ytool, Ztool - JogStepInc, RZtool, RYtool, RXtool);
        }
        else if (Vector == 31) {
          Tool_MatrixRev(Xtool, Ytool, Ztool + JogStepInc, RZtool, RYtool, RXtool);
        }
        else if (Vector == 40) {
          Tool_MatrixRev(Xtool, Ytool, Ztool, RZtool - JogStepInc, RYtool, RXtool);
        }
        else if (Vector == 41) {
          Tool_MatrixRev(Xtool, Ytool, Ztool, RZtool + JogStepInc, RYtool, RXtool);
        }
        else if (Vector == 50) {
          Tool_MatrixRev(Xtool, Ytool, Ztool, RZtool, RYtool - JogStepInc, RXtool);
        }
        else if (Vector == 51) {
          Tool_MatrixRev(Xtool, Ytool, Ztool, RZtool, RYtool + JogStepInc, RXtool);
        }
        else if (Vector == 60) {
          Tool_MatrixRev(Xtool, Ytool, Ztool, RZtool, RYtool, RXtool - JogStepInc);
        }
        else if (Vector == 61) {
          Tool_MatrixRev(Xtool, Ytool, Ztool, RZtool, RYtool, RXtool + JogStepInc);
        }


        JangleIn[0] = (J1StepM - J1zeroStep) / J1StepDeg;
        JangleIn[1] = (J2StepM - J2zeroStep) / J2StepDeg;
        JangleIn[2] = (J3StepM - J3zeroStep) / J3StepDeg;
        JangleIn[3] = (J4StepM - J4zeroStep) / J4StepDeg;
        JangleIn[4] = (J5StepM - J5zeroStep) / J5StepDeg;
        JangleIn[5] = (J6StepM - J6zeroStep) / J6StepDeg;

        if (JangleIn[4] > 0) {
          WristCon = "F";
        }
        else {
          WristCon = "N";
        }

        SolveFowardKinematic;

        xyzuvw_In[0] = xyzuvw_Out[0];
        xyzuvw_In[1] = xyzuvw_Out[1];
        xyzuvw_In[2] = xyzuvw_Out[2];
        xyzuvw_In[3] = xyzuvw_Out[3];
        xyzuvw_In[4] = xyzuvw_Out[4];
        xyzuvw_In[5] = xyzuvw_Out[5];

        SolveInverseKinematic();

        Tool_MatrixRev(Xtool, Ytool, Ztool, RZtool, RYtool, RXtool);

        //calc destination motor steps
        int J1futStepM = (JangleOut[0] + J1axisLimNeg) * J1StepDeg;
        int J2futStepM = (JangleOut[1] + J2axisLimNeg) * J2StepDeg;
        int J3futStepM = (JangleOut[2] + J3axisLimNeg) * J3StepDeg;
        int J4futStepM = (JangleOut[3] + J4axisLimNeg) * J4StepDeg;
        int J5futStepM = (JangleOut[4] + J5axisLimNeg) * J5StepDeg;
        int J6futStepM = (JangleOut[5] + J6axisLimNeg) * J6StepDeg;

        //calc delta from current to destination
        int J1stepDif = J1StepM - J1futStepM;
        int J2stepDif = J2StepM - J2futStepM;
        int J3stepDif = J3StepM - J3futStepM;
        int J4stepDif = J4StepM - J4futStepM;
        int J5stepDif = J5StepM - J5futStepM;
        int J6stepDif = J6StepM - J6futStepM;
        int J7stepDif = 0;
        int J8stepDif = 0;
        int J9stepDif = 0;

        //determine motor directions
        if (J1stepDif <= 0) {
          J1dir = 1;
        }
        else {
          J1dir = 0;
        }

        if (J2stepDif <= 0) {
          J2dir = 1;
        }
        else {
          J2dir = 0;
        }

        if (J3stepDif <= 0) {
          J3dir = 1;
        }
        else {
          J3dir = 0;
        }

        if (J4stepDif <= 0) {
          J4dir = 1;
        }
        else {
          J4dir = 0;
        }

        if (J5stepDif <= 0) {
          J5dir = 1;
        }
        else {
          J5dir = 0;
        }

        if (J6stepDif <= 0) {
          J6dir = 1;
        }
        else {
          J6dir = 0;
        }

        J7dir = 0;
        J8dir = 0;
        J9dir = 0;


        //determine if requested position is within axis limits
        if ((J1dir == 1 and (J1StepM + J1stepDif > J1StepLim)) or (J1dir == 0 and (J1StepM - J1stepDif < 0))) {
          J1axisFault = 1;
        }
        if ((J2dir == 1 and (J2StepM + J2stepDif > J2StepLim)) or (J2dir == 0 and (J2StepM - J2stepDif < 0))) {
          J2axisFault = 1;
        }
        if ((J3dir == 1 and (J3StepM + J3stepDif > J3StepLim)) or (J3dir == 0 and (J3StepM - J3stepDif < 0))) {
          J3axisFault = 1;
        }
        if ((J4dir == 1 and (J4StepM + J4stepDif > J4StepLim)) or (J4dir == 0 and (J4StepM - J4stepDif < 0))) {
          J4axisFault = 1;
        }
        if ((J5dir == 1 and (J5StepM + J5stepDif > J5StepLim)) or (J5dir == 0 and (J5StepM - J5stepDif < 0))) {
          J5axisFault = 1;
        }
        if ((J6dir == 1 and (J6StepM + J6stepDif > J6StepLim)) or (J6dir == 0 and (J6StepM - J6stepDif < 0))) {
          J6axisFault = 1;
        }
        TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault;


        //send move command if no axis limit error
        if (TotalAxisFault == 0 && KinematicError == 0) {
          resetEncoders();
          driveMotorsJ(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(J7stepDif), abs(J8stepDif), abs(J9stepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
          //checkEncoders();
          J1EncSteps = J1encPos.read() / J1encMult;
          J2EncSteps = J2encPos.read() / J2encMult;
          J3EncSteps = J3encPos.read() / J3encMult;
          J4EncSteps = J4encPos.read() / J4encMult;
          J5EncSteps = J5encPos.read() / J5encMult;
          J6EncSteps = J6encPos.read() / J6encMult;

          if (J1LoopMode == 0) {
            if (abs((J1EncSteps - J1StepM)) >= 5) {
              J1collisionTrue = 1;
              J1StepM = J1encPos.read() / J1encMult;
            }
          }
          if (J2LoopMode == 0) {
            if (abs((J2EncSteps - J2StepM)) >= 5) {
              J2collisionTrue = 1;
              J2StepM = J2encPos.read() / J2encMult;
            }
          }
          if (J3LoopMode == 0) {
            if (abs((J3EncSteps - J3StepM)) >= 5) {
              J3collisionTrue = 1;
              J3StepM = J3encPos.read() / J3encMult;
            }
          }
          if (J4LoopMode == 0) {
            if (abs((J4EncSteps - J4StepM)) >= 5) {
              J4collisionTrue = 1;
              J4StepM = J4encPos.read() / J4encMult;
            }
          }
          if (J5LoopMode == 0) {
            if (abs((J5EncSteps - J5StepM)) >= 5) {
              J5collisionTrue = 1;
              J5StepM = J5encPos.read() / J5encMult;
            }
          }
          if (J6LoopMode == 0) {
            if (abs((J6EncSteps - J6StepM)) >= 5) {
              J6collisionTrue = 1;
              J6StepM = J6encPos.read() / J6encMult;
            }
          }

          updatePos();
        }

        //stop loop if any serial command is recieved - but the expected command is "S" to stop the loop.

        char recieved = Serial.read();
        inData += recieved;
        if (recieved == '\n') {
          break;
        }

        //end loop
      }

      TotalCollision = J1collisionTrue + J2collisionTrue + J3collisionTrue + J4collisionTrue + J5collisionTrue + J6collisionTrue;
      if (TotalCollision > 0) {
        flag = "EC" + String(J1collisionTrue) + String(J2collisionTrue) + String(J3collisionTrue) + String(J4collisionTrue) + String(J5collisionTrue) + String(J6collisionTrue);
      }

      //send move command if no axis limit error
      if (TotalAxisFault == 0 && KinematicError == 0) {
        sendRobotPos();
      }
      else if (KinematicError == 1) {
        Alarm = "ER";
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      }
      else {
        Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault);
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      }

      inData = ""; // Clear recieved buffer
      ////////MOVE COMPLETE///////////
    }



    //----- Jog T ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "JT")
    {
      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;

      int J1axisFault = 0;
      int J2axisFault = 0;
      int J3axisFault = 0;
      int J4axisFault = 0;
      int J5axisFault = 0;
      int J6axisFault = 0;
      int TotalAxisFault = 0;

      String Alarm = "0";

      int SPstart = inData.indexOf('S');
      int AcStart = inData.indexOf('G');
      int DcStart = inData.indexOf('H');
      int RmStart = inData.indexOf('I');
      int LoopModeStart = inData.indexOf("Lm");

      String Dir = inData.substring(0, 2); // this should be Z0 or Z1
      float Dist = inData.substring(2, SPstart).toFloat();
      String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
      float SpeedVal = inData.substring(SPstart + 3, AcStart).toFloat();
      float ACCspd = inData.substring(AcStart + 1, DcStart).toInt();
      float DCCspd = inData.substring(DcStart + 1, RmStart).toInt();
      float ACCramp = inData.substring(RmStart + 1, LoopModeStart).toInt();
      String LoopMode = inData.substring(LoopModeStart + 2);
      LoopMode.trim();
      J1LoopMode = LoopMode.substring(0, 1).toInt();
      J2LoopMode = LoopMode.substring(1, 2).toInt();
      J3LoopMode = LoopMode.substring(2, 3).toInt();
      J4LoopMode = LoopMode.substring(3, 4).toInt();
      J5LoopMode = LoopMode.substring(4, 5).toInt();
      J6LoopMode = LoopMode.substring(5).toInt();

      if (Dir == "X0") {
        Tool_MatrixRev(Xtool + Dist, Ytool, Ztool, RZtool, RYtool, RXtool);
      }
      else if (Dir == "X1") {
        Tool_MatrixRev(Xtool - Dist, Ytool, Ztool, RZtool, RYtool, RXtool);
      }
      else if (Dir == "Y0") {
        Tool_MatrixRev(Xtool, Ytool + Dist, Ztool, RZtool, RYtool, RXtool);
      }
      else if (Dir == "Y1") {
        Tool_MatrixRev(Xtool, Ytool - Dist, Ztool, RZtool, RYtool, RXtool);
      }
      else if (Dir == "Z0") {
        Tool_MatrixRev(Xtool, Ytool, Ztool + Dist, RZtool, RYtool, RXtool);
      }
      else if (Dir == "Z1") {
        Tool_MatrixRev(Xtool, Ytool, Ztool - Dist, RZtool, RYtool, RXtool);
      }
      else if (Dir == "R0") {
        Tool_MatrixRev(Xtool, Ytool, Ztool, RZtool + Dist, RYtool, RXtool);
      }
      else if (Dir == "R1") {
        Tool_MatrixRev(Xtool, Ytool, Ztool, RZtool - Dist, RYtool, RXtool);
      }
      else if (Dir == "P0") {
        Tool_MatrixRev(Xtool, Ytool, Ztool, RZtool, RYtool + Dist, RXtool);
      }
      else if (Dir == "P1") {
        Tool_MatrixRev(Xtool, Ytool, Ztool, RZtool, RYtool - Dist, RXtool);
      }
      else if (Dir == "W0") {
        Tool_MatrixRev(Xtool, Ytool, Ztool, RZtool, RYtool, RXtool + Dist);
      }
      else if (Dir == "W1") {
        Tool_MatrixRev(Xtool, Ytool, Ztool, RZtool, RYtool, RXtool - Dist);
      }

      JangleIn[0] = (J1StepM - J1zeroStep) / J1StepDeg;
      JangleIn[1] = (J2StepM - J2zeroStep) / J2StepDeg;
      JangleIn[2] = (J3StepM - J3zeroStep) / J3StepDeg;
      JangleIn[3] = (J4StepM - J4zeroStep) / J4StepDeg;
      JangleIn[4] = (J5StepM - J5zeroStep) / J5StepDeg;
      JangleIn[5] = (J6StepM - J6zeroStep) / J6StepDeg;

      if (JangleIn[4] > 0) {
        WristCon = "F";
      }
      else {
        WristCon = "N";
      }

      SolveFowardKinematic;

      xyzuvw_In[0] = xyzuvw_Out[0];
      xyzuvw_In[1] = xyzuvw_Out[1];
      xyzuvw_In[2] = xyzuvw_Out[2];
      xyzuvw_In[3] = xyzuvw_Out[3];
      xyzuvw_In[4] = xyzuvw_Out[4];
      xyzuvw_In[5] = xyzuvw_Out[5];

      SolveInverseKinematic();

      Tool_MatrixRev(Xtool, Ytool, Ztool, RZtool, RYtool, RXtool);

      //calc destination motor steps
      int J1futStepM = (JangleOut[0] + J1axisLimNeg) * J1StepDeg;
      int J2futStepM = (JangleOut[1] + J2axisLimNeg) * J2StepDeg;
      int J3futStepM = (JangleOut[2] + J3axisLimNeg) * J3StepDeg;
      int J4futStepM = (JangleOut[3] + J4axisLimNeg) * J4StepDeg;
      int J5futStepM = (JangleOut[4] + J5axisLimNeg) * J5StepDeg;
      int J6futStepM = (JangleOut[5] + J6axisLimNeg) * J6StepDeg;

      //calc delta from current to destination
      int J1stepDif = J1StepM - J1futStepM;
      int J2stepDif = J2StepM - J2futStepM;
      int J3stepDif = J3StepM - J3futStepM;
      int J4stepDif = J4StepM - J4futStepM;
      int J5stepDif = J5StepM - J5futStepM;
      int J6stepDif = J6StepM - J6futStepM;
      int J7stepDif = 0;
      int J8stepDif = 0;
      int J9stepDif = 0;

      //determine motor directions
      if (J1stepDif <= 0) {
        J1dir = 1;
      }
      else {
        J1dir = 0;
      }

      if (J2stepDif <= 0) {
        J2dir = 1;
      }
      else {
        J2dir = 0;
      }

      if (J3stepDif <= 0) {
        J3dir = 1;
      }
      else {
        J3dir = 0;
      }

      if (J4stepDif <= 0) {
        J4dir = 1;
      }
      else {
        J4dir = 0;
      }

      if (J5stepDif <= 0) {
        J5dir = 1;
      }
      else {
        J5dir = 0;
      }

      if (J6stepDif <= 0) {
        J6dir = 1;
      }
      else {
        J6dir = 0;
      }

      J7dir = 0;

      //determine if requested position is within axis limits
      if ((J1dir == 1 and (J1StepM + J1stepDif > J1StepLim)) or (J1dir == 0 and (J1StepM - J1stepDif < 0))) {
        J1axisFault = 1;
      }
      if ((J2dir == 1 and (J2StepM + J2stepDif > J2StepLim)) or (J2dir == 0 and (J2StepM - J2stepDif < 0))) {
        J2axisFault = 1;
      }
      if ((J3dir == 1 and (J3StepM + J3stepDif > J3StepLim)) or (J3dir == 0 and (J3StepM - J3stepDif < 0))) {
        J3axisFault = 1;
      }
      if ((J4dir == 1 and (J4StepM + J4stepDif > J4StepLim)) or (J4dir == 0 and (J4StepM - J4stepDif < 0))) {
        J4axisFault = 1;
      }
      if ((J5dir == 1 and (J5StepM + J5stepDif > J5StepLim)) or (J5dir == 0 and (J5StepM - J5stepDif < 0))) {
        J5axisFault = 1;
      }
      if ((J6dir == 1 and (J6StepM + J6stepDif > J6StepLim)) or (J6dir == 0 and (J6StepM - J6stepDif < 0))) {
        J6axisFault = 1;
      }
      TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault;


      //send move command if no axis limit error
      if (TotalAxisFault == 0 && KinematicError == 0) {
        resetEncoders();
        driveMotorsJ(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(J7stepDif), abs(J8stepDif), abs(J9stepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
        checkEncoders();
        sendRobotPos();
      }
      else if (KinematicError == 1) {
        Alarm = "ER";
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      }
      else {
        Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault);
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      }

      inData = ""; // Clear recieved buffer
      ////////MOVE COMPLETE///////////
    }













    //----- MOVE V ------ VISION OFFSET ----------------------------------
    //-----------------------------------------------------------------------
    if (function == "MV")
    {
      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;

      int J1axisFault = 0;
      int J2axisFault = 0;
      int J3axisFault = 0;
      int J4axisFault = 0;
      int J5axisFault = 0;
      int J6axisFault = 0;
      int J7axisFault = 0;
      int J8axisFault = 0;
      int J9axisFault = 0;
      int TotalAxisFault = 0;

      int xStart = inData.indexOf("X");
      int yStart = inData.indexOf("Y");
      int zStart = inData.indexOf("Z");
      int rzStart = inData.indexOf("Rz");
      int ryStart = inData.indexOf("Ry");
      int rxStart = inData.indexOf("Rx");
      int J7Start = inData.indexOf("J7");
      int J8Start = inData.indexOf("J8");
      int J9Start = inData.indexOf("J9");
      int SPstart = inData.indexOf("S");
      int AcStart = inData.indexOf("Ac");
      int DcStart = inData.indexOf("Dc");
      int RmStart = inData.indexOf("Rm");
      int RndStart = inData.indexOf("Rnd");
      int WristConStart = inData.indexOf("W");
      int VisRotStart = inData.indexOf("Vr");
      int LoopModeStart = inData.indexOf("Lm");

      xyzuvw_In[0] = inData.substring(xStart + 1, yStart).toFloat();
      xyzuvw_In[1] = inData.substring(yStart + 1, zStart).toFloat();
      xyzuvw_In[2] = inData.substring(zStart + 1, rzStart).toFloat();
      xyzuvw_In[3] = inData.substring(rzStart + 2, ryStart).toFloat();
      xyzuvw_In[4] = inData.substring(ryStart + 2, rxStart).toFloat();
      xyzuvw_In[5] = inData.substring(rxStart + 2, J7Start).toFloat();
      J7_In = inData.substring(J7Start + 2, J8Start).toFloat();
      J8_In = inData.substring(J8Start + 2, J9Start).toFloat();
      J9_In = inData.substring(J9Start + 2, SPstart).toFloat();

      String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
      float SpeedVal = inData.substring(SPstart + 2, AcStart).toFloat();
      float ACCspd = inData.substring(AcStart + 2, DcStart).toFloat();
      float DCCspd = inData.substring(DcStart + 2, RmStart).toFloat();
      float ACCramp = inData.substring(RmStart + 2, RndStart).toFloat();
      float Rounding = inData.substring(RndStart + 3, WristConStart).toFloat();
      WristCon = inData.substring(WristConStart + 1, VisRotStart);
      float VisRot = inData.substring(VisRotStart + 2, LoopModeStart).toFloat();
      String LoopMode = inData.substring(LoopModeStart + 2);
      LoopMode.trim();
      J1LoopMode = LoopMode.substring(0, 1).toInt();
      J2LoopMode = LoopMode.substring(1, 2).toInt();
      J3LoopMode = LoopMode.substring(2, 3).toInt();
      J4LoopMode = LoopMode.substring(3, 4).toInt();
      J5LoopMode = LoopMode.substring(4, 5).toInt();
      J6LoopMode = LoopMode.substring(5).toInt();

      // offset tool rotation by the found vision angle
      Tool_MatrixRev(Xtool, Ytool, Ztool, RZtool - VisRot, RYtool, RXtool);
      debug = String(LoopMode);

      //solve kinematics
      SolveInverseKinematic();

      //calc destination motor steps
      int J1futStepM = (JangleOut[0] + J1axisLimNeg) * J1StepDeg;
      int J2futStepM = (JangleOut[1] + J2axisLimNeg) * J2StepDeg;
      int J3futStepM = (JangleOut[2] + J3axisLimNeg) * J3StepDeg;
      int J4futStepM = (JangleOut[3] + J4axisLimNeg) * J4StepDeg;
      int J5futStepM = (JangleOut[4] + J5axisLimNeg) * J5StepDeg;
      int J6futStepM = (JangleOut[5] + J6axisLimNeg) * J6StepDeg;
      int J7futStepM = (J7_In + J7axisLimNeg) * J7StepDeg;
      int J8futStepM = (J8_In + J8axisLimNeg) * J8StepDeg;
      int J9futStepM = (J9_In + J9axisLimNeg) * J9StepDeg;


      //calc delta from current to destination
      int J1stepDif = J1StepM - J1futStepM;
      int J2stepDif = J2StepM - J2futStepM;
      int J3stepDif = J3StepM - J3futStepM;
      int J4stepDif = J4StepM - J4futStepM;
      int J5stepDif = J5StepM - J5futStepM;
      int J6stepDif = J6StepM - J6futStepM;
      int J7stepDif = J7StepM - J7futStepM;
      int J8stepDif = J8StepM - J8futStepM;
      int J9stepDif = J9StepM - J9futStepM;

      // put tool roation back where it was
      Tool_MatrixRev(Xtool, Ytool, Ztool, RZtool, RYtool, RXtool);

      //determine motor directions
      if (J1stepDif <= 0) {
        J1dir = 1;
      }
      else {
        J1dir = 0;
      }

      if (J2stepDif <= 0) {
        J2dir = 1;
      }
      else {
        J2dir = 0;
      }

      if (J3stepDif <= 0) {
        J3dir = 1;
      }
      else {
        J3dir = 0;
      }

      if (J4stepDif <= 0) {
        J4dir = 1;
      }
      else {
        J4dir = 0;
      }

      if (J5stepDif <= 0) {
        J5dir = 1;
      }
      else {
        J5dir = 0;
      }

      if (J6stepDif <= 0) {
        J6dir = 1;
      }
      else {
        J6dir = 0;
      }

      if (J7stepDif <= 0) {
        J7dir = 1;
      }
      else {
        J7dir = 0;
      }

      if (J8stepDif <= 0) {
        J8dir = 1;
      }
      else {
        J8dir = 0;
      }

      if (J9stepDif <= 0) {
        J9dir = 1;
      }
      else {
        J9dir = 0;
      }



      //determine if requested position is within axis limits
      if ((J1dir == 1 and (J1StepM + J1stepDif > J1StepLim)) or (J1dir == 0 and (J1StepM - J1stepDif < 0))) {
        J1axisFault = 1;
      }
      if ((J2dir == 1 and (J2StepM + J2stepDif > J2StepLim)) or (J2dir == 0 and (J2StepM - J2stepDif < 0))) {
        J2axisFault = 1;
      }
      if ((J3dir == 1 and (J3StepM + J3stepDif > J3StepLim)) or (J3dir == 0 and (J3StepM - J3stepDif < 0))) {
        J3axisFault = 1;
      }
      if ((J4dir == 1 and (J4StepM + J4stepDif > J4StepLim)) or (J4dir == 0 and (J4StepM - J4stepDif < 0))) {
        J4axisFault = 1;
      }
      if ((J5dir == 1 and (J5StepM + J5stepDif > J5StepLim)) or (J5dir == 0 and (J5StepM - J5stepDif < 0))) {
        J5axisFault = 1;
      }
      if ((J6dir == 1 and (J6StepM + J6stepDif > J6StepLim)) or (J6dir == 0 and (J6StepM - J6stepDif < 0))) {
        J6axisFault = 1;
      }
      if ((J7dir == 1 and (J7StepM + J7stepDif > J7StepLim)) or (J7dir == 0 and (J7StepM - J7stepDif < 0))) {
        J7axisFault = 1;
      }
      if ((J8dir == 1 and (J8StepM + J8stepDif > J8StepLim)) or (J8dir == 0 and (J8StepM - J8stepDif < 0))) {
        J8axisFault = 1;
      }
      if ((J9dir == 1 and (J9StepM + J9stepDif > J9StepLim)) or (J9dir == 0 and (J9StepM - J9stepDif < 0))) {
        J9axisFault = 1;
      }
      TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault + J7axisFault + J8axisFault + J9axisFault;


      //send move command if no axis limit error
      if (TotalAxisFault == 0 && KinematicError == 0) {
        resetEncoders();
        driveMotorsJ(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(J7stepDif), abs(J8stepDif), abs(J9stepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
        checkEncoders();
        sendRobotPos();
      }
      else if (KinematicError == 1) {
        Alarm = "ER";
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      }
      else {
        Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault) + String(J7axisFault) + String(J8axisFault) + String(J9axisFault);
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      }



      inData = ""; // Clear recieved buffer
      ////////MOVE COMPLETE///////////
    }








    //----- MOVE IN JOINTS ROTATION  ---------------------------------------------------
    //-----------------------------------------------------------------------

    if (function == "RJ") {
      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;

      int J1axisFault = 0;
      int J2axisFault = 0;
      int J3axisFault = 0;
      int J4axisFault = 0;
      int J5axisFault = 0;
      int J6axisFault = 0;
      int J7axisFault = 0;
      int J8axisFault = 0;
      int J9axisFault = 0;
      int TotalAxisFault = 0;

      int J1stepStart = inData.indexOf("A");
      int J2stepStart = inData.indexOf("B");
      int J3stepStart = inData.indexOf("C");
      int J4stepStart = inData.indexOf("D");
      int J5stepStart = inData.indexOf("E");
      int J6stepStart = inData.indexOf("F");
      int J7Start = inData.indexOf("J7");
      int J8Start = inData.indexOf("J8");
      int J9Start = inData.indexOf("J9");
      int SPstart = inData.indexOf("S");
      int AcStart = inData.indexOf("Ac");
      int DcStart = inData.indexOf("Dc");
      int RmStart = inData.indexOf("Rm");
      int WristConStart = inData.indexOf("W");
      int LoopModeStart = inData.indexOf("Lm");

      float J1Angle;
      float J2Angle;
      float J3Angle;
      float J4Angle;
      float J5Angle;
      float J6Angle;

      J1Angle = inData.substring(J1stepStart + 1, J2stepStart).toFloat();
      J2Angle = inData.substring(J2stepStart + 1, J3stepStart).toFloat();
      J3Angle = inData.substring(J3stepStart + 1, J4stepStart).toFloat();
      J4Angle = inData.substring(J4stepStart + 1, J5stepStart).toFloat();
      J5Angle = inData.substring(J5stepStart + 1, J6stepStart).toFloat();
      J6Angle = inData.substring(J6stepStart + 1, J7Start).toFloat();
      J7_In = inData.substring(J7Start + 2, J8Start).toFloat();
      J8_In = inData.substring(J8Start + 2, J9Start).toFloat();
      J9_In = inData.substring(J9Start + 2, SPstart).toFloat();
      String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
      float SpeedVal = inData.substring(SPstart + 2, AcStart).toFloat();
      float ACCspd = inData.substring(AcStart + 2, DcStart).toFloat();
      float DCCspd = inData.substring(DcStart + 2, RmStart).toFloat();
      float ACCramp = inData.substring(RmStart + 2, WristConStart).toFloat();
      WristCon = inData.substring(WristConStart + 1, LoopModeStart);
      String LoopMode = inData.substring(LoopModeStart + 2);
      LoopMode.trim();
      J1LoopMode = LoopMode.substring(0, 1).toInt();
      J2LoopMode = LoopMode.substring(1, 2).toInt();
      J3LoopMode = LoopMode.substring(2, 3).toInt();
      J4LoopMode = LoopMode.substring(3, 4).toInt();
      J5LoopMode = LoopMode.substring(4, 5).toInt();
      J6LoopMode = LoopMode.substring(5).toInt();

      int J1futStepM = (J1Angle + J1axisLimNeg) * J1StepDeg;
      int J2futStepM = (J2Angle + J2axisLimNeg) * J2StepDeg;
      int J3futStepM = (J3Angle + J3axisLimNeg) * J3StepDeg;
      int J4futStepM = (J4Angle + J4axisLimNeg) * J4StepDeg;
      int J5futStepM = (J5Angle + J5axisLimNeg) * J5StepDeg;
      int J6futStepM = (J6Angle + J6axisLimNeg) * J6StepDeg;
      int J7futStepM = (J7_In + J7axisLimNeg) * J7StepDeg;
      int J8futStepM = (J8_In + J8axisLimNeg) * J8StepDeg;
      int J9futStepM = (J9_In + J9axisLimNeg) * J9StepDeg;

      //calc delta from current to destination
      int J1stepDif = J1StepM - J1futStepM;
      int J2stepDif = J2StepM - J2futStepM;
      int J3stepDif = J3StepM - J3futStepM;
      int J4stepDif = J4StepM - J4futStepM;
      int J5stepDif = J5StepM - J5futStepM;
      int J6stepDif = J6StepM - J6futStepM;
      int J7stepDif = J7StepM - J7futStepM;
      int J8stepDif = J8StepM - J8futStepM;
      int J9stepDif = J9StepM - J9futStepM;


      //determine motor directions
      if (J1stepDif <= 0) {
        J1dir = 1;
      }
      else {
        J1dir = 0;
      }

      if (J2stepDif <= 0) {
        J2dir = 1;
      }
      else {
        J2dir = 0;
      }

      if (J3stepDif <= 0) {
        J3dir = 1;
      }
      else {
        J3dir = 0;
      }

      if (J4stepDif <= 0) {
        J4dir = 1;
      }
      else {
        J4dir = 0;
      }

      if (J5stepDif <= 0) {
        J5dir = 1;
      }
      else {
        J5dir = 0;
      }

      if (J6stepDif <= 0) {
        J6dir = 1;
      }
      else {
        J6dir = 0;
      }

      if (J7stepDif <= 0) {
        J7dir = 1;
      }
      else {
        J7dir = 0;
      }

      if (J8stepDif <= 0) {
        J8dir = 1;
      }
      else {
        J8dir = 0;
      }

      if (J9stepDif <= 0) {
        J9dir = 1;
      }
      else {
        J9dir = 0;
      }


      //determine if requested position is within axis limits
      if ((J1dir == 1 and (J1StepM + J1stepDif > J1StepLim)) or (J1dir == 0 and (J1StepM - J1stepDif < 0))) {
        J1axisFault = 1;
      }
      if ((J2dir == 1 and (J2StepM + J2stepDif > J2StepLim)) or (J2dir == 0 and (J2StepM - J2stepDif < 0))) {
        J2axisFault = 1;
      }
      if ((J3dir == 1 and (J3StepM + J3stepDif > J3StepLim)) or (J3dir == 0 and (J3StepM - J3stepDif < 0))) {
        J3axisFault = 1;
      }
      if ((J4dir == 1 and (J4StepM + J4stepDif > J4StepLim)) or (J4dir == 0 and (J4StepM - J4stepDif < 0))) {
        J4axisFault = 1;
      }
      if ((J5dir == 1 and (J5StepM + J5stepDif > J5StepLim)) or (J5dir == 0 and (J5StepM - J5stepDif < 0))) {
        J5axisFault = 1;
      }
      if ((J6dir == 1 and (J6StepM + J6stepDif > J6StepLim)) or (J6dir == 0 and (J6StepM - J6stepDif < 0))) {
        J6axisFault = 1;
      }
      if ((J7dir == 1 and (J7StepM + J7stepDif > J7StepLim)) or (J7dir == 0 and (J7StepM - J7stepDif < 0))) {
        J7axisFault = 1;
      }
      if ((J8dir == 1 and (J8StepM + J8stepDif > J8StepLim)) or (J8dir == 0 and (J8StepM - J8stepDif < 0))) {
        J8axisFault = 1;
      }
      if ((J9dir == 1 and (J9StepM + J9stepDif > J9StepLim)) or (J9dir == 0 and (J9StepM - J9stepDif < 0))) {
        J9axisFault = 1;
      }
      TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault + J7axisFault + J8axisFault + J9axisFault;


      //send move command if no axis limit error
      if (TotalAxisFault == 0 && KinematicError == 0) {
        resetEncoders();
        driveMotorsJ(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(J7stepDif), abs(J8stepDif), abs(J9stepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
        checkEncoders();
        sendRobotPos();
      }
      else if (KinematicError == 1) {
        Alarm = "ER";
        delay(5);
        Serial.println(Alarm);
      }
      else {
        Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault) + String(J7axisFault) + String(J8axisFault) + String(J9axisFault);
        delay(5);
        Serial.println(Alarm);
      }


      inData = ""; // Clear recieved buffer
      ////////MOVE COMPLETE///////////

    }



    //----- MOVE L ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "ML" and flag == "")
    {
      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;

      float curDelay;

      String nextCMDtype;
      String test;

      int J1axisFault = 0;
      int J2axisFault = 0;
      int J3axisFault = 0;
      int J4axisFault = 0;
      int J5axisFault = 0;
      int J6axisFault = 0;
      int J7axisFault = 0;
      int J8axisFault = 0;
      int J9axisFault = 0;
      int TotalAxisFault = 0;

      //String Alarm = "0";

      float curWayDis;
      float speedSP;

      float Xvect;
      float Yvect;
      float Zvect;
      float RZvect;
      float RYvect;
      float RXvect;

      int xStart = inData.indexOf("X");
      int yStart = inData.indexOf("Y");
      int zStart = inData.indexOf("Z");
      int rzStart = inData.indexOf("Rz");
      int ryStart = inData.indexOf("Ry");
      int rxStart = inData.indexOf("Rx");
      int J7Start = inData.indexOf("J7");
      int J8Start = inData.indexOf("J8");
      int J9Start = inData.indexOf("J9");
      int SPstart = inData.indexOf("S");
      int AcStart = inData.indexOf("Ac");
      int DcStart = inData.indexOf("Dc");
      int RmStart = inData.indexOf("Rm");
      int RndStart = inData.indexOf("Rnd");
      int WristConStart = inData.indexOf("W");
      int LoopModeStart = inData.indexOf("Lm");
      int DisWristStart = inData.indexOf("Q");


      xyzuvw_Temp[0] = inData.substring(xStart + 1, yStart).toFloat();
      xyzuvw_Temp[1] = inData.substring(yStart + 1, zStart).toFloat();
      xyzuvw_Temp[2] = inData.substring(zStart + 1, rzStart).toFloat();
      xyzuvw_Temp[3] = inData.substring(rzStart + 2, ryStart).toFloat();
      xyzuvw_Temp[4] = inData.substring(ryStart + 2, rxStart).toFloat();
      xyzuvw_Temp[5] = inData.substring(rxStart + 2, J7Start).toFloat();
      J7_In = inData.substring(J7Start + 2, J8Start).toFloat();
      J8_In = inData.substring(J8Start + 2, J9Start).toFloat();
      J9_In = inData.substring(J9Start + 2, SPstart).toFloat();

      String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
      float SpeedVal = inData.substring(SPstart + 2, AcStart).toFloat();
      float ACCspd = inData.substring(AcStart + 2, DcStart).toFloat();
      float DCCspd = inData.substring(DcStart + 2, RmStart).toFloat();
      float ACCramp = inData.substring(RmStart + 2, RndStart).toFloat();
      float Rounding = inData.substring(RndStart + 3, WristConStart).toFloat();
      WristCon = inData.substring(WristConStart + 1, LoopModeStart);
      String LoopMode = inData.substring(LoopModeStart + 2, DisWristStart);
      String DisWrist = inData.substring(DisWristStart + 1);
      DisWrist.trim();

      J1LoopMode = LoopMode.substring(0, 1).toInt();
      J2LoopMode = LoopMode.substring(1, 2).toInt();
      J3LoopMode = LoopMode.substring(2, 3).toInt();
      J4LoopMode = LoopMode.substring(3, 4).toInt();
      J5LoopMode = LoopMode.substring(4, 5).toInt();
      J6LoopMode = LoopMode.substring(5).toInt();


      ///// rounding logic /////
      if (cmdBuffer2 != "") {
        checkData = cmdBuffer2;
        checkData.trim();
        nextCMDtype = checkData.substring(0, 1);
        checkData = checkData.substring(2);
      }
      if (splineTrue == true and Rounding > 0 and nextCMDtype == "M") {
        //calculate new end point before rounding arc
        updatePos();
        //vector
        float Xvect = xyzuvw_Temp[0] - xyzuvw_Out[0];
        float Yvect = xyzuvw_Temp[1] - xyzuvw_Out[1];
        float Zvect = xyzuvw_Temp[2] - xyzuvw_Out[2];
        float RZvect = xyzuvw_Temp[3] - xyzuvw_Out[3];
        float RYvect = xyzuvw_Temp[4] - xyzuvw_Out[4];
        float RXvect = xyzuvw_Temp[5] - xyzuvw_Out[5];
        //start pos
        float Xstart = xyzuvw_Out[0];
        float Ystart = xyzuvw_Out[1];
        float Zstart = xyzuvw_Out[2];
        float RZstart = xyzuvw_Out[3];
        float RYstart = xyzuvw_Out[4];
        float RXstart = xyzuvw_Out[5];
        //line dist
        float lineDist = pow((pow((Xvect), 2) + pow((Yvect), 2) + pow((Zvect), 2) + pow((RZvect), 2) + pow((RYvect), 2) + pow((RXvect), 2)), .5);
        if (Rounding > (lineDist * .45)) {
          Rounding = lineDist * .45;
        }
        float newDistPerc = 1 - (Rounding / lineDist);
        //cropped destination (new end point before rounding arc)
        xyzuvw_In[0] = Xstart + (Xvect * newDistPerc);
        xyzuvw_In[1] = Ystart + (Yvect * newDistPerc);
        xyzuvw_In[2] = Zstart + (Zvect * newDistPerc);
        xyzuvw_In[3] = RZstart + (RZvect * newDistPerc);
        xyzuvw_In[4] = RYstart + (RYvect * newDistPerc);
        xyzuvw_In[5] = RXstart + (RXvect * newDistPerc);
        xStart = checkData.indexOf("X");
        yStart = checkData.indexOf("Y");
        zStart = checkData.indexOf("Z");
        rzStart = checkData.indexOf("Rz");
        ryStart = checkData.indexOf("Ry");
        rxStart = checkData.indexOf("Rx");
        J7Start = checkData.indexOf("J7");
        J8Start = checkData.indexOf("J8");
        J9Start = checkData.indexOf("J9");
        //get arc end point (next move in queue)
        rndArcEnd[0] = checkData.substring(xStart + 1, yStart).toFloat();
        rndArcEnd[1] = checkData.substring(yStart + 1, zStart).toFloat();
        rndArcEnd[2] = checkData.substring(zStart + 1, rzStart).toFloat();
        rndArcEnd[3] = checkData.substring(rzStart + 2, ryStart).toFloat();
        rndArcEnd[4] = checkData.substring(ryStart + 2, rxStart).toFloat();
        rndArcEnd[5] = checkData.substring(rxStart + 2, J7Start).toFloat();
        //arc vector
        Xvect = rndArcEnd[0] - xyzuvw_Temp[0];
        Yvect = rndArcEnd[1] - xyzuvw_Temp[1];
        Zvect = rndArcEnd[2] - xyzuvw_Temp[2];
        RZvect = rndArcEnd[3] - xyzuvw_Temp[3];
        RYvect = rndArcEnd[4] - xyzuvw_Temp[4];
        RXvect = rndArcEnd[5] - xyzuvw_Temp[5];
        //end arc start pos
        Xstart = xyzuvw_Temp[0];
        Ystart = xyzuvw_Temp[1];
        Zstart = xyzuvw_Temp[2];
        RZstart = xyzuvw_Temp[3];
        RYstart = xyzuvw_Temp[4];
        RXstart = xyzuvw_Temp[5];
        //line dist
        lineDist = pow((pow((Xvect), 2) + pow((Yvect), 2) + pow((Zvect), 2) + pow((RZvect), 2) + pow((RYvect), 2) + pow((RXvect), 2)), .5);
        if (Rounding > (lineDist * .45)) {
          Rounding = lineDist * .45;
        }
        newDistPerc = (Rounding / lineDist);
        //calculated arc end postion
        rndArcEnd[0] = Xstart + (Xvect * newDistPerc);
        rndArcEnd[1] = Ystart + (Yvect * newDistPerc);
        rndArcEnd[2] = Zstart + (Zvect * newDistPerc);
        rndArcEnd[3] = RZstart + (RZvect * newDistPerc);
        rndArcEnd[4] = RYstart + (RYvect * newDistPerc);
        rndArcEnd[5] = RXstart + (RXvect * newDistPerc);
        //calculate arc center point
        rndCalcCen[0] = (xyzuvw_In[0] + rndArcEnd[0]) / 2;
        rndCalcCen[1] = (xyzuvw_In[1] + rndArcEnd[1]) / 2;
        rndCalcCen[2] = (xyzuvw_In[2] + rndArcEnd[2]) / 2;
        rndCalcCen[3] = (xyzuvw_In[3] + rndArcEnd[3]) / 2;
        rndCalcCen[4] = (xyzuvw_In[4] + rndArcEnd[4]) / 2;
        rndCalcCen[5] = (xyzuvw_In[5] + rndArcEnd[5]) / 2;
        rndArcMid[0] = (xyzuvw_Temp[0] + rndCalcCen[0]) / 2;
        rndArcMid[1] = (xyzuvw_Temp[1] + rndCalcCen[1]) / 2;
        rndArcMid[2] = (xyzuvw_Temp[2] + rndCalcCen[2]) / 2;
        rndArcMid[3] = (xyzuvw_Temp[3] + rndCalcCen[3]) / 2;
        rndArcMid[4] = (xyzuvw_Temp[4] + rndCalcCen[4]) / 2;
        rndArcMid[5] = (xyzuvw_Temp[5] + rndCalcCen[5]) / 2;
        //set arc move to be executed
        rndData = "X" + String(rndArcMid[0]) + "Y" + String(rndArcMid[1]) + "Z" + String(rndArcMid[2]) + "Rz" + String(rndArcMid[3]) + "Ry" + String(rndArcMid[4]) + "Rx" + String(rndArcMid[5]) + "Ex" + String(rndArcEnd[0]) + "Ey" + String(rndArcEnd[1]) + "Ez" + String(rndArcEnd[2]) + "Tr" + String(xyzuvw_Temp[6]) + "S" + SpeedType  + String(SpeedVal) + "Ac" + String(ACCspd) + "Dc" + String(DCCspd) + "Rm" + String(ACCramp) + "W" + WristCon;
        function = "MA";
        rndTrue = true;
      }
      else {
        updatePos();
        xyzuvw_In[0] = xyzuvw_Temp[0];
        xyzuvw_In[1] = xyzuvw_Temp[1];
        xyzuvw_In[2] = xyzuvw_Temp[2];
        xyzuvw_In[3] = xyzuvw_Temp[3];
        xyzuvw_In[4] = xyzuvw_Temp[4];
        xyzuvw_In[5] = xyzuvw_Temp[5];
      }



      //xyz vector
      Xvect = xyzuvw_In[0] - xyzuvw_Out[0];
      Yvect = xyzuvw_In[1] - xyzuvw_Out[1];
      Zvect = xyzuvw_In[2] - xyzuvw_Out[2];

      //rz vector - eval if crosses over from 180 to -180
      if (xyzuvw_In[3] > 0 and xyzuvw_Out[3] < 0 ) {
        RZvect = (180 - xyzuvw_In[3]) - (180 + xyzuvw_Out[3]);
      }
      else if (xyzuvw_In[3] < 0 and xyzuvw_Out[3] > 0 ) {
        RZvect = (180 + xyzuvw_In[3]) - (180 - xyzuvw_Out[3]);
      }
      else {
        RZvect = xyzuvw_In[3] - xyzuvw_Out[3];
      }

      //ry vector
      RYvect = xyzuvw_In[4] - xyzuvw_Out[4];

      //rx vector - eval if crosses over from 180 to -180
      if (xyzuvw_In[5] > 0 and xyzuvw_Out[5] < 0 ) {
        RXvect = (180 - xyzuvw_In[5]) - (180 + xyzuvw_Out[5]);
      }
      else if (xyzuvw_In[5] < 0 and xyzuvw_Out[5] > 0 ) {
        RXvect = (180 + xyzuvw_In[5]) - (180 - xyzuvw_Out[5]);
      }
      else {
        RXvect = xyzuvw_In[5] - xyzuvw_Out[5];
      }


      //start pos
      float Xstart = xyzuvw_Out[0];
      float Ystart = xyzuvw_Out[1];
      float Zstart = xyzuvw_Out[2];
      float RZstart = xyzuvw_Out[3];
      float RYstart = xyzuvw_Out[4];
      float RXstart = xyzuvw_Out[5];


      //line dist and determine way point gap
      float lineDist = pow((pow((Xvect), 2) + pow((Yvect), 2) + pow((Zvect), 2) + pow((RZvect), 2) + pow((RYvect), 2) + pow((RXvect), 2)), .5);
      if (lineDist > 0) {

        float wayPts = lineDist / linWayDistSP;
        float wayPerc =  1 / wayPts;

        //pre calculate entire move and speeds

        SolveInverseKinematic();
        //calc destination motor steps for precalc
        int J1futStepM = (JangleOut[0] + J1axisLimNeg) * J1StepDeg;
        int J2futStepM = (JangleOut[1] + J2axisLimNeg) * J2StepDeg;
        int J3futStepM = (JangleOut[2] + J3axisLimNeg) * J3StepDeg;
        int J4futStepM = (JangleOut[3] + J4axisLimNeg) * J4StepDeg;
        int J5futStepM = (JangleOut[4] + J5axisLimNeg) * J5StepDeg;
        int J6futStepM = (JangleOut[5] + J6axisLimNeg) * J6StepDeg;

        //calc delta from current to destination fpr precalc
        int J1stepDif = J1StepM - J1futStepM;
        int J2stepDif = J2StepM - J2futStepM;
        int J3stepDif = J3StepM - J3futStepM;
        int J4stepDif = J4StepM - J4futStepM;
        int J5stepDif = J5StepM - J5futStepM;
        int J6stepDif = J6StepM - J6futStepM;

        //FIND HIGHEST STEP FOR PRECALC
        int HighStep = J1stepDif;
        if (J2stepDif > HighStep)
        {
          HighStep = J2stepDif;
        }
        if (J3stepDif > HighStep)
        {
          HighStep = J3stepDif;
        }
        if (J4stepDif > HighStep)
        {
          HighStep = J4stepDif;
        }
        if (J5stepDif > HighStep)
        {
          HighStep = J5stepDif;
        }
        if (J6stepDif > HighStep)
        {
          HighStep = J6stepDif;
        }


        /////PRE CALC SPEEDS//////
        float calcStepGap;

        //determine steps
        float ACCStep = HighStep * (ACCspd / 100);
        float NORStep = HighStep * ((100 - ACCspd - DCCspd) / 100);
        float DCCStep = HighStep * (DCCspd / 100);

        //set speed for seconds or mm per sec
        if (SpeedType == "s") {
          speedSP = (SpeedVal  * 1000000) * 1.2;
        }
        else if ((SpeedType == "m")) {
          speedSP = ((lineDist / SpeedVal) * 1000000) * 1.2;
        }

        //calc step gap for seconds or mm per sec
        if (SpeedType == "s" or SpeedType == "m" ) {
          float zeroStepGap = speedSP / HighStep;
          float zeroACCstepInc = (zeroStepGap * (100 / ACCramp)) / ACCStep;
          float zeroACCtime = ((ACCStep) * zeroStepGap) + ((ACCStep - 9) * (((ACCStep) * (zeroACCstepInc / 2))));
          float zeroNORtime = NORStep * zeroStepGap;
          float zeroDCCstepInc = (zeroStepGap * (100 / ACCramp)) / DCCStep;
          float zeroDCCtime = ((DCCStep) * zeroStepGap) + ((DCCStep - 9) * (((DCCStep) * (zeroDCCstepInc / 2))));
          float zeroTOTtime = zeroACCtime + zeroNORtime + zeroDCCtime;
          float overclockPerc = speedSP / zeroTOTtime;
          calcStepGap = zeroStepGap * overclockPerc;
          if (calcStepGap <= minSpeedDelay) {
            calcStepGap = minSpeedDelay;
            speedViolation = "1";
          }
        }

        //calc step gap for percentage
        else if (SpeedType == "p") {
          calcStepGap = minSpeedDelay / (SpeedVal / 100);
        }

        //calculate final step increments
        float calcACCstepInc = (calcStepGap * (100 / ACCramp)) / ACCStep;
        float calcDCCstepInc = (calcStepGap * (100 / ACCramp)) / DCCStep;
        float calcACCstartDel = (calcACCstepInc * ACCStep) * 2;
        float calcDCCendDel = (calcDCCstepInc * DCCStep) * 2;


        //calc way pt speeds
        float ACCwayPts = wayPts * (ACCspd / 100);
        float NORwayPts = wayPts * ((100 - ACCspd - DCCspd) / 100);
        float DCCwayPts = wayPts * (DCCspd / 100);

        //calc way inc for lin way steps
        float ACCwayInc = (calcACCstartDel - calcStepGap) / ACCwayPts;
        float DCCwayInc = (calcDCCendDel - calcStepGap) / DCCwayPts;

        //set starting delsy
        if (rndTrue == true) {
          curDelay = rndSpeed;
        }
        else {
          curDelay = calcACCstartDel;
        }


        // calc external axis way pt moves
        int J7futStepM = (J7_In + J7axisLimNeg) * J7StepDeg;
        int J7stepDif = (J7StepM - J7futStepM) / (wayPts - 1);
        int J8futStepM = (J8_In + J8axisLimNeg) * J8StepDeg;
        int J8stepDif = (J8StepM - J8futStepM) / (wayPts - 1);
        int J9futStepM = (J9_In + J9axisLimNeg) * J9StepDeg;
        int J9stepDif = (J9StepM - J9futStepM) / (wayPts - 1);


        if (J7stepDif <= 0) {
          J7dir = 1;
        }
        else {
          J7dir = 0;
        }

        if (J8stepDif <= 0) {
          J8dir = 1;
        }
        else {
          J8dir = 0;
        }

        if (J9stepDif <= 0) {
          J9dir = 1;
        }
        else {
          J9dir = 0;
        }


        resetEncoders();
        /////////////////////////////////////////////////
        //loop through waypoints
        for (int i = 0; i <= wayPts + 1; i++) {

          ////DELAY CALC/////
          if (i <= ACCwayPts) {
            curDelay = curDelay - (ACCwayInc);
          }
          else if (i >= (wayPts - DCCwayPts)) {
            curDelay = curDelay + (DCCwayInc);
          }
          else {
            curDelay = calcStepGap;
          }


          curDelay = calcStepGap;

          float curWayPerc = wayPerc * i;
          xyzuvw_In[0] = Xstart + (Xvect * curWayPerc);
          xyzuvw_In[1] = Ystart + (Yvect * curWayPerc);
          xyzuvw_In[2] = Zstart + (Zvect * curWayPerc);
          xyzuvw_In[3] = RZstart + (RZvect * curWayPerc);
          xyzuvw_In[4] = RYstart + (RYvect * curWayPerc);
          xyzuvw_In[5] = RXstart + (RXvect * curWayPerc);

          SolveInverseKinematic();

          //calc destination motor steps
          int J1futStepM = (JangleOut[0] + J1axisLimNeg) * J1StepDeg;
          int J2futStepM = (JangleOut[1] + J2axisLimNeg) * J2StepDeg;
          int J3futStepM = (JangleOut[2] + J3axisLimNeg) * J3StepDeg;
          int J4futStepM = (JangleOut[3] + J4axisLimNeg) * J4StepDeg;
          int J5futStepM = (JangleOut[4] + J5axisLimNeg) * J5StepDeg;
          int J6futStepM = (JangleOut[5] + J6axisLimNeg) * J6StepDeg;

          //calc delta from current to destination
          int J1stepDif = J1StepM - J1futStepM;
          int J2stepDif = J2StepM - J2futStepM;
          int J3stepDif = J3StepM - J3futStepM;
          int J4stepDif = J4StepM - J4futStepM;
          int J5stepDif = J5StepM - J5futStepM;
          int J6stepDif = J6StepM - J6futStepM;

          //determine motor directions
          if (J1stepDif <= 0) {
            J1dir = 1;
          }
          else {
            J1dir = 0;
          }

          if (J2stepDif <= 0) {
            J2dir = 1;
          }
          else {
            J2dir = 0;
          }

          if (J3stepDif <= 0) {
            J3dir = 1;
          }
          else {
            J3dir = 0;
          }

          if (J4stepDif <= 0) {
            J4dir = 1;
          }
          else {
            J4dir = 0;
          }

          if (J5stepDif <= 0) {
            J5dir = 1;
          }
          else {
            J5dir = 0;
          }

          if (J6stepDif <= 0) {
            J6dir = 1;
          }
          else {
            J6dir = 0;
          }



          //determine if requested position is within axis limits
          if ((J1dir == 1 and (J1StepM + J1stepDif > J1StepLim)) or (J1dir == 0 and (J1StepM - J1stepDif < 0))) {
            J1axisFault = 1;
          }
          if ((J2dir == 1 and (J2StepM + J2stepDif > J2StepLim)) or (J2dir == 0 and (J2StepM - J2stepDif < 0))) {
            J2axisFault = 1;
          }
          if ((J3dir == 1 and (J3StepM + J3stepDif > J3StepLim)) or (J3dir == 0 and (J3StepM - J3stepDif < 0))) {
            J3axisFault = 1;
          }
          if ((J4dir == 1 and (J4StepM + J4stepDif > J4StepLim)) or (J4dir == 0 and (J4StepM - J4stepDif < 0))) {
            J4axisFault = 1;
          }
          if ((J5dir == 1 and (J5StepM + J5stepDif > J5StepLim)) or (J5dir == 0 and (J5StepM - J5stepDif < 0))) {
            J5axisFault = 1;
          }
          if ((J6dir == 1 and (J6StepM + J6stepDif > J6StepLim)) or (J6dir == 0 and (J6StepM - J6stepDif < 0))) {
            J6axisFault = 1;
          }
          if ((J7dir == 1 and (J7StepM + J7stepDif > J7StepLim)) or (J7dir == 0 and (J7StepM - J7stepDif < 0))) {
            J7axisFault = 1;
          }
          if ((J8dir == 1 and (J8StepM + J8stepDif > J8StepLim)) or (J8dir == 0 and (J8StepM - J8stepDif < 0))) {
            J8axisFault = 1;
          }
          if ((J9dir == 1 and (J9StepM + J9stepDif > J9StepLim)) or (J9dir == 0 and (J9StepM - J9stepDif < 0))) {
            J9axisFault = 1;
          }
          TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault + J7axisFault + J8axisFault + J9axisFault;

          //send move command if no axis limit error
          if (TotalAxisFault == 0 && KinematicError == 0) {
            driveMotorsL(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(J7stepDif), abs(J8stepDif), abs(J9stepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, curDelay);
            updatePos();
            rndSpeed = curDelay;
          }
          else if (KinematicError == 1) {
            Alarm = "ER";
            if (splineTrue == false) {
              delay(5);
              Serial.println(Alarm);
            }
          }
          else {
            Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault) + String(J7axisFault) + String(J8axisFault) + String(J9axisFault);
            if (splineTrue == false) {
              delay(5);
              Serial.println(Alarm);
            }
          }
        }

      }

      checkEncoders();
      if (splineTrue == false) {
        sendRobotPos();
      }
      inData = ""; // Clear recieved buffer
      ////////MOVE COMPLETE///////////

    }




    //----- MOVE J ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "MJ")
    {
      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;

      int J1axisFault = 0;
      int J2axisFault = 0;
      int J3axisFault = 0;
      int J4axisFault = 0;
      int J5axisFault = 0;
      int J6axisFault = 0;
      int J7axisFault = 0;
      int J8axisFault = 0;
      int J9axisFault = 0;
      int TotalAxisFault = 0;

      int xStart = inData.indexOf("X");
      int yStart = inData.indexOf("Y");
      int zStart = inData.indexOf("Z");
      int rzStart = inData.indexOf("Rz");
      int ryStart = inData.indexOf("Ry");
      int rxStart = inData.indexOf("Rx");
      int J7Start = inData.indexOf("J7");
      int J8Start = inData.indexOf("J8");
      int J9Start = inData.indexOf("J9");
      int SPstart = inData.indexOf("S");
      int AcStart = inData.indexOf("Ac");
      int DcStart = inData.indexOf("Dc");
      int RmStart = inData.indexOf("Rm");
      int RndStart = inData.indexOf("Rnd");
      int WristConStart = inData.indexOf("W");
      int LoopModeStart = inData.indexOf("Lm");

      xyzuvw_In[0] = inData.substring(xStart + 1, yStart).toFloat();
      xyzuvw_In[1] = inData.substring(yStart + 1, zStart).toFloat();
      xyzuvw_In[2] = inData.substring(zStart + 1, rzStart).toFloat();
      xyzuvw_In[3] = inData.substring(rzStart + 2, ryStart).toFloat();
      xyzuvw_In[4] = inData.substring(ryStart + 2, rxStart).toFloat();
      xyzuvw_In[5] = inData.substring(rxStart + 2, J7Start).toFloat();
      J7_In = inData.substring(J7Start + 2, J8Start).toFloat();
      J8_In = inData.substring(J8Start + 2, J9Start).toFloat();
      J9_In = inData.substring(J9Start + 2, SPstart).toFloat();

      String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
      float SpeedVal = inData.substring(SPstart + 2, AcStart).toFloat();
      float ACCspd = inData.substring(AcStart + 2, DcStart).toFloat();
      float DCCspd = inData.substring(DcStart + 2, RmStart).toFloat();
      float ACCramp = inData.substring(RmStart + 2, RndStart).toFloat();
      float Rounding = inData.substring(RndStart + 3, WristConStart).toFloat();
      WristCon = inData.substring(WristConStart + 1, LoopModeStart);
      String LoopMode = inData.substring(LoopModeStart + 2);
      LoopMode.trim();
      J1LoopMode = LoopMode.substring(0, 1).toInt();
      J2LoopMode = LoopMode.substring(1, 2).toInt();
      J3LoopMode = LoopMode.substring(2, 3).toInt();
      J4LoopMode = LoopMode.substring(3, 4).toInt();
      J5LoopMode = LoopMode.substring(4, 5).toInt();
      J6LoopMode = LoopMode.substring(5).toInt();


      SolveInverseKinematic();

      //calc destination motor steps
      int J1futStepM = (JangleOut[0] + J1axisLimNeg) * J1StepDeg;
      int J2futStepM = (JangleOut[1] + J2axisLimNeg) * J2StepDeg;
      int J3futStepM = (JangleOut[2] + J3axisLimNeg) * J3StepDeg;
      int J4futStepM = (JangleOut[3] + J4axisLimNeg) * J4StepDeg;
      int J5futStepM = (JangleOut[4] + J5axisLimNeg) * J5StepDeg;
      int J6futStepM = (JangleOut[5] + J6axisLimNeg) * J6StepDeg;
      int J7futStepM = (J7_In + J7axisLimNeg) * J7StepDeg;
      int J8futStepM = (J8_In + J8axisLimNeg) * J8StepDeg;
      int J9futStepM = (J9_In + J9axisLimNeg) * J9StepDeg;


      //calc delta from current to destination
      int J1stepDif = J1StepM - J1futStepM;
      int J2stepDif = J2StepM - J2futStepM;
      int J3stepDif = J3StepM - J3futStepM;
      int J4stepDif = J4StepM - J4futStepM;
      int J5stepDif = J5StepM - J5futStepM;
      int J6stepDif = J6StepM - J6futStepM;
      int J7stepDif = J7StepM - J7futStepM;
      int J8stepDif = J8StepM - J8futStepM;
      int J9stepDif = J9StepM - J9futStepM;

      //determine motor directions
      if (J1stepDif <= 0) {
        J1dir = 1;
      }
      else {
        J1dir = 0;
      }

      if (J2stepDif <= 0) {
        J2dir = 1;
      }
      else {
        J2dir = 0;
      }

      if (J3stepDif <= 0) {
        J3dir = 1;
      }
      else {
        J3dir = 0;
      }

      if (J4stepDif <= 0) {
        J4dir = 1;
      }
      else {
        J4dir = 0;
      }

      if (J5stepDif <= 0) {
        J5dir = 1;
      }
      else {
        J5dir = 0;
      }

      if (J6stepDif <= 0) {
        J6dir = 1;
      }
      else {
        J6dir = 0;
      }

      if (J7stepDif <= 0) {
        J7dir = 1;
      }
      else {
        J7dir = 0;
      }

      if (J8stepDif <= 0) {
        J8dir = 1;
      }
      else {
        J8dir = 0;
      }

      if (J9stepDif <= 0) {
        J9dir = 1;
      }
      else {
        J9dir = 0;
      }



      //determine if requested position is within axis limits
      if ((J1dir == 1 and (J1StepM + J1stepDif > J1StepLim)) or (J1dir == 0 and (J1StepM - J1stepDif < 0))) {
        J1axisFault = 1;
      }
      if ((J2dir == 1 and (J2StepM + J2stepDif > J2StepLim)) or (J2dir == 0 and (J2StepM - J2stepDif < 0))) {
        J2axisFault = 1;
      }
      if ((J3dir == 1 and (J3StepM + J3stepDif > J3StepLim)) or (J3dir == 0 and (J3StepM - J3stepDif < 0))) {
        J3axisFault = 1;
      }
      if ((J4dir == 1 and (J4StepM + J4stepDif > J4StepLim)) or (J4dir == 0 and (J4StepM - J4stepDif < 0))) {
        J4axisFault = 1;
      }
      if ((J5dir == 1 and (J5StepM + J5stepDif > J5StepLim)) or (J5dir == 0 and (J5StepM - J5stepDif < 0))) {
        J5axisFault = 1;
      }
      if ((J6dir == 1 and (J6StepM + J6stepDif > J6StepLim)) or (J6dir == 0 and (J6StepM - J6stepDif < 0))) {
        J6axisFault = 1;
      }
      if ((J7dir == 1 and (J7StepM + J7stepDif > J7StepLim)) or (J7dir == 0 and (J7StepM - J7stepDif < 0))) {
        J7axisFault = 1;
      }
      if ((J8dir == 1 and (J8StepM + J8stepDif > J8StepLim)) or (J8dir == 0 and (J8StepM - J8stepDif < 0))) {
        J8axisFault = 1;
      }
      if ((J9dir == 1 and (J9StepM + J9stepDif > J9StepLim)) or (J9dir == 0 and (J9StepM - J9stepDif < 0))) {
        J9axisFault = 1;
      }
      TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault + J7axisFault + J8axisFault + J9axisFault;


      //send move command if no axis limit error
      if (TotalAxisFault == 0 && KinematicError == 0) {
        resetEncoders();
        driveMotorsJ(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(J7stepDif), abs(J8stepDif), abs(J9stepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
        checkEncoders();
        sendRobotPos();
      }
      else if (KinematicError == 1) {
        Alarm = "ER";
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      }
      else {
        Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault) + String(J7axisFault) + String(J8axisFault) + String(J9axisFault);
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      }



      inData = ""; // Clear recieved buffer
      ////////MOVE COMPLETE///////////
    }





    //----- MOVE G ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "MG")
    {
      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;

      int J1axisFault = 0;
      int J2axisFault = 0;
      int J3axisFault = 0;
      int J4axisFault = 0;
      int J5axisFault = 0;
      int J6axisFault = 0;
      int J7axisFault = 0;
      int J8axisFault = 0;
      int J9axisFault = 0;
      int TotalAxisFault = 0;

      int xStart = inData.indexOf("X");
      int yStart = inData.indexOf("Y");
      int zStart = inData.indexOf("Z");
      int rzStart = inData.indexOf("Rz");
      int ryStart = inData.indexOf("Ry");
      int rxStart = inData.indexOf("Rx");
      int J7Start = inData.indexOf("J7");
      int J8Start = inData.indexOf("J8");
      int J9Start = inData.indexOf("J9");
      int SPstart = inData.indexOf("S");
      int AcStart = inData.indexOf("Ac");
      int DcStart = inData.indexOf("Dc");
      int RmStart = inData.indexOf("Rm");
      int RndStart = inData.indexOf("Rnd");
      int WristConStart = inData.indexOf("W");
      int LoopModeStart = inData.indexOf("Lm");

      xyzuvw_In[0] = inData.substring(xStart + 1, yStart).toFloat();
      xyzuvw_In[1] = inData.substring(yStart + 1, zStart).toFloat();
      xyzuvw_In[2] = inData.substring(zStart + 1, rzStart).toFloat();
      xyzuvw_In[3] = inData.substring(rzStart + 2, ryStart).toFloat();
      xyzuvw_In[4] = inData.substring(ryStart + 2, rxStart).toFloat();
      xyzuvw_In[5] = inData.substring(rxStart + 2, J7Start).toFloat();
      J7_In = inData.substring(J7Start + 2, J8Start).toFloat();
      J8_In = inData.substring(J8Start + 2, J9Start).toFloat();
      J9_In = inData.substring(J9Start + 2, SPstart).toFloat();

      String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
      float SpeedVal = inData.substring(SPstart + 2, AcStart).toFloat();
      float ACCspd = inData.substring(AcStart + 2, DcStart).toFloat();
      float DCCspd = inData.substring(DcStart + 2, RmStart).toFloat();
      float ACCramp = inData.substring(RmStart + 2, RndStart).toFloat();
      float Rounding = inData.substring(RndStart + 3, WristConStart).toFloat();
      WristCon = inData.substring(WristConStart + 1, LoopModeStart);
      String LoopMode = inData.substring(LoopModeStart + 2);
      LoopMode.trim();
      J1LoopMode = LoopMode.substring(0, 1).toInt();
      J2LoopMode = LoopMode.substring(1, 2).toInt();
      J3LoopMode = LoopMode.substring(2, 3).toInt();
      J4LoopMode = LoopMode.substring(3, 4).toInt();
      J5LoopMode = LoopMode.substring(4, 5).toInt();
      J6LoopMode = LoopMode.substring(5).toInt();


      SolveInverseKinematic();

      //calc destination motor steps
      int J1futStepM = (JangleOut[0] + J1axisLimNeg) * J1StepDeg;
      int J2futStepM = (JangleOut[1] + J2axisLimNeg) * J2StepDeg;
      int J3futStepM = (JangleOut[2] + J3axisLimNeg) * J3StepDeg;
      int J4futStepM = (JangleOut[3] + J4axisLimNeg) * J4StepDeg;
      int J5futStepM = (JangleOut[4] + J5axisLimNeg) * J5StepDeg;
      int J6futStepM = (JangleOut[5] + J6axisLimNeg) * J6StepDeg;
      int J7futStepM = (J7_In + J7axisLimNeg) * J7StepDeg;
      int J8futStepM = (J8_In + J8axisLimNeg) * J8StepDeg;
      int J9futStepM = (J9_In + J9axisLimNeg) * J9StepDeg;


      //calc delta from current to destination
      int J1stepDif = J1StepM - J1futStepM;
      int J2stepDif = J2StepM - J2futStepM;
      int J3stepDif = J3StepM - J3futStepM;
      int J4stepDif = J4StepM - J4futStepM;
      int J5stepDif = J5StepM - J5futStepM;
      int J6stepDif = J6StepM - J6futStepM;
      int J7stepDif = J7StepM - J7futStepM;
      int J8stepDif = J8StepM - J8futStepM;
      int J9stepDif = J9StepM - J9futStepM;

      //determine motor directions
      if (J1stepDif <= 0) {
        J1dir = 1;
      }
      else {
        J1dir = 0;
      }

      if (J2stepDif <= 0) {
        J2dir = 1;
      }
      else {
        J2dir = 0;
      }

      if (J3stepDif <= 0) {
        J3dir = 1;
      }
      else {
        J3dir = 0;
      }

      if (J4stepDif <= 0) {
        J4dir = 1;
      }
      else {
        J4dir = 0;
      }

      if (J5stepDif <= 0) {
        J5dir = 1;
      }
      else {
        J5dir = 0;
      }

      if (J6stepDif <= 0) {
        J6dir = 1;
      }
      else {
        J6dir = 0;
      }

      if (J7stepDif <= 0) {
        J7dir = 1;
      }
      else {
        J7dir = 0;
      }

      if (J8stepDif <= 0) {
        J8dir = 1;
      }
      else {
        J8dir = 0;
      }

      if (J9stepDif <= 0) {
        J9dir = 1;
      }
      else {
        J9dir = 0;
      }



      //determine if requested position is within axis limits
      if ((J1dir == 1 and (J1StepM + J1stepDif > J1StepLim)) or (J1dir == 0 and (J1StepM - J1stepDif < 0))) {
        J1axisFault = 1;
      }
      if ((J2dir == 1 and (J2StepM + J2stepDif > J2StepLim)) or (J2dir == 0 and (J2StepM - J2stepDif < 0))) {
        J2axisFault = 1;
      }
      if ((J3dir == 1 and (J3StepM + J3stepDif > J3StepLim)) or (J3dir == 0 and (J3StepM - J3stepDif < 0))) {
        J3axisFault = 1;
      }
      if ((J4dir == 1 and (J4StepM + J4stepDif > J4StepLim)) or (J4dir == 0 and (J4StepM - J4stepDif < 0))) {
        J4axisFault = 1;
      }
      if ((J5dir == 1 and (J5StepM + J5stepDif > J5StepLim)) or (J5dir == 0 and (J5StepM - J5stepDif < 0))) {
        J5axisFault = 1;
      }
      if ((J6dir == 1 and (J6StepM + J6stepDif > J6StepLim)) or (J6dir == 0 and (J6StepM - J6stepDif < 0))) {
        J6axisFault = 1;
      }
      if ((J7dir == 1 and (J7StepM + J7stepDif > J7StepLim)) or (J7dir == 0 and (J7StepM - J7stepDif < 0))) {
        J7axisFault = 1;
      }
      if ((J8dir == 1 and (J8StepM + J8stepDif > J8StepLim)) or (J8dir == 0 and (J8StepM - J8stepDif < 0))) {
        J8axisFault = 1;
      }
      if ((J9dir == 1 and (J9StepM + J9stepDif > J9StepLim)) or (J9dir == 0 and (J9StepM - J9stepDif < 0))) {
        J9axisFault = 1;
      }
      TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault + J7axisFault + J8axisFault + J9axisFault;


      //send move command if no axis limit error
      if (TotalAxisFault == 0 && KinematicError == 0) {
        resetEncoders();
        driveMotorsG(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(J7stepDif), abs(J8stepDif), abs(J9stepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
        checkEncoders();
        sendRobotPos();
      }
      else if (KinematicError == 1) {
        Alarm = "ER";
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      }
      else {
        Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault) + String(J7axisFault) + String(J8axisFault) + String(J9axisFault);
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      }



      inData = ""; // Clear recieved buffer
      ////////MOVE COMPLETE///////////
    }






    //----- MOVE C (Cirlce) ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "MC") {

      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;

      int J1axisFault = 0;
      int J2axisFault = 0;
      int J3axisFault = 0;
      int J4axisFault = 0;
      int J5axisFault = 0;
      int J6axisFault = 0;
      int TotalAxisFault = 0;

      String Alarm = "0";
      float curWayDis;
      float speedSP;
      float Xvect;
      float Yvect;
      float Zvect;
      float calcStepGap;
      float theta;
      int Cdir;
      float axis [3];
      float axisTemp [3];
      float startVect [3];
      float Rotation [3][3];
      float DestPt [3];
      float a;
      float b;
      float c;
      float d;
      float aa;
      float bb;
      float cc;
      float dd;
      float bc;
      float ad;
      float ac;
      float ab;
      float bd;
      float cd;

      int xStart = inData.indexOf("Cx");
      int yStart = inData.indexOf("Cy");
      int zStart = inData.indexOf("Cz");
      int rzStart = inData.indexOf("Rz");
      int ryStart = inData.indexOf("Ry");
      int rxStart = inData.indexOf("Rx");
      int xMidIndex = inData.indexOf("Bx");
      int yMidIndex = inData.indexOf("By");
      int zMidIndex = inData.indexOf("Bz");
      int xEndIndex = inData.indexOf("Px");
      int yEndIndex = inData.indexOf("Py");
      int zEndIndex = inData.indexOf("Pz");
      int tStart = inData.indexOf("Tr");
      int SPstart = inData.indexOf("S");
      int AcStart = inData.indexOf("Ac");
      int DcStart = inData.indexOf("Dc");
      int RmStart = inData.indexOf("Rm");
      int WristConStart = inData.indexOf("W");
      int LoopModeStart = inData.indexOf("Lm");


      float xBeg = inData.substring(xStart + 2, yStart).toFloat();
      float yBeg = inData.substring(yStart + 2, zStart).toFloat();
      float zBeg = inData.substring(zStart + 2, rzStart).toFloat();
      float rzBeg = inData.substring(rzStart + 2, ryStart).toFloat();
      float ryBeg = inData.substring(ryStart + 2, rxStart).toFloat();
      float rxBeg = inData.substring(rxStart + 2, xMidIndex).toFloat();
      float xMid = inData.substring(xMidIndex + 2, yMidIndex).toFloat();
      float yMid = inData.substring(yMidIndex + 2, zMidIndex).toFloat();
      float zMid = inData.substring(zMidIndex + 2, xEndIndex).toFloat();
      float xEnd = inData.substring(xEndIndex + 2, yEndIndex).toFloat();
      float yEnd = inData.substring(yEndIndex + 2, zEndIndex).toFloat();
      float zEnd = inData.substring(zEndIndex + 2, tStart).toFloat();
      xyzuvw_In[6] = inData.substring(tStart + 2, SPstart).toFloat();
      String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
      float SpeedVal = inData.substring(SPstart + 2, AcStart).toFloat();
      float ACCspd = inData.substring(AcStart + 2, DcStart).toFloat();
      float DCCspd = inData.substring(DcStart + 2, RmStart).toFloat();
      float ACCramp = inData.substring(RmStart + 2, WristConStart).toFloat();
      WristCon = inData.substring(WristConStart + 1, LoopModeStart);
      String LoopMode = inData.substring(LoopModeStart + 2);
      LoopMode.trim();
      J1LoopMode = LoopMode.substring(0, 1).toInt();
      J2LoopMode = LoopMode.substring(1, 2).toInt();
      J3LoopMode = LoopMode.substring(2, 3).toInt();
      J4LoopMode = LoopMode.substring(3, 4).toInt();
      J5LoopMode = LoopMode.substring(4, 5).toInt();
      J6LoopMode = LoopMode.substring(5).toInt();

      //calc vector from start point of circle (mid) to center of circle (beg)
      Xvect = xMid - xBeg;
      Yvect = yMid - yBeg;
      Zvect = zMid - zBeg;
      //get radius - distance from first point (center of circle) to second point (start point of circle)
      float Radius = pow((pow((Xvect), 2) + pow((Yvect), 2) + pow((Zvect), 2)), .5);

      //set center coordinates of circle to first point (beg) as this is the center of our circle
      float Px = xBeg ;
      float Py = yBeg ;
      float Pz = zBeg ;

      //define start vetor (mid) point is start of circle
      startVect [0] = (xMid - Px);
      startVect [1] = (yMid - Py);
      startVect [2] = (zMid - Pz);
      //get vectors from center of circle to  mid target (start) and end target then normalize
      float vect_Bmag = pow((pow((xMid - Px), 2) + pow((yMid - Py), 2) + pow((zMid - Pz), 2)), .5);
      float vect_Bx = (xMid - Px) / vect_Bmag;
      float vect_By = (yMid - Py) / vect_Bmag;
      float vect_Bz = (zMid - Pz) / vect_Bmag;
      float vect_Cmag = pow((pow((xEnd - Px), 2) + pow((yEnd - Py), 2) + pow((zEnd - Pz), 2)), .5);
      float vect_Cx = (xEnd - Px) / vect_Cmag;
      float vect_Cy = (yEnd - Py) / vect_Cmag;
      float vect_Cz = (zEnd - Pz) / vect_Cmag;
      //get cross product of vectors b & c than apply to axis matrix
      float CrossX = (vect_By * vect_Cz) - (vect_Bz * vect_Cy);
      float CrossY = (vect_Bz * vect_Cx) - (vect_Bx * vect_Cz);
      float CrossZ = (vect_Bx * vect_Cy) - (vect_By * vect_Cx);
      axis [0] = CrossX / sqrt((CrossX * CrossX) + (CrossY * CrossY) + (CrossZ * CrossZ));
      axis [1] = CrossY / sqrt((CrossX * CrossX) + (CrossY * CrossY) + (CrossZ * CrossZ));
      axis [2] = CrossZ / sqrt((CrossX * CrossX) + (CrossY * CrossY) + (CrossZ * CrossZ));
      //get radian angle between vectors using acos of dot product
      float BCradians = acos((vect_Bx * vect_Cx + vect_By * vect_Cy + vect_Bz * vect_Cz) / (sqrt(pow(vect_Bx , 2) + pow(vect_Cy , 2) + pow(vect_Bz , 2)) * sqrt(pow(vect_Cx , 2) + pow(vect_Cy , 2) + pow(vect_Cz , 2)))  );
      //get arc degree
      float ABdegrees = degrees(BCradians);
      //get direction from angle
      if (ABdegrees > 0) {
        Cdir = 1;
      }
      else {
        Cdir = -1;
      }

      //get circumference and calc way pt gap
      float lineDist = 2 * 3.14159265359 * Radius;
      float wayPts = lineDist / linWayDistSP;

      float wayPerc = 1 / wayPts;
      //cacl way pt angle
      float theta_Deg = ((360 * Cdir) / (wayPts));

      //determine steps
      int HighStep = lineDist / .05;
      float ACCStep = HighStep * (ACCspd / 100);
      float NORStep = HighStep * ((100 - ACCspd - DCCspd) / 100);
      float DCCStep = HighStep * (DCCspd / 100);

      //set speed for seconds or mm per sec
      if (SpeedType == "s") {
        speedSP = (SpeedVal * 1000000) * 1.75;
      }
      else if (SpeedType == "m") {
        speedSP = ((lineDist / SpeedVal) * 1000000) * 1.75;
      }

      //calc step gap for seconds or mm per sec
      if (SpeedType == "s" or SpeedType == "m" ) {
        float zeroStepGap = speedSP / HighStep;
        float zeroACCstepInc = (zeroStepGap * (100 / ACCramp)) / ACCStep;
        float zeroACCtime = ((ACCStep) * zeroStepGap) + ((ACCStep - 9) * (((ACCStep) * (zeroACCstepInc / 2))));
        float zeroNORtime = NORStep * zeroStepGap;
        float zeroDCCstepInc = (zeroStepGap * (100 / ACCramp)) / DCCStep;
        float zeroDCCtime = ((DCCStep) * zeroStepGap) + ((DCCStep - 9) * (((DCCStep) * (zeroDCCstepInc / 2))));
        float zeroTOTtime = zeroACCtime + zeroNORtime + zeroDCCtime;
        float overclockPerc = speedSP / zeroTOTtime;
        calcStepGap = zeroStepGap * overclockPerc;
        if (calcStepGap <= minSpeedDelay) {
          calcStepGap = minSpeedDelay;
          speedViolation = "1";
        }
      }

      //calc step gap for percentage
      else if (SpeedType == "p") {
        calcStepGap = minSpeedDelay / (SpeedVal / 100);
      }

      //calculate final step increments
      float calcACCstepInc = (calcStepGap * (100 / ACCramp)) / ACCStep;
      float calcDCCstepInc = (calcStepGap * (100 / ACCramp)) / DCCStep;
      float calcACCstartDel = (calcACCstepInc * ACCStep) * 2;
      float calcDCCendDel = (calcDCCstepInc * DCCStep) * 2;


      //calc way pt speeds
      float ACCwayPts = wayPts * (ACCspd / 100);
      float NORwayPts = wayPts * ((100 - ACCspd - DCCspd) / 100);
      float DCCwayPts = wayPts * (DCCspd / 100);

      //calc way inc for lin way steps
      float ACCwayInc = (calcACCstartDel - calcStepGap) / ACCwayPts;
      float DCCwayInc = (calcDCCendDel - calcStepGap) / DCCwayPts;

      //set starting delsy
      float curDelay = calcACCstartDel;

      //set starting angle first way pt
      float cur_deg = theta_Deg;

      /////////////////////////////////////
      //loop through waypoints
      ////////////////////////////////////

      resetEncoders();

      for (int i = 1; i <= wayPts; i++) {

        theta = radians(cur_deg);
        //use euler rodrigues formula to find rotation vector
        a = cos(theta / 2.0);
        b = -axis [0] * sin(theta / 2.0);
        c = -axis [1] * sin(theta / 2.0);
        d = -axis [2] * sin(theta / 2.0);
        aa = a * a;
        bb = b * b;
        cc = c * c;
        dd = d * d;
        bc = b * c;
        ad = a * d;
        ac = a * c;
        ab = a * b;
        bd = b * d;
        cd = c * d;
        Rotation [0][0] = aa + bb - cc - dd;
        Rotation [0][1] = 2 * (bc + ad);
        Rotation [0][2] = 2 * (bd - ac);
        Rotation [1][0] = 2 * (bc - ad);
        Rotation [1][1] = aa + cc - bb - dd;
        Rotation [1][2] = 2 * (cd + ab);
        Rotation [2][0] = 2 * (bd + ac);
        Rotation [2][1] = 2 * (cd - ab);
        Rotation [2][2] = aa + dd - bb - cc;

        //get product of current rotation and start vector
        DestPt[0] = (Rotation [0][0] * startVect[0]) + (Rotation [0][1] * startVect[1]) + (Rotation [0][2] * startVect[2]);
        DestPt[1] = (Rotation [1][0] * startVect[0]) + (Rotation [1][1] * startVect[1]) + (Rotation [1][2] * startVect[2]);
        DestPt[2] = (Rotation [2][0] * startVect[0]) + (Rotation [2][1] * startVect[1]) + (Rotation [2][2] * startVect[2]);

        ////DELAY CALC/////
        if (i <= ACCwayPts) {
          curDelay = curDelay - (ACCwayInc);
        }
        else if (i >= (wayPts - DCCwayPts)) {
          curDelay = curDelay + (DCCwayInc);
        }
        else {
          curDelay = calcStepGap;
        }

        //shift way pts back to orignal origin and calc kinematics for way pt movement
        xyzuvw_In[0] = (DestPt[0]) + Px;
        xyzuvw_In[1] = (DestPt[1]) + Py;
        xyzuvw_In[2] = (DestPt[2]) + Pz;
        xyzuvw_In[3] = rzBeg;
        xyzuvw_In[4] = ryBeg;
        xyzuvw_In[5] = rxBeg;

        SolveInverseKinematic();

        //calc destination motor steps
        int J1futStepM = (JangleOut[0] + J1axisLimNeg) * J1StepDeg;
        int J2futStepM = (JangleOut[1] + J2axisLimNeg) * J2StepDeg;
        int J3futStepM = (JangleOut[2] + J3axisLimNeg) * J3StepDeg;
        int J4futStepM = (JangleOut[3] + J4axisLimNeg) * J4StepDeg;
        int J5futStepM = (JangleOut[4] + J5axisLimNeg) * J5StepDeg;
        int J6futStepM = (JangleOut[5] + J6axisLimNeg) * J6StepDeg;

        //calc delta from current to destination
        int J1stepDif = J1StepM - J1futStepM;
        int J2stepDif = J2StepM - J2futStepM;
        int J3stepDif = J3StepM - J3futStepM;
        int J4stepDif = J4StepM - J4futStepM;
        int J5stepDif = J5StepM - J5futStepM;
        int J6stepDif = J6StepM - J6futStepM;
        int J7stepDif = 0;
        int J8stepDif = 0;
        int J9stepDif = 0;

        //determine motor directions
        if (J1stepDif <= 0) {
          J1dir = 1;
        }
        else {
          J1dir = 0;
        }

        if (J2stepDif <= 0) {
          J2dir = 1;
        }
        else {
          J2dir = 0;
        }

        if (J3stepDif <= 0) {
          J3dir = 1;
        }
        else {
          J3dir = 0;
        }

        if (J4stepDif <= 0) {
          J4dir = 1;
        }
        else {
          J4dir = 0;
        }

        if (J5stepDif <= 0) {
          J5dir = 1;
        }
        else {
          J5dir = 0;
        }

        if (J6stepDif <= 0) {
          J6dir = 1;
        }
        else {
          J6dir = 0;
        }

        J7dir = 0;
        J8dir = 0;
        J9dir = 0;

        //determine if requested position is within axis limits
        if ((J1dir == 1 and (J1StepM + J1stepDif > J1StepLim)) or (J1dir == 0 and (J1StepM - J1stepDif < 0))) {
          J1axisFault = 1;
        }
        if ((J2dir == 1 and (J2StepM + J2stepDif > J2StepLim)) or (J2dir == 0 and (J2StepM - J2stepDif < 0))) {
          J2axisFault = 1;
        }
        if ((J3dir == 1 and (J3StepM + J3stepDif > J3StepLim)) or (J3dir == 0 and (J3StepM - J3stepDif < 0))) {
          J3axisFault = 1;
        }
        if ((J4dir == 1 and (J4StepM + J4stepDif > J4StepLim)) or (J4dir == 0 and (J4StepM - J4stepDif < 0))) {
          J4axisFault = 1;
        }
        if ((J5dir == 1 and (J5StepM + J5stepDif > J5StepLim)) or (J5dir == 0 and (J5StepM - J5stepDif < 0))) {
          J5axisFault = 1;
        }
        if ((J6dir == 1 and (J6StepM + J6stepDif > J6StepLim)) or (J6dir == 0 and (J6StepM - J6stepDif < 0))) {
          J6axisFault = 1;
        }
        TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault;



        if (TotalAxisFault == 0 && KinematicError == 0) {
          driveMotorsL(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(J7stepDif), abs(J8stepDif), abs(J9stepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, curDelay);
        }
        else if (KinematicError == 1) {
          Alarm = "ER";
          delay(5);
          Serial.println(Alarm);
        }
        else {
          Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault);
          delay(5);
          Serial.println(Alarm);
        }


        //increment angle
        cur_deg += theta_Deg;

      }

      checkEncoders();
      sendRobotPos();


      inData = ""; // Clear recieved buffer
      ////////MOVE COMPLETE///////////
    }







    //----- MOVE A (Arc) ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "MA" and flag == "") {

      if (rndTrue == true) {
        inData = rndData;
      }

      float curDelay;

      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;

      int J1axisFault = 0;
      int J2axisFault = 0;
      int J3axisFault = 0;
      int J4axisFault = 0;
      int J5axisFault = 0;
      int J6axisFault = 0;
      int TotalAxisFault = 0;

      //String Alarm = "0";
      float curWayDis;
      float speedSP;
      float Xvect;
      float Yvect;
      float Zvect;
      float calcStepGap;
      float theta;
      float axis [3];
      float axisTemp [3];
      float startVect [3];
      float Rotation [3][3];
      float DestPt [3];
      float a;
      float b;
      float c;
      float d;
      float aa;
      float bb;
      float cc;
      float dd;
      float bc;
      float ad;
      float ac;
      float ab;
      float bd;
      float cd;

      int xMidIndex = inData.indexOf("X");
      int yMidIndex = inData.indexOf("Y");
      int zMidIndex = inData.indexOf("Z");
      int rzIndex = inData.indexOf("Rz");
      int ryIndex = inData.indexOf("Ry");
      int rxIndex = inData.indexOf("Rx");

      int xEndIndex = inData.indexOf("Ex");
      int yEndIndex = inData.indexOf("Ey");
      int zEndIndex = inData.indexOf("Ez");
      int tStart = inData.indexOf("Tr");
      int SPstart = inData.indexOf("S");
      int AcStart = inData.indexOf("Ac");
      int DcStart = inData.indexOf("Dc");
      int RmStart = inData.indexOf("Rm");
      int WristConStart = inData.indexOf("W");
      int LoopModeStart = inData.indexOf("Lm");

      updatePos();

      float xBeg = xyzuvw_Out[0];
      float yBeg = xyzuvw_Out[1];
      float zBeg = xyzuvw_Out[2];
      float rzBeg = xyzuvw_Out[3];
      float ryBeg = xyzuvw_Out[4];
      float rxBeg = xyzuvw_Out[5];


      float xMid = inData.substring(xMidIndex + 1, yMidIndex).toFloat();
      float yMid = inData.substring(yMidIndex + 1, zMidIndex).toFloat();
      float zMid = inData.substring(zMidIndex + 1, rzIndex).toFloat();

      float rz = inData.substring(rzIndex + 2, ryIndex).toFloat();
      float ry = inData.substring(ryIndex + 2, rxIndex).toFloat();
      float rx = inData.substring(rxIndex + 2, xEndIndex).toFloat();


      float RZvect = rzBeg - rz;
      float RYvect = ryBeg - ry;
      float RXvect = rxBeg - rx;

      float xEnd = inData.substring(xEndIndex + 2, yEndIndex).toFloat();
      float yEnd = inData.substring(yEndIndex + 2, zEndIndex).toFloat();
      float zEnd = inData.substring(zEndIndex + 2, tStart).toFloat();


      xyzuvw_In[6] = inData.substring(tStart + 2, SPstart).toFloat();
      String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
      float SpeedVal = inData.substring(SPstart + 2, AcStart).toFloat();
      float ACCspd = inData.substring(AcStart + 2, DcStart).toFloat();
      float DCCspd = inData.substring(DcStart + 2, RmStart).toFloat();
      float ACCramp = inData.substring(RmStart + 2, WristConStart).toFloat();
      WristCon = inData.substring(WristConStart + 1, LoopModeStart);
      String LoopMode = inData.substring(LoopModeStart + 2);
      LoopMode.trim();
      J1LoopMode = LoopMode.substring(0, 1).toInt();
      J2LoopMode = LoopMode.substring(1, 2).toInt();
      J3LoopMode = LoopMode.substring(2, 3).toInt();
      J4LoopMode = LoopMode.substring(3, 4).toInt();
      J5LoopMode = LoopMode.substring(4, 5).toInt();
      J6LoopMode = LoopMode.substring(5).toInt();


      //determine length between each point (lengths of triangle)
      Xvect = xEnd - xMid;
      Yvect = yEnd - yMid;
      Zvect = zEnd - zMid;
      float aDist = pow((pow((Xvect), 2) + pow((Yvect), 2) + pow((Zvect), 2)), .5);
      Xvect = xEnd - xBeg;
      Yvect = yEnd - yBeg;
      Zvect = zEnd - zBeg;
      float bDist = pow((pow((Xvect), 2) + pow((Yvect), 2) + pow((Zvect), 2)), .5);
      Xvect = xMid - xBeg;
      Yvect = yMid - yBeg;
      Zvect = zMid - zBeg;
      float cDist = pow((pow((Xvect), 2) + pow((Yvect), 2) + pow((Zvect), 2)), .5);
      //use lengths between each point (lengths of triangle) to determine radius
      float s = (aDist + bDist + cDist) / 2;
      float Radius = aDist * bDist * cDist / 4 / sqrt(s * (s - aDist) * (s - bDist) * (s - cDist));
      //find barycentric coordinates of triangle (center of triangle)
      float BCx = pow(aDist, 2) * (pow(bDist, 2) + pow(cDist, 2) - pow(aDist, 2));
      float BCy = pow(bDist, 2) * (pow(cDist, 2) + pow(aDist, 2) - pow(bDist, 2));
      float BCz = pow(cDist, 2) * (pow(aDist, 2) + pow(bDist, 2) - pow(cDist, 2));
      //find center coordinates of circle - convert barycentric coordinates to cartesian coordinates - dot product of 3 points and barycentric coordiantes divided by sum of barycentric coordinates
      float Px = ((BCx * xBeg) + (BCy * xMid) + (BCz * xEnd)) / (BCx + BCy + BCz) ;
      float Py = ((BCx * yBeg) + (BCy * yMid) + (BCz * yEnd)) / (BCx + BCy + BCz) ;
      float Pz = ((BCx * zBeg) + (BCy * zMid) + (BCz * zEnd)) / (BCx + BCy + BCz) ;
      //define start vetor
      startVect [0] = (xBeg - Px);
      startVect [1] = (yBeg - Py);
      startVect [2] = (zBeg - Pz);
      //get 3 vectors from center of circle to begining target, mid target and end target then normalize
      float vect_Amag = pow((pow((xBeg - Px), 2) + pow((yBeg - Py), 2) + pow((zBeg - Pz), 2)), .5);
      float vect_Ax = (xBeg - Px) / vect_Amag;
      float vect_Ay = (yBeg - Py) / vect_Amag;
      float vect_Az = (zBeg - Pz) / vect_Amag;
      float vect_Bmag = pow((pow((xMid - Px), 2) + pow((yMid - Py), 2) + pow((zMid - Pz), 2)), .5);
      float vect_Bx = (xMid - Px) / vect_Bmag;
      float vect_By = (yMid - Py) / vect_Bmag;
      float vect_Bz = (zMid - Pz) / vect_Bmag;
      float vect_Cmag = pow((pow((xEnd - Px), 2) + pow((yEnd - Py), 2) + pow((zEnd - Pz), 2)), .5);
      float vect_Cx = (xEnd - Px) / vect_Cmag;
      float vect_Cy = (yEnd - Py) / vect_Cmag;
      float vect_Cz = (zEnd - Pz) / vect_Cmag;
      //get cross product of vectors a & c than apply to axis matrix
      float CrossX = (vect_Ay * vect_Bz) - (vect_Az * vect_By);
      float CrossY = (vect_Az * vect_Bx) - (vect_Ax * vect_Bz);
      float CrossZ = (vect_Ax * vect_By) - (vect_Ay * vect_Bx);
      axis [0] = CrossX / sqrt((CrossX * CrossX) + (CrossY * CrossY) + (CrossZ * CrossZ));
      axis [1] = CrossY / sqrt((CrossX * CrossX) + (CrossY * CrossY) + (CrossZ * CrossZ));
      axis [2] = CrossZ / sqrt((CrossX * CrossX) + (CrossY * CrossY) + (CrossZ * CrossZ));
      //get radian angle between vectors using acos of dot product
      float ABradians = acos((vect_Ax * vect_Bx + vect_Ay * vect_By + vect_Az * vect_Bz) / (sqrt(pow(vect_Ax , 2) + pow(vect_Ay , 2) + pow(vect_Az , 2)) * sqrt(pow(vect_Bx , 2) + pow(vect_By , 2) + pow(vect_Bz , 2)))  );
      float BCradians = acos((vect_Bx * vect_Cx + vect_By * vect_Cy + vect_Bz * vect_Cz) / (sqrt(pow(vect_Bx , 2) + pow(vect_By , 2) + pow(vect_Bz , 2)) * sqrt(pow(vect_Cx , 2) + pow(vect_Cy , 2) + pow(vect_Cz , 2)))  );
      //get total degrees of both arcs
      float ABdegrees = degrees(ABradians + BCradians);
      //get arc length and calc way pt gap

      float anglepercent = ABdegrees / 360;
      float circumference = 2 * 3.14159265359 * Radius;
      float lineDist = circumference * anglepercent;
      float wayPts = lineDist / linWayDistSP;

      float wayPerc = 1 / wayPts;
      //cacl way pt angle
      float theta_Deg = (ABdegrees / wayPts);

      //determine steps
      int HighStep = lineDist / .05;
      float ACCStep = HighStep * (ACCspd / 100);
      float NORStep = HighStep * ((100 - ACCspd - DCCspd) / 100);
      float DCCStep = HighStep * (DCCspd / 100);

      //set speed for seconds or mm per sec
      if (SpeedType == "s") {
        speedSP = (SpeedVal * 1000000) * 1.2;
      }
      else if (SpeedType == "m") {
        speedSP = ((lineDist / SpeedVal) * 1000000) * 1.2;
      }

      //calc step gap for seconds or mm per sec
      if (SpeedType == "s" or SpeedType == "m" ) {
        float zeroStepGap = speedSP / HighStep;
        float zeroACCstepInc = (zeroStepGap * (100 / ACCramp)) / ACCStep;
        float zeroACCtime = ((ACCStep) * zeroStepGap) + ((ACCStep - 9) * (((ACCStep) * (zeroACCstepInc / 2))));
        float zeroNORtime = NORStep * zeroStepGap;
        float zeroDCCstepInc = (zeroStepGap * (100 / ACCramp)) / DCCStep;
        float zeroDCCtime = ((DCCStep) * zeroStepGap) + ((DCCStep - 9) * (((DCCStep) * (zeroDCCstepInc / 2))));
        float zeroTOTtime = zeroACCtime + zeroNORtime + zeroDCCtime;
        float overclockPerc = speedSP / zeroTOTtime;
        calcStepGap = zeroStepGap * overclockPerc;
        if (calcStepGap <= minSpeedDelay) {
          calcStepGap = minSpeedDelay;
          speedViolation = "1";
        }
      }

      //calc step gap for percentage
      else if (SpeedType == "p") {
        calcStepGap = minSpeedDelay / (SpeedVal / 100);
      }

      //calculate final step increments
      float calcACCstepInc = (calcStepGap * (100 / ACCramp)) / ACCStep;
      float calcDCCstepInc = (calcStepGap * (100 / ACCramp)) / DCCStep;
      float calcACCstartDel = (calcACCstepInc * ACCStep) * 2;
      float calcDCCendDel = (calcDCCstepInc * DCCStep) * 2;


      //calc way pt speeds
      float ACCwayPts = wayPts * (ACCspd / 100);
      float NORwayPts = wayPts * ((100 - ACCspd - DCCspd) / 100);
      float DCCwayPts = wayPts * (DCCspd / 100);

      //calc way inc for lin way steps
      float ACCwayInc = (calcACCstartDel - calcStepGap) / ACCwayPts;
      float DCCwayInc = (calcDCCendDel - calcStepGap) / DCCwayPts;

      //set starting delsy
      if (rndTrue == true) {
        curDelay = rndSpeed;
      }
      else {
        curDelay = calcACCstartDel;
      }


      //set starting angle first way pt
      float cur_deg = theta_Deg;

      /////////////////////////////////////
      //loop through waypoints
      ////////////////////////////////////

      resetEncoders();

      for (int i = 0; i <= wayPts - 1; i++) {

        theta = radians(cur_deg);
        //use euler rodrigues formula to find rotation vector
        a = cos(theta / 2.0);
        b = -axis [0] * sin(theta / 2.0);
        c = -axis [1] * sin(theta / 2.0);
        d = -axis [2] * sin(theta / 2.0);
        aa = a * a;
        bb = b * b;
        cc = c * c;
        dd = d * d;
        bc = b * c;
        ad = a * d;
        ac = a * c;
        ab = a * b;
        bd = b * d;
        cd = c * d;
        Rotation [0][0] = aa + bb - cc - dd;
        Rotation [0][1] = 2 * (bc + ad);
        Rotation [0][2] = 2 * (bd - ac);
        Rotation [1][0] = 2 * (bc - ad);
        Rotation [1][1] = aa + cc - bb - dd;
        Rotation [1][2] = 2 * (cd + ab);
        Rotation [2][0] = 2 * (bd + ac);
        Rotation [2][1] = 2 * (cd - ab);
        Rotation [2][2] = aa + dd - bb - cc;

        //get product of current rotation and start vector
        DestPt[0] = (Rotation [0][0] * startVect[0]) + (Rotation [0][1] * startVect[1]) + (Rotation [0][2] * startVect[2]);
        DestPt[1] = (Rotation [1][0] * startVect[0]) + (Rotation [1][1] * startVect[1]) + (Rotation [1][2] * startVect[2]);
        DestPt[2] = (Rotation [2][0] * startVect[0]) + (Rotation [2][1] * startVect[1]) + (Rotation [2][2] * startVect[2]);

        ////DELAY CALC/////
        if (rndTrue == true) {
          curDelay = rndSpeed;
        }
        else if (i <= ACCwayPts) {
          curDelay = curDelay - (ACCwayInc);
        }
        else if (i >= (wayPts - DCCwayPts)) {
          curDelay = curDelay + (DCCwayInc);
        }
        else {
          curDelay = calcStepGap;
        }

        //shift way pts back to orignal origin and calc kinematics for way pt movement
        float curWayPerc = wayPerc * i;
        xyzuvw_In[0] = (DestPt[0]) + Px;
        xyzuvw_In[1] = (DestPt[1]) + Py;
        xyzuvw_In[2] = (DestPt[2]) + Pz;
        xyzuvw_In[3] = rzBeg - (RZvect * curWayPerc);
        xyzuvw_In[4] = ryBeg - (RYvect * curWayPerc);
        xyzuvw_In[5] = rxBeg - (RXvect * curWayPerc);


        SolveInverseKinematic();

        //calc destination motor steps
        int J1futStepM = (JangleOut[0] + J1axisLimNeg) * J1StepDeg;
        int J2futStepM = (JangleOut[1] + J2axisLimNeg) * J2StepDeg;
        int J3futStepM = (JangleOut[2] + J3axisLimNeg) * J3StepDeg;
        int J4futStepM = (JangleOut[3] + J4axisLimNeg) * J4StepDeg;
        int J5futStepM = (JangleOut[4] + J5axisLimNeg) * J5StepDeg;
        int J6futStepM = (JangleOut[5] + J6axisLimNeg) * J6StepDeg;

        //calc delta from current to destination
        int J1stepDif = J1StepM - J1futStepM;
        int J2stepDif = J2StepM - J2futStepM;
        int J3stepDif = J3StepM - J3futStepM;
        int J4stepDif = J4StepM - J4futStepM;
        int J5stepDif = J5StepM - J5futStepM;
        int J6stepDif = J6StepM - J6futStepM;
        int J7stepDif = 0;
        int J8stepDif = 0;
        int J9stepDif = 0;

        //determine motor directions
        if (J1stepDif <= 0) {
          J1dir = 1;
        }
        else {
          J1dir = 0;
        }

        if (J2stepDif <= 0) {
          J2dir = 1;
        }
        else {
          J2dir = 0;
        }

        if (J3stepDif <= 0) {
          J3dir = 1;
        }
        else {
          J3dir = 0;
        }

        if (J4stepDif <= 0) {
          J4dir = 1;
        }
        else {
          J4dir = 0;
        }

        if (J5stepDif <= 0) {
          J5dir = 1;
        }
        else {
          J5dir = 0;
        }

        if (J6stepDif <= 0) {
          J6dir = 1;
        }
        else {
          J6dir = 0;
        }

        J7dir = 0;
        J8dir = 0;
        J9dir = 0;

        //determine if requested position is within axis limits
        if ((J1dir == 1 and (J1StepM + J1stepDif > J1StepLim)) or (J1dir == 0 and (J1StepM - J1stepDif < 0))) {
          J1axisFault = 1;
        }
        if ((J2dir == 1 and (J2StepM + J2stepDif > J2StepLim)) or (J2dir == 0 and (J2StepM - J2stepDif < 0))) {
          J2axisFault = 1;
        }
        if ((J3dir == 1 and (J3StepM + J3stepDif > J3StepLim)) or (J3dir == 0 and (J3StepM - J3stepDif < 0))) {
          J3axisFault = 1;
        }
        if ((J4dir == 1 and (J4StepM + J4stepDif > J4StepLim)) or (J4dir == 0 and (J4StepM - J4stepDif < 0))) {
          J4axisFault = 1;
        }
        if ((J5dir == 1 and (J5StepM + J5stepDif > J5StepLim)) or (J5dir == 0 and (J5StepM - J5stepDif < 0))) {
          J5axisFault = 1;
        }
        if ((J6dir == 1 and (J6StepM + J6stepDif > J6StepLim)) or (J6dir == 0 and (J6StepM - J6stepDif < 0))) {
          J6axisFault = 1;
        }
        TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault;


        //send move command if no axis limit error
        if (TotalAxisFault == 0 && KinematicError == 0) {
          driveMotorsL(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(J7stepDif), abs(J8stepDif), abs(J9stepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, curDelay);
        }
        else if (KinematicError == 1) {
          Alarm = "ER";
          if (splineTrue == false) {
            delay(5);
            Serial.println(Alarm);
          }
        }
        else {
          Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault);
          if (splineTrue == false) {
            delay(5);
            Serial.println(Alarm);
          }
        }

        //increment angle
        cur_deg += theta_Deg;

      }
      checkEncoders();
      rndTrue = false;
      inData = ""; // Clear recieved buffer
      if (splineTrue == false) {
        sendRobotPos();
      }
      ////////MOVE COMPLETE///////////
    }

    else
    {
      inData = ""; // Clear recieved buffer
    }

    //shift cmd buffer
    inData = "";
    cmdBuffer1 = "";
    shiftCMDarray();

  }
}
