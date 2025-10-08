//VERSION 6.2

/*  AR4 Robot Control Software
    Copyright (c) 2024, Chris Annin
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
// 4.2 - 1/12/24 - bug fix - step direction delay
// 4.3 - 1/21/24 - Gcode to SD card.  Estop button interrupt.
// 4.3.1 - 2/1/24 bug fix - vision snap and find drop down
// 4.4 - 3/2/24 added kinematic error handling
// 4.5 - 6/29/24 simplified drive motors functions with arrays
// 5.0 - 7/14/24 updated kinematics
// 5.1 - 2/15/25 Modbus option
// 5.2 - 6/7/25 Modbus option
// 6.0 - 6/7/25 Virtual Robot
// 6.1 - 8/29/25 updated accel and decel, auto calibrate & microsteps
// 6.2 - 8/29/25 9/12/25 changed bootstrap theme, xbox upgrade

#include <math.h>
#include <limits>
#include <avr/pgmspace.h>
#include <Encoder.h>
#include <SPI.h>
#include <SD.h>
#include <stdexcept>
#include <ModbusMaster.h>
#pragma GCC diagnostic ignored "-Warray-bounds"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wsequence-point"
#pragma GCC diagnostic ignored "-Wunused-value"
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#pragma GCC diagnostic ignored "-Waddress"
#pragma GCC diagnostic ignored "-Wall"

#define Table_Size 6
typedef float Matrix4x4[16];
typedef float tRobot[66];

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
const int J9stepPin = 40;
const int J9dirPin = 41;

const int J1calPin = 26;
const int J2calPin = 27;
const int J3calPin = 28;
const int J4calPin = 29;
const int J5calPin = 30;
const int J6calPin = 31;
const int J7calPin = 36;
const int J8calPin = 37;
const int J9calPin = 38;

const int EstopPin = 39;




//set encoder multiplier
float J1encMult = 10;
float J2encMult = 10;
float J3encMult = 10;
float J4encMult = 10;
float J5encMult = 5;
float J6encMult = 10;
int encOffset = 50;

//set encoder pins
Encoder J1encPos(14, 15);
Encoder J2encPos(17, 16);
Encoder J3encPos(19, 18);
Encoder J4encPos(20, 21);
Encoder J5encPos(23, 22);
Encoder J6encPos(24, 25);

ModbusMaster node;


// GLOBAL VARS //

//define axis limits in degrees
float J1axisLimPos = 170;
float J1axisLimNeg = 170;
float J2axisLimPos = 90;
float J2axisLimNeg = 42;
float J3axisLimPos = 52;
float J3axisLimNeg = 89;
float J4axisLimPos = 180;
float J4axisLimNeg = 180;
float J5axisLimPos = 105;
float J5axisLimNeg = 105;
float J6axisLimPos = 180;
float J6axisLimNeg = 180;
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
float J1StepDeg = 88.888;
float J2StepDeg = 111.111;
float J3StepDeg = 111.111;
float J4StepDeg = 99.555;
float J5StepDeg = 43.720;
float J6StepDeg = 44.444;
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

//step at axis zero
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
float J5calBaseOff = 3.1;
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
float minSpeedDelay = 200;
float maxMMperSec = 192;
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
const int numJoints = 9;
typedef float tRobotJoints[ROBOT_nDOFs];
typedef float tRobotPose[ROBOT_nDOFs];

//declare in out vars
float xyzuvw_Out[ROBOT_nDOFs];
float xyzuvw_In[ROBOT_nDOFs];
float xyzuvw_Temp[ROBOT_nDOFs];

float JangleOut[ROBOT_nDOFs];
float JangleIn[ROBOT_nDOFs];
float joints_estimate[ROBOT_nDOFs];
float SolutionMatrix[ROBOT_nDOFs][4];

//external axis
float J7_pos;
float J8_pos;
float J9_pos;

float J7_In;
float J8_In;
float J9_In;

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
bool estopActive;

float Xtool = 0;
float Ytool = 0;
float Ztool = 0;
float RZtool = 0;
float RYtool = 0;
float RXtool = 0;


//DENAVIT HARTENBERG PARAMETERS

float DHparams[6][4] = {
  { 0, 0, 169.77, 0 },
  { -90, -90, 0, 64.2 },
  { 0, 0, 0, 305 },
  { 0, -90, 222.63, 0 },
  { 0, 90, 0, 0 },
  { 180, -90, 41, 0 }
};


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MATRIX OPERATION
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//This allow to return a array as an argument instead of using global pointer

#define Matrix_Multiply(out, inA, inB) \
  (out)[0] = (inA)[0] * (inB)[0] + (inA)[4] * (inB)[1] + (inA)[8] * (inB)[2]; \
  (out)[1] = (inA)[1] * (inB)[0] + (inA)[5] * (inB)[1] + (inA)[9] * (inB)[2]; \
  (out)[2] = (inA)[2] * (inB)[0] + (inA)[6] * (inB)[1] + (inA)[10] * (inB)[2]; \
  (out)[3] = 0; \
  (out)[4] = (inA)[0] * (inB)[4] + (inA)[4] * (inB)[5] + (inA)[8] * (inB)[6]; \
  (out)[5] = (inA)[1] * (inB)[4] + (inA)[5] * (inB)[5] + (inA)[9] * (inB)[6]; \
  (out)[6] = (inA)[2] * (inB)[4] + (inA)[6] * (inB)[5] + (inA)[10] * (inB)[6]; \
  (out)[7] = 0; \
  (out)[8] = (inA)[0] * (inB)[8] + (inA)[4] * (inB)[9] + (inA)[8] * (inB)[10]; \
  (out)[9] = (inA)[1] * (inB)[8] + (inA)[5] * (inB)[9] + (inA)[9] * (inB)[10]; \
  (out)[10] = (inA)[2] * (inB)[8] + (inA)[6] * (inB)[9] + (inA)[10] * (inB)[10]; \
  (out)[11] = 0; \
  (out)[12] = (inA)[0] * (inB)[12] + (inA)[4] * (inB)[13] + (inA)[8] * (inB)[14] + (inA)[12]; \
  (out)[13] = (inA)[1] * (inB)[12] + (inA)[5] * (inB)[13] + (inA)[9] * (inB)[14] + (inA)[13]; \
  (out)[14] = (inA)[2] * (inB)[12] + (inA)[6] * (inB)[13] + (inA)[10] * (inB)[14] + (inA)[14]; \
  (out)[15] = 1;

#define Matrix_Inv(out, in) \
  (out)[0] = (in)[0]; \
  (out)[1] = (in)[4]; \
  (out)[2] = (in)[8]; \
  (out)[3] = 0; \
  (out)[4] = (in)[1]; \
  (out)[5] = (in)[5]; \
  (out)[6] = (in)[9]; \
  (out)[7] = 0; \
  (out)[8] = (in)[2]; \
  (out)[9] = (in)[6]; \
  (out)[10] = (in)[10]; \
  (out)[11] = 0; \
  (out)[12] = -((in)[0] * (in)[12] + (in)[1] * (in)[13] + (in)[2] * (in)[14]); \
  (out)[13] = -((in)[4] * (in)[12] + (in)[5] * (in)[13] + (in)[6] * (in)[14]); \
  (out)[14] = -((in)[8] * (in)[12] + (in)[9] * (in)[13] + (in)[10] * (in)[14]); \
  (out)[15] = 1;

#define Matrix_Copy(out, in) \
  (out)[0] = (in)[0]; \
  (out)[1] = (in)[1]; \
  (out)[2] = (in)[2]; \
  (out)[3] = (in)[3]; \
  (out)[4] = (in)[4]; \
  (out)[5] = (in)[5]; \
  (out)[6] = (in)[6]; \
  (out)[7] = (in)[7]; \
  (out)[8] = (in)[8]; \
  (out)[9] = (in)[9]; \
  (out)[10] = (in)[10]; \
  (out)[11] = (in)[11]; \
  (out)[12] = (in)[12]; \
  (out)[13] = (in)[13]; \
  (out)[14] = (in)[14]; \
  (out)[15] = (in)[15];

#define Matrix_Eye(inout) \
  (inout)[0] = 1; \
  (inout)[1] = 0; \
  (inout)[2] = 0; \
  (inout)[3] = 0; \
  (inout)[4] = 0; \
  (inout)[5] = 1; \
  (inout)[6] = 0; \
  (inout)[7] = 0; \
  (inout)[8] = 0; \
  (inout)[9] = 0; \
  (inout)[10] = 1; \
  (inout)[11] = 0; \
  (inout)[12] = 0; \
  (inout)[13] = 0; \
  (inout)[14] = 0; \
  (inout)[15] = 1;

#define Matrix_Multiply_Cumul(inout, inB) \
  { \
    Matrix4x4 out; \
    Matrix_Multiply(out, inout, inB); \
    Matrix_Copy(inout, out); \
  }


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//DECLARATION OF VARIABLES
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/// DHM Table parameters
#define DHM_Alpha 0
#define DHM_A 1
#define DHM_Theta 2
#define DHM_D 3


/// Custom robot base (user frame)
Matrix4x4 Robot_BaseFrame = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

/// Custom robot tool (tool frame, end of arm tool or TCP)
Matrix4x4 Robot_ToolFrame = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

/// Robot parameters
/// All robot data is held in a large array
tRobot Robot_Data = { 0 };


//These global variable are also pointers, allowing to put the variables inside the Robot_Data
/// DHM table
float *Robot_Kin_DHM_Table = Robot_Data + 0 * Table_Size;

/// xyzwpr of the base
float *Robot_Kin_Base = Robot_Data + 6 * Table_Size;

/// xyzwpr of the tool
float *Robot_Kin_Tool = Robot_Data + 7 * Table_Size;

/// Robot lower limits
float *Robot_JointLimits_Upper = Robot_Data + 8 * Table_Size;

/// Robot upper limits
float *Robot_JointLimits_Lower = Robot_Data + 9 * Table_Size;

/// Robot axis senses
float *Robot_Senses = Robot_Data + 10 * Table_Size;

// A value mappings

float *Robot_Kin_DHM_L1 = Robot_Kin_DHM_Table + 0 * Table_Size;
float *Robot_Kin_DHM_L2 = Robot_Kin_DHM_Table + 1 * Table_Size;
float *Robot_Kin_DHM_L3 = Robot_Kin_DHM_Table + 2 * Table_Size;
float *Robot_Kin_DHM_L4 = Robot_Kin_DHM_Table + 3 * Table_Size;
float *Robot_Kin_DHM_L5 = Robot_Kin_DHM_Table + 4 * Table_Size;
float *Robot_Kin_DHM_L6 = Robot_Kin_DHM_Table + 5 * Table_Size;


float &Robot_Kin_DHM_A2(Robot_Kin_DHM_Table[1 * Table_Size + 1]);
float &Robot_Kin_DHM_A3(Robot_Kin_DHM_Table[2 * Table_Size + 1]);
float &Robot_Kin_DHM_A4(Robot_Kin_DHM_Table[3 * Table_Size + 1]);

// D value mappings
float &Robot_Kin_DHM_D1(Robot_Kin_DHM_Table[0 * Table_Size + 3]);
float &Robot_Kin_DHM_D2(Robot_Kin_DHM_Table[1 * Table_Size + 3]);
float &Robot_Kin_DHM_D4(Robot_Kin_DHM_Table[3 * Table_Size + 3]);
float &Robot_Kin_DHM_D6(Robot_Kin_DHM_Table[5 * Table_Size + 3]);

// Theta value mappings (mastering)
float &Robot_Kin_DHM_Theta1(Robot_Kin_DHM_Table[0 * Table_Size + 2]);
float &Robot_Kin_DHM_Theta2(Robot_Kin_DHM_Table[1 * Table_Size + 2]);
float &Robot_Kin_DHM_Theta3(Robot_Kin_DHM_Table[2 * Table_Size + 2]);
float &Robot_Kin_DHM_Theta4(Robot_Kin_DHM_Table[3 * Table_Size + 2]);
float &Robot_Kin_DHM_Theta5(Robot_Kin_DHM_Table[4 * Table_Size + 2]);
float &Robot_Kin_DHM_Theta6(Robot_Kin_DHM_Table[5 * Table_Size + 2]);



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool containsNullByte(float value) {
  const unsigned char *bytes = reinterpret_cast<const unsigned char *>(&value);
  for (size_t i = 0; i < sizeof(float); ++i) {
    if (bytes[i] == 0x00) {
      return true;
    }
  }
  return false;
}

bool isValidResult(float value) {
  // Check for NaN or Inf, which are typical results of invalid operations
  return !std::isnan(value) && !std::isinf(value);
}


//This function sets the variable inside Robot_Data to the DHparams
void robot_set_AR() {
  robot_data_reset();

  // Alpha parameters
  Robot_Kin_DHM_L1[DHM_Alpha] = DHparams[0][1] * M_PI / 180;
  Robot_Kin_DHM_L2[DHM_Alpha] = DHparams[1][1] * M_PI / 180;
  Robot_Kin_DHM_L3[DHM_Alpha] = DHparams[2][1] * M_PI / 180;
  Robot_Kin_DHM_L4[DHM_Alpha] = DHparams[3][1] * M_PI / 180;
  Robot_Kin_DHM_L5[DHM_Alpha] = DHparams[4][1] * M_PI / 180;
  Robot_Kin_DHM_L6[DHM_Alpha] = DHparams[5][1] * M_PI / 180;

  // Theta parameters
  Robot_Kin_DHM_L1[DHM_Theta] = DHparams[0][0] * M_PI / 180;
  Robot_Kin_DHM_L2[DHM_Theta] = DHparams[1][0] * M_PI / 180;
  Robot_Kin_DHM_L3[DHM_Theta] = DHparams[2][0] * M_PI / 180;
  Robot_Kin_DHM_L4[DHM_Theta] = DHparams[3][0] * M_PI / 180;
  Robot_Kin_DHM_L5[DHM_Theta] = DHparams[4][0] * M_PI / 180;
  Robot_Kin_DHM_L6[DHM_Theta] = DHparams[5][0] * M_PI / 180;

  // A parameters
  Robot_Kin_DHM_L1[DHM_A] = DHparams[0][3];
  Robot_Kin_DHM_L2[DHM_A] = DHparams[1][3];
  Robot_Kin_DHM_L3[DHM_A] = DHparams[2][3];
  Robot_Kin_DHM_L4[DHM_A] = DHparams[3][3];
  Robot_Kin_DHM_L5[DHM_A] = DHparams[4][3];
  Robot_Kin_DHM_L6[DHM_A] = DHparams[5][3];

  // D parameters
  Robot_Kin_DHM_L1[DHM_D] = DHparams[0][2];
  Robot_Kin_DHM_L2[DHM_D] = DHparams[1][2];
  Robot_Kin_DHM_L3[DHM_D] = DHparams[2][2];
  Robot_Kin_DHM_L4[DHM_D] = DHparams[3][2];
  Robot_Kin_DHM_L5[DHM_D] = DHparams[4][2];
  Robot_Kin_DHM_L6[DHM_D] = DHparams[5][2];


  Robot_JointLimits_Lower[0] = J1axisLimNeg;
  Robot_JointLimits_Upper[0] = J1axisLimPos;
  Robot_JointLimits_Lower[1] = J2axisLimNeg;
  Robot_JointLimits_Upper[1] = J2axisLimPos;
  Robot_JointLimits_Lower[2] = J3axisLimNeg;
  Robot_JointLimits_Upper[2] = J3axisLimPos;
  Robot_JointLimits_Lower[3] = J4axisLimNeg;
  Robot_JointLimits_Upper[3] = J4axisLimPos;
  Robot_JointLimits_Lower[4] = J5axisLimNeg;
  Robot_JointLimits_Upper[4] = J5axisLimPos;
  Robot_JointLimits_Lower[5] = J6axisLimNeg;
  Robot_JointLimits_Upper[5] = J6axisLimPos;
}

void robot_data_reset() {
  // Reset user base and tool frames
  Matrix_Eye(Robot_BaseFrame);
  Matrix_Eye(Robot_ToolFrame);

  // Reset internal base frame and tool frames
  for (int i = 0; i < 6; i++) {
    Robot_Kin_Base[i] = 0.0;
  }

  // Reset joint senses and joint limits
  for (int i = 0; i < ROBOT_nDOFs; i++) {
    Robot_Senses[i] = +1.0;
  }
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MATRICE OPERATIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<typename T>
bool robot_joints_valid(const T joints[ROBOT_nDOFs]) {

  for (int i = 0; i < ROBOT_nDOFs; i++) {
    if (joints[i] < -Robot_JointLimits_Lower[i] || joints[i] > Robot_JointLimits_Upper[i]) {
      return false;
    }
  }
  return true;
}


//This function returns a 4x4 matrix as an argument (pose) following the modified DH rules for the inputs T rx, T tx, T rz and T tz source : https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
template<typename T>
void DHM_2_pose(T rx, T tx, T rz, T tz, Matrix4x4 pose) {
  T crx;
  T srx;
  T crz;
  T srz;
  crx = cos(rx);
  srx = sin(rx);
  crz = cos(rz);
  srz = sin(rz);
  pose[0] = crz;
  pose[4] = -srz;
  pose[8] = 0.0;
  pose[12] = tx;
  pose[1] = crx * srz;
  pose[5] = crx * crz;
  pose[9] = -srx;
  pose[13] = -tz * srx;
  pose[2] = srx * srz;
  pose[6] = crz * srx;
  pose[10] = crx;
  pose[14] = tz * crx;
  pose[3] = 0.0;
  pose[7] = 0.0;
  pose[11] = 0.0;
  pose[15] = 1.0;
}


//This function tranforms a coordinate system xyzwpr into a 4x4 matrix and return it as an argument.
template<typename T>
void xyzwpr_2_pose(const T xyzwpr[6], Matrix4x4 pose) {
  T srx;
  T crx;
  T sry;
  T cry;
  T srz;
  T crz;
  T H_tmp;
  srx = sin(xyzwpr[3]);
  crx = cos(xyzwpr[3]);
  sry = sin(xyzwpr[4]);
  cry = cos(xyzwpr[4]);
  srz = sin(xyzwpr[5]);
  crz = cos(xyzwpr[5]);
  pose[0] = cry * crz;
  pose[4] = -cry * srz;
  pose[8] = sry;
  pose[12] = xyzwpr[0];
  H_tmp = crz * srx;
  pose[1] = crx * srz + H_tmp * sry;
  crz *= crx;
  pose[5] = crz - srx * sry * srz;
  pose[9] = -cry * srx;
  pose[13] = xyzwpr[1];
  pose[2] = srx * srz - crz * sry;
  pose[6] = H_tmp + crx * sry * srz;
  pose[10] = crx * cry;
  pose[14] = xyzwpr[2];
  pose[3] = 0.0;
  pose[7] = 0.0;
  pose[11] = 0.0;
  pose[15] = 1.0;
}


/// Calculate the [x,y,z,u,v,w] position with rotation vector for a pose target
template<typename T>
void pose_2_xyzuvw(const Matrix4x4 pose, T out[6]) {
  T sin_angle;
  T angle;
  T vector[3];
  int iidx;
  int vector_tmp;
  signed char b_I[9];
  out[0] = pose[12];
  out[1] = pose[13];
  out[2] = pose[14];
  sin_angle = (((pose[0] + pose[5]) + pose[10]) - 1.0) * 0.5;
  if (sin_angle <= -1.0) {
    sin_angle = -1.0;
  }

  if (sin_angle >= 1.0) {
    sin_angle = 1.0;
  }

  angle = acos(sin_angle);
  if (angle < 1.0E-6) {
    vector[0] = 0.0;
    vector[1] = 0.0;
    vector[2] = 0.0;
  } else {
    sin_angle = sin(angle);
    if (abs(sin_angle) < 1.0E-6) {  //IMPOTANT : cosinus of 90 give a really small number instead of 0, the result is forced back to what it should
      sin_angle = pose[0];
      iidx = 0;
      if (pose[0] < pose[5]) {
        sin_angle = pose[5];
        iidx = 1;
      }

      if (sin_angle < pose[10]) {
        sin_angle = pose[10];
        iidx = 2;
      }

      for (vector_tmp = 0; vector_tmp < 9; vector_tmp++) {
        b_I[vector_tmp] = 0;
      }

      b_I[0] = 1;
      b_I[4] = 1;
      b_I[8] = 1;
      sin_angle = 2.0 * (1.0 + sin_angle);
      if (sin_angle <= 0.0) {
        sin_angle = 0.0;
      } else {
        sin_angle = sqrt(sin_angle);
      }

      vector_tmp = iidx << 2;
      vector[0] = (pose[vector_tmp] + static_cast<T>(b_I[3 * iidx])) / sin_angle;
      vector[1] = (pose[1 + vector_tmp] + static_cast<T>(b_I[1 + 3 * iidx]))
                  / sin_angle;
      vector[2] = (pose[2 + vector_tmp] + static_cast<T>(b_I[2 + 3 * iidx]))
                  / sin_angle;
      angle = M_PI;
    } else {
      sin_angle = 1.0 / (2.0 * sin_angle);
      vector[0] = (pose[6] - pose[9]) * sin_angle;
      vector[1] = (pose[8] - pose[2]) * sin_angle;
      vector[2] = (pose[1] - pose[4]) * sin_angle;
    }
  }

  sin_angle = angle * 180.0 / M_PI;
  out[3] = vector[0] * sin_angle * M_PI / 180.0;
  out[4] = vector[1] * sin_angle * M_PI / 180.0;
  out[5] = vector[2] * sin_angle * M_PI / 180.0;
}


//This function tranforms a coordinate system xyzwpr into a 4x4 matrix using UR euler rules and return it as an argument.
template<typename T>
void xyzuvw_2_pose(const T xyzuvw[6], Matrix4x4 pose) {
  T s;
  T angle;
  T axisunit[3];
  T ex;
  T c;
  T pose_tmp;
  T b_pose_tmp;
  s = sqrt((xyzuvw[3] * xyzuvw[3] + xyzuvw[4] * xyzuvw[4]) + xyzuvw[5] * xyzuvw[5]);
  angle = s * 180.0 / M_PI;
  if (abs(angle) < 1.0E-6) {  //IMPOTANT : cosinus of 90 give a really small number instead of 0, the result is forced back to what it should
    memset(&pose[0], 0, sizeof(T) << 4);
    pose[0] = 1.0;
    pose[5] = 1.0;
    pose[10] = 1.0;
    pose[15] = 1.0;
  } else {
    axisunit[1] = abs(xyzuvw[4]);
    axisunit[2] = abs(xyzuvw[5]);
    ex = abs(xyzuvw[3]);
    if (abs(xyzuvw[3]) < axisunit[1]) {
      ex = axisunit[1];
    }

    if (ex < axisunit[2]) {
      ex = axisunit[2];
    }

    if (ex < 1.0E-6) {  //IMPOTANT : cosinus of 90 give a really small number instead of 0, the result is forced back to what it should
      memset(&pose[0], 0, sizeof(T) << 4);
      pose[0] = 1.0;
      pose[5] = 1.0;
      pose[10] = 1.0;
      pose[15] = 1.0;
    } else {
      axisunit[0] = xyzuvw[3] / s;
      axisunit[1] = xyzuvw[4] / s;
      axisunit[2] = xyzuvw[5] / s;
      s = angle * 3.1415926535897931 / 180.0;
      c = cos(s);
      s = sin(s);
      angle = axisunit[0] * axisunit[0];
      pose[0] = angle + c * (1.0 - angle);
      angle = axisunit[0] * axisunit[1] * (1.0 - c);
      ex = axisunit[2] * s;
      pose[4] = angle - ex;
      pose_tmp = axisunit[0] * axisunit[2] * (1.0 - c);
      b_pose_tmp = axisunit[1] * s;
      pose[8] = pose_tmp + b_pose_tmp;
      pose[1] = angle + ex;
      angle = axisunit[1] * axisunit[1];
      pose[5] = angle + (1.0 - angle) * c;
      angle = axisunit[1] * axisunit[2] * (1.0 - c);
      ex = axisunit[0] * s;
      pose[9] = angle - ex;
      pose[2] = pose_tmp - b_pose_tmp;
      pose[6] = angle + ex;
      angle = axisunit[2] * axisunit[2];
      pose[10] = angle + (1.0 - angle) * c;
      pose[3] = 0.0;
      pose[7] = 0.0;
      pose[11] = 0.0;
      pose[15] = 1.0;
    }
  }

  pose[12] = xyzuvw[0];
  pose[13] = xyzuvw[1];
  pose[14] = xyzuvw[2];
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//FOWARD KINEMATICS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//This function input the JxangleIn into an array, send it to the foward kinematic solver and output the result into the position variables
void SolveFowardKinematics() {

  robot_set_AR();

  float target_xyzuvw[6];
  float joints[ROBOT_nDOFs];

  for (int i = 0; i < ROBOT_nDOFs; i++) {
    joints[i] = JangleIn[i];
  }

  forward_kinematics_robot_xyzuvw(joints, target_xyzuvw);

  xyzuvw_Out[0] = target_xyzuvw[0];
  xyzuvw_Out[1] = target_xyzuvw[1];
  xyzuvw_Out[2] = target_xyzuvw[2];
  xyzuvw_Out[3] = target_xyzuvw[3] / M_PI * 180;
  xyzuvw_Out[4] = target_xyzuvw[4] / M_PI * 180;
  xyzuvw_Out[5] = target_xyzuvw[5] / M_PI * 180;
}



template<typename T>
void forward_kinematics_arm(const T *joints, Matrix4x4 pose) {
  xyzwpr_2_pose(Robot_Kin_Base, pose);
  for (int i = 0; i < ROBOT_nDOFs; i++) {
    Matrix4x4 hi;
    float *dhm_i = Robot_Kin_DHM_Table + i * Table_Size;
    T ji_rad = joints[i] * Robot_Senses[i] * M_PI / 180.0;
    DHM_2_pose(dhm_i[0], dhm_i[1], dhm_i[2] + ji_rad, dhm_i[3], hi);
    Matrix_Multiply_Cumul(pose, hi);
  }
  Matrix4x4 tool_pose;
  xyzwpr_2_pose(Robot_Kin_Tool, tool_pose);
  Matrix_Multiply_Cumul(pose, tool_pose);
}


template<typename T>
void forward_kinematics_robot_xyzuvw(const T joints[ROBOT_nDOFs], T target_xyzuvw[6]) {
  Matrix4x4 pose;
  forward_kinematics_robot(joints, pose);  //send the joints values and return the pose matrix as an argument
  pose_2_xyzuvw(pose, target_xyzuvw);      //send the pose matrix and return the xyzuvw values in an array as an argument
}

//Calculate de foward kinematic of the robot without the tool
template<typename T>
void forward_kinematics_robot(const T joints[ROBOT_nDOFs], Matrix4x4 target) {
  Matrix4x4 invBaseFrame;
  Matrix4x4 pose_arm;
  Matrix_Inv(invBaseFrame, Robot_BaseFrame);  // invRobot_Tool could be precalculated, the tool does not change so often
  forward_kinematics_arm(joints, pose_arm);
  Matrix_Multiply(target, invBaseFrame, pose_arm);
  Matrix_Multiply_Cumul(target, Robot_ToolFrame);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//REVERSE KINEMATICS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void updatejoints() {

  for (int i = 0; i > ROBOT_nDOFs; i++) {
    JangleIn[i] = JangleOut[i];
  }
}

void JointEstimate() {

  for (int i = 0; i < ROBOT_nDOFs; i++) {
    joints_estimate[i] = JangleIn[i];
  }
}

void SolveInverseKinematics() {

  float joints[ROBOT_nDOFs];
  float target[6];

  float solbuffer[ROBOT_nDOFs] = { 0 };
  int NumberOfSol = 0;
  int solVal = 0;

  KinematicError = 0;

  JointEstimate();
  target[0] = xyzuvw_In[0];
  target[1] = xyzuvw_In[1];
  target[2] = xyzuvw_In[2];
  target[3] = xyzuvw_In[3] * M_PI / 180;
  target[4] = xyzuvw_In[4] * M_PI / 180;
  target[5] = xyzuvw_In[5] * M_PI / 180;

  // Serial.println("X : " + String(target[0]) + " Y : " + String(target[1]) + " Z : " + String(target[2]) + " rx : " + String(xyzuvw_In[3]) + " ry : " + String(xyzuvw_In[4]) + " rz : " + String(xyzuvw_In[5]));


  for (int i = -3; i <= 3; i++) {
    joints_estimate[4] = i * 30;
    int success = inverse_kinematics_robot_xyzuvw<float>(target, joints, joints_estimate);
    if (success) {
      if (solbuffer[4] != joints[4]) {
        if (robot_joints_valid(joints)) {
          for (int j = 0; j < ROBOT_nDOFs; j++) {
            solbuffer[j] = joints[j];
            SolutionMatrix[j][NumberOfSol] = solbuffer[j];
          }
          if (NumberOfSol <= 6) {
            NumberOfSol++;
          }
        }
      }
    } else {
      KinematicError = 1;
    }
  }

  joints_estimate[4] = JangleIn[4];


  solVal = 0;
  for (int i = 0; i < ROBOT_nDOFs; i++) {
    if ((abs(joints_estimate[i] - SolutionMatrix[i][0]) > 20) and NumberOfSol > 1) {
      solVal = 1;
    } else if ((abs(joints_estimate[i] - SolutionMatrix[i][1]) > 20) and NumberOfSol > 1) {
      solVal = 0;
    }


    // Serial.println(String(i) + "  Joint estimate : " + String(joints_estimate[i]) + " // Joint sol 1 : " + String(SolutionMatrix[i][0]) + " // Joint sol 2 : " + String(SolutionMatrix[i][1]));
  }

  if (NumberOfSol == 0) {
    KinematicError = 1;
  }


  // Serial.println("Sol : " + String(solVal) + " Nb sol : " + String(NumberOfSol));

  for (int i = 0; i < ROBOT_nDOFs; i++) {
    JangleOut[i] = SolutionMatrix[i][solVal];
  }
}





template<typename T>
int inverse_kinematics_robot(const Matrix4x4 target, T joints[ROBOT_nDOFs], const T *joints_estimate) {
  Matrix4x4 invToolFrame;
  Matrix4x4 pose_arm;
  int nsol;
  Matrix_Inv(invToolFrame, Robot_ToolFrame);  // invRobot_Tool could be precalculated, the tool does not change so often
  Matrix_Multiply(pose_arm, Robot_BaseFrame, target);
  Matrix_Multiply_Cumul(pose_arm, invToolFrame);
  if (joints_estimate != nullptr) {
    inverse_kinematics_raw(pose_arm, Robot_Data, joints_estimate, joints, &nsol);
  } else {
    // Warning! This is dangerous if joints does not have a valid/reasonable result
    T joints_approx[6];
    memcpy(joints_approx, joints, ROBOT_nDOFs * sizeof(T));
    inverse_kinematics_raw(pose_arm, Robot_Data, joints_approx, joints, &nsol);
  }
  if (nsol == 0) {
    return 0;
  }

  return 1;
}


template<typename T>
int inverse_kinematics_robot_xyzuvw(const T target_xyzuvw1[6], T joints[ROBOT_nDOFs], const T *joints_estimate) {

  Matrix4x4 pose;
  xyzuvw_2_pose(target_xyzuvw1, pose);
  return inverse_kinematics_robot(pose, joints, joints_estimate);
}


template<typename T>
void inverse_kinematics_raw(const T pose[16], const tRobot DK, const T joints_approx_in[6], T joints[6], int *nsol) {
  int i0;
  T base[16];
  T joints_approx[6];
  T tool[16];
  int i;
  T Hout[16];
  T b_Hout[9];
  T dv0[4];
  bool guard1 = false;
  T make_sqrt;
  T P04[4];
  T q1;
  int i1;
  T c_Hout[16];
  T k2;
  T k1;
  T ai;
  T B;
  T C;
  T s31;
  T c31;
  T q13_idx_2;
  T bb_div_cc;
  T q13_idx_0;
  for (i0 = 0; i0 < 6; i0++) {
    joints_approx[i0] = DK[60 + i0] * joints_approx_in[i0];
  }

  //debug = String(Robot_Data[13]) + " * " + String(Robot_Data[19]) + " * " + String(Robot_Data[21]);

  xyzwpr_2_pose(*(T(*)[6]) & DK[36], base);
  xyzwpr_2_pose(*(T(*)[6]) & DK[42], tool);
  for (i0 = 0; i0 < 4; i0++) {
    i = i0 << 2;
    Hout[i] = base[i0];
    Hout[1 + i] = base[i0 + 4];
    Hout[2 + i] = base[i0 + 8];
    Hout[3 + i] = base[i0 + 12];
  }

  for (i0 = 0; i0 < 3; i0++) {
    i = i0 << 2;
    Hout[3 + i] = 0.0;
    b_Hout[3 * i0] = -Hout[i];
    b_Hout[1 + 3 * i0] = -Hout[1 + i];
    b_Hout[2 + 3 * i0] = -Hout[2 + i];
  }

  for (i0 = 0; i0 < 3; i0++) {
    Hout[12 + i0] = (b_Hout[i0] * base[12] + b_Hout[i0 + 3] * base[13]) + b_Hout[i0 + 6] * base[14];
  }

  for (i0 = 0; i0 < 4; i0++) {
    i = i0 << 2;
    base[i] = tool[i0];
    base[1 + i] = tool[i0 + 4];
    base[2 + i] = tool[i0 + 8];
    base[3 + i] = tool[i0 + 12];
  }

  for (i0 = 0; i0 < 3; i0++) {
    i = i0 << 2;
    base[3 + i] = 0.0;
    b_Hout[3 * i0] = -base[i];
    b_Hout[1 + 3 * i0] = -base[1 + i];
    b_Hout[2 + 3 * i0] = -base[2 + i];
  }

  for (i0 = 0; i0 < 3; i0++) {
    base[12 + i0] = (b_Hout[i0] * tool[12] + b_Hout[i0 + 3] * tool[13]) + b_Hout[i0 + 6] * tool[14];
  }

  dv0[0] = 0.0;
  dv0[1] = 0.0;
  dv0[2] = -DK[33];
  dv0[3] = 1.0;
  for (i0 = 0; i0 < 4; i0++) {
    for (i = 0; i < 4; i++) {
      i1 = i << 2;
      c_Hout[i0 + i1] = ((Hout[i0] * pose[i1] + Hout[i0 + 4] * pose[1 + i1]) + Hout[i0 + 8] * pose[2 + i1]) + Hout[i0 + 12] * pose[3 + i1];
    }

    P04[i0] = 0.0;
    for (i = 0; i < 4; i++) {
      i1 = i << 2;
      make_sqrt = ((c_Hout[i0] * base[i1] + c_Hout[i0 + 4] * base[1 + i1]) + c_Hout[i0 + 8] * base[2 + i1]) + c_Hout[i0 + 12] * base[3 + i1];
      tool[i0 + i1] = make_sqrt;
      P04[i0] += make_sqrt * dv0[i];
    }
  }

  guard1 = false;
  if (DK[9] == 0.0) {
    q1 = atan2(P04[1], P04[0]);
    guard1 = true;
  } else {
    make_sqrt = (P04[0] * P04[0] + P04[1] * P04[1]) - DK[9] * DK[9];
    if (make_sqrt < 0.0) {
      for (i = 0; i < 6; i++) {
        joints[i] = 0.0;
      }

      *nsol = 0;
    } else {
      q1 = atan2(P04[1], P04[0]) - atan2(DK[9], sqrt(make_sqrt));
      guard1 = true;
    }
  }

  if (guard1) {
    k2 = P04[2] - DK[3];
    k1 = (cos(q1) * P04[0] + sin(q1) * P04[1]) - DK[7];
    ai = (((k1 * k1 + k2 * k2) - DK[13] * DK[13]) - DK[21] * DK[21]) - DK[19] * DK[19];
    B = 2.0 * DK[21] * DK[13];
    C = 2.0 * DK[19] * DK[13];
    s31 = 0.0;
    c31 = 0.0;
    if (C == 0.0) {
      s31 = -ai / B;
      make_sqrt = 1.0 - s31 * s31;
      if (make_sqrt >= 0.0) {
        c31 = sqrt(make_sqrt);
      }
    } else {
      q13_idx_2 = C * C;
      bb_div_cc = B * B / q13_idx_2;
      make_sqrt = 2.0 * ai * B / q13_idx_2;
      make_sqrt = make_sqrt * make_sqrt - 4.0 * ((1.0 + bb_div_cc) * (ai * ai / q13_idx_2 - 1.0));
      if (make_sqrt >= 0.0) {
        s31 = (-2.0 * ai * B / q13_idx_2 + sqrt(make_sqrt)) / (2.0 * (1.0 + bb_div_cc));
        c31 = (ai + B * s31) / C;
      }
    }

    if ((make_sqrt >= 0.0) && (abs(s31) <= 1.0)) {
      B = atan2(s31, c31);
      make_sqrt = cos(B);
      ai = sin(B);
      C = (DK[13] - DK[21] * ai) + DK[19] * make_sqrt;
      make_sqrt = DK[21] * make_sqrt + DK[19] * ai;
      q13_idx_0 = q1 + -DK[2];
      k2 = atan2(C * k1 - make_sqrt * k2, C * k2 + make_sqrt * k1) + (-DK[8] - M_PI / 2);
      q13_idx_2 = B + -DK[14];
      bb_div_cc = joints_approx[3] * M_PI / 180.0 - (-DK[20]);
      q1 = q13_idx_0 + DK[2];
      B = k2 + DK[8];
      C = q13_idx_2 + DK[14];
      make_sqrt = B + C;
      s31 = cos(make_sqrt);
      c31 = cos(q1);
      Hout[0] = s31 * c31;
      ai = sin(q1);
      Hout[4] = s31 * ai;
      make_sqrt = sin(make_sqrt);
      Hout[8] = -make_sqrt;
      Hout[12] = (DK[3] * make_sqrt - DK[7] * s31) - DK[13] * cos(C);
      Hout[1] = -sin(B + C) * c31;
      Hout[5] = -sin(B + C) * ai;
      Hout[9] = -s31;
      Hout[13] = (DK[3] * s31 + DK[7] * make_sqrt) + DK[13] * sin(C);
      Hout[2] = -ai;
      Hout[6] = c31;
      Hout[10] = 0.0;
      Hout[14] = 0.0;
      Hout[3] = 0.0;
      Hout[7] = 0.0;
      Hout[11] = 0.0;
      Hout[15] = 1.0;
      for (i0 = 0; i0 < 4; i0++) {
        for (i = 0; i < 4; i++) {
          i1 = i << 2;
          base[i0 + i1] = ((Hout[i0] * tool[i1] + Hout[i0 + 4] * tool[1 + i1]) + Hout[i0 + 8] * tool[2 + i1]) + Hout[i0 + 12] * tool[3 + i1];
        }
      }

      make_sqrt = 1.0 - base[9] * base[9];
      if (make_sqrt <= 0.0) {
        make_sqrt = 0.0;
      } else {
        make_sqrt = sqrt(make_sqrt);
      }

      if (make_sqrt < 1.0E-6) {
        C = atan2(make_sqrt, base[9]);
        make_sqrt = sin(bb_div_cc);
        ai = cos(bb_div_cc);
        make_sqrt = atan2(make_sqrt * base[0] + ai * base[2], make_sqrt * base[2] - ai * base[0]);
      } else if (joints_approx[4] >= 0.0) {
        bb_div_cc = atan2(base[10] / make_sqrt, -base[8] / make_sqrt);
        C = atan2(make_sqrt, base[9]);
        make_sqrt = sin(C);
        make_sqrt = atan2(base[5] / make_sqrt, -base[1] / make_sqrt);
      } else {
        bb_div_cc = atan2(-base[10] / make_sqrt, base[8] / make_sqrt);
        C = atan2(-make_sqrt, base[9]);
        make_sqrt = sin(C);
        make_sqrt = atan2(base[5] / make_sqrt, -base[1] / make_sqrt);
      }

      joints[0] = q13_idx_0;
      joints[3] = bb_div_cc + -DK[20];
      joints[1] = k2;
      joints[4] = C + -DK[26];
      joints[2] = q13_idx_2;
      joints[5] = make_sqrt + (-DK[32] + M_PI);
      make_sqrt = joints[5];
      if (joints[5] > 3.1415926535897931) {
        make_sqrt = joints[5] - M_PI * 2;
      } else {
        if (joints[5] <= -M_PI) {
          make_sqrt = joints[5] + M_PI * 2;
        }
      }

      joints[5] = make_sqrt;
      for (i0 = 0; i0 < 6; i0++) {
        joints[i0] = DK[60 + i0] * (joints[i0] * 180.0 / M_PI);
      }

      *nsol = 1.0;
    } else {
      for (i = 0; i < 6; i++) {
        joints[i] = 0.0;
      }

      *nsol = 0;
    }
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

  SolveFowardKinematics();
}



void correctRobotPos() {

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


  SolveFowardKinematics();

  String sendPos = "A" + String(JangleIn[0], 3) + "B" + String(JangleIn[1], 3) + "C" + String(JangleIn[2], 3) + "D" + String(JangleIn[3], 3) + "E" + String(JangleIn[4], 3) + "F" + String(JangleIn[5], 3) + "G" + String(xyzuvw_Out[0], 3) + "H" + String(xyzuvw_Out[1], 3) + "I" + String(xyzuvw_Out[2], 3) + "J" + String(xyzuvw_Out[3], 3) + "K" + String(xyzuvw_Out[4], 3) + "L" + String(xyzuvw_Out[5], 3) + "M" + speedViolation + "N" + debug + "O" + flag + "P" + J7_pos + "Q" + J8_pos + "R" + J9_pos;
  delay(5);
  Serial.println(sendPos);
  speedViolation = "0";
  flag = "";
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SD CARD
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void writeSD(String filename, String info) {
  SD.begin(BUILTIN_SDCARD);
  const char *fn = filename.c_str();
  File gcFile = SD.open(fn, FILE_WRITE);
  if (gcFile) {
    //Serial.print("Writing to test.txt...");
    gcFile.println(info);
    gcFile.close();
    //Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("EG");
  }
}

void deleteSD(String filename) {
  SD.begin(BUILTIN_SDCARD);
  const char *fn = filename.c_str();
  SD.remove(fn);
}

void printDirectory(File dir, int numTabs) {
  String filesSD;
  while (true) {

    File entry = dir.openNextFile();
    if (!entry) {
      // no more files
      Serial.println(filesSD);
      break;
    }
    if (entry.name() != "System Volume Information") {
      filesSD += entry.name();
      filesSD += ",";
    }
    entry.close();
  }
}




/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//DRIVE LIMIT
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void driveLimit(const int steps[], float SpeedVal) {

  const unsigned long DEBOUNCE_US = 3000;  // 3 ms
  unsigned long firstHighUs[numJoints] = { 0 };
  int calcStepGap = minSpeedDelay / (SpeedVal / 100);

  // Define arrays for calibration directions, motor directions, and direction pins
  int calDir[numJoints] = { J1CalDir, J2CalDir, J3CalDir, J4CalDir, J5CalDir, J6CalDir, J7CalDir, J8CalDir, J9CalDir };
  int motDir[numJoints] = { J1MotDir, J2MotDir, J3MotDir, J4MotDir, J5MotDir, J6MotDir, J7MotDir, J8MotDir, J9MotDir };
  int dirPins[numJoints] = { J1dirPin, J2dirPin, J3dirPin, J4dirPin, J5dirPin, J6dirPin, J7dirPin, J8dirPin, J9dirPin };

  // Define arrays for the current state, calibration pins, step pins, steps, completion status, and steps done
  int curState[numJoints];
  int calPins[numJoints] = { J1calPin, J2calPin, J3calPin, J4calPin, J5calPin, J6calPin, J7calPin, J8calPin, J9calPin };
  int stepPins[numJoints] = { J1stepPin, J2stepPin, J3stepPin, J4stepPin, J5stepPin, J6stepPin, J7stepPin, J8stepPin, J9stepPin };
  int stepsDone[numJoints] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  int complete[numJoints] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  for (int i = 0; i < numJoints; i++) {
    if ((calDir[i] == 1 && motDir[i] == 1) || (calDir[i] == 0 && motDir[i] == 0)) {
      digitalWrite(dirPins[i], HIGH);
    } else {
      digitalWrite(dirPins[i], LOW);
    }
  }

  for (int i = 0; i < numJoints; i++) {
    // Set complete if joint was not sent a limit value
    if (steps[i] == 0) {
      complete[i] = 1;
    }
  }

  //DRIVE MOTORS FOR CALIBRATION
  int DriveLimInProc = 1;
  while (DriveLimInProc == 1 && estopActive == false) {

    for (int i = 0; i < numJoints; i++) {
      // Evaluate each joint
      curState[i] = digitalRead(calPins[i]);

      // Debounced: set complete only if HIGH is stable for DEBOUNCE_US
      if (curState[i] == HIGH) {
        if (firstHighUs[i] == 0) firstHighUs[i] = micros();
        if ((micros() - firstHighUs[i]) >= DEBOUNCE_US) {
          complete[i] = 1;
        }
      } else {
        firstHighUs[i] = 0;  // reset if it ever goes LOW again
      }

      // Step the motor if not complete and curState is LOW
      if (stepsDone[i] < steps[i] && complete[i] == 0) {
        digitalWrite(stepPins[i], HIGH);
        delayMicroseconds(calcStepGap);
        digitalWrite(stepPins[i], LOW);
        stepsDone[i]++;
      } else if (stepsDone[i] >= steps[i]) {
        // Steps exceeded, sensor never triggered – consider it failed
        complete[i] = 1;
      }
    }

    // Check if all joints are complete
    int allComplete = 1;
    for (int i = 0; i < numJoints; i++) {
      if (complete[i] == 0) {
        allComplete = 0;
        break;
      }
    }

    if (allComplete == 1) {
      DriveLimInProc = 0;
    }

    // Delay before restarting the loop
    delayMicroseconds(100);
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

  if (abs((J1EncSteps - J1StepM)) >= encOffset) {
    if (J1LoopMode == 0) {
      J1collisionTrue = 1;
      J1StepM = J1encPos.read() / J1encMult;
    }
  }
  if (abs((J2EncSteps - J2StepM)) >= encOffset) {
    if (J2LoopMode == 0) {
      J2collisionTrue = 1;
      J2StepM = J2encPos.read() / J2encMult;
    }
  }
  if (abs((J3EncSteps - J3StepM)) >= encOffset) {
    if (J3LoopMode == 0) {
      J3collisionTrue = 1;
      J3StepM = J3encPos.read() / J3encMult;
    }
  }
  if (abs((J4EncSteps - J4StepM)) >= encOffset) {
    if (J4LoopMode == 0) {
      J4collisionTrue = 1;
      J4StepM = J4encPos.read() / J4encMult;
    }
  }
  if (abs((J5EncSteps - J5StepM)) >= encOffset) {
    if (J5LoopMode == 0) {
      J5collisionTrue = 1;
      J5StepM = J5encPos.read() / J5encMult;
    }
  }
  if (abs((J6EncSteps - J6StepM)) >= encOffset) {
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

void driveMotorsJ(int J1step, int J2step, int J3step, int J4step, int J5step, int J6step, int J7step, int J8step, int J9step,
                  int J1dir, int J2dir, int J3dir, int J4dir, int J5dir, int J6dir, int J7dir, int J8dir, int J9dir,
                  String SpeedType, float SpeedVal, float ACCspd, float DCCspd, float ACCramp) {
  // Array of steps and directions
  int steps[9] = { J1step, J2step, J3step, J4step, J5step, J6step, J7step, J8step, J9step };
  int dirs[9] = { J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir };

  // Array of active joints, current steps, PE, SE, LO, and their current states
  int active[9] = { 0 };
  int cur[9] = { 0 };
  int PE[9] = { 0 }, SE_1[9] = { 0 }, SE_2[9] = { 0 }, LO_1[9] = { 0 }, LO_2[9] = { 0 };
  int PEcur[9] = { 0 }, SE_1cur[9] = { 0 }, SE_2cur[9] = { 0 };

  // Array of step and direction pins
  int stepPins[9] = { J1stepPin, J2stepPin, J3stepPin, J4stepPin, J5stepPin, J6stepPin, J7stepPin, J8stepPin, J9stepPin };
  int dirPins[9] = { J1dirPin, J2dirPin, J3dirPin, J4dirPin, J5dirPin, J6dirPin, J7dirPin, J8dirPin, J9dirPin };
  int motDirs[9] = { J1MotDir, J2MotDir, J3MotDir, J4MotDir, J5MotDir, J6MotDir, J7MotDir, J8MotDir, J9MotDir };

  // Initialize step monitors
  int stepMonitors[9] = { J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM, J7StepM, J8StepM, J9StepM };

  int HighStep = steps[0];
  int Jactive = 0;

  // FIND HIGHEST STEP
  for (int i = 1; i < 9; i++) {
    if (steps[i] > HighStep) {
      HighStep = steps[i];
    }
    if (steps[i] >= 1) {
      active[i] = 1;
      Jactive++;
    }
  }

  // SET DIRECTIONS
  for (int i = 0; i < 9; i++) {
    if (dirs[i] == motDirs[i]) {
      digitalWrite(dirPins[i], HIGH);
    } else {
      digitalWrite(dirPins[i], LOW);
    }
  }

  delayMicroseconds(15);

  /////CALC SPEEDS//////
  float calcStepGap;  // cruise delay (µs between highStep ticks)
  float speedSP;      // target total time in µs for the move
  float delay;

  // DETERMINE STEPS
  float ACCStep = HighStep * (ACCspd / 100.0f);
  float DCCStep = HighStep * (DCCspd / 100.0f);
  float NORStep = HighStep - ACCStep - DCCStep;

  // SET SPEED FOR SECONDS OR MM PER SEC
  if (SpeedType == "s") {
    speedSP = (SpeedVal * 1000000.0f) * 1.0f;
  } else if (SpeedType == "m") {
    float lineDist = pow(pow(xyzuvw_In[0] - xyzuvw_Out[0], 2) + pow(xyzuvw_In[1] - xyzuvw_Out[1], 2) + pow(xyzuvw_In[2] - xyzuvw_Out[2], 2), 0.5f);
    speedSP = ((lineDist / SpeedVal) * 1000000.0f) * 1.0f;
  }

  // fixed ramp factors (start/end slower than cruise)
  if(ACCramp < 10){
    ACCramp = 10;
  }
  const float k_acc = ACCramp / 10;
  const float k_dec = ACCramp / 10;

  if (SpeedType == "s" || SpeedType == "m") {
    // Solve cruise delay so total time matches speedSP.
    //
    // Total time T for a trapezoid (linear accel/decel):
    // T = ACCStep * (start+cruise)/2 + NORStep * (cruise) + DCCStep * (cruise+end)/2
    // Let start = k_acc * cruise, end = k_dec * cruise => solve for cruise:
    //
    // T = cruise * [ NORStep + (ACCStep*(1+k_acc) + DCCStep*(1+k_dec))/2 ]
    //
    float denom = NORStep + (ACCStep * (1.0f + k_acc) + DCCStep * (1.0f + k_dec)) * 0.5f;

    if (denom <= 0.0f) {
      // Fallback to constant speed if accel+decel consume everything
      calcStepGap = speedSP / max(HighStep, 1.0f);
    } else {
      calcStepGap = speedSP / denom;
    }

    if (calcStepGap < minSpeedDelay) {
      calcStepGap = minSpeedDelay;
      speedViolation = "1";
    }
  } else if (SpeedType == "p") {
    // Percentage mode unchanged
    calcStepGap = minSpeedDelay / (SpeedVal / 100.0f);
  }

  // With cruise known, define start/end delays and per-step increments
  float startDelay = calcStepGap * k_acc;  // slower than cruise
  float endDelay = calcStepGap * k_dec;    // slower than cruise

  // Linear ramp decrements/increments per step
  float calcACCstepInc = (ACCStep > 0.0f) ? (startDelay - calcStepGap) / ACCStep : 0.0f;  // subtract each step
  float calcDCCstepInc = (DCCStep > 0.0f) ? (endDelay - calcStepGap) / DCCStep : 0.0f;    // add each step

  // Start at the slow end of accel (or keep rounding behavior)
  float calcACCstartDel = startDelay;
  float curDelay = (rndTrue == true) ? rndSpeed : calcACCstartDel;
  rndTrue = false;

  ///// DRIVE MOTORS /////
  unsigned long moveStart = micros();
  int highStepCur = 0;

  while ((cur[0] < steps[0] || cur[1] < steps[1] || cur[2] < steps[2] || cur[3] < steps[3] || cur[4] < steps[4] || cur[5] < steps[5] || cur[6] < steps[6] || cur[7] < steps[7] || cur[8] < steps[8]) && estopActive == false) {

    ////DELAY CALC/////
    if (highStepCur <= ACCStep) {
      // During accel, move from startDelay down to cruise
      curDelay -= calcACCstepInc;  // since calcACCstepInc = (start - cruise)/ACCStep > 0
    } else if (highStepCur >= (HighStep - DCCStep)) {
      // During decel, move from cruise up to endDelay
      curDelay += calcDCCstepInc;  // since calcDCCstepInc = (end - cruise)/DCCStep > 0
    } else {
      curDelay = calcStepGap;  // cruise
    }

    float distDelay = 30;
    float disDelayCur = 0;

    for (int i = 0; i < 9; i++) {
      if (cur[i] < steps[i]) {
        PE[i] = (HighStep / steps[i]);
        LO_1[i] = (HighStep - (steps[i] * PE[i]));
        SE_1[i] = (LO_1[i] > 0) ? (HighStep / LO_1[i]) : 0;
        LO_2[i] = (SE_1[i] > 0) ? (HighStep - ((steps[i] * PE[i]) + ((steps[i] * PE[i]) / SE_1[i]))) : 0;
        SE_2[i] = (LO_2[i] > 0) ? (HighStep / LO_2[i]) : 0;

        if (SE_2[i] == 0) {
          SE_2cur[i] = SE_2[i] + 1;
        }

        if (SE_2cur[i] != SE_2[i]) {
          SE_2cur[i]++;
          if (SE_1[i] == 0) {
            SE_1cur[i] = SE_1[i] + 1;
          }

          if (SE_1cur[i] != SE_1[i]) {
            SE_1cur[i]++;
            PEcur[i]++;

            if (PEcur[i] == PE[i]) {
              cur[i]++;
              PEcur[i] = 0;
              digitalWrite(stepPins[i], LOW);
              delayMicroseconds(distDelay);
              disDelayCur += distDelay;

              if (dirs[i] == 0) {
                stepMonitors[i]--;
              } else {
                stepMonitors[i]++;
              }
            }
          } else {
            SE_1cur[i] = 0;
          }
        } else {
          SE_2cur[i] = 0;
        }
      }
    }

    // Increment current step
    highStepCur++;
    for (int i = 0; i < 9; i++) {
      digitalWrite(stepPins[i], HIGH);
    }

    delay = curDelay - disDelayCur;
    if (delay < minSpeedDelay) {
      delay = minSpeedDelay;
    }
    delayMicroseconds(delay);
  }
  unsigned long moveEnd = micros();
  float elapsedSeconds = (moveEnd - moveStart) / 1000000.0f;
  //debug = String(elapsedSeconds);

  // Set rounding speed to last move speed
  rndSpeed = curDelay;

  // Update the original step monitor variables
  J1StepM = stepMonitors[0];
  J2StepM = stepMonitors[1];
  J3StepM = stepMonitors[2];
  J4StepM = stepMonitors[3];
  J5StepM = stepMonitors[4];
  J6StepM = stepMonitors[5];
  J7StepM = stepMonitors[6];
  J8StepM = stepMonitors[7];
  J9StepM = stepMonitors[8];
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//DRIVE MOTORS G
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void driveMotorsG(int J1step, int J2step, int J3step, int J4step, int J5step, int J6step, int J7step, int J8step, int J9step, int J1dir, int J2dir, int J3dir, int J4dir, int J5dir, int J6dir, int J7dir, int J8dir, int J9dir, String SpeedType, float SpeedVal, float ACCspd, float DCCspd, float ACCramp) {
  int steps[] = { J1step, J2step, J3step, J4step, J5step, J6step, J7step, J8step, J9step };
  int dirs[] = { J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir };
  int motDirs[] = { J1MotDir, J2MotDir, J3MotDir, J4MotDir, J5MotDir, J6MotDir, J7MotDir, J8MotDir, J9MotDir };
  int stepPins[] = { J1stepPin, J2stepPin, J3stepPin, J4stepPin, J5stepPin, J6stepPin, J7stepPin, J8stepPin, J9stepPin };
  int dirPins[] = { J1dirPin, J2dirPin, J3dirPin, J4dirPin, J5dirPin, J6dirPin, J7dirPin, J8dirPin, J9dirPin };
  int stepMonitors[] = { J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM, J7StepM, J8StepM, J9StepM };

  // FIND HIGHEST STEP
  int HighStep = 0;
  for (int i = 0; i < 9; i++) {
    if (steps[i] > HighStep) {
      HighStep = steps[i];
    }
  }

  // FIND ACTIVE JOINTS
  int Jactive = 0;
  for (int i = 0; i < 9; i++) {
    if (steps[i] >= 1) {
      Jactive++;
    }
  }

  // Array of active joints, current steps, PE, SE, LO, and their current states
  int active[9] = { 0 };
  int cur[9] = { 0 };
  int PE[9] = { 0 }, SE_1[9] = { 0 }, SE_2[9] = { 0 }, LO_1[9] = { 0 }, LO_2[9] = { 0 };
  int PEcur[9] = { 0 }, SE_1cur[9] = { 0 }, SE_2cur[9] = { 0 };

  int highStepCur = 0;
  float curDelay = 0;
  float speedSP;
  float moveDist;

  // SET DIRECTIONS
  for (int i = 0; i < 9; i++) {
    if (dirs[i] == motDirs[i]) {
      digitalWrite(dirPins[i], HIGH);
    } else {
      digitalWrite(dirPins[i], LOW);
    }
  }

  delayMicroseconds(15);

  ///// CALC SPEEDS /////
  float calcStepGap;
  speedViolation = "0";  // Reset speed violation flag

  // Set speed for seconds or mm per sec
  if (SpeedType == "s") {
    speedSP = (SpeedVal * 1000000) * 1.2;
    calcStepGap = speedSP / HighStep;
  } else if (SpeedType == "m") {
    if (SpeedVal >= maxMMperSec) {
      SpeedVal = maxMMperSec;
      speedViolation = "1";
    }
    SpeedVal = ((SpeedVal / maxMMperSec) * 100);
    calcStepGap = minSpeedDelay / (SpeedVal / 100);
  } else if (SpeedType == "p") {
    calcStepGap = minSpeedDelay / (SpeedVal / 100);
  }

  // Ensure calcStepGap is not less than minSpeedDelay
  if (calcStepGap <= minSpeedDelay) {
    calcStepGap = minSpeedDelay;
    speedViolation = "1";
  }

  ///// DRIVE MOTORS /////
  while ((cur[0] != steps[0] || cur[1] != steps[1] || cur[2] != steps[2] || cur[3] != steps[3] || cur[4] != steps[4] || cur[5] != steps[5] || cur[6] != steps[6] || cur[7] != steps[7] || cur[8] != steps[8]) && estopActive == false) {
    curDelay = calcStepGap;

    float distDelay = 30;
    float disDelayCur = 0;

    for (int i = 0; i < 9; i++) {
      if (cur[i] < steps[i]) {
        PE[i] = (HighStep / steps[i]);
        LO_1[i] = (HighStep - (steps[i] * PE[i]));
        SE_1[i] = LO_1[i] > 0 ? (HighStep / LO_1[i]) : 0;
        LO_2[i] = SE_1[i] > 0 ? HighStep - ((steps[i] * PE[i]) + ((steps[i] * PE[i]) / SE_1[i])) : 0;
        SE_2[i] = LO_2[i] > 0 ? (HighStep / LO_2[i]) : 0;

        if (SE_2[i] == 0) SE_2cur[i] = SE_2[i] + 1;
        if (SE_2cur[i] != SE_2[i]) {
          SE_2cur[i]++;
          if (SE_1[i] == 0) SE_1cur[i] = SE_1[i] + 1;
          if (SE_1cur[i] != SE_1[i]) {
            SE_1cur[i]++;
            PEcur[i]++;
            if (PEcur[i] == PE[i]) {
              cur[i]++;
              PEcur[i] = 0;
              digitalWrite(stepPins[i], LOW);
              delayMicroseconds(distDelay);
              disDelayCur += distDelay;
              stepMonitors[i] += (dirs[i] == 0) ? -1 : 1;
            }
          } else {
            SE_1cur[i] = 0;
          }
        } else {
          SE_2cur[i] = 0;
        }
      }
    }

    highStepCur++;
    for (int i = 0; i < 9; i++) {
      digitalWrite(stepPins[i], HIGH);
    }
    delayMicroseconds(curDelay - disDelayCur);
  }

  // set rounding speed to last move speed
  rndSpeed = curDelay;

  // assign the updated values back to the original step monitors
  J1StepM = stepMonitors[0];
  J2StepM = stepMonitors[1];
  J3StepM = stepMonitors[2];
  J4StepM = stepMonitors[3];
  J5StepM = stepMonitors[4];
  J6StepM = stepMonitors[5];
  J7StepM = stepMonitors[6];
  J8StepM = stepMonitors[7];
  J9StepM = stepMonitors[8];
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//DRIVE MOTORS L
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void driveMotorsL(int J1step, int J2step, int J3step, int J4step, int J5step, int J6step, int J7step, int J8step, int J9step, int J1dir, int J2dir, int J3dir, int J4dir, int J5dir, int J6dir, int J7dir, int J8dir, int J9dir, float curDelay) {
  // Array of steps, directions, pins, motor directions, and step counters
  int steps[9] = { J1step, J2step, J3step, J4step, J5step, J6step, J7step, J8step, J9step };
  int dirs[9] = { J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir };
  int dirPins[9] = { J1dirPin, J2dirPin, J3dirPin, J4dirPin, J5dirPin, J6dirPin, J7dirPin, J8dirPin, J9dirPin };
  int stepPins[9] = { J1stepPin, J2stepPin, J3stepPin, J4stepPin, J5stepPin, J6stepPin, J7stepPin, J8stepPin, J9stepPin };
  int motDirs[9] = { J1MotDir, J2MotDir, J3MotDir, J4MotDir, J5MotDir, J6MotDir, J7MotDir, J8MotDir, J9MotDir };
  int stepMonitors[9] = { J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM, J7StepM, J8StepM, J9StepM };

  // Array of active joints, current steps, PE, SE, LO, and their current states
  int active[9] = { 0 };
  int cur[9] = { 0 };
  int PE[9] = { 0 }, SE_1[9] = { 0 }, SE_2[9] = { 0 }, LO_1[9] = { 0 }, LO_2[9] = { 0 };
  int PEcur[9] = { 0 }, SE_1cur[9] = { 0 }, SE_2cur[9] = { 0 };

  // FIND HIGHEST STEP
  int HighStep = 0;
  for (int i = 0; i < 9; i++) {
    if (steps[i] > HighStep) {
      HighStep = steps[i];
    }
  }

  // FIND ACTIVE JOINTS
  for (int i = 0; i < 9; i++) {
    if (steps[i] >= 1) {
      active[i] = 1;
    }
  }

  // Process lookahead
  if (splineTrue) {
    processSerial();
  }

  // SET DIRECTIONS
  for (int i = 0; i < 9; i++) {
    if (dirs[i] == motDirs[i]) {
      digitalWrite(dirPins[i], HIGH);
    } else {
      digitalWrite(dirPins[i], LOW);
    }
  }

  delayMicroseconds(15);

  int highStepCur = 0;

  // DRIVE MOTORS
  while ((cur[0] < steps[0] || cur[1] < steps[1] || cur[2] < steps[2] || cur[3] < steps[3] || cur[4] < steps[4] || cur[5] < steps[5] || cur[6] < steps[6] || cur[7] < steps[7] || cur[8] < steps[8]) && !estopActive) {
    float distDelay = 30;
    float disDelayCur = 0;

    // Process lookahead
    if (splineTrue) {
      processSerial();
    }

    // Iterate through each joint
    for (int i = 0; i < 9; i++) {
      if (cur[i] < steps[i]) {
        PE[i] = (HighStep / steps[i]);
        LO_1[i] = (HighStep - (steps[i] * PE[i]));
        SE_1[i] = (LO_1[i] > 0) ? (HighStep / LO_1[i]) : 0;
        LO_2[i] = (SE_1[i] > 0) ? HighStep - ((steps[i] * PE[i]) + ((steps[i] * PE[i]) / SE_1[i])) : 0;
        SE_2[i] = (LO_2[i] > 0) ? (HighStep / LO_2[i]) : 0;

        if (SE_2[i] == 0) {
          SE_2cur[i] = 1;
        }
        if (SE_2cur[i] != SE_2[i]) {
          SE_2cur[i]++;
          if (SE_1[i] == 0) {
            SE_1cur[i] = 1;
          }
          if (SE_1cur[i] != SE_1[i]) {
            SE_1cur[i]++;
            PEcur[i]++;
            if (PEcur[i] == PE[i]) {
              cur[i]++;
              PEcur[i] = 0;
              digitalWrite(stepPins[i], LOW);
              delayMicroseconds(distDelay);
              disDelayCur += distDelay;
              stepMonitors[i] += (dirs[i] == 0) ? -1 : 1;
            }
          } else {
            SE_1cur[i] = 0;
          }
        } else {
          SE_2cur[i] = 0;
        }
      }
    }

    // Increment current step
    highStepCur++;
    for (int i = 0; i < 9; i++) {
      digitalWrite(stepPins[i], HIGH);
    }
    delayMicroseconds(curDelay - disDelayCur);
  }

  // Assign the updated values back to the original step monitors
  J1StepM = stepMonitors[0];
  J2StepM = stepMonitors[1];
  J3StepM = stepMonitors[2];
  J4StepM = stepMonitors[3];
  J5StepM = stepMonitors[4];
  J6StepM = stepMonitors[5];
  J7StepM = stepMonitors[6];
  J8StepM = stepMonitors[7];
  J9StepM = stepMonitors[8];
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MOVE J
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void moveJ(String inData, bool response, bool precalc, bool simspeed) {

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


  SolveInverseKinematics();

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

  if (precalc) {
    J1StepM = J1futStepM;
    J2StepM = J2futStepM;
    J3StepM = J3futStepM;
    J4StepM = J4futStepM;
    J5StepM = J5futStepM;
    J6StepM = J6futStepM;
    J7StepM = J7futStepM;
    J8StepM = J8futStepM;
    J9StepM = J9futStepM;
  }

  else {
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
    J1dir = (J1stepDif <= 0) ? 1 : 0;
    J2dir = (J2stepDif <= 0) ? 1 : 0;
    J3dir = (J3stepDif <= 0) ? 1 : 0;
    J4dir = (J4stepDif <= 0) ? 1 : 0;
    J5dir = (J5stepDif <= 0) ? 1 : 0;
    J6dir = (J6stepDif <= 0) ? 1 : 0;
    J7dir = (J7stepDif <= 0) ? 1 : 0;
    J8dir = (J8stepDif <= 0) ? 1 : 0;
    J9dir = (J9stepDif <= 0) ? 1 : 0;

    // Arrays for joint properties
    int dir[numJoints] = { J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir };
    int StepM[numJoints] = { J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM, J7StepM, J8StepM, J9StepM };
    int stepDif[numJoints] = { J1stepDif, J2stepDif, J3stepDif, J4stepDif, J5stepDif, J6stepDif, J7stepDif, J8stepDif, J9stepDif };
    int StepLim[numJoints] = { J1StepLim, J2StepLim, J3StepLim, J4StepLim, J5StepLim, J6StepLim, J7StepLim, J8StepLim, J9StepLim };
    int axisFault[numJoints] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    // Loop to check axis limits and set faults
    for (int i = 0; i < numJoints; ++i) {
      if ((dir[i] == 1 && (StepM[i] + stepDif[i] > StepLim[i])) || (dir[i] == 0 && (StepM[i] - stepDif[i] < 0))) {
        axisFault[i] = 1;
      }
    }

    // Assign fault values back to individual variables
    J1axisFault = axisFault[0];
    J2axisFault = axisFault[1];
    J3axisFault = axisFault[2];
    J4axisFault = axisFault[3];
    J5axisFault = axisFault[4];
    J6axisFault = axisFault[5];
    J7axisFault = axisFault[6];
    J8axisFault = axisFault[7];
    J9axisFault = axisFault[8];

    // Calculate total axis fault
    TotalAxisFault = 0;
    for (int i = 0; i < numJoints; ++i) {
      TotalAxisFault += axisFault[i];
    }

    //send move command if no axis limit error
    if (TotalAxisFault == 0 && KinematicError == 0) {
      resetEncoders();
      if (simspeed) {
        driveMotorsG(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(J7stepDif), abs(J8stepDif), abs(J9stepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
      } else {
        driveMotorsJ(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(J7stepDif), abs(J8stepDif), abs(J9stepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
      }
      checkEncoders();
      if (response == true) {
        sendRobotPos();
      }
    } else if (KinematicError == 1) {
      Alarm = "ER";
      delay(5);
      Serial.println(Alarm);
      Alarm = "0";
    } else {
      Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault) + String(J7axisFault) + String(J8axisFault) + String(J9axisFault);
      delay(5);
      Serial.println(Alarm);
      Alarm = "0";
    }

    inData = "";  // Clear recieved buffer
                  ////////MOVE COMPLETE///////////
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//COMMUNICATIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



int32_t modbusQuerry(String inData, int function) {
  int32_t result;
  int32_t response;
  int32_t response2;
  int slaveIdIndex = inData.indexOf('A');
  int MBaddressIndex = inData.indexOf('B');
  int MBvalIndex = inData.indexOf('C');
  int SlaveID = inData.substring(slaveIdIndex + 1, MBaddressIndex).toInt();
  int MBaddress = inData.substring(MBaddressIndex + 1, MBvalIndex).toInt();
  int MBval = inData.substring(MBvalIndex + 1).toInt();
  node = ModbusMaster();
  node.begin(SlaveID, Serial8);
  if (function == 1) {
    result = node.readCoils(MBaddress, 1);
    if (result == node.ku8MBSuccess) {
      response = node.getResponseBuffer(0);
      return response;
    } else {
      response = -1;
      return response;
    }
  } else if (function == 2) {
    result = node.readDiscreteInputs(MBaddress, 1);
    if (result == node.ku8MBSuccess) {
      response = node.getResponseBuffer(0);
      return response;
    } else {
      response = -1;
      return response;
    }
  } else if (function == 3) {
    result = node.readHoldingRegisters(MBaddress, MBval);
    if (result == node.ku8MBSuccess) {
      response = node.getResponseBuffer(0);
      return response;
    } else {
      response = -1;
      return response;
    }
  } else if (function == 4) {
    result = node.readInputRegisters(MBaddress, MBval);
    if (result == node.ku8MBSuccess) {
      response = node.getResponseBuffer(0);
      return response;
    } else {
      response = -1;
      return response;
    }
  } else if (function == 15) {
    result = node.writeSingleCoil(MBaddress, MBval);
    if (result == node.ku8MBSuccess) {
      response = 1;
      return response;
    } else {
      response = -1;
      return response;
    }
  } else if (function == 6) {
    result = node.writeSingleRegister(MBaddress, MBval);
    if (result == node.ku8MBSuccess) {
      response = 1;
      return response;
    } else {
      response = -1;
      return response;
    }
  } else {
    response = -1;
    return response;
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
        } else {
          Serial.println(Alarm);
          Alarm = "0";
        }
      }

      recData = "";  // Clear recieved buffer

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
                } else {
                  Serial.println(Alarm);
                  Alarm = "0";
                }
              }
              recData = "";  // Clear recieved buffer
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


void EstopProg() {
  estopActive = true;
  flag = "EB";
  sendRobotPos();
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MAIN
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  // run once:
  Serial.begin(9600);
  Serial8.begin(38400);  // Use Serial8 (pins 34 and 35)
  // Initialize Modbus communication
  node.begin(1, Serial8);



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


  pinMode(EstopPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EstopPin), EstopProg, LOW);



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
    estopActive = false;
    inData = cmdBuffer1;
    inData.trim();
    String function = inData.substring(0, 2);
    inData = inData.substring(2);
    KinematicError = 0;
    debug = "";


    //-----MODBUS READ HOLDING REGISTER - FUNCTION 03--------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "BA") {
      int32_t result = modbusQuerry(inData, 3);
      if (result == -1) {
        Serial.println("Modbus Error");
      } else {
        Serial.println(result);
      }
    }


    //-----MODBUS READ COIL - FUNCTION 01--------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "BB") {
      int32_t result = modbusQuerry(inData, 1);
      if (result == -1) {
        Serial.println("Modbus Error");
      } else {
        Serial.println(result);
      }
    }

    //-----MODBUS READ INPUT - FUNCTION 02--------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "BC") {
      int32_t result = modbusQuerry(inData, 2);
      if (result == -1) {
        Serial.println("Modbus Error");
      } else {
        Serial.println(result);
      }
    }

    //-----MODBUS READ INPUT REGISTER - FUNCTION 04--------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "BD") {
      int32_t result = modbusQuerry(inData, 4);
      if (result == -1) {
        Serial.println("Modbus Error");
      } else {
        Serial.println(result);
      }
    }

    //-----MODBUS WRITE COIL - FUNCTION 15--------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "BE") {
      int32_t result = modbusQuerry(inData, 15);
      if (result == -1) {
        Serial.println("Modbus Error");
      } else {
        Serial.println("Write Success");
      }
    }

    //-----MODBUS WRITE REGISTER - FUNCTION 6--------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "BF") {
      int32_t result = modbusQuerry(inData, 6);
      if (result == -1) {
        Serial.println("Modbus Error");
      } else {
        Serial.println("Write Success");
      }
    }





    //-----QUERRY DRIVE MODBUS--------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "MQ") {
      uint8_t result;
      int16_t highRegister;

      // Modbus read
      result = node.readHoldingRegisters(0x1207, 2);

      if (result == node.ku8MBSuccess) {

        highRegister = node.getResponseBuffer(0);
        Serial.println(highRegister);

      } else {
        Serial.println("Modbus error: ");
        //Serial.println(result, HEX);
      }

      delay(1000);
    }

    //-----HOME MOTOR DRIVE MODBUS--------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "HD") {
      uint8_t result;

      // Address and value to write
      uint16_t registerAddress1 = 0x020D;  // P0213 - DI3
      uint16_t registerAddress2 = 0x020C;  // P0212 - DI2
      //uint16_t registerAddress = 0x1207;  // P1807 - absolute position counter
      uint16_t valueOn = 1;   // Value to write to the register
      uint16_t valueOff = 0;  // Value to write to the register

      // Write the value to the register
      result = node.writeSingleRegister(registerAddress1, valueOn);
      delay(50);
      result = node.writeSingleRegister(registerAddress2, valueOn);
      delay(50);
      result = node.writeSingleRegister(registerAddress1, valueOff);
      delay(50);
      result = node.writeSingleRegister(registerAddress2, valueOff);

      if (result == node.ku8MBSuccess) {
        Serial.println("Write successful");
      } else {
        //Serial.println("Modbus Error: ");
        Serial.println(result, HEX);
      }

      delay(50);
    }

    //-----RESET DRIVE MODBUS--------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "RR") {
      uint8_t result;

      // Address and value to write
      uint16_t registerAddress1 = 0x020D;  // P0213 - DI3 INPUT
      uint16_t registerAddress2 = 0x0203;  // P0203 - DI3 FUNCTION SELECTION

      uint16_t valueOn = 1;
      uint16_t valueOff = 0;
      uint16_t homingMode = 33;
      uint16_t resetMode = 2;


      result = node.writeSingleRegister(registerAddress2, resetMode);
      delay(50);
      result = node.writeSingleRegister(registerAddress1, valueOn);
      delay(50);
      result = node.writeSingleRegister(registerAddress2, homingMode);
      delay(50);


      if (result == node.ku8MBSuccess) {
        Serial.println("Write successful");
      } else {
        Serial.println("fail");
      }

      delay(50);
    }

    //-----RESET DRIVE MODBUS--------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "FR") {
      uint8_t result;

      // Address and value to write
      uint16_t registerAddress1 = 0x0B01;  // P1101 - fault reset

      uint16_t valueOn = 1;
      uint16_t valueOff = 0;

      result = node.writeSingleRegister(registerAddress1, valueOn);
      delay(50);
      result = node.writeSingleRegister(registerAddress1, valueOff);
      delay(50);


      if (result == node.ku8MBSuccess) {
        Serial.println("Write successful");
      } else {
        Serial.println("fail");
      }

      delay(50);
    }

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
    if (function == "CL") {
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
    if (function == "SE") {
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
    if (function == "RE") {
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
    if (function == "RP") {
      //close serial so next command can be read in
      delay(5);
      if (Alarm == "0") {
        sendRobotPos();
      } else {
        Serial.println(Alarm);
        Alarm = "0";
      }
    }



    //-----COMMAND HOME POSITION---------------------------------------------------
    //-----------------------------------------------------------------------

    //For debugging
    if (function == "HM") {

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
      J1dir = (J1stepDif <= 0) ? 1 : 0;
      J2dir = (J2stepDif <= 0) ? 1 : 0;
      J3dir = (J3stepDif <= 0) ? 1 : 0;
      J4dir = (J4stepDif <= 0) ? 1 : 0;
      J5dir = (J5stepDif <= 0) ? 1 : 0;
      J6dir = (J6stepDif <= 0) ? 1 : 0;
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
    if (function == "CP") {
      correctRobotPos();
    }

    //-----COMMAND UPDATE PARAMS---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "UP") {
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

      Robot_Kin_Tool[0] = inData.substring(TFxStart + 1, TFyStart).toFloat();
      Robot_Kin_Tool[1] = inData.substring(TFyStart + 1, TFzStart).toFloat();
      Robot_Kin_Tool[2] = inData.substring(TFzStart + 1, TFrzStart).toFloat();
      Robot_Kin_Tool[3] = inData.substring(TFrzStart + 1, TFryStart).toFloat() * M_PI / 180;
      Robot_Kin_Tool[4] = inData.substring(TFryStart + 1, TFrxStart).toFloat() * M_PI / 180;
      Robot_Kin_Tool[5] = inData.substring(TFrxStart + 1).toFloat() * M_PI / 180;
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

      Serial.print("Done");
    }

    //-----COMMAND CALIBRATE EXTERNAL AXIS---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "CE") {
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
    if (function == "Z7") {
      J7StepM = 0;
      sendRobotPos();
    }

    //-----COMMAND ZERO J8---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "Z8") {
      J8StepM = 0;
      sendRobotPos();
    }

    //-----COMMAND ZERO J9---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "Z9") {
      J9StepM = 0;
      sendRobotPos();
    }


    //-----COMMAND TO WAIT TIME---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "WT") {
      int WTstart = inData.indexOf('S');
      float WaitTime = inData.substring(WTstart + 1).toFloat();
      int WaitTimeMS = WaitTime * 1000;
      delay(WaitTimeMS);
      Serial.println("WTdone");
    }


    //-----COMMAND SET OUTPUT ON---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "ON") {
      int ONstart = inData.indexOf('X');
      int outputNum = inData.substring(ONstart + 1).toInt();
      digitalWrite(outputNum, HIGH);
      delay(5);
      Serial.println("Done");
    }
    //-----COMMAND SET OUTPUT OFF---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "OF") {
      int ONstart = inData.indexOf('X');
      int outputNum = inData.substring(ONstart + 1).toInt();
      digitalWrite(outputNum, LOW);
      delay(5);
      Serial.println("Done");
    }

    //-----COMMAND TO WAIT MODBUS COIL---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "WJ") {
      int32_t result = -2;
      String MBquery = "";
      int slaveIndex = inData.indexOf('A');
      int inputIndex = inData.indexOf('B');
      int valueIndex = inData.indexOf('C');
      int timoutIndex = inData.indexOf('D');
      int slaveID = inData.substring(slaveIndex + 1, inputIndex).toInt();
      int input = inData.substring(inputIndex + 1, valueIndex).toInt();
      int value = inData.substring(valueIndex + 1, timoutIndex).toInt();
      int timeout = inData.substring(timoutIndex + 1).toInt();
      unsigned long timeoutMillis = timeout * 1000;
      unsigned long startTime = millis();
      MBquery = "A" + String(slaveID) + "B" + String(input) + "C1";
      while ((millis() - startTime < timeoutMillis) && (result != value)) {
        result = modbusQuerry(MBquery, 1);
        delay(100);
      }
      delay(5);
      Serial.print("Done");
    }

    //-----COMMAND TO WAIT MODBUS INPUT---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "WK") {
      int32_t result = -2;
      String MBquery = "";
      int slaveIndex = inData.indexOf('A');
      int inputIndex = inData.indexOf('B');
      int valueIndex = inData.indexOf('C');
      int timoutIndex = inData.indexOf('D');
      int slaveID = inData.substring(slaveIndex + 1, inputIndex).toInt();
      int input = inData.substring(inputIndex + 1, valueIndex).toInt();
      int value = inData.substring(valueIndex + 1, timoutIndex).toInt();
      int timeout = inData.substring(timoutIndex + 1).toInt();
      unsigned long timeoutMillis = timeout * 1000;
      unsigned long startTime = millis();
      MBquery = "A" + String(slaveID) + "B" + String(input) + "C1";
      while ((millis() - startTime < timeoutMillis) && (result != value)) {
        result = modbusQuerry(MBquery, 2);
        delay(100);
      }
      delay(5);
      Serial.print("Done");
    }

    //-----COMMAND TO SET MODBUS COIL---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "SC") {
      int32_t result = -2;
      String MBquery = "";
      int slaveIndex = inData.indexOf('A');
      int inputIndex = inData.indexOf('B');
      int valueIndex = inData.indexOf('C');
      int slaveID = inData.substring(slaveIndex + 1, inputIndex).toInt();
      int input = inData.substring(inputIndex + 1, valueIndex).toInt();
      int value = inData.substring(valueIndex + 1).toInt();
      MBquery = "A" + String(slaveID) + "B" + String(input) + "C" + String(value);
      result = modbusQuerry(MBquery, 15);
      delay(5);
      Serial.println(result);
    }

    //-----COMMAND TO SET MODBUS OUTPUT REGISTER---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "SO") {
      int32_t result = -2;
      String MBquery = "";
      int slaveIndex = inData.indexOf('A');
      int inputIndex = inData.indexOf('B');
      int valueIndex = inData.indexOf('C');
      int slaveID = inData.substring(slaveIndex + 1, inputIndex).toInt();
      int input = inData.substring(inputIndex + 1, valueIndex).toInt();
      int value = inData.substring(valueIndex + 1).toInt();
      MBquery = "A" + String(slaveID) + "B" + String(input) + "C" + String(value);
      result = modbusQuerry(MBquery, 6);
      delay(5);
      Serial.println(result);
    }


    //-----COMMAND SEND POSITION---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "SP") {
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
    if (function == "TM") {
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

      SolveInverseKinematics();

      String echo = "";
      delay(5);
      Serial.println(inData);
    }
    //-----COMMAND TO CALIBRATE---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "LL") {
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

      int Jreq[9] = { J1req, J2req, J3req, J4req, J5req, J6req, J7req, J8req, J9req };
      int JStepLim[9] = { J1StepLim, J2StepLim, J3StepLim, J4StepLim, J5StepLim, J6StepLim, J7StepLim, J8StepLim, J9StepLim };
      int JcalPin[9] = { J1calPin, J2calPin, J3calPin, J4calPin, J5calPin, J6calPin, J7calPin, J8calPin, J9calPin };
      int JStep[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

      for (int i = 0; i < 9; i++) {
        if (Jreq[i] == 1) {
          JStep[i] = JStepLim[i];
        }
      }

      //--CALL FUNCT TO DRIVE TO LIMITS--
      SpeedIn = 40;
      driveLimit(JStep, SpeedIn);




      //set master steps and center step

      if (J1req == 1) {
        if (J1CalDir == 1) {
          J1StepM = ((J1axisLim) + J1calBaseOff + J1calOff) * J1StepDeg;
          J1stepCen = ((J1axisLimPos) + J1calBaseOff + J1calOff) * J1StepDeg;
        } else {
          J1StepM = (0 + J1calBaseOff + J1calOff) * J1StepDeg;
          J1stepCen = ((J1axisLimNeg)-J1calBaseOff - J1calOff) * J1StepDeg;
        }
      }
      if (J2req == 1) {
        if (J2CalDir == 1) {
          J2StepM = ((J2axisLim) + J2calBaseOff + J2calOff) * J2StepDeg;
          J2stepCen = ((J2axisLimPos) + J2calBaseOff + J2calOff) * J2StepDeg;
        } else {
          J2StepM = (0 + J2calBaseOff + J2calOff) * J2StepDeg;
          J2stepCen = ((J2axisLimNeg)-J2calBaseOff - J2calOff) * J2StepDeg;
        }
      }
      if (J3req == 1) {
        if (J3CalDir == 1) {
          J3StepM = ((J3axisLim) + J3calBaseOff + J3calOff) * J3StepDeg;
          J3stepCen = ((J3axisLimPos) + J3calBaseOff + J3calOff) * J3StepDeg;
        } else {
          J3StepM = (0 + J3calBaseOff + J3calOff) * J3StepDeg;
          J3stepCen = ((J3axisLimNeg)-J3calBaseOff - J3calOff) * J3StepDeg;
        }
      }
      if (J4req == 1) {
        if (J4CalDir == 1) {
          J4StepM = ((J4axisLim) + J4calBaseOff + J4calOff) * J4StepDeg;
          J4stepCen = ((J4axisLimPos) + J4calBaseOff + J4calOff) * J4StepDeg;
        } else {
          J4StepM = (0 + J4calBaseOff + J4calOff) * J4StepDeg;
          J4stepCen = ((J4axisLimNeg)-J4calBaseOff - J4calOff) * J4StepDeg;
        }
      }
      if (J5req == 1) {
        if (J5CalDir == 1) {
          J5StepM = ((J5axisLim) + J5calBaseOff + J5calOff) * J5StepDeg;
          J5stepCen = ((J5axisLimPos) + J5calBaseOff + J5calOff) * J5StepDeg;
          J5step90 = (((J5axisLimNeg) + J5calBaseOff + J5calOff) - 90) * J5StepDeg;
        } else {
          J5StepM = (0 + J5calBaseOff + J5calOff) * J5StepDeg;
          J5stepCen = ((J5axisLimNeg)-J5calBaseOff - J5calOff) * J5StepDeg;
          J5step90 = (((J5axisLimNeg)-J5calBaseOff - J5calOff) + 90) * J5StepDeg;
        }
      }
      if (J6req == 1) {
        if (J6CalDir == 1) {
          J6StepM = ((J6axisLim) + J6calBaseOff + J6calOff) * J6StepDeg;
          J6stepCen = ((J6axisLimPos) + J6calBaseOff + J6calOff) * J6StepDeg;
        } else {
          J6StepM = (0 + J6calBaseOff + J6calOff) * J6StepDeg;
          J6stepCen = ((J6axisLimNeg)-J6calBaseOff - J6calOff) * J6StepDeg;
        }
      }
      if (J7req == 1) {
        if (J7CalDir == 1) {
          J7StepM = ((J7axisLim) + J7calBaseOff + J7calOff) * J7StepDeg;
          J7stepCen = ((J7axisLimPos) + J7calBaseOff + J7calOff) * J7StepDeg;
        } else {
          J7StepM = (0 + J7calBaseOff + J7calOff) * J7StepDeg;
          J7stepCen = ((J7axisLimNeg)-J7calBaseOff - J7calOff) * J7StepDeg;
        }
      }
      if (J8req == 1) {
        if (J8CalDir == 1) {
          J8StepM = ((J8axisLim) + J8calBaseOff + J8calOff) * J8StepDeg;
          J8stepCen = ((J8axisLimPos) + J8calBaseOff + J8calOff) * J8StepDeg;
        } else {
          J8StepM = (0 + J8calBaseOff + J8calOff) * J8StepDeg;
          J8stepCen = ((J8axisLimNeg)-J8calBaseOff - J8calOff) * J8StepDeg;
        }
      }
      if (J9req == 1) {
        if (J9CalDir == 1) {
          J9StepM = ((J9axisLim) + J9calBaseOff + J9calOff) * J9StepDeg;
          J9stepCen = ((J9axisLimPos) + J9calBaseOff + J9calOff) * J9StepDeg;
        } else {
          J9StepM = (0 + J9calBaseOff + J9calOff) * J9StepDeg;
          J9stepCen = ((J9axisLimNeg)-J9calBaseOff - J9calOff) * J9StepDeg;
        }
      }


      //move to center
      /// J1 ///
      if (J1CalDir) {
        J1dir = 0;
      } else {
        J1dir = 1;
      }
      /// J2 ///
      if (J2CalDir) {
        J2dir = 0;
      } else {
        J2dir = 1;
      }
      /// J3 ///
      if (J3CalDir) {
        J3dir = 0;
      } else {
        J3dir = 1;
      }
      /// J4 ///
      if (J4CalDir) {
        J4dir = 0;
      } else {
        J4dir = 1;
      }
      /// J5 ///
      if (J5CalDir) {
        J5dir = 0;
      } else {
        J5dir = 1;
      }
      /// J6 ///
      if (J6CalDir) {
        J6dir = 0;
      } else {
        J6dir = 1;
      }
      /// J7 ///
      if (J7CalDir) {
        J7dir = 0;
      } else {
        J7dir = 1;
      }
      /// J8 ///
      if (J8CalDir) {
        J8dir = 0;
      } else {
        J8dir = 1;
      }
      /// J9 ///
      if (J9CalDir) {
        J9dir = 0;
      } else {
        J9dir = 1;
      }

      float ACCspd = 10;
      float DCCspd = 10;
      String SpeedType = "p";
      float SpeedVal = 50;
      float ACCramp = 50;

      driveMotorsJ(J1stepCen, J2stepCen, J3stepCen, J4stepCen, J5step90, J6stepCen, J7stepCen, J8stepCen, J9stepCen, J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
      sendRobotPos();
      inData = "";  // Clear recieved buffer
    }











    //----- LIVE CARTESIAN JOG  ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "LC") {
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
      float ACCspd = 0;
      float DCCspd = 0;
      float ACCramp = 10;
      WristCon = inData.substring(WristConStart + 1, LoopModeStart);
      String LoopMode = inData.substring(LoopModeStart + 2);
      LoopMode.trim();
      J1LoopMode = LoopMode.substring(0, 1).toInt();
      J2LoopMode = LoopMode.substring(1, 2).toInt();
      J3LoopMode = LoopMode.substring(2, 3).toInt();
      J4LoopMode = LoopMode.substring(3, 4).toInt();
      J5LoopMode = LoopMode.substring(4, 5).toInt();
      J6LoopMode = LoopMode.substring(5).toInt();

      inData = "";  // Clear recieved buffer

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

        SolveInverseKinematics();

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
        J1dir = (J1stepDif <= 0) ? 1 : 0;
        J2dir = (J2stepDif <= 0) ? 1 : 0;
        J3dir = (J3stepDif <= 0) ? 1 : 0;
        J4dir = (J4stepDif <= 0) ? 1 : 0;
        J5dir = (J5stepDif <= 0) ? 1 : 0;
        J6dir = (J6stepDif <= 0) ? 1 : 0;
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
          driveMotorsJ(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(J7stepDif), abs(J8stepDif), abs(J9stepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
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
      } else if (KinematicError == 1) {
        Alarm = "ER";
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      } else {
        Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault);
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      }

      inData = "";  // Clear recieved buffer
      ////////MOVE COMPLETE///////////
    }







    //----- LIVE JOINT JOG  ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "LJ") {

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
      float ACCspd = 0;
      float DCCspd = 0;
      float ACCramp = 10;
      WristCon = inData.substring(WristConStart + 1, LoopModeStart);
      String LoopMode = inData.substring(LoopModeStart + 2);
      LoopMode.trim();
      J1LoopMode = LoopMode.substring(0, 1).toInt();
      J2LoopMode = LoopMode.substring(1, 2).toInt();
      J3LoopMode = LoopMode.substring(2, 3).toInt();
      J4LoopMode = LoopMode.substring(3, 4).toInt();
      J5LoopMode = LoopMode.substring(4, 5).toInt();
      J6LoopMode = LoopMode.substring(5).toInt();

      inData = "";  // Clear recieved buffer

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
        J1dir = (J1stepDif <= 0) ? 1 : 0;
        J2dir = (J2stepDif <= 0) ? 1 : 0;
        J3dir = (J3stepDif <= 0) ? 1 : 0;
        J4dir = (J4stepDif <= 0) ? 1 : 0;
        J5dir = (J5stepDif <= 0) ? 1 : 0;
        J6dir = (J6stepDif <= 0) ? 1 : 0;
        J7dir = (J7stepDif <= 0) ? 1 : 0;
        J8dir = (J8stepDif <= 0) ? 1 : 0;
        J9dir = (J9stepDif <= 0) ? 1 : 0;

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
          driveMotorsJ(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(J7stepDif), abs(J8stepDif), abs(J9stepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
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
      } else if (KinematicError == 1) {
        Alarm = "ER";
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      } else {
        Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault) + String(J7axisFault) + String(J8axisFault) + String(J9axisFault);
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      }

      inData = "";  // Clear recieved buffer
      ////////MOVE COMPLETE///////////
    }








    //----- LIVE TOOL JOG  ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "LT") {
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

      float Xtool = Robot_Kin_Tool[0];
      float Ytool = Robot_Kin_Tool[1];
      float Ztool = Robot_Kin_Tool[2];
      float RZtool = Robot_Kin_Tool[3];
      float RYtool = Robot_Kin_Tool[4];
      float RXtool = Robot_Kin_Tool[5];

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

      inData = "";  // Clear recieved buffer


      Xtool = Robot_Kin_Tool[0];
      Ytool = Robot_Kin_Tool[1];
      Ztool = Robot_Kin_Tool[2];
      RXtool = Robot_Kin_Tool[3];
      RYtool = Robot_Kin_Tool[4];
      RZtool = Robot_Kin_Tool[5];

      JangleIn[0] = (J1StepM - J1zeroStep) / J1StepDeg;
      JangleIn[1] = (J2StepM - J2zeroStep) / J2StepDeg;
      JangleIn[2] = (J3StepM - J3zeroStep) / J3StepDeg;
      JangleIn[3] = (J4StepM - J4zeroStep) / J4StepDeg;
      JangleIn[4] = (J5StepM - J5zeroStep) / J5StepDeg;
      JangleIn[5] = (J6StepM - J6zeroStep) / J6StepDeg;

      while (JogInPoc == true) {

        if (Vector == 10) {
          Robot_Kin_Tool[0] = Robot_Kin_Tool[0] - 1;
        }
        if (Vector == 11) {
          Robot_Kin_Tool[0] = Robot_Kin_Tool[0] + 1;
        }

        if (Vector == 20) {
          Robot_Kin_Tool[1] = Robot_Kin_Tool[1] - 1;
        }
        if (Vector == 21) {
          Robot_Kin_Tool[1] = Robot_Kin_Tool[1] + 1;
        }

        if (Vector == 30) {
          Robot_Kin_Tool[2] = Robot_Kin_Tool[2] - 1;
        }
        if (Vector == 31) {
          Robot_Kin_Tool[2] = Robot_Kin_Tool[2] + 1;
        }

        if (Vector == 60) {
          Robot_Kin_Tool[3] = Robot_Kin_Tool[3] - 1 * M_PI / 180;
        }
        if (Vector == 61) {
          Robot_Kin_Tool[3] = Robot_Kin_Tool[3] + 1 * M_PI / 180;
        }

        if (Vector == 50) {
          Robot_Kin_Tool[4] = Robot_Kin_Tool[4] - 1 * M_PI / 180;
        }
        if (Vector == 51) {
          Robot_Kin_Tool[4] = Robot_Kin_Tool[4] + 1 * M_PI / 180;
        }

        if (Vector == 40) {
          Robot_Kin_Tool[5] = Robot_Kin_Tool[5] - 1 * M_PI / 180;
        }
        if (Vector == 41) {
          Robot_Kin_Tool[5] = Robot_Kin_Tool[5] + 1 * M_PI / 180;
        }



        xyzuvw_In[0] = xyzuvw_Out[0];
        xyzuvw_In[1] = xyzuvw_Out[1];
        xyzuvw_In[2] = xyzuvw_Out[2];
        xyzuvw_In[3] = xyzuvw_Out[3];
        xyzuvw_In[4] = xyzuvw_Out[4];
        xyzuvw_In[5] = xyzuvw_Out[5];

        SolveInverseKinematics();

        Robot_Kin_Tool[0] = Xtool;
        Robot_Kin_Tool[1] = Ytool;
        Robot_Kin_Tool[2] = Ztool;
        Robot_Kin_Tool[3] = RXtool;
        Robot_Kin_Tool[4] = RYtool;
        Robot_Kin_Tool[5] = RZtool;

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
        J1dir = (J1stepDif <= 0) ? 1 : 0;
        J2dir = (J2stepDif <= 0) ? 1 : 0;
        J3dir = (J3stepDif <= 0) ? 1 : 0;
        J4dir = (J4stepDif <= 0) ? 1 : 0;
        J5dir = (J5stepDif <= 0) ? 1 : 0;
        J6dir = (J6stepDif <= 0) ? 1 : 0;
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
          driveMotorsJ(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(J7stepDif), abs(J8stepDif), abs(J9stepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
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
      } else if (KinematicError == 1) {
        Alarm = "ER";
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      } else {
        Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault);
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      }

      inData = "";  // Clear recieved buffer
      ////////MOVE COMPLETE///////////
    }



    //----- Jog T ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "JT") {
      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;

      float Xtool = Robot_Kin_Tool[0];
      float Ytool = Robot_Kin_Tool[1];
      float Ztool = Robot_Kin_Tool[2];
      float RZtool = Robot_Kin_Tool[3];
      float RYtool = Robot_Kin_Tool[4];
      float RXtool = Robot_Kin_Tool[5];

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

      String Dir = inData.substring(0, 2);  // this should be Z0 or Z1
      float Dist = inData.substring(2, SPstart).toFloat();
      String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
      float SpeedVal = inData.substring(SPstart + 2, AcStart).toFloat();
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
        Robot_Kin_Tool[0] = Robot_Kin_Tool[0] + Dist;
      } else if (Dir == "X1") {
        Robot_Kin_Tool[0] = Robot_Kin_Tool[0] - Dist;
      } else if (Dir == "Y0") {
        Robot_Kin_Tool[1] = Robot_Kin_Tool[1] + Dist;
      } else if (Dir == "Y1") {
        Robot_Kin_Tool[1] = Robot_Kin_Tool[1] - Dist;
      } else if (Dir == "Z0") {
        Robot_Kin_Tool[2] = Robot_Kin_Tool[2] + Dist;
      } else if (Dir == "Z1") {
        Robot_Kin_Tool[2] = Robot_Kin_Tool[2] - Dist;
      } else if (Dir == "R0") {
        Robot_Kin_Tool[5] = Robot_Kin_Tool[5] + Dist * M_PI / 180;
      } else if (Dir == "R1") {
        Robot_Kin_Tool[5] = Robot_Kin_Tool[5] - Dist * M_PI / 180;
      } else if (Dir == "P0") {
        Robot_Kin_Tool[4] = Robot_Kin_Tool[4] + Dist * M_PI / 180;
      } else if (Dir == "P1") {
        Robot_Kin_Tool[4] = Robot_Kin_Tool[4] - Dist * M_PI / 180;
      } else if (Dir == "W0") {
        Robot_Kin_Tool[3] = Robot_Kin_Tool[3] + Dist * M_PI / 180;
      } else if (Dir == "W1") {
        Robot_Kin_Tool[3] = Robot_Kin_Tool[3] - Dist * M_PI / 180;
      }


      JangleIn[0] = (J1StepM - J1zeroStep) / J1StepDeg;
      JangleIn[1] = (J2StepM - J2zeroStep) / J2StepDeg;
      JangleIn[2] = (J3StepM - J3zeroStep) / J3StepDeg;
      JangleIn[3] = (J4StepM - J4zeroStep) / J4StepDeg;
      JangleIn[4] = (J5StepM - J5zeroStep) / J5StepDeg;
      JangleIn[5] = (J6StepM - J6zeroStep) / J6StepDeg;


      xyzuvw_In[0] = xyzuvw_Out[0];
      xyzuvw_In[1] = xyzuvw_Out[1];
      xyzuvw_In[2] = xyzuvw_Out[2];
      xyzuvw_In[3] = xyzuvw_Out[3];
      xyzuvw_In[4] = xyzuvw_Out[4];
      xyzuvw_In[5] = xyzuvw_Out[5];

      SolveInverseKinematics();

      Robot_Kin_Tool[0] = Xtool;
      Robot_Kin_Tool[1] = Ytool;
      Robot_Kin_Tool[2] = Ztool;
      Robot_Kin_Tool[3] = RZtool;
      Robot_Kin_Tool[4] = RYtool;
      Robot_Kin_Tool[5] = RXtool;


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
      J1dir = (J1stepDif <= 0) ? 1 : 0;
      J2dir = (J2stepDif <= 0) ? 1 : 0;
      J3dir = (J3stepDif <= 0) ? 1 : 0;
      J4dir = (J4stepDif <= 0) ? 1 : 0;
      J5dir = (J5stepDif <= 0) ? 1 : 0;
      J6dir = (J6stepDif <= 0) ? 1 : 0;
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

      debug = String(SpeedVal);
      //send move command if no axis limit error
      if (TotalAxisFault == 0 && KinematicError == 0) {
        resetEncoders();
        driveMotorsJ(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(J7stepDif), abs(J8stepDif), abs(J9stepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
        checkEncoders();
        sendRobotPos();
      } else if (KinematicError == 1) {
        Alarm = "ER";
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      } else {
        Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault);
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      }

      inData = "";  // Clear recieved buffer
      ////////MOVE COMPLETE///////////
    }













    //----- MOVE V ------ VISION OFFSET ----------------------------------
    //-----------------------------------------------------------------------
    if (function == "MV") {
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

      //get current tool rotation
      float RXtool = Robot_Kin_Tool[5];


      // offset tool rotation by the found vision angle
      Robot_Kin_Tool[5] = Robot_Kin_Tool[5] - VisRot * M_PI / 180;

      //solve kinematics
      SolveInverseKinematics();

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
      Robot_Kin_Tool[5] = RXtool;

      //determine motor directions
      J1dir = (J1stepDif <= 0) ? 1 : 0;
      J2dir = (J2stepDif <= 0) ? 1 : 0;
      J3dir = (J3stepDif <= 0) ? 1 : 0;
      J4dir = (J4stepDif <= 0) ? 1 : 0;
      J5dir = (J5stepDif <= 0) ? 1 : 0;
      J6dir = (J6stepDif <= 0) ? 1 : 0;
      J7dir = (J7stepDif <= 0) ? 1 : 0;
      J8dir = (J8stepDif <= 0) ? 1 : 0;
      J9dir = (J9stepDif <= 0) ? 1 : 0;



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
      } else if (KinematicError == 1) {
        Alarm = "ER";
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      } else {
        Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault) + String(J7axisFault) + String(J8axisFault) + String(J9axisFault);
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      }



      inData = "";  // Clear recieved buffer
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
      J1dir = (J1stepDif <= 0) ? 1 : 0;
      J2dir = (J2stepDif <= 0) ? 1 : 0;
      J3dir = (J3stepDif <= 0) ? 1 : 0;
      J4dir = (J4stepDif <= 0) ? 1 : 0;
      J5dir = (J5stepDif <= 0) ? 1 : 0;
      J6dir = (J6stepDif <= 0) ? 1 : 0;
      J7dir = (J7stepDif <= 0) ? 1 : 0;
      J8dir = (J8stepDif <= 0) ? 1 : 0;
      J9dir = (J9stepDif <= 0) ? 1 : 0;


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
      } else if (KinematicError == 1) {
        Alarm = "ER";
        delay(5);
        Serial.println(Alarm);
      } else {
        Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault) + String(J7axisFault) + String(J8axisFault) + String(J9axisFault);
        delay(5);
        Serial.println(Alarm);
      }


      inData = "";  // Clear recieved buffer
      ////////MOVE COMPLETE///////////
    }



    //----- MOVE L ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "ML" and flag == "") {
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
        rndData = "X" + String(rndArcMid[0]) + "Y" + String(rndArcMid[1]) + "Z" + String(rndArcMid[2]) + "Rz" + String(rndArcMid[3]) + "Ry" + String(rndArcMid[4]) + "Rx" + String(rndArcMid[5]) + "Ex" + String(rndArcEnd[0]) + "Ey" + String(rndArcEnd[1]) + "Ez" + String(rndArcEnd[2]) + "Tr" + String(xyzuvw_Temp[6]) + "S" + SpeedType + String(SpeedVal) + "Ac" + String(ACCspd) + "Dc" + String(DCCspd) + "Rm" + String(ACCramp) + "W" + WristCon;
        function = "MA";
        rndTrue = true;
      } else {
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
      RZvect = xyzuvw_In[3] - xyzuvw_Out[3];
      RYvect = xyzuvw_In[4] - xyzuvw_Out[4];
      RXvect = xyzuvw_In[5] - xyzuvw_Out[5];


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
        float wayPerc = 1 / wayPts;

        //pre calculate entire move and speeds

        SolveInverseKinematics();
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
        if (J2stepDif > HighStep) {
          HighStep = J2stepDif;
        }
        if (J3stepDif > HighStep) {
          HighStep = J3stepDif;
        }
        if (J4stepDif > HighStep) {
          HighStep = J4stepDif;
        }
        if (J5stepDif > HighStep) {
          HighStep = J5stepDif;
        }
        if (J6stepDif > HighStep) {
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
          speedSP = (SpeedVal * 1000000) * 1.2;
        } else if ((SpeedType == "m")) {
          speedSP = ((lineDist / SpeedVal) * 1000000) * 1.2;
        }

        //calc step gap for seconds or mm per sec
        if (SpeedType == "s" or SpeedType == "m") {
          float zeroStepGap = speedSP / HighStep;
          float zeroACCstepInc = (zeroStepGap * (100 / ACCramp)) / ACCStep;
          float zeroACCtime = ((ACCStep)*zeroStepGap) + ((ACCStep - 9) * (((ACCStep) * (zeroACCstepInc / 2))));
          float zeroNORtime = NORStep * zeroStepGap;
          float zeroDCCstepInc = (zeroStepGap * (100 / ACCramp)) / DCCStep;
          float zeroDCCtime = ((DCCStep)*zeroStepGap) + ((DCCStep - 9) * (((DCCStep) * (zeroDCCstepInc / 2))));
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
        } else {
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
        } else {
          J7dir = 0;
        }

        if (J8stepDif <= 0) {
          J8dir = 1;
        } else {
          J8dir = 0;
        }

        if (J9stepDif <= 0) {
          J9dir = 1;
        } else {
          J9dir = 0;
        }


        resetEncoders();
        /////////////////////////////////////////////////
        //loop through waypoints
        for (int i = 0; i <= wayPts + 1; i++) {

          ////DELAY CALC/////
          if (i <= ACCwayPts) {
            curDelay = curDelay - (ACCwayInc);
          } else if (i >= (wayPts - DCCwayPts)) {
            curDelay = curDelay + (DCCwayInc);
          } else {
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

          SolveInverseKinematics();

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
          J1dir = (J1stepDif <= 0) ? 1 : 0;
          J2dir = (J2stepDif <= 0) ? 1 : 0;
          J3dir = (J3stepDif <= 0) ? 1 : 0;
          J4dir = (J4stepDif <= 0) ? 1 : 0;
          J5dir = (J5stepDif <= 0) ? 1 : 0;
          J6dir = (J6stepDif <= 0) ? 1 : 0;

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
          } else if (KinematicError == 1) {
            Alarm = "ER";
            if (splineTrue == false) {
              delay(5);
              Serial.println(Alarm);
            }
          } else {
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
      inData = "";  // Clear recieved buffer
      ////////MOVE COMPLETE///////////
    }




    //----- MOVE J ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "MJ") {
      moveJ(inData, true, false, false);
    }

    //----- MOVE G ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "MG") {
      moveJ(inData, true, false, true);
    }


    //----- DELETE PROG FROM SD CARD ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "DG") {
      SD.begin(BUILTIN_SDCARD);
      int fileStart = inData.indexOf("Fn");
      String filename = inData.substring(fileStart + 2);
      const char *fn = filename.c_str();
      if (SD.exists(fn)) {
        deleteSD(filename);
        Serial.println("P");
      } else {
        Serial.println("F");
      }
    }

    //----- READ FILES FROM SD CARD ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "RG") {
      File root;
      SD.begin(BUILTIN_SDCARD);
      root = SD.open("/");
      printDirectory(root, 0);
    }


    //----- WRITE COMMAND TO SD CARD ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "WC") {
      SD.begin(BUILTIN_SDCARD);
      int fileStart = inData.indexOf("Fn");
      String filename = inData.substring(fileStart + 2);
      const char *fn = filename.c_str();
      String info = inData.substring(0, fileStart);
      writeSD(fn, info);
      //moveJ(info, false, true, false);
      sendRobotPos();
    }

    //----- PLAY FILE ON SD CARD ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "PG") {
      File gcFile;
      String Cmd;
      SD.begin(BUILTIN_SDCARD);
      int fileStart = inData.indexOf("Fn");
      String filename = inData.substring(fileStart + 2);
      const char *fn = filename.c_str();
      gcFile = SD.open(fn);
      if (!gcFile) {
        Serial.println("EG");
        while (1)
          ;
      }
      while (gcFile.available() && estopActive == false) {
        Cmd = gcFile.readStringUntil('\n');
        //CARTESIAN CMD
        if (Cmd.substring(0, 1) == "X") {
          updatePos();
          moveJ(Cmd, false, false, true);
        }
        //PRECALC'D CMD - not currently used, needs position handling
        else {
          int i1 = Cmd.indexOf(',');
          int i2 = Cmd.indexOf(',', i1 + 1);
          int i3 = Cmd.indexOf(',', i2 + 1);
          int i4 = Cmd.indexOf(',', i3 + 1);
          int i5 = Cmd.indexOf(',', i4 + 1);
          int i6 = Cmd.indexOf(',', i5 + 1);
          int i7 = Cmd.indexOf(',', i6 + 1);
          int i8 = Cmd.indexOf(',', i7 + 1);
          int i9 = Cmd.indexOf(',', i8 + 1);
          int i10 = Cmd.indexOf(',', i9 + 1);
          int i11 = Cmd.indexOf(',', i10 + 1);
          int i12 = Cmd.indexOf(',', i11 + 1);
          int i13 = Cmd.indexOf(',', i12 + 1);
          int i14 = Cmd.indexOf(',', i13 + 1);
          int i15 = Cmd.indexOf(',', i14 + 1);
          int i16 = Cmd.indexOf(',', i15 + 1);
          int i17 = Cmd.indexOf(',', i16 + 1);
          int i18 = Cmd.indexOf(',', i17 + 1);
          int i19 = Cmd.indexOf(',', i18 + 1);
          int i20 = Cmd.indexOf(',', i19 + 1);
          int i21 = Cmd.indexOf(',', i20 + 1);
          int i22 = Cmd.indexOf(',', i21 + 1);
          int i23 = Cmd.indexOf(',', i22 + 1);
          int J1step = Cmd.substring(0, i1).toInt();
          int J2step = Cmd.substring(i1 + 1, i2).toInt();
          int J3step = Cmd.substring(i2 + 1, i3).toInt();
          int J4step = Cmd.substring(i3 + 1, i4).toInt();
          int J5step = Cmd.substring(i4 + 1, i5).toInt();
          int J6step = Cmd.substring(i5 + 1, i6).toInt();
          int J7step = Cmd.substring(i6 + 1, i7).toInt();
          int J8step = Cmd.substring(i7 + 1, i8).toInt();
          int J9step = Cmd.substring(i8 + 1, i9).toInt();
          int J1dir = Cmd.substring(i9 + 1, i10).toInt();
          int J2dir = Cmd.substring(i10 + 1, i11).toInt();
          int J3dir = Cmd.substring(i11 + 1, i12).toInt();
          int J4dir = Cmd.substring(i12 + 1, i13).toInt();
          int J5dir = Cmd.substring(i13 + 1, i14).toInt();
          int J6dir = Cmd.substring(i14 + 1, i15).toInt();
          int J7dir = Cmd.substring(i15 + 1, i16).toInt();
          int J8dir = Cmd.substring(i16 + 1, i17).toInt();
          int J9dir = Cmd.substring(i17 + 1, i18).toInt();
          String SpeedType = Cmd.substring(i18 + 1, i19);
          float SpeedVal = Cmd.substring(i19 + 1, i20).toFloat();
          float ACCspd = Cmd.substring(i20 + 1, i21).toFloat();
          float DCCspd = Cmd.substring(i21 + 1, i22).toFloat();
          float ACCramp = Cmd.substring(i22 + 1).toFloat();
          driveMotorsG(J1step, J2step, J3step, J4step, J5step, J6step, J7step, J8step, J9step, J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
        }
      }
      gcFile.close();
      sendRobotPos();
    }



    //----- WRITE PRE-CALC'D MOVE TO SD CARD ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "WG") {
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

      String info;

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
      int fileStart = inData.indexOf("Fn");

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
      String LoopMode = inData.substring(LoopModeStart + 2, fileStart);
      String filename = inData.substring(fileStart + 2);

      J1LoopMode = LoopMode.substring(0, 1).toInt();
      J2LoopMode = LoopMode.substring(1, 2).toInt();
      J3LoopMode = LoopMode.substring(2, 3).toInt();
      J4LoopMode = LoopMode.substring(3, 4).toInt();
      J5LoopMode = LoopMode.substring(4, 5).toInt();
      J6LoopMode = LoopMode.substring(5).toInt();

      SolveInverseKinematics();

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

      //set step
      J1StepM = J1futStepM;
      J2StepM = J2futStepM;
      J3StepM = J3futStepM;
      J4StepM = J4futStepM;
      J5StepM = J5futStepM;
      J6StepM = J6futStepM;
      J7StepM = J7futStepM;
      J8StepM = J8futStepM;
      J9StepM = J9futStepM;

      //determine motor directions
      J1dir = (J1stepDif <= 0) ? 1 : 0;
      J2dir = (J2stepDif <= 0) ? 1 : 0;
      J3dir = (J3stepDif <= 0) ? 1 : 0;
      J4dir = (J4stepDif <= 0) ? 1 : 0;
      J5dir = (J5stepDif <= 0) ? 1 : 0;
      J6dir = (J6stepDif <= 0) ? 1 : 0;
      J7dir = (J7stepDif <= 0) ? 1 : 0;
      J8dir = (J8stepDif <= 0) ? 1 : 0;
      J9dir = (J9stepDif <= 0) ? 1 : 0;


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
        info = String(abs(J1stepDif)) + "," + String(abs(J2stepDif)) + "," + String(abs(J3stepDif)) + "," + String(abs(J4stepDif)) + "," + String(abs(J5stepDif)) + "," + String(abs(J6stepDif)) + "," + String(abs(J7stepDif)) + "," + String(abs(J8stepDif)) + "," + String(abs(J9stepDif)) + "," + String(J1dir) + "," + String(J2dir) + "," + String(J3dir) + "," + String(J4dir) + "," + String(J5dir) + "," + String(J6dir) + "," + String(J7dir) + "," + String(J8dir) + "," + String(J9dir) + "," + String(SpeedType) + "," + String(SpeedVal) + "," + String(ACCspd) + "," + String(DCCspd) + "," + String(ACCramp);
        writeSD(filename, info);
        sendRobotPos();
      } else if (KinematicError == 1) {
        Alarm = "ER";
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      } else {
        Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault) + String(J7axisFault) + String(J8axisFault) + String(J9axisFault);
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      }

      inData = "";  // Clear recieved buffer
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
      float axis[3];
      float axisTemp[3];
      float startVect[3];
      float Rotation[3][3];
      float DestPt[3];
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
      float Px = xBeg;
      float Py = yBeg;
      float Pz = zBeg;

      //define start vetor (mid) point is start of circle
      startVect[0] = (xMid - Px);
      startVect[1] = (yMid - Py);
      startVect[2] = (zMid - Pz);
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
      axis[0] = CrossX / sqrt((CrossX * CrossX) + (CrossY * CrossY) + (CrossZ * CrossZ));
      axis[1] = CrossY / sqrt((CrossX * CrossX) + (CrossY * CrossY) + (CrossZ * CrossZ));
      axis[2] = CrossZ / sqrt((CrossX * CrossX) + (CrossY * CrossY) + (CrossZ * CrossZ));
      //get radian angle between vectors using acos of dot product
      float BCradians = acos((vect_Bx * vect_Cx + vect_By * vect_Cy + vect_Bz * vect_Cz) / (sqrt(pow(vect_Bx, 2) + pow(vect_Cy, 2) + pow(vect_Bz, 2)) * sqrt(pow(vect_Cx, 2) + pow(vect_Cy, 2) + pow(vect_Cz, 2))));
      //get arc degree
      float ABdegrees = degrees(BCradians);
      //get direction from angle
      if (ABdegrees > 0) {
        Cdir = 1;
      } else {
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
      } else if (SpeedType == "m") {
        speedSP = ((lineDist / SpeedVal) * 1000000) * 1.75;
      }

      //calc step gap for seconds or mm per sec
      if (SpeedType == "s" or SpeedType == "m") {
        float zeroStepGap = speedSP / HighStep;
        float zeroACCstepInc = (zeroStepGap * (100 / ACCramp)) / ACCStep;
        float zeroACCtime = ((ACCStep)*zeroStepGap) + ((ACCStep - 9) * (((ACCStep) * (zeroACCstepInc / 2))));
        float zeroNORtime = NORStep * zeroStepGap;
        float zeroDCCstepInc = (zeroStepGap * (100 / ACCramp)) / DCCStep;
        float zeroDCCtime = ((DCCStep)*zeroStepGap) + ((DCCStep - 9) * (((DCCStep) * (zeroDCCstepInc / 2))));
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
        b = -axis[0] * sin(theta / 2.0);
        c = -axis[1] * sin(theta / 2.0);
        d = -axis[2] * sin(theta / 2.0);
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
        Rotation[0][0] = aa + bb - cc - dd;
        Rotation[0][1] = 2 * (bc + ad);
        Rotation[0][2] = 2 * (bd - ac);
        Rotation[1][0] = 2 * (bc - ad);
        Rotation[1][1] = aa + cc - bb - dd;
        Rotation[1][2] = 2 * (cd + ab);
        Rotation[2][0] = 2 * (bd + ac);
        Rotation[2][1] = 2 * (cd - ab);
        Rotation[2][2] = aa + dd - bb - cc;

        //get product of current rotation and start vector
        DestPt[0] = (Rotation[0][0] * startVect[0]) + (Rotation[0][1] * startVect[1]) + (Rotation[0][2] * startVect[2]);
        DestPt[1] = (Rotation[1][0] * startVect[0]) + (Rotation[1][1] * startVect[1]) + (Rotation[1][2] * startVect[2]);
        DestPt[2] = (Rotation[2][0] * startVect[0]) + (Rotation[2][1] * startVect[1]) + (Rotation[2][2] * startVect[2]);

        ////DELAY CALC/////
        if (i <= ACCwayPts) {
          curDelay = curDelay - (ACCwayInc);
        } else if (i >= (wayPts - DCCwayPts)) {
          curDelay = curDelay + (DCCwayInc);
        } else {
          curDelay = calcStepGap;
        }

        //shift way pts back to orignal origin and calc kinematics for way pt movement
        xyzuvw_In[0] = (DestPt[0]) + Px;
        xyzuvw_In[1] = (DestPt[1]) + Py;
        xyzuvw_In[2] = (DestPt[2]) + Pz;
        xyzuvw_In[3] = rzBeg;
        xyzuvw_In[4] = ryBeg;
        xyzuvw_In[5] = rxBeg;

        SolveInverseKinematics();

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
        J1dir = (J1stepDif <= 0) ? 1 : 0;
        J2dir = (J2stepDif <= 0) ? 1 : 0;
        J3dir = (J3stepDif <= 0) ? 1 : 0;
        J4dir = (J4stepDif <= 0) ? 1 : 0;
        J5dir = (J5stepDif <= 0) ? 1 : 0;
        J6dir = (J6stepDif <= 0) ? 1 : 0;
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
        } else if (KinematicError == 1) {
          Alarm = "ER";
          delay(5);
          Serial.println(Alarm);
        } else {
          Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault);
          delay(5);
          Serial.println(Alarm);
        }


        //increment angle
        cur_deg += theta_Deg;
      }

      checkEncoders();
      sendRobotPos();


      inData = "";  // Clear recieved buffer
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
      float axis[3];
      float axisTemp[3];
      float startVect[3];
      float Rotation[3][3];
      float DestPt[3];
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
      float Px = ((BCx * xBeg) + (BCy * xMid) + (BCz * xEnd)) / (BCx + BCy + BCz);
      float Py = ((BCx * yBeg) + (BCy * yMid) + (BCz * yEnd)) / (BCx + BCy + BCz);
      float Pz = ((BCx * zBeg) + (BCy * zMid) + (BCz * zEnd)) / (BCx + BCy + BCz);
      //define start vetor
      startVect[0] = (xBeg - Px);
      startVect[1] = (yBeg - Py);
      startVect[2] = (zBeg - Pz);
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
      axis[0] = CrossX / sqrt((CrossX * CrossX) + (CrossY * CrossY) + (CrossZ * CrossZ));
      axis[1] = CrossY / sqrt((CrossX * CrossX) + (CrossY * CrossY) + (CrossZ * CrossZ));
      axis[2] = CrossZ / sqrt((CrossX * CrossX) + (CrossY * CrossY) + (CrossZ * CrossZ));
      //get radian angle between vectors using acos of dot product
      float ABradians = acos((vect_Ax * vect_Bx + vect_Ay * vect_By + vect_Az * vect_Bz) / (sqrt(pow(vect_Ax, 2) + pow(vect_Ay, 2) + pow(vect_Az, 2)) * sqrt(pow(vect_Bx, 2) + pow(vect_By, 2) + pow(vect_Bz, 2))));
      float BCradians = acos((vect_Bx * vect_Cx + vect_By * vect_Cy + vect_Bz * vect_Cz) / (sqrt(pow(vect_Bx, 2) + pow(vect_By, 2) + pow(vect_Bz, 2)) * sqrt(pow(vect_Cx, 2) + pow(vect_Cy, 2) + pow(vect_Cz, 2))));
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
      } else if (SpeedType == "m") {
        speedSP = ((lineDist / SpeedVal) * 1000000) * 1.2;
      }

      //calc step gap for seconds or mm per sec
      if (SpeedType == "s" or SpeedType == "m") {
        float zeroStepGap = speedSP / HighStep;
        float zeroACCstepInc = (zeroStepGap * (100 / ACCramp)) / ACCStep;
        float zeroACCtime = ((ACCStep)*zeroStepGap) + ((ACCStep - 9) * (((ACCStep) * (zeroACCstepInc / 2))));
        float zeroNORtime = NORStep * zeroStepGap;
        float zeroDCCstepInc = (zeroStepGap * (100 / ACCramp)) / DCCStep;
        float zeroDCCtime = ((DCCStep)*zeroStepGap) + ((DCCStep - 9) * (((DCCStep) * (zeroDCCstepInc / 2))));
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
      } else {
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
        b = -axis[0] * sin(theta / 2.0);
        c = -axis[1] * sin(theta / 2.0);
        d = -axis[2] * sin(theta / 2.0);
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
        Rotation[0][0] = aa + bb - cc - dd;
        Rotation[0][1] = 2 * (bc + ad);
        Rotation[0][2] = 2 * (bd - ac);
        Rotation[1][0] = 2 * (bc - ad);
        Rotation[1][1] = aa + cc - bb - dd;
        Rotation[1][2] = 2 * (cd + ab);
        Rotation[2][0] = 2 * (bd + ac);
        Rotation[2][1] = 2 * (cd - ab);
        Rotation[2][2] = aa + dd - bb - cc;

        //get product of current rotation and start vector
        DestPt[0] = (Rotation[0][0] * startVect[0]) + (Rotation[0][1] * startVect[1]) + (Rotation[0][2] * startVect[2]);
        DestPt[1] = (Rotation[1][0] * startVect[0]) + (Rotation[1][1] * startVect[1]) + (Rotation[1][2] * startVect[2]);
        DestPt[2] = (Rotation[2][0] * startVect[0]) + (Rotation[2][1] * startVect[1]) + (Rotation[2][2] * startVect[2]);

        ////DELAY CALC/////
        if (rndTrue == true) {
          curDelay = rndSpeed;
        } else if (i <= ACCwayPts) {
          curDelay = curDelay - (ACCwayInc);
        } else if (i >= (wayPts - DCCwayPts)) {
          curDelay = curDelay + (DCCwayInc);
        } else {
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


        SolveInverseKinematics();

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
        J1dir = (J1stepDif <= 0) ? 1 : 0;
        J2dir = (J2stepDif <= 0) ? 1 : 0;
        J3dir = (J3stepDif <= 0) ? 1 : 0;
        J4dir = (J4stepDif <= 0) ? 1 : 0;
        J5dir = (J5stepDif <= 0) ? 1 : 0;
        J6dir = (J6stepDif <= 0) ? 1 : 0;
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
        } else if (KinematicError == 1) {
          Alarm = "ER";
          if (splineTrue == false) {
            delay(5);
            Serial.println(Alarm);
          }
        } else {
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
      inData = "";  // Clear recieved buffer
      if (splineTrue == false) {
        sendRobotPos();
      }
      ////////MOVE COMPLETE///////////
    }

    else {
      inData = "";  // Clear recieved buffer
    }

    //shift cmd buffer
    inData = "";
    cmdBuffer1 = "";
    shiftCMDarray();
  }
}
