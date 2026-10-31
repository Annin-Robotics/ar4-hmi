//VERSION 8.1

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
// 6.2 - 8/29/25 changed bootstrap theme, xbox upgrade
// 6.3 - 10/8/25 - JK - added beta Linux support
// 6.4 - 10/29/25 - added set robot command to store HW and version to eprom, MK4 update, fixed tool jog, re-added 2 step calibration, add servo amp test
// 6.5 - 2/15/26 - gcode bug fix / update program flow with run once and run program loop
// 6.6 - 2/22/26 - update kinematic solver to reduce J4/6 wrap | reimplement wrist N/F config
// 6.7 - 3/11/26 MB holding reg bug fix
// 6.8 - 3/31/26 - MK5 update - hall effect cal high low
// 7.0 - 6/28/26 - JSON and API updates
// 8.0 - 10/7/26 - Phase 2 unification: runtime PinMap (AR4/TST), Encoder pointers constructed at boot, reboot after set_robot_id
//                 motion streaming: queued-move ack protocol ("q":1), one reply per command, soft stop command,
//                 reliable Move L blending (waits for next move), tangent blend arc, Move L endpoint overshoot fix,
//                 Move L / Move A / blend orientation interpolated as a true rotation (fixes wrist flips near 180 deg),
//                 live joint-angle telemetry while streaming ({"msg":"tele"}) so the PC virtual robot can mirror the arm
// 8.1 - 10/30/26 - kinematics: IK solves at the wrist singularity (J5 = 0 found no solution before), accurate J4/J6
//                 near J5 = 0, J4/J6 unwrapped into the joint limits instead of jumping 360 deg at +/-180, waypoint
//                 jump guard for Move L / Move A / live jogs, Cartesian Rx/Ry/Rz jog rotates about the world axes.
//                 Solver shared with ARrobots/src/kinematics.cpp (PC virtual robot) - keep both in step.

const char *FIRMWARE_VERSION = "8.1";

//////////////////////////////////////////////////////////////////////////////
//DEBUGGING
//////////////////////////////////////////////////////////////////////////////
// Define function aliases for debugging but only if debugging is enabled

#include <math.h>
#include <limits>
#include <avr/pgmspace.h>
#include <Encoder.h>
#include <SPI.h>
#include <SD.h>
#include <stdexcept>
#include <ModbusMaster.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#pragma GCC diagnostic ignored "-Warray-bounds"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wsequence-point"
#pragma GCC diagnostic ignored "-Wunused-value"
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#pragma GCC diagnostic ignored "-Waddress"
#pragma GCC diagnostic ignored "-Wall"

bool DEBUG = false;
// These Debug printers do nothing unless DEBUG = true
#define DEBUG_PRINT(x) \
  do { \
    if (DEBUG) Serial.print(x); \
  } while (0)
#define DEBUG_PRINTLN(x) \
  do { \
    if (DEBUG) Serial.println(x); \
  } while (0)

#define Table_Size 6
typedef float Matrix4x4[16];
typedef float tRobot[66];
#define ROBOT_nDOFs 6
const int numJoints = 9;

String cmdBuffer1;
String cmdBuffer2;
String cmdBuffer3;
String inData;
String recData;
String checkData;
String function;
volatile byte state = LOW;

////////////////////////////////////////////////////////////////
// PIN MAP — selected at boot from EEPROM robot model (Phase 2)
// -1 = pin/feature not present on this model
////////////////////////////////////////////////////////////////
struct PinMap {
  int stepPin[9];
  int dirPin[9];
  int calPin[9];
  int calPinMode[9];   // INPUT or INPUT_PULLUP
  int faultPin[9];     // drive fault inputs (TST only)
  int estopPin;
  int estop2Pin;
  int encA[6];         // encoder channel A pins (AR4 family only)
  int encB[6];         // encoder channel B pins
  float minSpeedDelay; // model-specific, not user configurable
  float maxMMperSec;
};

static const PinMap PINS_AR4 = {
  { 0, 2, 4, 6, 8, 10, 12, 32, 40 },                    // stepPin J1-J9
  { 1, 3, 5, 7, 9, 11, 13, 33, 41 },                    // dirPin  J1-J9
  { 26, 27, 28, 29, 30, 31, 36, 37, 38 },               // calPin  J1-J9
  { INPUT_PULLUP, INPUT_PULLUP, INPUT_PULLUP,
    INPUT, INPUT, INPUT, INPUT, INPUT, INPUT },          // calPinMode
  { -1, -1, -1, -1, -1, -1, -1, -1, -1 },               // faultPin (none)
  39,                                                    // estopPin
  -1,                                                    // estop2Pin (none)
  { 14, 17, 19, 20, 23, 24 },                            // encA J1-J6
  { 15, 16, 18, 21, 22, 25 },                            // encB J1-J6
  200.0f,                                                // minSpeedDelay
  192.0f                                                 // maxMMperSec
};

static const PinMap PINS_TST = {
  { 2, 4, 6, 8, 10, 12, 14, 16, 18 },                   // stepPin J1-J9
  { 3, 5, 7, 9, 11, 13, 15, 17, 19 },                   // dirPin  J1-J9
  { 20, 21, 22, 23, 24, 25, 26, 27, 28 },               // calPin  J1-J9
  { INPUT_PULLUP, INPUT_PULLUP, INPUT_PULLUP,
    INPUT_PULLUP, INPUT_PULLUP, INPUT_PULLUP,
    INPUT_PULLUP, INPUT_PULLUP, INPUT_PULLUP },          // calPinMode (hall, all)
  { 29, 30, 31, 32, 33, 34, 35, 36, 37 },               // faultPin J1-J9
  38,                                                    // estopPin
  39,                                                    // estop2Pin
  { -1, -1, -1, -1, -1, -1 },                            // encA (no encoders)
  { -1, -1, -1, -1, -1, -1 },                            // encB
  40.0f,                                                 // minSpeedDelay
  192.0f                                                 // maxMMperSec
};

const PinMap* activePins = nullptr;  // set in setup() from EEPROM model

// Capability flags — behavior code checks these, never the model string (Phase 3).
struct RobotCapabilities {
  bool hasEncoders;     // Teensy-connected quadrature encoders (AR4 family)
  bool hasDriveModbus;  // servo drives on Serial1 RS485 bus (TST) — used Phase 4
  bool calZeroesDrives; // calibration zeroes drive positions (TST) — used Phase 5
};

static const RobotCapabilities CAPS_AR4 = { true,  false, false };
static const RobotCapabilities CAPS_TST = { false, true,  true  };

const RobotCapabilities* CAP = nullptr;  // set in setup() alongside applyPinMap()

// Runtime pin globals — populated by applyPinMap() at boot.
// Names preserved so all existing call sites work unchanged.
int J1stepPin = -1, J1dirPin = -1;
int J2stepPin = -1, J2dirPin = -1;
int J3stepPin = -1, J3dirPin = -1;
int J4stepPin = -1, J4dirPin = -1;
int J5stepPin = -1, J5dirPin = -1;
int J6stepPin = -1, J6dirPin = -1;
int J7stepPin = -1, J7dirPin = -1;
int J8stepPin = -1, J8dirPin = -1;
int J9stepPin = -1, J9dirPin = -1;

int J1calPin = -1, J2calPin = -1, J3calPin = -1;
int J4calPin = -1, J5calPin = -1, J6calPin = -1;
int J7calPin = -1, J8calPin = -1, J9calPin = -1;

int J1faultPin = -1, J2faultPin = -1, J3faultPin = -1;
int J4faultPin = -1, J5faultPin = -1, J6faultPin = -1;
int J7faultPin = -1, J8faultPin = -1, J9faultPin = -1;

int EstopPin = -1;
int Estop2Pin = -1;

// Model-specific speed constants — set by applyPinMap() at boot
float minSpeedDelay = 200;
float maxMMperSec = 192;

// Copies the selected PinMap into the runtime pin globals and
// model-specific speed constants. Called once from setup() when configured.
void applyPinMap(const PinMap* m) {
  activePins = m;

  J1stepPin = m->stepPin[0]; J1dirPin = m->dirPin[0];
  J2stepPin = m->stepPin[1]; J2dirPin = m->dirPin[1];
  J3stepPin = m->stepPin[2]; J3dirPin = m->dirPin[2];
  J4stepPin = m->stepPin[3]; J4dirPin = m->dirPin[3];
  J5stepPin = m->stepPin[4]; J5dirPin = m->dirPin[4];
  J6stepPin = m->stepPin[5]; J6dirPin = m->dirPin[5];
  J7stepPin = m->stepPin[6]; J7dirPin = m->dirPin[6];
  J8stepPin = m->stepPin[7]; J8dirPin = m->dirPin[7];
  J9stepPin = m->stepPin[8]; J9dirPin = m->dirPin[8];

  J1calPin = m->calPin[0]; J2calPin = m->calPin[1]; J3calPin = m->calPin[2];
  J4calPin = m->calPin[3]; J5calPin = m->calPin[4]; J6calPin = m->calPin[5];
  J7calPin = m->calPin[6]; J8calPin = m->calPin[7]; J9calPin = m->calPin[8];

  J1faultPin = m->faultPin[0]; J2faultPin = m->faultPin[1]; J3faultPin = m->faultPin[2];
  J4faultPin = m->faultPin[3]; J5faultPin = m->faultPin[4]; J6faultPin = m->faultPin[5];
  J7faultPin = m->faultPin[6]; J8faultPin = m->faultPin[7]; J9faultPin = m->faultPin[8];

  EstopPin = m->estopPin;
  Estop2Pin = m->estop2Pin;

  minSpeedDelay = m->minSpeedDelay;
  maxMMperSec = m->maxMMperSec;
}


//set encoder multiplier
float J1encMult = 5;
float J2encMult = 5;
float J3encMult = 5;
float J4encMult = 5;
float J5encMult = 2.5;
float J6encMult = 5;
int encOffset = 50;

//set encoder pins
// Encoder objects are constructed in setup() only when the model has
// encoders (pins come from the active PinMap). Global constructors would
// attach pin-change interrupts before setup() runs — fatal on the TST,
// where those pins are step/dir outputs. (Phase 2)
Encoder* J1encPos = nullptr;
Encoder* J2encPos = nullptr;
Encoder* J3encPos = nullptr;
Encoder* J4encPos = nullptr;
Encoder* J5encPos = nullptr;
Encoder* J6encPos = nullptr;
// (interim `bool hasEncoders` folded into CAP->hasEncoders in Phase 3)

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

uint8_t limitSensor[numJoints] = { LOW, LOW, LOW, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH };

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



//FIRMWARE CALIBRATION OFFSET - this is combined with the HMI software offset
float J1calBaseOff = -6.2;
float J2calBaseOff = 3.8;
float J3calBaseOff = 1.4;
float J4calBaseOff = -.8;
float J5calBaseOff = 5.6;
float J6calBaseOff = .5;
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
// minSpeedDelay / maxMMperSec are model-specific; declared above with the PinMap (Phase 2)
float linWayDistSP = 1;
String debug = "";
String flag = "";
const int TRACKrotdir = 0;
float JogStepInc = .5;
float JogStepTargetF[6] = { 0, 0, 0, 0, 0, 0 };
bool JogStepTargetInit = false;
bool splineResume = false;

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


typedef float tRobotJoints[ROBOT_nDOFs];
typedef float tRobotPose[ROBOT_nDOFs];

//declare in out vars
float xyzuvw_Out[ROBOT_nDOFs];
float xyzuvw_In[ROBOT_nDOFs];
float xyzuvw_Temp[ROBOT_nDOFs];
Matrix4x4 FK_Pose;

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

// ---- motion streaming (8.0) ----
// Streamed moves carry "q":1 and are only accepted while spline mode is active.
// They are acknowledged on receipt ({"msg":"ack"}) and send no completion reply;
// spline_end replies with robot_pos once all queued motion has finished.
bool suppressReplies = false;   // true while executing a queued move
bool stopRequested = false;     // soft stop received, reply pending
bool cmdExecuting = false;      // a command from cmdBuffer1 is being processed
bool motionFault = false;       // set whenever an alarm is printed
const int IN_QUEUE_SIZE = 4;
String inQueue[IN_QUEUE_SIZE];  // complete lines read from serial, not yet in cmdBuffers
bool inQueueQ[IN_QUEUE_SIZE];   // line is a queued (streamed) move
int inQueueCount = 0;
const unsigned long BLEND_WAIT_MS = 300;  // max wait for the next move before deciding on a blend
const unsigned long TELE_INTERVAL_MS = 50; // live joint-angle messages while streaming (20 per second)
unsigned long lastTeleMs = 0;
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

// ============================================================================
// EEPROM Configuration
// ============================================================================

// EEPROM Memory Map
#define EEPROM_MAGIC_ADDR 0            // 4 bytes - magic number to verify valid data
#define EEPROM_DEBUG_ADDR 4            // 1 byte = whether debug is active on boot
#define EEPROM_ROBOT_MODEL_ADDR 5      // 32 bytes - robot model string
#define EEPROM_ROBOT_VERSION_ADDR 37   // 32 bytes - robot version string
#define EEPROM_DRIVER_BOARD_ADDR 69    // 32 bytes - driver board string
#define EEPROM_SERIAL_NUMBER_ADDR 101  // 32 bytes - serial number string
#define EEPROM_ASSET_TAG_ADDR 133      // 32 bytes - asset tag string

#define EEPROM_MAGIC_NUMBER 0x41523401  // "AR4" + version 01


// Default values (used if EEPROM not initialized)
const char *DEFAULT_ROBOT_MODEL = "Unset";
const char *DEFAULT_ROBOT_VERSION = "Unset";
const char *DEFAULT_DRIVER_BOARD = "Unset";
const char *DEFAULT_SERIAL_NUMBER = "Unset";
const char *DEFAULT_ASSET_TAG = "Unset";
const bool DEFAULT_DEBUG = DEBUG;

String robot_model = DEFAULT_ROBOT_MODEL;
String robot_version = DEFAULT_ROBOT_VERSION;
String driver_board = DEFAULT_DRIVER_BOARD;
String serial_number = DEFAULT_SERIAL_NUMBER;
String asset_tag = DEFAULT_ASSET_TAG;

// Safe mode: true when EEPROM is uninitialized or model is unset.
// In safe mode, no pins are configured, no encoders constructed, no interrupts attached.
// Only hello and set_robot_id commands are accepted.
bool safeMode = false;


// ============================================================================
// EEPROM Functions
// ============================================================================

bool is_eeprom_initialized() {
  /*
     * Check if EEPROM has been initialized with valid data.
     * 
     * Returns:
     *   true if EEPROM contains valid robot configuration
     *   false if EEPROM is uninitialized or corrupted
     */
  uint32_t magic;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);
  return (magic == EEPROM_MAGIC_NUMBER);
}

void load_debug_from_eeprom() {
  if (is_eeprom_initialized()) {
    // Read from EEPROM
    // If Debug is defined in EEPROM then turn it on now
    bool debugBuf = false;
    EEPROM.get(EEPROM_DEBUG_ADDR, debugBuf);
    if (debugBuf) {
      DEBUG = true;
      DEBUG_PRINTLN("Loaded DEBUG=True from EEPROM - Setting DEBUG to True");
    }
  } else {
    Serial.println("EEPROM not initialized in load_debug");
  }
}

void save_debug_to_eeprom(bool value) {
  bool debugBuf = value;
  DEBUG_PRINT("Saving Value: ");
  DEBUG_PRINT(debugBuf);
  DEBUG_PRINTLN(" to EEPROM");
  EEPROM.put(EEPROM_DEBUG_ADDR, debugBuf);
  bool debugTest;
  EEPROM.get(EEPROM_DEBUG_ADDR, debugTest);

  if (debugTest != value) {
    Serial.println("Error saving Debug Persistence - Values don't match");
  }
}

void load_robot_id_from_eeprom() {
  /*
     * Load robot model and version from EEPROM.
     * If EEPROM not initialized, use default values.
     */
  if (is_eeprom_initialized()) {
    // Read from EEPROM
    char charBuffer[31];
    EEPROM.get(EEPROM_ROBOT_MODEL_ADDR, charBuffer);
    robot_model = charBuffer;
    DEBUG_PRINT("Debug - Loaded Robot Model from EEPROM: ");
    DEBUG_PRINTLN(robot_model);
    EEPROM.get(EEPROM_ROBOT_VERSION_ADDR, charBuffer);
    robot_version = charBuffer;
    DEBUG_PRINT("Debug - Loaded Robot Version from EEPROM: ");
    DEBUG_PRINTLN(robot_version);
    EEPROM.get(EEPROM_DRIVER_BOARD_ADDR, charBuffer);
    driver_board = charBuffer;
    DEBUG_PRINT("Debug - Loaded Driver Board from EEPROM: ");
    DEBUG_PRINTLN(driver_board);
    EEPROM.get(EEPROM_SERIAL_NUMBER_ADDR, charBuffer);
    serial_number = charBuffer;
    DEBUG_PRINT("Debug - Loaded Serial Number from EEPROM: ");
    DEBUG_PRINTLN(serial_number);
    EEPROM.get(EEPROM_ASSET_TAG_ADDR, charBuffer);
    asset_tag = charBuffer;
    DEBUG_PRINT("Debug - Loaded Asset Tag from EEPROM: ");
    DEBUG_PRINTLN(asset_tag);

  } else {
    Serial.println("EEPROM not initialized in load_robot_id");
    robot_model = DEFAULT_ROBOT_MODEL;
    robot_version = DEFAULT_ROBOT_VERSION;
    driver_board = DEFAULT_DRIVER_BOARD;
    serial_number = DEFAULT_SERIAL_NUMBER;
    asset_tag = DEFAULT_ASSET_TAG;
  }
}

void save_robot_id_to_eeprom(const String robot_model, const String robot_version, const String driver_board, const String serial_number, const String asset_tag) {
  /*
     * Save robot model and version to EEPROM.
     * 
     * Args:
     *   robot_model: Robot model string (max 31 chars)
     *   robot_version: Robot version string (max 31 chars)
     *   driver_board: Driver board string (max 31 chars)
     *   serial_number: Serial number string (max 31 chars)
     *   asset_tag: Asset Tag string (max 31 chars)
     */
  // Write magic number
  uint32_t magic = EEPROM_MAGIC_NUMBER;
  EEPROM.put(EEPROM_MAGIC_ADDR, magic);

  char charBuffer[32] = { 0 };

  // Write robot model
  if (robot_model != "NA") {
    robot_model.toCharArray(charBuffer, sizeof(charBuffer));
    EEPROM.put(EEPROM_ROBOT_MODEL_ADDR, charBuffer);
  }

  // Write robot version
  if (robot_version != "NA") {
    robot_version.toCharArray(charBuffer, sizeof(charBuffer));
    EEPROM.put(EEPROM_ROBOT_VERSION_ADDR, charBuffer);
  }

  // Write driver board
  if (driver_board != "NA") {
    driver_board.toCharArray(charBuffer, sizeof(charBuffer));
    EEPROM.put(EEPROM_DRIVER_BOARD_ADDR, charBuffer);
  }

  // Write Serial Number
  if (serial_number != "NA") {
    serial_number.toCharArray(charBuffer, sizeof(charBuffer));
    EEPROM.put(EEPROM_SERIAL_NUMBER_ADDR, charBuffer);
  }

  // Write Asset Tag
  if (asset_tag != "NA") {
    asset_tag.toCharArray(charBuffer, sizeof(charBuffer));
    EEPROM.put(EEPROM_ASSET_TAG_ADDR, charBuffer);
  }
}

void reboot() {
  DEBUG_PRINT("Rebooting Driver Board: ");
  DEBUG_PRINTLN(driver_board);
  if (driver_board.indexOf("Teensy") >= 0) {
    DEBUG_PRINTLN("Teensy 3.x / 4.x: ARM system reset");
    SCB_AIRCR = 0x05FA0004;
    while (true)
      ;
  } else {
    // Unknown type — fallback or safe no-op
    Serial.println("Unknown board type, no reboot performed.");
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Persistent Hardware / Query Functions
/////////////////////////////////////////////////////////////////////////////////////////////////////

void handle_hello_command() {
  /*
     * Responds to HELLO command with system information in JSON format.
     * Reads robot model/version from EEPROM (or uses defaults).
     * 
     * Command: {"cmd":"hello"}
     * Response: {"DriverModel":"...","FirmwareVersion":"...","RobotModel":"...","RobotVersion":"...","Configured":true}
     */

  bool configured = is_eeprom_initialized() && robot_model != "Unset" && robot_model.length() > 0;

  String response = "{";
  response += "\"DriverModel\":\"" + String(driver_board) + "\",";
  response += "\"FirmwareVersion\":\"" + String(FIRMWARE_VERSION) + "\",";
  response += "\"RobotModel\":\"" + String(robot_model) + "\",";
  response += "\"RobotVersion\":\"" + String(robot_version) + "\",";
  response += "\"SerialNumber\":\"" + String(serial_number) + "\",";
  response += "\"AssetTag\":\"" + String(asset_tag) + "\",";
  response += "\"Configured\":" + String(configured ? "true" : "false");
  response += "}";

  Serial.println(response);
}


void handle_set_robot_id_command(String robot_model, String robot_version, String driver_board, String serial_number, String asset_tag) {
  /*
     * Set robot model and version, save to EEPROM.
     * 
     * Response: Done\n (on success) or Error\n (on failure)
     */

  if (robot_model.length() == 0) {
    DEBUG_PRINTLN("No Robot Model Provided - Not Setting");
    robot_model = "NA";
  } else if (robot_model.length() > 31) {
    Serial.println("Error: Robot Model too long (max 31 chars)");
    return;
  }

  if (robot_version.length() == 0) {
    DEBUG_PRINTLN("No Robot Version Provded - Not Setting");
    robot_version = "NA";
    if (robot_version.length() > 31) {
      Serial.println("Error: Robot Version too long (max 31 chars)");
      return;
    }
  }

  if (driver_board.length() == 0) {
    DEBUG_PRINTLN("No Driver Board Provded - Not Setting");
    driver_board = "NA";
  } else if (driver_board.length() > 31) {
    Serial.println("Error: Driver Board too long (max 31 chars)");
    return;
  }

  if (serial_number.length() == 0) {
    DEBUG_PRINTLN("No Serial Number - Not Setting");
    serial_number = "NA";
  } else if (serial_number.length() > 31) {
    Serial.println("Error: Version too long (max 31 chars)");
    return;
  }

  if (asset_tag.length() == 0) {
    DEBUG_PRINTLN("No Asset Tag Provded - Not Setting");
    asset_tag = "NA";
  } else if (asset_tag.length() > 31) {
    Serial.println("Error: Asset Tag too long (max 31 chars)");
    return;
  }

  robot_model.trim();
  robot_version.trim();
  driver_board.trim();
  serial_number.trim();
  asset_tag.trim();

  DEBUG_PRINT("Debug - Setting Robot Model: ");
  DEBUG_PRINTLN(robot_model);
  DEBUG_PRINT("Debug - Setting Robot Version: ");
  DEBUG_PRINTLN(robot_version);
  DEBUG_PRINT("Debug - Setting Driver Board: ");
  DEBUG_PRINTLN(driver_board);
  DEBUG_PRINT("Debug - Setting Serial Number: ");
  DEBUG_PRINTLN(serial_number);
  DEBUG_PRINT("Debug - Setting Asset Tag: ");
  DEBUG_PRINTLN(asset_tag);

  // Capture current model before saving — reboot only if pin-map family changes
  String prev_model = ::robot_model;

  // Save to EEPROM
  save_robot_id_to_eeprom(robot_model, robot_version, driver_board, serial_number, asset_tag);

  // Reload globals from EEPROM (refreshes in-memory state and verifies write)
  String expected_model = robot_model;
  String expected_version = robot_version;
  load_robot_id_from_eeprom();

  bool verified = true;
  if (expected_model != "NA" && ::robot_model != expected_model) {
    DEBUG_PRINT("EEPROM verify failed for model: expected '");
    DEBUG_PRINT(expected_model);
    DEBUG_PRINT("' got '");
    DEBUG_PRINT(::robot_model);
    DEBUG_PRINTLN("'");
    verified = false;
  }
  if (expected_version != "NA" && ::robot_version != expected_version) {
    DEBUG_PRINT("EEPROM verify failed for version: expected '");
    DEBUG_PRINT(expected_version);
    DEBUG_PRINT("' got '");
    DEBUG_PRINT(::robot_version);
    DEBUG_PRINTLN("'");
    verified = false;
  }

  if (verified) {
    // Reboot only when the PIN MAP FAMILY changes. All AR3/AR4 variants share
    // one pinout; TST has its own. Version changes (MK4 -> MK5 etc.) never
    // require a reboot. Also reboot when leaving safe mode — the board needs
    // a clean boot to apply a pin map for the first time.
    bool prevIsTST = (prev_model == "TST");
    bool newIsTST = (::robot_model == "TST");
    if (prevIsTST != newIsTST || safeMode) {
      Serial.println("Done:Rebooting");
      // Flush + delay ensures the HMI receives the response before USB drops.
      Serial.flush();
      delay(250);
      SCB_AIRCR = 0x05FA0004;  // ARM software reset
      while (true) {}          // unreachable; wait for reset
    }
    Serial.println("Done");
  } else {
    Serial.println("Error: EEPROM verification failed");
  }
}

///// CSV VERIFICATION /////

bool splitCsvFields(const String &input, String fields[], int expectedCount) {
  int start = 0;
  int fieldIndex = 0;

  while (fieldIndex < expectedCount) {
    int commaIndex = input.indexOf(',', start);

    if (commaIndex == -1) {
      fields[fieldIndex++] = input.substring(start);
      break;
    } else {
      fields[fieldIndex++] = input.substring(start, commaIndex);
      start = commaIndex + 1;
    }
  }

  return (fieldIndex == expectedCount);
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
  // Rotation matrix -> rotation vector, computed in double precision.
  // Uses atan2 + a normalized axis so the result stays correct near 180 deg
  // (tool pointing straight down), where dividing by sin(angle) in float
  // used to return a wrong-length axis and a garbage orientation.
  const double r00 = pose[0], r10 = pose[1], r20 = pose[2];
  const double r01 = pose[4], r11 = pose[5], r21 = pose[6];
  const double r02 = pose[8], r12 = pose[9], r22 = pose[10];

  out[0] = pose[12];
  out[1] = pose[13];
  out[2] = pose[14];

  double v[3] = { r21 - r12, r02 - r20, r10 - r01 };  // 2*sin(angle)*axis
  double vnorm = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  double c = fmax(-1.0, fmin(1.0, ((r00 + r11 + r22) - 1.0) * 0.5));
  double angle = atan2(0.5 * vnorm, c);
  double axis[3] = { 0.0, 0.0, 0.0 };

  if (angle < 1.0E-9) {
    // no rotation
  } else if (c > -0.5) {
    // well-conditioned: axis from the skew-symmetric part
    axis[0] = v[0] / vnorm;
    axis[1] = v[1] / vnorm;
    axis[2] = v[2] / vnorm;
  } else {
    // near 180 deg: (R + R^T)/2 - c*I = (1 - c) * axis * axis^T,
    // so take its largest column as the axis direction
    const double S[3][3] = {
      { r00 - c, 0.5 * (r01 + r10), 0.5 * (r02 + r20) },
      { 0.5 * (r01 + r10), r11 - c, 0.5 * (r12 + r21) },
      { 0.5 * (r02 + r20), 0.5 * (r12 + r21), r22 - c }
    };
    int k = 0;
    if (S[1][1] > S[k][k]) k = 1;
    if (S[2][2] > S[k][k]) k = 2;
    double col[3] = { S[0][k], S[1][k], S[2][k] };
    double n = sqrt(col[0] * col[0] + col[1] * col[1] + col[2] * col[2]);
    axis[0] = col[0] / n;
    axis[1] = col[1] / n;
    axis[2] = col[2] / n;
    // pick the sign that agrees with the skew part (irrelevant at exactly 180)
    if (axis[0] * v[0] + axis[1] * v[1] + axis[2] * v[2] < 0.0) {
      axis[0] = -axis[0];
      axis[1] = -axis[1];
      axis[2] = -axis[2];
    }
  }

  out[3] = static_cast<T>(axis[0] * angle);
  out[4] = static_cast<T>(axis[1] * angle);
  out[5] = static_cast<T>(axis[2] * angle);
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
void SolveForwardKinematics() {
  float target_xyzuvw[6];
  float joints[ROBOT_nDOFs];

  for (int i = 0; i < ROBOT_nDOFs; i++) {
    joints[i] = JangleIn[i];
  }

  //forward_kinematics_robot_xyzuvw(joints, target_xyzuvw);
  forward_kinematics_robot(joints, FK_Pose);
  pose_2_xyzuvw(FK_Pose, target_xyzuvw);

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

// --- solver tuning (keep in step with ARrobots/src/kinematics.cpp) -----------------------
static const float IK_SING_SIN_EPS = 1e-4f;  // |sin J5| below this: wrist singular (J5 < 0.006 deg)
static const float IK_DUP_TOL_DEG = 1e-3f;   // solutions closer than this on every joint are one solution
// Interpolated motion (Move L / Move A / live jogs) steps ~1 mm at a time, so a joint that
// moves more than this between two consecutive IK solutions is a discontinuity (wrist flip
// through a singularity, J4/J6 wrapping at +/-180), not a move to perform: the move stops with
// a kinematic error instead of spinning a joint.
static const float IK_MAX_WAYPOINT_JUMP_DEG = 90.0f;

void updatejoints() {
  for (int i = 0; i < ROBOT_nDOFs; i++) {  // was "i > ROBOT_nDOFs": the loop never ran
    JangleIn[i] = JangleOut[i];
  }
}

void JointEstimate() {
  for (int i = 0; i < ROBOT_nDOFs; i++) {
    joints_estimate[i] = JangleIn[i];
  }
}

static inline float wrap180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

static inline float angdiff_deg(float a, float b) {
  float d = a - b;
  while (d > 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

static inline int sgn_with_db(float v, float db) {
  if (v > db) return +1;
  if (v < -db) return -1;
  return 0;  // in deadband
}

static inline bool ik_same_solution(const float a[ROBOT_nDOFs], const float b[ROBOT_nDOFs]) {
  for (int j = 0; j < ROBOT_nDOFs; j++) {
    if (fabsf(angdiff_deg(a[j], b[j])) > IK_DUP_TOL_DEG) return false;
  }
  return true;
}

// Among a, a + 360 and a - 360 pick the value inside the joint limits closest to ref.
// A wrist joint reported as -180 while the robot sits at +179.9 is the same angle; without
// this the robot wound the joint a full turn.
static inline float ik_unwrap_into_limits(float a, float ref, float lowerMag, float upper) {
  float best = a;
  float bestD = fabsf(a - ref);
  const float cands[2] = { a + 360.0f, a - 360.0f };
  for (int k = 0; k < 2; k++) {
    float c = cands[k];
    if (c < -lowerMag || c > upper) continue;
    float d = fabsf(c - ref);
    if (d < bestD) {
      best = c;
      bestD = d;
    }
  }
  return best;
}

// Solve IK for xyzuvw_In (mm, rotation vector in degrees) from the current joints JangleIn,
// honouring WristCon ("F" J5 >= 0, "N" J5 <= 0, "A" automatic). Result in JangleOut.
// maxJumpDeg > 0 rejects a solution that moves any joint farther than that from JangleIn
// (see IK_MAX_WAYPOINT_JUMP_DEG). KinematicError = 1 when there is no acceptable solution;
// JangleOut is then left unchanged.
void SolveInverseKinematicsGuarded(float maxJumpDeg) {

  float joints[ROBOT_nDOFs];
  float target[6];
  int NumberOfSol = 0;

  KinematicError = 0;

  JointEstimate();
  target[0] = xyzuvw_In[0];
  target[1] = xyzuvw_In[1];
  target[2] = xyzuvw_In[2];
  target[3] = xyzuvw_In[3] * M_PI / 180;
  target[4] = xyzuvw_In[4] * M_PI / 180;
  target[5] = xyzuvw_In[5] * M_PI / 180;

  // One call per wrist branch (J5 >= 0, J5 < 0): the raw solver only looks at the sign of
  // the J5 estimate, so this covers every solution. At the singularity both branches give
  // the same joints and the duplicate is dropped (the old code compared the first solution
  // against a zero-filled buffer, so a solution with J5 == 0 was always thrown away).
  const float branchEst[2] = { 30.0f, -30.0f };
  for (int b = 0; b < 2; b++) {
    joints_estimate[4] = branchEst[b];
    if (!inverse_kinematics_robot_xyzuvw<float>(target, joints, joints_estimate)) continue;

    for (int j = 0; j < ROBOT_nDOFs; j++) {
      joints[j] = wrap180(joints[j]);
      joints[j] = ik_unwrap_into_limits(joints[j], JangleIn[j], Robot_JointLimits_Lower[j], Robot_JointLimits_Upper[j]);
    }
    if (!robot_joints_valid(joints)) continue;

    bool dup = false;
    for (int s = 0; s < NumberOfSol && !dup; s++) {
      float stored[ROBOT_nDOFs];
      for (int j = 0; j < ROBOT_nDOFs; j++) stored[j] = SolutionMatrix[j][s];
      dup = ik_same_solution(stored, joints);
    }
    if (dup) continue;

    for (int j = 0; j < ROBOT_nDOFs; j++) SolutionMatrix[j][NumberOfSol] = joints[j];
    NumberOfSol++;
  }
  joints_estimate[4] = JangleIn[4];

  if (NumberOfSol == 0) {
    KinematicError = 1;
    return;
  }

  // ---- choose the branch -----------------------------------------------------------
  char wc = 0;
  if (WristCon.length() > 0) wc = toupper(WristCon.charAt(0));

  const float J5_DB = 0.5f;    // deg: J5 sign deadband
  const float SING_DB = 2.0f;  // deg: near-singular zone
  const float curWristSum = JangleIn[3] + JangleIn[5];

  int desiredW5 = sgn_with_db(JangleIn[4], J5_DB);
  if (desiredW5 == 0) desiredW5 = +1;
  if (wc == 'F') desiredW5 = +1;
  if (wc == 'N') desiredW5 = -1;

  int best = -1;
  float bestCost = 1e30f;

  for (int s = 0; s < NumberOfSol; s++) {
    float j5 = SolutionMatrix[4][s];
    float absJ5 = fabsf(j5);
    int solW5 = sgn_with_db(j5, J5_DB);

    // Joint travel from the current position (solutions are already unwrapped, so a plain
    // difference is the real distance the joint would move).
    float cost = 0.0f;
    for (int j = 0; j < ROBOT_nDOFs; j++) {
      cost += fabsf(SolutionMatrix[j][s] - joints_estimate[j]);
    }

    if (wc == 'F' || wc == 'N') {
      // Requested wrist: strict away from the singularity, strong bias close to it.
      if (absJ5 > SING_DB) {
        if (solW5 != 0 && solW5 != desiredW5) continue;
        if (solW5 == 0) cost += 50.0f;
      } else if (solW5 != 0 && solW5 != desiredW5) {
        cost += 200.0f;
      }
    } else if (wc == 'A') {
      // Automatic: allow a flip, mild bias toward the current branch.
      if (absJ5 > SING_DB && solW5 != 0 && solW5 != desiredW5) cost += 20.0f;
    } else {
      // Unknown / empty: keep the current branch.
      if (absJ5 > SING_DB) {
        if (solW5 != 0 && solW5 != desiredW5) continue;
      } else if (solW5 != 0 && solW5 != desiredW5) {
        cost += 200.0f;
      }
    }

    // Near the singularity only J4 + J6 is meaningful: keep it continuous.
    if (absJ5 <= SING_DB) {
      float solWristSum = SolutionMatrix[3][s] + SolutionMatrix[5][s];
      cost += 5.0f * fabsf(angdiff_deg(solWristSum, curWristSum));
    }

    if (cost < bestCost) {
      bestCost = cost;
      best = s;
    }
  }

  // The wrist filter rejected everything: fall back to the closest solution.
  if (best < 0) {
    bestCost = 1e30f;
    for (int s = 0; s < NumberOfSol; s++) {
      float cost = 0.0f;
      for (int j = 0; j < ROBOT_nDOFs; j++) {
        cost += fabsf(SolutionMatrix[j][s] - joints_estimate[j]);
      }
      if (cost < bestCost) {
        bestCost = cost;
        best = s;
      }
    }
  }

  if (maxJumpDeg > 0.0f) {
    for (int i = 0; i < ROBOT_nDOFs; i++) {
      if (fabsf(SolutionMatrix[i][best] - JangleIn[i]) > maxJumpDeg) {
        KinematicError = 1;
        return;
      }
    }
  }

  for (int i = 0; i < ROBOT_nDOFs; i++) {
    JangleOut[i] = SolutionMatrix[i][best];
  }
}

void SolveInverseKinematics() {
  SolveInverseKinematicsGuarded(0.0f);
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

      // |sin(J5)| from the two off-axis terms. The old (1 - cos^2) form loses every digit in
      // float near J5 = 0, so J4/J6 came out as noise there (the wrist "teleported") and the
      // exact rest pose (J5 = 0) found no solution at all.
      make_sqrt = sqrt(base[8] * base[8] + base[10] * base[10]);

      if (make_sqrt < IK_SING_SIN_EPS) {
        // Wrist singularity: the J4 and J6 axes line up and only J4 + J6 is defined.
        // Keep J4 at the estimate (current joint) and give J6 the rest.
        C = (base[9] >= 0.0) ? 0.0 : M_PI;
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

void sendRobotPosRaw() {

  updatePos();

  String sendPos = "{\"msg\":\"robot_pos\",\"j\":[";

  // J1–J6
  sendPos += String(JangleIn[0], 3) + ",";
  sendPos += String(JangleIn[1], 3) + ",";
  sendPos += String(JangleIn[2], 3) + ",";
  sendPos += String(JangleIn[3], 3) + ",";
  sendPos += String(JangleIn[4], 3) + ",";
  sendPos += String(JangleIn[5], 3) + "],";

  // Pose + external axes
  sendPos += "\"pose\":[";
  sendPos += String(xyzuvw_Out[0], 3) + ",";
  sendPos += String(xyzuvw_Out[1], 3) + ",";
  sendPos += String(xyzuvw_Out[2], 3) + ",";
  sendPos += String(xyzuvw_Out[3], 3) + ",";
  sendPos += String(xyzuvw_Out[4], 3) + ",";
  sendPos += String(xyzuvw_Out[5], 3) + ",";
  sendPos += String(J7_pos) + ",";
  sendPos += String(J8_pos) + ",";
  sendPos += String(J9_pos) + "],";

  // Status fields
  sendPos += "\"speed_violation\":" + speedViolation + ",";
  sendPos += "\"debug\":\"" + debug + "\",";
  sendPos += "\"flag\":\"" + flag + "\"}";

  delay(5);
  Serial.println(sendPos);

  speedViolation = "0";
  flag = "";
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

  SolveForwardKinematics();
}


// Completion reply. Suppressed while a queued move executes (the move was already
// acknowledged) and after a soft stop (finishStop sends the single reply).
void sendRobotPos() {
  if (suppressReplies || stopRequested) {
    updatePos();
    return;
  }
  sendRobotPosRaw();
}

// Lightweight acknowledgement for a queued move. No kinematics or delays, because it
// can be sent from inside the step loop.
void sendQueueAck() {
  int freeSlots = (cmdBuffer2 == "" ? 1 : 0) + (cmdBuffer3 == "" ? 1 : 0);
  Serial.print("{\"msg\":\"ack\",\"free\":");
  Serial.print(freeSlots);
  Serial.println("}");
}

// Live joint angles for the PC's virtual robot while streaming. Angles come straight from
// the step counters of the running move (no kinematics), so this is cheap enough to call
// from inside the step loops. Rate limited to TELE_INTERVAL_MS.
void maybeSendTelemetry(const int steps[9]) {
  if (!splineTrue) return;
  unsigned long now = millis();
  if (now - lastTeleMs < TELE_INTERVAL_MS) return;
  lastTeleMs = now;
  const int zero[6] = { J1zeroStep, J2zeroStep, J3zeroStep, J4zeroStep, J5zeroStep, J6zeroStep };
  const float stepDeg[6] = { J1StepDeg, J2StepDeg, J3StepDeg, J4StepDeg, J5StepDeg, J6StepDeg };
  Serial.print("{\"msg\":\"tele\",\"j\":[");
  for (int i = 0; i < 6; i++) {
    Serial.print((steps[i] - zero[i]) / stepDeg[i], 2);
    if (i < 5) Serial.print(",");
  }
  Serial.println("]}");
}

void sendRobotPosSpline() {

  updatePos();

  String sendPos = "{\"msg\":\"robot_pos\",\"j\":[";

  sendPos += String(JangleIn[0], 3) + ",";
  sendPos += String(JangleIn[1], 3) + ",";
  sendPos += String(JangleIn[2], 3) + ",";
  sendPos += String(JangleIn[3], 3) + ",";
  sendPos += String(JangleIn[4], 3) + ",";
  sendPos += String(JangleIn[5], 3) + "],";

  sendPos += "\"pose\":[";
  sendPos += String(xyzuvw_Out[0], 3) + ",";
  sendPos += String(xyzuvw_Out[1], 3) + ",";
  sendPos += String(xyzuvw_Out[2], 3) + ",";
  sendPos += String(xyzuvw_Out[3], 3) + ",";
  sendPos += String(xyzuvw_Out[4], 3) + ",";
  sendPos += String(xyzuvw_Out[5], 3) + ",";
  sendPos += String(J7_pos) + ",";
  sendPos += String(J8_pos) + ",";
  sendPos += String(J9_pos) + "],";

  sendPos += "\"speed_violation\":" + speedViolation + ",";
  sendPos += "\"debug\":\"" + debug + "\",";
  sendPos += "\"flag\":\"" + flag + "\"}";

  delay(5);
  Serial.println(sendPos);

  speedViolation = "0";
}

void correctRobotPos() {
  if (!CAP->hasEncoders) { return; }  // open-loop models: commanded steps are truth (Phase 3)

  J1StepM = J1encPos->read() / J1encMult;
  J2StepM = J2encPos->read() / J2encMult;
  J3StepM = J3encPos->read() / J3encMult;
  J4StepM = J4encPos->read() / J4encMult;
  J5StepM = J5encPos->read() / J5encMult;
  J6StepM = J6encPos->read() / J6encMult;

  JangleIn[0] = (J1StepM - J1zeroStep) / J1StepDeg;
  JangleIn[1] = (J2StepM - J2zeroStep) / J2StepDeg;
  JangleIn[2] = (J3StepM - J3zeroStep) / J3StepDeg;
  JangleIn[3] = (J4StepM - J4zeroStep) / J4StepDeg;
  JangleIn[4] = (J5StepM - J5zeroStep) / J5StepDeg;
  JangleIn[5] = (J6StepM - J6zeroStep) / J6StepDeg;


  SolveForwardKinematics();
  sendRobotPos();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SD CARD
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


static bool sd_ok = false;

const char *sdCodeMeaning(uint8_t code) {
  switch (code) {
    case 0x04: return "CMD17 read block failed (card error response)";
    case 0x0A: return "ACMD41 init timeout";
    case 0x02: return "CMD8 not accepted (not SD/unsupported)";
    default: return "see code/data";
  }
}

String egSD(const char *stage) {
  uint8_t code = SD.sdfs.sdErrorCode();
  uint8_t data = SD.sdfs.sdErrorData();

  String s = "EG: ";
  s += stage;
  s += " | code=0x";
  s += String(code, HEX);
  s += " data=0x";
  s += String(data, HEX);
  s += " | ";
  s += sdCodeMeaning(code);
  return s;
}

bool initSD() {
  if (sd_ok) return true;
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println(egSD("begin fail"));
    return false;
  }
  sd_ok = true;
  return true;
}

void writeSD(const String &filename, const String &info) {
  if (!initSD()) return;

  File f = SD.open(filename.c_str(), FILE_WRITE);
  if (!f) {
    Serial.println(egSD("open fail"));
    return;
  }
  f.println(info);
  f.close();
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

bool driveLimit(const int steps[], float SpeedVal) {

  const unsigned long DEBOUNCE_US = 3000;  // 3 ms
  unsigned long firstHighUs[numJoints] = { 0 };

  int calcStepGap = minSpeedDelay / (SpeedVal / 100);

  int calDir[numJoints] = { J1CalDir, J2CalDir, J3CalDir, J4CalDir, J5CalDir, J6CalDir, J7CalDir, J8CalDir, J9CalDir };
  int motDir[numJoints] = { J1MotDir, J2MotDir, J3MotDir, J4MotDir, J5MotDir, J6MotDir, J7MotDir, J8MotDir, J9MotDir };
  int dirPins[numJoints] = { J1dirPin, J2dirPin, J3dirPin, J4dirPin, J5dirPin, J6dirPin, J7dirPin, J8dirPin, J9dirPin };

  int curState[numJoints] = { 0 };
  int calPins[numJoints] = { J1calPin, J2calPin, J3calPin, J4calPin, J5calPin, J6calPin, J7calPin, J8calPin, J9calPin };
  int stepPins[numJoints] = { J1stepPin, J2stepPin, J3stepPin, J4stepPin, J5stepPin, J6stepPin, J7stepPin, J8stepPin, J9stepPin };

  int stepsDone[numJoints] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  int complete[numJoints] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  int failed[numJoints] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  int limitSeen[numJoints] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  for (int i = 0; i < numJoints; i++) {
    if ((calDir[i] == 1 && motDir[i] == 1) || (calDir[i] == 0 && motDir[i] == 0)) {
      digitalWrite(dirPins[i], HIGH);
    } else {
      digitalWrite(dirPins[i], LOW);
    }

    digitalWrite(stepPins[i], LOW);
  }

  for (int i = 0; i < numJoints; i++) {
    if (steps[i] == 0) {
      complete[i] = 1;
    }
  }

  int DriveLimInProc = 1;

  while (DriveLimInProc == 1 && estopActive == false) {

    for (int i = 0; i < numJoints; i++) {

      if (complete[i] == 1) {
        continue;
      }

      curState[i] = digitalRead(calPins[i]);

      if (curState[i] == limitSensor[i]) {

        if (firstHighUs[i] == 0) {
          firstHighUs[i] = micros();
          limitSeen[i] = 1;
        }

        if ((micros() - firstHighUs[i]) >= DEBOUNCE_US) {
          complete[i] = 1;
        }

      } else {
        firstHighUs[i] = 0;
        limitSeen[i] = 0;
      }

      if (stepsDone[i] < steps[i] && complete[i] == 0 && limitSeen[i] == 0) {
        digitalWrite(stepPins[i], HIGH);
        delayMicroseconds(5);
        digitalWrite(stepPins[i], LOW);

        stepsDone[i]++;

        delayMicroseconds(calcStepGap);

      } else if (stepsDone[i] >= steps[i] && complete[i] == 0) {
        failed[i] = 1;
        complete[i] = 1;
      }
    }

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

    delayMicroseconds(100);
  }

  for (int i = 0; i < numJoints; i++) {
    if (failed[i] == 1) {
      return false;
    }
  }

  return true;
}


void backOff(uint8_t J1req, uint8_t J2req, uint8_t J3req, uint8_t J4req, uint8_t J5req,
             uint8_t J6req, uint8_t J7req, uint8_t J8req, uint8_t J9req,
             float SpeedVal,
             int backoffSteps) {

  int calcStepGap = minSpeedDelay / (SpeedVal / 100);

  // SET DIRECTIONS
  digitalWrite(J1dirPin, (J1CalDir == J1MotDir) ? LOW : HIGH);
  digitalWrite(J2dirPin, (J2CalDir == J2MotDir) ? LOW : HIGH);
  digitalWrite(J3dirPin, (J3CalDir == J3MotDir) ? LOW : HIGH);
  digitalWrite(J4dirPin, (J4CalDir == J4MotDir) ? LOW : HIGH);
  digitalWrite(J5dirPin, (J5CalDir == J5MotDir) ? LOW : HIGH);
  digitalWrite(J6dirPin, (J6CalDir == J6MotDir) ? LOW : HIGH);
  digitalWrite(J7dirPin, (J7CalDir == J7MotDir) ? LOW : HIGH);
  digitalWrite(J8dirPin, (J8CalDir == J8MotDir) ? LOW : HIGH);
  digitalWrite(J9dirPin, (J9CalDir == J9MotDir) ? LOW : HIGH);

  // Make sure step pins start LOW
  digitalWrite(J1stepPin, LOW);
  digitalWrite(J2stepPin, LOW);
  digitalWrite(J3stepPin, LOW);
  digitalWrite(J4stepPin, LOW);
  digitalWrite(J5stepPin, LOW);
  digitalWrite(J6stepPin, LOW);
  digitalWrite(J7stepPin, LOW);
  digitalWrite(J8stepPin, LOW);
  digitalWrite(J9stepPin, LOW);

  auto pulseStep = [](uint8_t pin) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(5);
    digitalWrite(pin, LOW);
  };

  for (int step = 0; step < backoffSteps && estopActive == false; step++) {

    if (J1req == 1) pulseStep(J1stepPin);
    if (J2req == 1) pulseStep(J2stepPin);
    if (J3req == 1) pulseStep(J3stepPin);
    if (J4req == 1) pulseStep(J4stepPin);
    if (J5req == 1) pulseStep(J5stepPin);
    if (J6req == 1) pulseStep(J6stepPin);
    if (J7req == 1) pulseStep(J7stepPin);
    if (J8req == 1) pulseStep(J8stepPin);
    if (J9req == 1) pulseStep(J9stepPin);

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

  if (!CAP->hasEncoders) { return; }  // no encoders to sync (Phase 3)

  //set encoders to current position
  J1encPos->write(J1StepM * J1encMult);
  J2encPos->write(J2StepM * J2encMult);
  J3encPos->write(J3StepM * J3encMult);
  J4encPos->write(J4StepM * J4encMult);
  J5encPos->write(J5StepM * J5encMult);
  J6encPos->write(J6StepM * J6encMult);
  //delayMicroseconds(5);
}

void checkEncoders() {
  if (!CAP->hasEncoders) { return; }  // open-loop models: nothing to check (Phase 3)
  //read encoders
  J1EncSteps = J1encPos->read() / J1encMult;
  J2EncSteps = J2encPos->read() / J2encMult;
  J3EncSteps = J3encPos->read() / J3encMult;
  J4EncSteps = J4encPos->read() / J4encMult;
  J5EncSteps = J5encPos->read() / J5encMult;
  J6EncSteps = J6encPos->read() / J6encMult;

  if (abs((J1EncSteps - J1StepM)) >= encOffset) {
    if (J1LoopMode == 0) {
      J1collisionTrue = 1;
      J1StepM = J1encPos->read() / J1encMult;
    }
  }
  if (abs((J2EncSteps - J2StepM)) >= encOffset) {
    if (J2LoopMode == 0) {
      J2collisionTrue = 1;
      J2StepM = J2encPos->read() / J2encMult;
    }
  }
  if (abs((J3EncSteps - J3StepM)) >= encOffset) {
    if (J3LoopMode == 0) {
      J3collisionTrue = 1;
      J3StepM = J3encPos->read() / J3encMult;
    }
  }
  if (abs((J4EncSteps - J4StepM)) >= encOffset) {
    if (J4LoopMode == 0) {
      J4collisionTrue = 1;
      J4StepM = J4encPos->read() / J4encMult;
    }
  }
  if (abs((J5EncSteps - J5StepM)) >= encOffset) {
    if (J5LoopMode == 0) {
      J5collisionTrue = 1;
      J5StepM = J5encPos->read() / J5encMult;
    }
  }
  if (abs((J6EncSteps - J6StepM)) >= encOffset) {
    if (J6LoopMode == 0) {
      J6collisionTrue = 1;
      J6StepM = J6encPos->read() / J6encMult;
    }
  }

  TotalCollision = J1collisionTrue + J2collisionTrue + J3collisionTrue + J4collisionTrue + J5collisionTrue + J6collisionTrue;
  if (TotalCollision > 0) {
    flag = "EC" + String(J1collisionTrue) + String(J2collisionTrue) + String(J3collisionTrue) + String(J4collisionTrue) + String(J5collisionTrue) + String(J6collisionTrue);
  }
}





/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Spline Helpers
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ---- Orientation interpolation (8.1) ----------------------------------------------------
// Pose orientation slots [3..5] are a rotation vector (axis * angle, degrees). Blending those
// three numbers linearly is only safe for small changes: near 180 deg the same orientation can
// be written as +180 or -180 about the same axis, and a linear blend between the two passes
// through ~0 (tool pointing the opposite way). These helpers rotate along the shortest path
// between the two orientations instead (slerp). Double precision; output angle is <= 180 deg,
// which IK accepts like any other representation of the same orientation.

static void ori_rv2R(const double rvDeg[3], double R[9]) {
  double th = sqrt(rvDeg[0] * rvDeg[0] + rvDeg[1] * rvDeg[1] + rvDeg[2] * rvDeg[2]) * M_PI / 180.0;
  if (th < 1e-12) {
    R[0] = 1; R[1] = 0; R[2] = 0;
    R[3] = 0; R[4] = 1; R[5] = 0;
    R[6] = 0; R[7] = 0; R[8] = 1;
    return;
  }
  double thDeg = th * 180.0 / M_PI;
  double kx = rvDeg[0] / thDeg, ky = rvDeg[1] / thDeg, kz = rvDeg[2] / thDeg;
  double c = cos(th), sn = sin(th), v = 1.0 - c;
  R[0] = c + kx * kx * v;       R[1] = kx * ky * v - kz * sn;  R[2] = kx * kz * v + ky * sn;
  R[3] = ky * kx * v + kz * sn;  R[4] = c + ky * ky * v;       R[5] = ky * kz * v - kx * sn;
  R[6] = kz * kx * v - ky * sn;  R[7] = kz * ky * v + kx * sn;  R[8] = c + kz * kz * v;
}

static void ori_R2rv(const double R[9], double rvDeg[3]) {
  // quaternion (Shepperd), stable at 0 and 180 deg
  double w, x, y, z, sq;
  double tr = R[0] + R[4] + R[8];
  if (tr > 0) {
    sq = sqrt(tr + 1.0) * 2.0;
    w = 0.25 * sq; x = (R[7] - R[5]) / sq; y = (R[2] - R[6]) / sq; z = (R[3] - R[1]) / sq;
  } else if (R[0] > R[4] && R[0] > R[8]) {
    sq = sqrt(1.0 + R[0] - R[4] - R[8]) * 2.0;
    w = (R[7] - R[5]) / sq; x = 0.25 * sq; y = (R[1] + R[3]) / sq; z = (R[2] + R[6]) / sq;
  } else if (R[4] > R[8]) {
    sq = sqrt(1.0 + R[4] - R[0] - R[8]) * 2.0;
    w = (R[2] - R[6]) / sq; x = (R[1] + R[3]) / sq; y = 0.25 * sq; z = (R[5] + R[7]) / sq;
  } else {
    sq = sqrt(1.0 + R[8] - R[0] - R[4]) * 2.0;
    w = (R[3] - R[1]) / sq; x = (R[2] + R[6]) / sq; y = (R[5] + R[7]) / sq; z = 0.25 * sq;
  }
  if (w < 0) { w = -w; x = -x; y = -y; z = -z; }
  double n = sqrt(x * x + y * y + z * z);
  if (n < 1e-12) {
    rvDeg[0] = rvDeg[1] = rvDeg[2] = 0.0;
    return;
  }
  double angDeg = 2.0 * atan2(n, w) * 180.0 / M_PI;
  rvDeg[0] = x / n * angDeg;
  rvDeg[1] = y / n * angDeg;
  rvDeg[2] = z / n * angDeg;
}

// An orientation path is held in a plain double[12]: [0..8] start rotation matrix,
// [9..11] relative rotation start -> end (rotation vector, deg, <= 180). Plain arrays
// keep the Arduino IDE's auto-generated prototypes valid.
// Returns the size of the rotation in degrees.
float oriInterpInit(double path[12], const float startDeg[3], const float endDeg[3]) {
  double *Rs = &path[0];
  double *w = &path[9];
  double a[3] = { startDeg[0], startDeg[1], startDeg[2] };
  double b[3] = { endDeg[0], endDeg[1], endDeg[2] };
  double Re[9], Rrel[9];
  ori_rv2R(a, Rs);
  ori_rv2R(b, Re);
  for (int r = 0; r < 3; r++)  // Rrel = Rs^T * Re
    for (int c = 0; c < 3; c++)
      Rrel[r * 3 + c] = Rs[0 * 3 + r] * Re[0 * 3 + c] + Rs[1 * 3 + r] * Re[1 * 3 + c] + Rs[2 * 3 + r] * Re[2 * 3 + c];
  ori_R2rv(Rrel, w);
  return (float)sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
}

void oriInterpAt(const double path[12], float t, float outDeg[3]) {
  const double *Rs = &path[0];
  const double *w = &path[9];
  double wt[3] = { w[0] * t, w[1] * t, w[2] * t };
  double Rw[9], R[9], rv[3];
  ori_rv2R(wt, Rw);
  for (int r = 0; r < 3; r++)  // R = Rs * Rw
    for (int c = 0; c < 3; c++)
      R[r * 3 + c] = Rs[r * 3 + 0] * Rw[0 * 3 + c] + Rs[r * 3 + 1] * Rw[1 * 3 + c] + Rs[r * 3 + 2] * Rw[2 * 3 + c];
  ori_R2rv(R, rv);
  outDeg[0] = (float)rv[0];
  outDeg[1] = (float)rv[1];
  outDeg[2] = (float)rv[2];
}

// Convenience: orientation part of a 6-slot pose, interpolated a -> b at fraction t
void oriSlerpPose(const float a[6], const float b[6], float t, float out[6]) {
  double path[12];
  oriInterpInit(path, &a[3], &b[3]);
  oriInterpAt(path, t, &out[3]);
}

bool getNextSplinePose(float nextPose[6]) {
  if (cmdBuffer2 == "") return false;

  JsonDocument nextDoc;
  DeserializationError err = deserializeJson(nextDoc, cmdBuffer2);
  if (err) return false;

  // Only blend into another linear move. move_j uses a different pose order
  // ([X,Y,Z,Rz,Ry,Rx]) and is not a straight line, so a Cartesian arc into it is meaningless.
  const char *nextCmd = nextDoc["cmd"] | "";
  if (strcmp(nextCmd, "move_l") != 0) {
    return false;
  }

  JsonArray pose = nextDoc["pose"].as<JsonArray>();
  if (pose.isNull() || pose.size() < 6) return false;

  for (int i = 0; i < 6; i++) {
    nextPose[i] = pose[i] | 0.0f;
  }

  return true;
}

bool cropMoveLEndForRounding(
  const float startPose[6],
  const float targetPose[6],
  const float nextPose[6],
  float rounding,
  float croppedPose[6]) {
  float v1[6];
  float v2[6];
  float dist1 = 0.0f;
  float dist2 = 0.0f;

  for (int i = 0; i < 6; i++) {
    v1[i] = targetPose[i] - startPose[i];
    v2[i] = nextPose[i] - targetPose[i];
    dist1 += v1[i] * v1[i];
    dist2 += v2[i] * v2[i];
  }

  dist1 = sqrt(dist1);
  dist2 = sqrt(dist2);

  if (dist1 <= 0.0f || dist2 <= 0.0f || rounding <= 0.0f) {
    return false;
  }

  if (rounding > dist1 * 0.45f) rounding = dist1 * 0.45f;
  if (rounding > dist2 * 0.45f) rounding = dist2 * 0.45f;

  float cropPerc = 1.0f - (rounding / dist1);

  for (int i = 0; i < 6; i++) {
    croppedPose[i] = startPose[i] + (v1[i] * cropPerc);
  }

  return true;
}



bool buildSplineBlendArc(
  const float startPose[6],
  const float cornerPose[6],
  const float nextPose[6],
  float rounding,
  float croppedCurrentEnd[6],
  float blendArcMid[6],
  float blendArcEnd[3]) {
  // Rounding is a radius in mm, so segment lengths use XYZ only. Orientation is
  // cropped by the same fraction as position.
  float v1[6];
  float v2[6];
  float dist1 = 0.0f;
  float dist2 = 0.0f;

  for (int i = 0; i < 6; i++) {
    v1[i] = cornerPose[i] - startPose[i];
    v2[i] = nextPose[i] - cornerPose[i];
  }
  for (int i = 0; i < 3; i++) {
    dist1 += v1[i] * v1[i];
    dist2 += v2[i] * v2[i];
  }

  dist1 = sqrt(dist1);
  dist2 = sqrt(dist2);

  if (dist1 <= 0.001f || dist2 <= 0.001f || rounding <= 0.0f) {
    return false;
  }

  // Symmetric tangent distance so the arc is tangent to both lines.
  float r = rounding;
  if (r > dist1 * 0.45f) r = dist1 * 0.45f;
  if (r > dist2 * 0.45f) r = dist2 * 0.45f;

  float u1[3], u2[3], bis[3];
  for (int i = 0; i < 3; i++) {
    u1[i] = v1[i] / dist1;
    u2[i] = v2[i] / dist2;
    bis[i] = u2[i] - u1[i];  // points to the inside of the corner
  }
  float bisLen = sqrt(bis[0] * bis[0] + bis[1] * bis[1] + bis[2] * bis[2]);

  // Interior angle at the corner: cos(alpha) = dot(-u1, u2)
  float cosAlpha = -(u1[0] * u2[0] + u1[1] * u2[1] + u1[2] * u2[2]);
  if (cosAlpha > 1.0f) cosAlpha = 1.0f;
  if (cosAlpha < -1.0f) cosAlpha = -1.0f;
  float halfAlpha = acos(cosAlpha) * 0.5f;
  float cosHalf = cos(halfAlpha);
  float sinHalf = sin(halfAlpha);

  // Straight continuation (no corner) or near reversal: no arc.
  if (bisLen < 0.01f || cosHalf < 0.05f) {
    return false;
  }

  float curCropPerc = 1.0f - (r / dist1);
  float nextCropPerc = (r / dist2);
  float croppedNextStart[6];

  for (int i = 0; i < 3; i++) {
    croppedCurrentEnd[i] = startPose[i] + (v1[i] * curCropPerc);
    croppedNextStart[i] = cornerPose[i] + (v2[i] * nextCropPerc);
  }
  // orientation: rotate along the shortest path, never blend the raw numbers
  oriSlerpPose(startPose, cornerPose, curCropPerc, croppedCurrentEnd);
  oriSlerpPose(cornerPose, nextPose, nextCropPerc, croppedNextStart);

  // Midpoint of the tangent arc lies on the bisector, r*(1 - sin(a/2))/cos(a/2) from the corner.
  float midDist = r * (1.0f - sinHalf) / cosHalf;
  for (int i = 0; i < 3; i++) {
    blendArcMid[i] = cornerPose[i] + (bis[i] / bisLen) * midDist;
  }
  oriSlerpPose(croppedCurrentEnd, croppedNextStart, 0.5f, blendArcMid);

  blendArcEnd[0] = croppedNextStart[0];
  blendArcEnd[1] = croppedNextStart[1];
  blendArcEnd[2] = croppedNextStart[2];

  return true;
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
  if (ACCramp < 10) {
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

    // Streaming: keep reading queued moves / stop requests during joint moves
    if (splineTrue) {
      processSerial();
      maybeSendTelemetry(stepMonitors);
    }

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
      maybeSendTelemetry(stepMonitors);
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

void moveJ(JsonDocument &doc, bool response, bool precalc, bool simspeed) {

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

  JsonArray pose = doc["pose"];

  xyzuvw_In[0] = pose[0] | 0.0;
  xyzuvw_In[1] = pose[1] | 0.0;
  xyzuvw_In[2] = pose[2] | 0.0;
  xyzuvw_In[3] = pose[3] | 0.0;
  xyzuvw_In[4] = pose[4] | 0.0;
  xyzuvw_In[5] = pose[5] | 0.0;

  // Optional J7/J8/J9 in pose[6:8]
  // If omitted, leave them at current position
  J7_In = (pose.size() > 6) ? (pose[6] | J7_In) : J7_In;
  J8_In = (pose.size() > 7) ? (pose[7] | J8_In) : J8_In;
  J9_In = (pose.size() > 8) ? (pose[8] | J9_In) : J9_In;

  const char *spd = doc["spd_type"] | "percent";
  String SpeedType =
    (strcmp(spd, "seconds") == 0) ? "s" : (strcmp(spd, "mmps") == 0) ? "m"
                                                                     : "p";

  float SpeedVal = doc["spd"] | 0.0;
  float ACCspd = doc["acc"] | 0.0;
  float DCCspd = doc["dec"] | 0.0;
  float ACCramp = doc["ramp"] | 0.0;

  WristCon = doc["w"] | "A";

  String LoopMode = doc["loop_mode"] | "000000";
  LoopMode.trim();
  J1LoopMode = LoopMode.substring(0, 1).toInt();
  J2LoopMode = LoopMode.substring(1, 2).toInt();
  J3LoopMode = LoopMode.substring(2, 3).toInt();
  J4LoopMode = LoopMode.substring(3, 4).toInt();
  J5LoopMode = LoopMode.substring(4, 5).toInt();
  J6LoopMode = LoopMode.substring(5, 6).toInt();


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

      Alarm = "{\"msg\":\"error\",\"code\":\"R\"}";
      delay(5);
      Serial.println(Alarm);
      motionFault = true;
      Alarm = "0";

    } else {

      Alarm = "{\"msg\":\"error\",\"code\":\"L\",\"axes\":[";
      Alarm += String(J1axisFault) + ",";
      Alarm += String(J2axisFault) + ",";
      Alarm += String(J3axisFault) + ",";
      Alarm += String(J4axisFault) + ",";
      Alarm += String(J5axisFault) + ",";
      Alarm += String(J6axisFault) + ",";
      Alarm += String(J7axisFault) + ",";
      Alarm += String(J8axisFault) + ",";
      Alarm += String(J9axisFault) + "]}";

      delay(5);
      Serial.println(Alarm);
      motionFault = true;
      Alarm = "0";
    }

    inData = "";  // Clear recieved buffer
                  ////////MOVE COMPLETE///////////
  }
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//LIVE JOINT JOG
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void liveJointJog(JsonDocument &doc) {

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

  int axis = doc["axis"] | 0;
  int dirCmd = doc["dir"] | 0;

  const char *spd = doc["spd_type"] | "percent";
  String SpeedType =
    (strcmp(spd, "seconds") == 0) ? "s" : (strcmp(spd, "mmps") == 0) ? "m"
                                                                     : "p";

  float SpeedVal = doc["spd"] | 25;

  // HARD CODE ACC,DCC,RAMP FOR LIVE ACTION
  float ACCspd = 0;
  float DCCspd = 0;
  float ACCramp = 10;

  // NON HARD CODE OPTION
  // float ACCspd  = doc["acc"]  | 0.0;
  // float DCCspd  = doc["dec"]  | 0.0;
  // float ACCramp = doc["ramp"] | 10.0;

  WristCon = doc["w"] | "A";

  String LoopMode = doc["loop_mode"] | "000000";
  LoopMode.trim();
  J1LoopMode = LoopMode.substring(0, 1).toInt();
  J2LoopMode = LoopMode.substring(1, 2).toInt();
  J3LoopMode = LoopMode.substring(2, 3).toInt();
  J4LoopMode = LoopMode.substring(3, 4).toInt();
  J5LoopMode = LoopMode.substring(4, 5).toInt();
  J6LoopMode = LoopMode.substring(5, 6).toInt();

  delay(5);
  Serial.println("Jog Initiated");
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

  while (JogInPoc == true) {

    float jogStep = (dirCmd < 0) ? -0.25 : 0.25;

    if (axis == 1) { J1Angle = JangleIn[0] + jogStep; }
    if (axis == 2) { J2Angle = JangleIn[1] + jogStep; }
    if (axis == 3) { J3Angle = JangleIn[2] + jogStep; }
    if (axis == 4) { J4Angle = JangleIn[3] + jogStep; }
    if (axis == 5) { J5Angle = JangleIn[4] + jogStep; }
    if (axis == 6) { J6Angle = JangleIn[5] + jogStep; }
    if (axis == 7) { J7Angle = J7_pos + jogStep; }
    if (axis == 8) { J8Angle = J8_pos + jogStep; }
    if (axis == 9) { J9Angle = J9_pos + jogStep; }

    int J1futStepM = (J1Angle + J1axisLimNeg) * J1StepDeg;
    int J2futStepM = (J2Angle + J2axisLimNeg) * J2StepDeg;
    int J3futStepM = (J3Angle + J3axisLimNeg) * J3StepDeg;
    int J4futStepM = (J4Angle + J4axisLimNeg) * J4StepDeg;
    int J5futStepM = (J5Angle + J5axisLimNeg) * J5StepDeg;
    int J6futStepM = (J6Angle + J6axisLimNeg) * J6StepDeg;
    int J7futStepM = (J7Angle + J7axisLimNeg) * J7StepDeg;
    int J8futStepM = (J8Angle + J8axisLimNeg) * J8StepDeg;
    int J9futStepM = (J9Angle + J9axisLimNeg) * J9StepDeg;

    int J1stepDif = J1StepM - J1futStepM;
    int J2stepDif = J2StepM - J2futStepM;
    int J3stepDif = J3StepM - J3futStepM;
    int J4stepDif = J4StepM - J4futStepM;
    int J5stepDif = J5StepM - J5futStepM;
    int J6stepDif = J6StepM - J6futStepM;
    int J7stepDif = J7StepM - J7futStepM;
    int J8stepDif = J8StepM - J8futStepM;
    int J9stepDif = J9StepM - J9futStepM;

    J1dir = (J1stepDif <= 0) ? 1 : 0;
    J2dir = (J2stepDif <= 0) ? 1 : 0;
    J3dir = (J3stepDif <= 0) ? 1 : 0;
    J4dir = (J4stepDif <= 0) ? 1 : 0;
    J5dir = (J5stepDif <= 0) ? 1 : 0;
    J6dir = (J6stepDif <= 0) ? 1 : 0;
    J7dir = (J7stepDif <= 0) ? 1 : 0;
    J8dir = (J8stepDif <= 0) ? 1 : 0;
    J9dir = (J9stepDif <= 0) ? 1 : 0;

    if ((J1dir == 1 && (J1StepM + J1stepDif > J1StepLim)) || (J1dir == 0 && (J1StepM - J1stepDif < 0))) { J1axisFault = 1; }
    if ((J2dir == 1 && (J2StepM + J2stepDif > J2StepLim)) || (J2dir == 0 && (J2StepM - J2stepDif < 0))) { J2axisFault = 1; }
    if ((J3dir == 1 && (J3StepM + J3stepDif > J3StepLim)) || (J3dir == 0 && (J3StepM - J3stepDif < 0))) { J3axisFault = 1; }
    if ((J4dir == 1 && (J4StepM + J4stepDif > J4StepLim)) || (J4dir == 0 && (J4StepM - J4stepDif < 0))) { J4axisFault = 1; }
    if ((J5dir == 1 && (J5StepM + J5stepDif > J5StepLim)) || (J5dir == 0 && (J5StepM - J5stepDif < 0))) { J5axisFault = 1; }
    if ((J6dir == 1 && (J6StepM + J6stepDif > J6StepLim)) || (J6dir == 0 && (J6StepM - J6stepDif < 0))) { J6axisFault = 1; }
    if ((J7dir == 1 && (J7StepM + J7stepDif > J7StepLim)) || (J7dir == 0 && (J7StepM - J7stepDif < 0))) { J7axisFault = 1; }
    if ((J8dir == 1 && (J8StepM + J8stepDif > J8StepLim)) || (J8dir == 0 && (J8StepM - J8stepDif < 0))) { J8axisFault = 1; }
    if ((J9dir == 1 && (J9StepM + J9stepDif > J9StepLim)) || (J9dir == 0 && (J9StepM - J9stepDif < 0))) { J9axisFault = 1; }

    TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault + J7axisFault + J8axisFault + J9axisFault;

    if (TotalAxisFault == 0 && KinematicError == 0) {
      driveMotorsJ(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif),
                   abs(J7stepDif), abs(J8stepDif), abs(J9stepDif),
                   J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir,
                   SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
      updatePos();
    }

    char received = Serial.read();
    if (received == '\n') {
      break;
    }
  }

  TotalCollision = J1collisionTrue + J2collisionTrue + J3collisionTrue + J4collisionTrue + J5collisionTrue + J6collisionTrue;
  if (TotalCollision > 0) {
    flag = "EC" + String(J1collisionTrue) + String(J2collisionTrue) + String(J3collisionTrue) + String(J4collisionTrue) + String(J5collisionTrue) + String(J6collisionTrue);
  }

  if (TotalAxisFault == 0 && KinematicError == 0) {
    sendRobotPos();
  } else if (KinematicError == 1) {

    Alarm = "{\"msg\":\"error\",\"code\":\"R\"}";
    delay(5);
    Serial.println(Alarm);
      motionFault = true;
    Alarm = "0";

  } else {

    Alarm = "{\"msg\":\"error\",\"code\":\"L\",\"axes\":[";
    Alarm += String(J1axisFault) + ",";
    Alarm += String(J2axisFault) + ",";
    Alarm += String(J3axisFault) + ",";
    Alarm += String(J4axisFault) + ",";
    Alarm += String(J5axisFault) + ",";
    Alarm += String(J6axisFault) + ",";
    Alarm += String(J7axisFault) + ",";
    Alarm += String(J8axisFault) + ",";
    Alarm += String(J9axisFault) + "]}";

    delay(5);
    Serial.println(Alarm);
      motionFault = true;
    Alarm = "0";
  }
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//LIVE CARTESIAN JOG
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void liveCartJog(JsonDocument &doc) {
  delay(5);
  Serial.println("Jog Initiated");

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

  const char *axis = doc["axis"] | "";
  const char *dir = doc["dir"] | "pos";

  const char *spd = doc["spd_type"] | "percent";
  String SpeedType =
    (strcmp(spd, "seconds") == 0) ? "s" : (strcmp(spd, "mmps") == 0) ? "m"
                                                                     : "p";

  float SpeedVal = doc["spd"] | 25;
  float ACCspd = 0;
  float DCCspd = 0;
  float ACCramp = 10;

  WristCon = doc["w"] | "A";

  String LoopMode = doc["loop_mode"] | "000000";
  LoopMode.trim();
  J1LoopMode = LoopMode.substring(0, 1).toInt();
  J2LoopMode = LoopMode.substring(1, 2).toInt();
  J3LoopMode = LoopMode.substring(2, 3).toInt();
  J4LoopMode = LoopMode.substring(3, 4).toInt();
  J5LoopMode = LoopMode.substring(4, 5).toInt();
  J6LoopMode = LoopMode.substring(5, 6).toInt();

  inData = "";  // Clear received buffer

  // ------------------------------------------------------------
  // Persistent commanded world pose for this jog session: seeded once from the
  // current FK pose, then moved every loop. Rotations are applied about the WORLD
  // axes (Rz turns about the base Z axis, as the button says). The old code added
  // degrees to one component of the rotation vector, which is not a rotation about
  // any fixed axis and reverses direction once the vector passes 180 deg (the usual
  // tool-down orientation sits right there).
  // ------------------------------------------------------------
  Matrix4x4 jogCommandPose;
  for (int i = 0; i < 16; i++) {
    jogCommandPose[i] = FK_Pose[i];
  }

  // Fractional joint-step remainder accumulation
  float jogStepErr[6] = { 0, 0, 0, 0, 0, 0 };

  while (JogInPoc == true) {
    // Reset faults each cycle
    J1axisFault = 0;
    J2axisFault = 0;
    J3axisFault = 0;
    J4axisFault = 0;
    J5axisFault = 0;
    J6axisFault = 0;
    TotalAxisFault = 0;
    KinematicError = 0;

    float step = JogStepInc;
    if (strcmp(dir, "neg") == 0) {
      step = -step;
    }

    // ----------------------------------------------------------
    // Move the persistent commanded pose (never reset it from FK each cycle).
    // ----------------------------------------------------------
    if (strcmp(axis, "X") == 0) {
      jogCommandPose[12] += step;
    } else if (strcmp(axis, "Y") == 0) {
      jogCommandPose[13] += step;
    } else if (strcmp(axis, "Z") == 0) {
      jogCommandPose[14] += step;
    } else if (strcmp(axis, "Rx") == 0 || strcmp(axis, "Ry") == 0 || strcmp(axis, "Rz") == 0) {
      float delta_xyzuvw[6] = { 0, 0, 0, 0, 0, 0 };
      float stepRad = step * M_PI / 180.0f;
      if (strcmp(axis, "Rx") == 0) delta_xyzuvw[3] = stepRad;
      if (strcmp(axis, "Ry") == 0) delta_xyzuvw[4] = stepRad;
      if (strcmp(axis, "Rz") == 0) delta_xyzuvw[5] = stepRad;

      Matrix4x4 deltaPose;
      xyzuvw_2_pose(delta_xyzuvw, deltaPose);

      // world-frame rotation about the TCP: R = Rdelta * R, position unchanged
      Matrix4x4 composedPose;
      Matrix_Multiply(composedPose, deltaPose, jogCommandPose);
      for (int i = 0; i < 12; i++) {
        jogCommandPose[i] = composedPose[i];
      }
    }

    // Feed the commanded pose into IK
    float target_xyzuvw[6];
    pose_2_xyzuvw(jogCommandPose, target_xyzuvw);

    xyzuvw_In[0] = target_xyzuvw[0];
    xyzuvw_In[1] = target_xyzuvw[1];
    xyzuvw_In[2] = target_xyzuvw[2];
    xyzuvw_In[3] = target_xyzuvw[3] * 180.0f / M_PI;
    xyzuvw_In[4] = target_xyzuvw[4] * 180.0f / M_PI;
    xyzuvw_In[5] = target_xyzuvw[5] * 180.0f / M_PI;

    // guarded: a joint jumping 90+ deg for a 1 mm / 1 deg step is a wrist flip, not a jog
    SolveInverseKinematicsGuarded(IK_MAX_WAYPOINT_JUMP_DEG);

    // Stop integrating farther away if IK failed
    if (KinematicError == 1) {
      break;
    }

    // ------------------------------------------------------------
    // Convert floating joint targets to integer motor steps.
    // Preserve sub-step fractions so tiny moves are not discarded.
    // ------------------------------------------------------------
    float j1Desired = (JangleOut[0] + J1axisLimNeg) * J1StepDeg + jogStepErr[0];
    float j2Desired = (JangleOut[1] + J2axisLimNeg) * J2StepDeg + jogStepErr[1];
    float j3Desired = (JangleOut[2] + J3axisLimNeg) * J3StepDeg + jogStepErr[2];
    float j4Desired = (JangleOut[3] + J4axisLimNeg) * J4StepDeg + jogStepErr[3];
    float j5Desired = (JangleOut[4] + J5axisLimNeg) * J5StepDeg + jogStepErr[4];
    float j6Desired = (JangleOut[5] + J6axisLimNeg) * J6StepDeg + jogStepErr[5];

    int J1futStepM = lroundf(j1Desired);
    int J2futStepM = lroundf(j2Desired);
    int J3futStepM = lroundf(j3Desired);
    int J4futStepM = lroundf(j4Desired);
    int J5futStepM = lroundf(j5Desired);
    int J6futStepM = lroundf(j6Desired);

    jogStepErr[0] = j1Desired - J1futStepM;
    jogStepErr[1] = j2Desired - J2futStepM;
    jogStepErr[2] = j3Desired - J3futStepM;
    jogStepErr[3] = j4Desired - J4futStepM;
    jogStepErr[4] = j5Desired - J5futStepM;
    jogStepErr[5] = j6Desired - J6futStepM;

    // calc delta from current to destination
    int J1stepDif = J1StepM - J1futStepM;
    int J2stepDif = J2StepM - J2futStepM;
    int J3stepDif = J3StepM - J3futStepM;
    int J4stepDif = J4StepM - J4futStepM;
    int J5stepDif = J5StepM - J5futStepM;
    int J6stepDif = J6StepM - J6futStepM;
    int J7stepDif = 0;
    int J8stepDif = 0;
    int J9stepDif = 0;

    // determine motor directions
    J1dir = (J1stepDif <= 0) ? 1 : 0;
    J2dir = (J2stepDif <= 0) ? 1 : 0;
    J3dir = (J3stepDif <= 0) ? 1 : 0;
    J4dir = (J4stepDif <= 0) ? 1 : 0;
    J5dir = (J5stepDif <= 0) ? 1 : 0;
    J6dir = (J6stepDif <= 0) ? 1 : 0;
    J7dir = 0;
    J8dir = 0;
    J9dir = 0;

    // determine if requested position is within axis limits
    if ((J1dir == 1 && (J1StepM + J1stepDif > J1StepLim)) || (J1dir == 0 && (J1StepM - J1stepDif < 0))) {
      J1axisFault = 1;
    }
    if ((J2dir == 1 && (J2StepM + J2stepDif > J2StepLim)) || (J2dir == 0 && (J2StepM - J2stepDif < 0))) {
      J2axisFault = 1;
    }
    if ((J3dir == 1 && (J3StepM + J3stepDif > J3StepLim)) || (J3dir == 0 && (J3StepM - J3stepDif < 0))) {
      J3axisFault = 1;
    }
    if ((J4dir == 1 && (J4StepM + J4stepDif > J4StepLim)) || (J4dir == 0 && (J4StepM - J4stepDif < 0))) {
      J4axisFault = 1;
    }
    if ((J5dir == 1 && (J5StepM + J5stepDif > J5StepLim)) || (J5dir == 0 && (J5StepM - J5stepDif < 0))) {
      J5axisFault = 1;
    }
    if ((J6dir == 1 && (J6StepM + J6stepDif > J6StepLim)) || (J6dir == 0 && (J6StepM - J6stepDif < 0))) {
      J6axisFault = 1;
    }

    TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault;

    // send move command if no axis limit error
    if (TotalAxisFault == 0 && KinematicError == 0) {
      driveMotorsJ(
        abs(J1stepDif), abs(J2stepDif), abs(J3stepDif),
        abs(J4stepDif), abs(J5stepDif), abs(J6stepDif),
        abs(J7stepDif), abs(J8stepDif), abs(J9stepDif),
        J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir,
        SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
      updatePos();
    }

    // stop loop if any serial command is received
    if (Serial.available() > 0) {
      char received = Serial.read();
      inData += received;
      if (received == '\n') {
        break;
      }
    }
  }

  TotalCollision = J1collisionTrue + J2collisionTrue + J3collisionTrue + J4collisionTrue + J5collisionTrue + J6collisionTrue;

  if (TotalCollision > 0) {
    flag = "EC" + String(J1collisionTrue) + String(J2collisionTrue) + String(J3collisionTrue) + String(J4collisionTrue) + String(J5collisionTrue) + String(J6collisionTrue);
  }

  if (TotalAxisFault == 0 && KinematicError == 0) {
    sendRobotPos();
  } else if (KinematicError == 1) {

    Alarm = "{\"msg\":\"error\",\"code\":\"R\"}";
    delay(5);
    Serial.println(Alarm);
      motionFault = true;
    Alarm = "0";

  } else {

    Alarm = "{\"msg\":\"error\",\"code\":\"L\",\"axes\":[";
    Alarm += String(J1axisFault) + ",";
    Alarm += String(J2axisFault) + ",";
    Alarm += String(J3axisFault) + ",";
    Alarm += String(J4axisFault) + ",";
    Alarm += String(J5axisFault) + ",";
    Alarm += String(J6axisFault) + "]}";



    delay(5);
    Serial.println(Alarm);
      motionFault = true;
    Alarm = "0";
  }

  inData = "";  // Clear received buffer
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//LIVE TOOL JOG
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void liveToolJog(JsonDocument &doc) {
  delay(5);
  Serial.println("Jog Initiated");

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

  const char *axis = doc["axis"] | "";
  const char *dir = doc["dir"] | "pos";
  const char *spd = doc["spd_type"] | "percent";

  String SpeedType =
    (strcmp(spd, "seconds") == 0) ? "s" : (strcmp(spd, "mmps") == 0) ? "m"
                                                                     : "p";

  float SpeedVal = doc["spd"] | 25;
  float ACCspd = 0;
  float DCCspd = 0;
  float ACCramp = 10;

  WristCon = doc["w"] | "A";

  String LoopMode = doc["loop_mode"] | "111111";
  LoopMode.trim();
  J1LoopMode = LoopMode.substring(0, 1).toInt();
  J2LoopMode = LoopMode.substring(1, 2).toInt();
  J3LoopMode = LoopMode.substring(2, 3).toInt();
  J4LoopMode = LoopMode.substring(3, 4).toInt();
  J5LoopMode = LoopMode.substring(4, 5).toInt();
  J6LoopMode = LoopMode.substring(5).toInt();

  inData = "";  // Clear received buffer

  // ------------------------------------------------------------------
  // Persistent commanded pose for this jog session.
  // Seed it ONCE from the current actual FK pose, then increment it
  // every loop. Do NOT rebuild it from FK_Pose each cycle.
  // ------------------------------------------------------------------
  Matrix4x4 jogCommandPose;
  for (int i = 0; i < 16; i++) {
    jogCommandPose[i] = FK_Pose[i];
  }

  // Optional joint-step remainder accumulation. Keep it here too.
  float jogStepErr[6] = { 0, 0, 0, 0, 0, 0 };

  while (JogInPoc == true) {

    float step_lin = JogStepInc;
    float step_rot = JogStepInc * M_PI / 180.0f;

    float signed_lin = (strcmp(dir, "pos") == 0) ? step_lin : -step_lin;
    float signed_rot = (strcmp(dir, "pos") == 0) ? step_rot : -step_rot;

    // ----------------------------------------------------------------
    // Increment the persistent commanded pose in TOOL coordinates.
    // Use the current commanded pose axes, not the actual FK pose axes.
    // ----------------------------------------------------------------
    if (strcmp(axis, "X") == 0) {
      jogCommandPose[12] += jogCommandPose[0] * signed_lin;
      jogCommandPose[13] += jogCommandPose[1] * signed_lin;
      jogCommandPose[14] += jogCommandPose[2] * signed_lin;

    } else if (strcmp(axis, "Y") == 0) {
      jogCommandPose[12] += jogCommandPose[4] * signed_lin;
      jogCommandPose[13] += jogCommandPose[5] * signed_lin;
      jogCommandPose[14] += jogCommandPose[6] * signed_lin;

    } else if (strcmp(axis, "Z") == 0) {
      jogCommandPose[12] += jogCommandPose[8] * signed_lin;
      jogCommandPose[13] += jogCommandPose[9] * signed_lin;
      jogCommandPose[14] += jogCommandPose[10] * signed_lin;

    } else if (strcmp(axis, "Rx") == 0 || strcmp(axis, "Ry") == 0 || strcmp(axis, "Rz") == 0) {
      float delta_xyzuvw[6] = { 0, 0, 0, 0, 0, 0 };

      if (strcmp(axis, "Rx") == 0) delta_xyzuvw[3] = signed_rot;
      if (strcmp(axis, "Ry") == 0) delta_xyzuvw[4] = signed_rot;
      if (strcmp(axis, "Rz") == 0) delta_xyzuvw[5] = signed_rot;

      Matrix4x4 deltaPose;
      xyzuvw_2_pose(delta_xyzuvw, deltaPose);

      Matrix4x4 composedPose;
      Matrix_Multiply(composedPose, jogCommandPose, deltaPose);

      for (int i = 0; i < 16; i++) {
        jogCommandPose[i] = composedPose[i];
      }
    }

    // Convert commanded pose to xyzuvw target for IK
    float target_xyzuvw[6];
    pose_2_xyzuvw(jogCommandPose, target_xyzuvw);

    xyzuvw_In[0] = target_xyzuvw[0];
    xyzuvw_In[1] = target_xyzuvw[1];
    xyzuvw_In[2] = target_xyzuvw[2];
    xyzuvw_In[3] = target_xyzuvw[3] * 180.0f / M_PI;
    xyzuvw_In[4] = target_xyzuvw[4] * 180.0f / M_PI;
    xyzuvw_In[5] = target_xyzuvw[5] * 180.0f / M_PI;

    // Solve IK in jog mode (guarded: a joint jumping 90+ deg for a 1 mm step is a flip)
    SolveInverseKinematicsGuarded(IK_MAX_WAYPOINT_JUMP_DEG);

    // If IK failed, stop trying to integrate farther away
    if (KinematicError == 1) {
      break;
    }

    // ------------------------------------------------------------
    // Convert floating joint targets to integer motor steps.
    // Keep fractional remainder so tiny moves are not discarded.
    // ------------------------------------------------------------
    float j1Desired = (JangleOut[0] + J1axisLimNeg) * J1StepDeg + jogStepErr[0];
    float j2Desired = (JangleOut[1] + J2axisLimNeg) * J2StepDeg + jogStepErr[1];
    float j3Desired = (JangleOut[2] + J3axisLimNeg) * J3StepDeg + jogStepErr[2];
    float j4Desired = (JangleOut[3] + J4axisLimNeg) * J4StepDeg + jogStepErr[3];
    float j5Desired = (JangleOut[4] + J5axisLimNeg) * J5StepDeg + jogStepErr[4];
    float j6Desired = (JangleOut[5] + J6axisLimNeg) * J6StepDeg + jogStepErr[5];

    int J1futStepM = lroundf(j1Desired);
    int J2futStepM = lroundf(j2Desired);
    int J3futStepM = lroundf(j3Desired);
    int J4futStepM = lroundf(j4Desired);
    int J5futStepM = lroundf(j5Desired);
    int J6futStepM = lroundf(j6Desired);

    jogStepErr[0] = j1Desired - J1futStepM;
    jogStepErr[1] = j2Desired - J2futStepM;
    jogStepErr[2] = j3Desired - J3futStepM;
    jogStepErr[3] = j4Desired - J4futStepM;
    jogStepErr[4] = j5Desired - J5futStepM;
    jogStepErr[5] = j6Desired - J6futStepM;

    // Calculate delta from current actual step position to target step
    int J1stepDif = J1StepM - J1futStepM;
    int J2stepDif = J2StepM - J2futStepM;
    int J3stepDif = J3StepM - J3futStepM;
    int J4stepDif = J4StepM - J4futStepM;
    int J5stepDif = J5StepM - J5futStepM;
    int J6stepDif = J6StepM - J6futStepM;
    int J7stepDif = 0;
    int J8stepDif = 0;
    int J9stepDif = 0;

    // Determine motor directions
    J1dir = (J1stepDif <= 0) ? 1 : 0;
    J2dir = (J2stepDif <= 0) ? 1 : 0;
    J3dir = (J3stepDif <= 0) ? 1 : 0;
    J4dir = (J4stepDif <= 0) ? 1 : 0;
    J5dir = (J5stepDif <= 0) ? 1 : 0;
    J6dir = (J6stepDif <= 0) ? 1 : 0;
    J7dir = 0;
    J8dir = 0;
    J9dir = 0;

    J1axisFault = 0;
    J2axisFault = 0;
    J3axisFault = 0;
    J4axisFault = 0;
    J5axisFault = 0;
    J6axisFault = 0;

    // Determine if requested position is within axis limits
    if ((J1dir == 1 && (J1StepM + J1stepDif > J1StepLim)) || (J1dir == 0 && (J1StepM - J1stepDif < 0))) J1axisFault = 1;
    if ((J2dir == 1 && (J2StepM + J2stepDif > J2StepLim)) || (J2dir == 0 && (J2StepM - J2stepDif < 0))) J2axisFault = 1;
    if ((J3dir == 1 && (J3StepM + J3stepDif > J3StepLim)) || (J3dir == 0 && (J3StepM - J3stepDif < 0))) J3axisFault = 1;
    if ((J4dir == 1 && (J4StepM + J4stepDif > J4StepLim)) || (J4dir == 0 && (J4StepM - J4stepDif < 0))) J4axisFault = 1;
    if ((J5dir == 1 && (J5StepM + J5stepDif > J5StepLim)) || (J5dir == 0 && (J5StepM - J5stepDif < 0))) J5axisFault = 1;
    if ((J6dir == 1 && (J6StepM + J6stepDif > J6StepLim)) || (J6dir == 0 && (J6StepM - J6stepDif < 0))) J6axisFault = 1;

    TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault;

    if (TotalAxisFault == 0 && KinematicError == 0) {
      driveMotorsJ(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif),
                   abs(J7stepDif), abs(J8stepDif), abs(J9stepDif),
                   J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir,
                   SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
      updatePos();
    }

    char recieved = Serial.read();
    inData += recieved;
    if (recieved == '\n') {
      break;
    }
  }

  TotalCollision = J1collisionTrue + J2collisionTrue + J3collisionTrue + J4collisionTrue + J5collisionTrue + J6collisionTrue;
  if (TotalCollision > 0) {
    flag = "EC" + String(J1collisionTrue) + String(J2collisionTrue) + String(J3collisionTrue) + String(J4collisionTrue) + String(J5collisionTrue) + String(J6collisionTrue);
  }

  if (TotalAxisFault == 0 && KinematicError == 0) {
    sendRobotPos();
  } else if (KinematicError == 1) {

    Alarm = "{\"msg\":\"error\",\"code\":\"R\"}";
    delay(5);
    Serial.println(Alarm);
      motionFault = true;
    Alarm = "0";

  } else {

    Alarm = "{\"msg\":\"error\",\"code\":\"L\",\"axes\":[";
    Alarm += String(J1axisFault) + ",";
    Alarm += String(J2axisFault) + ",";
    Alarm += String(J3axisFault) + ",";
    Alarm += String(J4axisFault) + ",";
    Alarm += String(J5axisFault) + ",";
    Alarm += String(J6axisFault) + "]}";



    delay(5);
    Serial.println(Alarm);
      motionFault = true;
    Alarm = "0";
  }

  inData = "";  // Clear received buffer
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//STATIC TOOL JOG
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void jogToolMove(JsonDocument &doc) {
  delay(5);

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

  Alarm = "0";

  const char *axis = doc["axis"] | "";
  const char *dir = doc["dir"] | "pos";
  const char *spd = doc["spd_type"] | "percent";

  String SpeedType =
    (strcmp(spd, "seconds") == 0) ? "s" : (strcmp(spd, "mmps") == 0) ? "m"
                                                                     : "p";

  float step = doc["step"] | 1.0;
  float SpeedVal = doc["spd"] | 25;
  float ACCspd = doc["acc"] | 100;
  float DCCspd = doc["dec"] | 100;
  float ACCramp = doc["ramp"] | 100;

  WristCon = doc["w"] | "A";

  String LoopMode = doc["loop_mode"] | "111111";
  LoopMode.trim();
  J1LoopMode = LoopMode.substring(0, 1).toInt();
  J2LoopMode = LoopMode.substring(1, 2).toInt();
  J3LoopMode = LoopMode.substring(2, 3).toInt();
  J4LoopMode = LoopMode.substring(3, 4).toInt();
  J5LoopMode = LoopMode.substring(4, 5).toInt();
  J6LoopMode = LoopMode.substring(5).toInt();

  inData = "";  // Clear received buffer

  // ------------------------------------------------------------
  // Seed commanded pose ONCE from the current actual FK pose.
  // This is a one-shot move, not a live jog loop.
  // ------------------------------------------------------------
  Matrix4x4 cmdPose;
  for (int i = 0; i < 16; i++) {
    cmdPose[i] = FK_Pose[i];
  }

  float signed_lin = (strcmp(dir, "pos") == 0) ? step : -step;
  float signed_rot = ((strcmp(dir, "pos") == 0) ? step : -step) * M_PI / 180.0f;

  // ------------------------------------------------------------
  // Apply one TOOL-frame increment to the commanded pose
  // using the corrected live jog math approach.
  // ------------------------------------------------------------
  if (strcmp(axis, "X") == 0) {
    cmdPose[12] += cmdPose[0] * signed_lin;
    cmdPose[13] += cmdPose[1] * signed_lin;
    cmdPose[14] += cmdPose[2] * signed_lin;

  } else if (strcmp(axis, "Y") == 0) {
    cmdPose[12] += cmdPose[4] * signed_lin;
    cmdPose[13] += cmdPose[5] * signed_lin;
    cmdPose[14] += cmdPose[6] * signed_lin;

  } else if (strcmp(axis, "Z") == 0) {
    cmdPose[12] += cmdPose[8] * signed_lin;
    cmdPose[13] += cmdPose[9] * signed_lin;
    cmdPose[14] += cmdPose[10] * signed_lin;

  } else if (strcmp(axis, "Rx") == 0 || strcmp(axis, "Ry") == 0 || strcmp(axis, "Rz") == 0) {
    float delta_xyzuvw[6] = { 0, 0, 0, 0, 0, 0 };

    if (strcmp(axis, "Rx") == 0) delta_xyzuvw[3] = signed_rot;
    if (strcmp(axis, "Ry") == 0) delta_xyzuvw[4] = signed_rot;
    if (strcmp(axis, "Rz") == 0) delta_xyzuvw[5] = signed_rot;

    Matrix4x4 deltaPose;
    xyzuvw_2_pose(delta_xyzuvw, deltaPose);

    Matrix4x4 composedPose;
    Matrix_Multiply(composedPose, cmdPose, deltaPose);

    for (int i = 0; i < 16; i++) {
      cmdPose[i] = composedPose[i];
    }
  } else {
    Alarm = "{\"msg\":\"error\",\"code\":\"R\"}";
    delay(5);
    Serial.println(Alarm);
      motionFault = true;
    inData = "";
    return;
  }

  // ------------------------------------------------------------
  // Convert commanded pose into IK target
  // ------------------------------------------------------------
  float target_xyzuvw[6];
  pose_2_xyzuvw(cmdPose, target_xyzuvw);

  xyzuvw_In[0] = target_xyzuvw[0];
  xyzuvw_In[1] = target_xyzuvw[1];
  xyzuvw_In[2] = target_xyzuvw[2];
  xyzuvw_In[3] = target_xyzuvw[3] * 180.0f / M_PI;
  xyzuvw_In[4] = target_xyzuvw[4] * 180.0f / M_PI;
  xyzuvw_In[5] = target_xyzuvw[5] * 180.0f / M_PI;

  SolveInverseKinematics();

  // ------------------------------------------------------------
  // Convert IK joint result to motor steps
  // Single-shot move, so no live-loop remainder accumulator needed
  // ------------------------------------------------------------
  int J1futStepM = lroundf((JangleOut[0] + J1axisLimNeg) * J1StepDeg);
  int J2futStepM = lroundf((JangleOut[1] + J2axisLimNeg) * J2StepDeg);
  int J3futStepM = lroundf((JangleOut[2] + J3axisLimNeg) * J3StepDeg);
  int J4futStepM = lroundf((JangleOut[3] + J4axisLimNeg) * J4StepDeg);
  int J5futStepM = lroundf((JangleOut[4] + J5axisLimNeg) * J5StepDeg);
  int J6futStepM = lroundf((JangleOut[5] + J6axisLimNeg) * J6StepDeg);

  // calc delta from current to destination
  int J1stepDif = J1StepM - J1futStepM;
  int J2stepDif = J2StepM - J2futStepM;
  int J3stepDif = J3StepM - J3futStepM;
  int J4stepDif = J4StepM - J4futStepM;
  int J5stepDif = J5StepM - J5futStepM;
  int J6stepDif = J6StepM - J6futStepM;
  int J7stepDif = 0;
  int J8stepDif = 0;
  int J9stepDif = 0;

  // determine motor directions
  J1dir = (J1stepDif <= 0) ? 1 : 0;
  J2dir = (J2stepDif <= 0) ? 1 : 0;
  J3dir = (J3stepDif <= 0) ? 1 : 0;
  J4dir = (J4stepDif <= 0) ? 1 : 0;
  J5dir = (J5stepDif <= 0) ? 1 : 0;
  J6dir = (J6stepDif <= 0) ? 1 : 0;
  J7dir = 0;
  J8dir = 0;
  J9dir = 0;

  // determine if requested position is within axis limits
  if ((J1dir == 1 && (J1StepM + J1stepDif > J1StepLim)) || (J1dir == 0 && (J1StepM - J1stepDif < 0))) J1axisFault = 1;
  if ((J2dir == 1 && (J2StepM + J2stepDif > J2StepLim)) || (J2dir == 0 && (J2StepM - J2stepDif < 0))) J2axisFault = 1;
  if ((J3dir == 1 && (J3StepM + J3stepDif > J3StepLim)) || (J3dir == 0 && (J3StepM - J3stepDif < 0))) J3axisFault = 1;
  if ((J4dir == 1 && (J4StepM + J4stepDif > J4StepLim)) || (J4dir == 0 && (J4StepM - J4stepDif < 0))) J4axisFault = 1;
  if ((J5dir == 1 && (J5StepM + J5stepDif > J5StepLim)) || (J5dir == 0 && (J5StepM - J5stepDif < 0))) J5axisFault = 1;
  if ((J6dir == 1 && (J6StepM + J6stepDif > J6StepLim)) || (J6dir == 0 && (J6StepM - J6stepDif < 0))) J6axisFault = 1;

  TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault;

  if (TotalAxisFault == 0 && KinematicError == 0) {
    resetEncoders();
    driveMotorsJ(
      abs(J1stepDif), abs(J2stepDif), abs(J3stepDif),
      abs(J4stepDif), abs(J5stepDif), abs(J6stepDif),
      abs(J7stepDif), abs(J8stepDif), abs(J9stepDif),
      J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir,
      SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
    checkEncoders();
    sendRobotPos();
  } else if (KinematicError == 1) {

    Alarm = "{\"msg\":\"error\",\"code\":\"R\"}";
    delay(5);
    Serial.println(Alarm);
      motionFault = true;
    Alarm = "0";

  } else {

    Alarm = "{\"msg\":\"error\",\"code\":\"L\",\"axes\":[";
    Alarm += String(J1axisFault) + ",";
    Alarm += String(J2axisFault) + ",";
    Alarm += String(J3axisFault) + ",";
    Alarm += String(J4axisFault) + ",";
    Alarm += String(J5axisFault) + ",";
    Alarm += String(J6axisFault) + "]}";

    delay(5);
    Serial.println(Alarm);
      motionFault = true;
    Alarm = "0";
  }

  inData = "";  // Clear received buffer
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MOVE L
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void moveL(JsonDocument &doc) {
  int J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir;
  float curDelay;

  bool suppressAccel = false;
  bool suppressDecel = false;

  if (splineResume) {
    suppressAccel = true;
    splineResume = false;
  }

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

  float speedSP;
  float Xvect, Yvect, Zvect;

  // -----------------------------
  // Parse JSON inputs
  // JSON pose ordering is the robot's own slot order, the same as move_j
  // and the robot_pos reply: [X, Y, Z, Rz, Ry, Rx]  (GUI field labels)
  // -----------------------------
  JsonArray pose = doc["pose"].as<JsonArray>();
  if (pose.isNull() || pose.size() < 6) {
    Alarm = "{\"msg\":\"error\",\"code\":\"R\"}";
    delay(5);
    Serial.println(Alarm);
      motionFault = true;
    Alarm = "0";
    return;
  }

  xyzuvw_Temp[0] = pose[0] | 0.0f;  // X
  xyzuvw_Temp[1] = pose[1] | 0.0f;  // Y
  xyzuvw_Temp[2] = pose[2] | 0.0f;  // Z
  xyzuvw_Temp[3] = pose[3] | 0.0f;  // Rz (GUI)
  xyzuvw_Temp[4] = pose[4] | 0.0f;  // Ry
  xyzuvw_Temp[5] = pose[5] | 0.0f;  // Rx (GUI)

  JsonArray ext = doc["ext"].as<JsonArray>();
  J7_In = (!ext.isNull() && ext.size() > 0) ? (float)(ext[0] | 0.0f) : ((float)J7StepM / J7StepDeg - J7axisLimNeg);
  J8_In = (!ext.isNull() && ext.size() > 1) ? (float)(ext[1] | 0.0f) : ((float)J8StepM / J8StepDeg - J8axisLimNeg);
  J9_In = (!ext.isNull() && ext.size() > 2) ? (float)(ext[2] | 0.0f) : ((float)J9StepM / J9StepDeg - J9axisLimNeg);

  const char *spd = doc["spd_type"] | "percent";
  String SpeedType =
    (strcmp(spd, "seconds") == 0) ? "s" : (strcmp(spd, "mmps") == 0) ? "m"
                                                                     : "p";

  float SpeedVal = doc["spd"] | 25.0f;
  float ACCspd = doc["acc"] | 15.0f;
  float DCCspd = doc["dec"] | 15.0f;
  float ACCramp = doc["ramp"] | 80.0f;
  float Rounding = doc["rounding"] | 0.0f;

  WristCon = doc["w"] | "A";

  String LoopMode = doc["loop_mode"] | "111111";
  LoopMode.trim();
  J1LoopMode = LoopMode.substring(0, 1).toInt();
  J2LoopMode = LoopMode.substring(1, 2).toInt();
  J3LoopMode = LoopMode.substring(2, 3).toInt();
  J4LoopMode = LoopMode.substring(3, 4).toInt();
  J5LoopMode = LoopMode.substring(4, 5).toInt();
  J6LoopMode = LoopMode.substring(5).toInt();

  String DisWrist = doc["dis_wrist"] | "0";
  DisWrist.trim();

  // -----------------------------
  // Use current actual pose as start
  // -----------------------------
  updatePos();

  float originalTarget[6];
  float startPose[6];
  float nextPose[6];
  float croppedPose[6];
  float blendArcMid[6];
  float blendArcEnd[3];

  bool splineBlendActive = false;
  bool haveBlendArc = false;

  for (int i = 0; i < 6; i++) {
    originalTarget[i] = xyzuvw_Temp[i];
    startPose[i] = xyzuvw_Out[i];
  }

  // In spline mode, give the next command a moment to arrive before deciding on a blend.
  // The wait ends as soon as anything lands in cmdBuffer2 (next move or spline_end).
  if (Rounding > 0.0f && splineTrue && cmdBuffer2 == "") {
    unsigned long waitStart = millis();
    while (cmdBuffer2 == "" && splineTrue && !estopActive && (millis() - waitStart) < BLEND_WAIT_MS) {
      processSerial();
    }
  }

  if (Rounding > 0.0f && splineTrue && !estopActive) {
    if (getNextSplinePose(nextPose)) {
      if (buildSplineBlendArc(
            startPose,
            originalTarget,
            nextPose,
            Rounding,
            croppedPose,
            blendArcMid,
            blendArcEnd)) {
        splineBlendActive = true;
        haveBlendArc = true;
        suppressDecel = true;

        for (int i = 0; i < 6; i++) {
          xyzuvw_In[i] = croppedPose[i];
        }
      }
    }
  }

  if (!splineBlendActive) {
    xyzuvw_In[0] = xyzuvw_Temp[0];
    xyzuvw_In[1] = xyzuvw_Temp[1];
    xyzuvw_In[2] = xyzuvw_Temp[2];
    xyzuvw_In[3] = xyzuvw_Temp[3];
    xyzuvw_In[4] = xyzuvw_Temp[4];
    xyzuvw_In[5] = xyzuvw_Temp[5];
  }

  Xvect = xyzuvw_In[0] - xyzuvw_Out[0];
  Yvect = xyzuvw_In[1] - xyzuvw_Out[1];
  Zvect = xyzuvw_In[2] - xyzuvw_Out[2];

  float Xstart = xyzuvw_Out[0];
  float Ystart = xyzuvw_Out[1];
  float Zstart = xyzuvw_Out[2];

  // Orientation is interpolated as a rotation (see oriInterpInit), not per number
  double oriPath[12];
  float oriAngleDeg = oriInterpInit(oriPath, &xyzuvw_Out[3], &xyzuvw_In[3]);

  // Path length: XYZ distance combined with the true rotation angle (deg)
  float lineDist = pow(
    pow(Xvect, 2) + pow(Yvect, 2) + pow(Zvect, 2) + pow(oriAngleDeg, 2),
    0.5);

  if (lineDist <= 0.0f) {
    sendRobotPos();
    return;
  }

  // Whole number of waypoints so the last one lands exactly on the target
  float wayPts = ceilf(lineDist / linWayDistSP);
  if (wayPts < 1.0f) wayPts = 1.0f;
  float wayPerc = 1.0f / wayPts;

  // -----------------------------
  // Pre-calc move size for speed planning
  // -----------------------------
  SolveInverseKinematics();

  int J1futStepM = (JangleOut[0] + J1axisLimNeg) * J1StepDeg;
  int J2futStepM = (JangleOut[1] + J2axisLimNeg) * J2StepDeg;
  int J3futStepM = (JangleOut[2] + J3axisLimNeg) * J3StepDeg;
  int J4futStepM = (JangleOut[3] + J4axisLimNeg) * J4StepDeg;
  int J5futStepM = (JangleOut[4] + J5axisLimNeg) * J5StepDeg;
  int J6futStepM = (JangleOut[5] + J6axisLimNeg) * J6StepDeg;

  int J1stepDif = J1StepM - J1futStepM;
  int J2stepDif = J2StepM - J2futStepM;
  int J3stepDif = J3StepM - J3futStepM;
  int J4stepDif = J4StepM - J4futStepM;
  int J5stepDif = J5StepM - J5futStepM;
  int J6stepDif = J6StepM - J6futStepM;

  int HighStep = abs(J1stepDif);
  if (abs(J2stepDif) > HighStep) HighStep = abs(J2stepDif);
  if (abs(J3stepDif) > HighStep) HighStep = abs(J3stepDif);
  if (abs(J4stepDif) > HighStep) HighStep = abs(J4stepDif);
  if (abs(J5stepDif) > HighStep) HighStep = abs(J5stepDif);
  if (abs(J6stepDif) > HighStep) HighStep = abs(J6stepDif);

  if (HighStep < 1) HighStep = 1;

  float calcStepGap;

  float ACCStep = HighStep * (ACCspd / 100.0f);
  float NORStep = HighStep * ((100.0f - ACCspd - DCCspd) / 100.0f);
  float DCCStep = HighStep * (DCCspd / 100.0f);

  if (SpeedType == "s") {
    speedSP = (SpeedVal * 1000000.0f) * 1.2f;
  } else if (SpeedType == "m") {
    speedSP = ((lineDist / SpeedVal) * 1000000.0f) * 1.2f;
  }

  if (SpeedType == "s" || SpeedType == "m") {
    float zeroStepGap = speedSP / HighStep;
    float zeroACCstepInc = (ACCStep > 0.0f) ? (zeroStepGap * (100.0f / ACCramp)) / ACCStep : 0.0f;
    float zeroNORtime = NORStep * zeroStepGap;
    float zeroACCtime = ((ACCStep)*zeroStepGap) + ((ACCStep - 9.0f) * ((ACCStep) * (zeroACCstepInc / 2.0f)));
    float zeroDCCstepInc = (DCCStep > 0.0f) ? (zeroStepGap * (100.0f / ACCramp)) / DCCStep : 0.0f;
    float zeroDCCtime = ((DCCStep)*zeroStepGap) + ((DCCStep - 9.0f) * ((DCCStep) * (zeroDCCstepInc / 2.0f)));
    float zeroTOTtime = zeroACCtime + zeroNORtime + zeroDCCtime;
    float overclockPerc = speedSP / zeroTOTtime;
    calcStepGap = zeroStepGap * overclockPerc;
    if (calcStepGap <= minSpeedDelay) {
      calcStepGap = minSpeedDelay;
      speedViolation = "1";
    }
  } else {
    calcStepGap = minSpeedDelay / (SpeedVal / 100.0f);
  }

  float calcACCstepInc = (ACCStep > 0.0f) ? (calcStepGap * (100.0f / ACCramp)) / ACCStep : 0.0f;
  float calcDCCstepInc = (DCCStep > 0.0f) ? (calcStepGap * (100.0f / ACCramp)) / DCCStep : 0.0f;
  float calcACCstartDel = (calcACCstepInc * ACCStep) * 2.0f;
  float calcDCCendDel = (calcDCCstepInc * DCCStep) * 2.0f;

  float ACCwayPts = wayPts * (ACCspd / 100.0f);
  float DCCwayPts = wayPts * (DCCspd / 100.0f);

  float ACCwayInc = (ACCwayPts > 0.0f) ? (calcACCstartDel - calcStepGap) / ACCwayPts : 0.0f;
  float DCCwayInc = (DCCwayPts > 0.0f) ? (calcDCCendDel - calcStepGap) / DCCwayPts : 0.0f;

  if (suppressAccel) {
    curDelay = calcStepGap;
  } else {
    curDelay = calcACCstartDel;
  }

  int J7futStepM = (J7_In + J7axisLimNeg) * J7StepDeg;
  int J7stepDifFull = J7StepM - J7futStepM;

  int J8futStepM = (J8_In + J8axisLimNeg) * J8StepDeg;
  int J8stepDifFull = J8StepM - J8futStepM;

  int J9futStepM = (J9_In + J9axisLimNeg) * J9StepDeg;
  int J9stepDifFull = J9StepM - J9futStepM;

  // Persistent remainder for all axes
  float stepErr[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  resetEncoders();

  // i = 1..wayPts so the final waypoint is exactly 100% (old loop overshot the target)
  for (int i = 1; i <= (int)wayPts; i++) {

    if (estopActive || motionFault) break;

    J1axisFault = J2axisFault = J3axisFault = J4axisFault = J5axisFault = J6axisFault = 0;
    J7axisFault = J8axisFault = J9axisFault = 0;
    TotalAxisFault = 0;
    KinematicError = 0;

    if (!suppressAccel && i <= ACCwayPts) {
      curDelay = curDelay - ACCwayInc;
    } else if (!suppressDecel && i >= (wayPts - DCCwayPts)) {
      curDelay = curDelay + DCCwayInc;
    } else {
      curDelay = calcStepGap;
    }

    //curDelay = calcStepGap;

    float curWayPerc = wayPerc * i;

    xyzuvw_In[0] = Xstart + (Xvect * curWayPerc);
    xyzuvw_In[1] = Ystart + (Yvect * curWayPerc);
    xyzuvw_In[2] = Zstart + (Zvect * curWayPerc);
    oriInterpAt(oriPath, curWayPerc, &xyzuvw_In[3]);

    SolveInverseKinematicsGuarded(IK_MAX_WAYPOINT_JUMP_DEG);

    float j1Desired = (JangleOut[0] + J1axisLimNeg) * J1StepDeg + stepErr[0];
    float j2Desired = (JangleOut[1] + J2axisLimNeg) * J2StepDeg + stepErr[1];
    float j3Desired = (JangleOut[2] + J3axisLimNeg) * J3StepDeg + stepErr[2];
    float j4Desired = (JangleOut[3] + J4axisLimNeg) * J4StepDeg + stepErr[3];
    float j5Desired = (JangleOut[4] + J5axisLimNeg) * J5StepDeg + stepErr[4];
    float j6Desired = (JangleOut[5] + J6axisLimNeg) * J6StepDeg + stepErr[5];

    int J1futStepM_wp = lroundf(j1Desired);
    int J2futStepM_wp = lroundf(j2Desired);
    int J3futStepM_wp = lroundf(j3Desired);
    int J4futStepM_wp = lroundf(j4Desired);
    int J5futStepM_wp = lroundf(j5Desired);
    int J6futStepM_wp = lroundf(j6Desired);

    stepErr[0] = j1Desired - J1futStepM_wp;
    stepErr[1] = j2Desired - J2futStepM_wp;
    stepErr[2] = j3Desired - J3futStepM_wp;
    stepErr[3] = j4Desired - J4futStepM_wp;
    stepErr[4] = j5Desired - J5futStepM_wp;
    stepErr[5] = j6Desired - J6futStepM_wp;

    int J1stepDif_wp = J1StepM - J1futStepM_wp;
    int J2stepDif_wp = J2StepM - J2futStepM_wp;
    int J3stepDif_wp = J3StepM - J3futStepM_wp;
    int J4stepDif_wp = J4StepM - J4futStepM_wp;
    int J5stepDif_wp = J5StepM - J5futStepM_wp;
    int J6stepDif_wp = J6StepM - J6futStepM_wp;

    float j7Desired = (J7StepM - (J7stepDifFull * curWayPerc)) + stepErr[6];
    float j8Desired = (J8StepM - (J8stepDifFull * curWayPerc)) + stepErr[7];
    float j9Desired = (J9StepM - (J9stepDifFull * curWayPerc)) + stepErr[8];

    int J7futStepM_wp = lroundf(j7Desired);
    int J8futStepM_wp = lroundf(j8Desired);
    int J9futStepM_wp = lroundf(j9Desired);

    stepErr[6] = j7Desired - J7futStepM_wp;
    stepErr[7] = j8Desired - J8futStepM_wp;
    stepErr[8] = j9Desired - J9futStepM_wp;

    int J7stepDif_wp = J7StepM - J7futStepM_wp;
    int J8stepDif_wp = J8StepM - J8futStepM_wp;
    int J9stepDif_wp = J9StepM - J9futStepM_wp;

    J1dir = (J1stepDif_wp <= 0) ? 1 : 0;
    J2dir = (J2stepDif_wp <= 0) ? 1 : 0;
    J3dir = (J3stepDif_wp <= 0) ? 1 : 0;
    J4dir = (J4stepDif_wp <= 0) ? 1 : 0;
    J5dir = (J5stepDif_wp <= 0) ? 1 : 0;
    J6dir = (J6stepDif_wp <= 0) ? 1 : 0;
    J7dir = (J7stepDif_wp <= 0) ? 1 : 0;
    J8dir = (J8stepDif_wp <= 0) ? 1 : 0;
    J9dir = (J9stepDif_wp <= 0) ? 1 : 0;

    if ((J1dir == 1 && (J1StepM + J1stepDif_wp > J1StepLim)) || (J1dir == 0 && (J1StepM - J1stepDif_wp < 0))) J1axisFault = 1;
    if ((J2dir == 1 && (J2StepM + J2stepDif_wp > J2StepLim)) || (J2dir == 0 && (J2StepM - J2stepDif_wp < 0))) J2axisFault = 1;
    if ((J3dir == 1 && (J3StepM + J3stepDif_wp > J3StepLim)) || (J3dir == 0 && (J3StepM - J3stepDif_wp < 0))) J3axisFault = 1;
    if ((J4dir == 1 && (J4StepM + J4stepDif_wp > J4StepLim)) || (J4dir == 0 && (J4StepM - J4stepDif_wp < 0))) J4axisFault = 1;
    if ((J5dir == 1 && (J5StepM + J5stepDif_wp > J5StepLim)) || (J5dir == 0 && (J5StepM - J5stepDif_wp < 0))) J5axisFault = 1;
    if ((J6dir == 1 && (J6StepM + J6stepDif_wp > J6StepLim)) || (J6dir == 0 && (J6StepM - J6stepDif_wp < 0))) J6axisFault = 1;
    if ((J7dir == 1 && (J7StepM + J7stepDif_wp > J7StepLim)) || (J7dir == 0 && (J7StepM - J7stepDif_wp < 0))) J7axisFault = 1;
    if ((J8dir == 1 && (J8StepM + J8stepDif_wp > J8StepLim)) || (J8dir == 0 && (J8StepM - J8stepDif_wp < 0))) J8axisFault = 1;
    if ((J9dir == 1 && (J9StepM + J9stepDif_wp > J9StepLim)) || (J9dir == 0 && (J9StepM - J9stepDif_wp < 0))) J9axisFault = 1;

    TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault + J7axisFault + J8axisFault + J9axisFault;

    if (TotalAxisFault == 0 && KinematicError == 0) {
      driveMotorsL(
        abs(J1stepDif_wp), abs(J2stepDif_wp), abs(J3stepDif_wp),
        abs(J4stepDif_wp), abs(J5stepDif_wp), abs(J6stepDif_wp),
        abs(J7stepDif_wp), abs(J8stepDif_wp), abs(J9stepDif_wp),
        J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir,
        curDelay);
      updatePos();
      rndSpeed = curDelay;
    } else if (KinematicError == 1) {

      Alarm = "{\"msg\":\"error\",\"code\":\"R\"}";
      delay(5);
      Serial.println(Alarm);
      motionFault = true;
      Alarm = "0";
      break;

    } else {

      Alarm = "{\"msg\":\"error\",\"code\":\"L\",\"axes\":[";
      Alarm += String(J1axisFault) + ",";
      Alarm += String(J2axisFault) + ",";
      Alarm += String(J3axisFault) + ",";
      Alarm += String(J4axisFault) + ",";
      Alarm += String(J5axisFault) + ",";
      Alarm += String(J6axisFault) + ",";
      Alarm += String(J7axisFault) + ",";
      Alarm += String(J8axisFault) + ",";
      Alarm += String(J9axisFault) + "]}";

      delay(5);
      Serial.println(Alarm);
      motionFault = true;
      Alarm = "0";
      break;
    }
  }

  checkEncoders();

  if (haveBlendArc && !motionFault && !estopActive) {
    JsonDocument blendDoc;

    blendDoc["cmd"] = "move_a";

    JsonArray mid = blendDoc["mid"].to<JsonArray>();
    for (int i = 0; i < 6; i++) {
      mid.add(blendArcMid[i]);
    }

    JsonArray end = blendDoc["end"].to<JsonArray>();
    end.add(blendArcEnd[0]);
    end.add(blendArcEnd[1]);
    end.add(blendArcEnd[2]);

    blendDoc["spd"] = SpeedVal;
    blendDoc["spd_type"] = spd;
    blendDoc["acc"] = ACCspd;
    blendDoc["dec"] = DCCspd;
    blendDoc["ramp"] = ACCramp;
    blendDoc["w"] = WristCon;
    blendDoc["loop_mode"] = LoopMode;

    rndTrue = true;
    moveA(blendDoc);
    return;
  }

  // Exactly one reply. After a fault the alarm already was the reply.
  if (!motionFault) {
    sendRobotPos();
  }
  inData = "";
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MOVE A
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void moveA(JsonDocument &doc) {
  float curDelay;

  int J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir;
  int J1axisFault = 0;
  int J2axisFault = 0;
  int J3axisFault = 0;
  int J4axisFault = 0;
  int J5axisFault = 0;
  int J6axisFault = 0;
  int TotalAxisFault = 0;

  float speedSP;
  float Xvect, Yvect, Zvect;
  float calcStepGap;
  float theta;
  float axis[3];
  float startVect[3];
  float Rotation[3][3];
  float DestPt[3];
  float a, b, c, d, aa, bb, cc, dd, bc, ad, ac, ab, bd, cd;

  // -----------------------------
  // Parse JSON
  // mid = [X,Y,Z, then orientation in robot slot order: Rz,Ry,Rx GUI labels]
  // end = [X,Y,Z]
  // -----------------------------
  JsonArray mid = doc["mid"].as<JsonArray>();
  JsonArray end = doc["end"].as<JsonArray>();

  if (mid.isNull() || mid.size() < 6 || end.isNull() || end.size() < 3) {
    Alarm = "{\"msg\":\"error\",\"code\":\"R\"}";
    delay(5);
    Serial.println(Alarm);
      motionFault = true;
    Alarm = "0";
    return;
  }

  float xMid = mid[0] | 0.0f;
  float yMid = mid[1] | 0.0f;
  float zMid = mid[2] | 0.0f;
  float rx = mid[3] | 0.0f;
  float ry = mid[4] | 0.0f;
  float rz = mid[5] | 0.0f;

  float xEnd = end[0] | 0.0f;
  float yEnd = end[1] | 0.0f;
  float zEnd = end[2] | 0.0f;

  xyzuvw_In[6] = doc["tr"] | 0.0f;

  const char *spd = doc["spd_type"] | "percent";
  String SpeedType =
    (strcmp(spd, "seconds") == 0) ? "s" : (strcmp(spd, "mmps") == 0) ? "m"
                                                                     : "p";

  float SpeedVal = doc["spd"] | 25.0f;
  float ACCspd = doc["acc"] | 15.0f;
  float DCCspd = doc["dec"] | 15.0f;
  float ACCramp = doc["ramp"] | 80.0f;

  WristCon = doc["w"] | "A";

  String LoopMode = doc["loop_mode"] | "111111";
  LoopMode.trim();
  J1LoopMode = LoopMode.substring(0, 1).toInt();
  J2LoopMode = LoopMode.substring(1, 2).toInt();
  J3LoopMode = LoopMode.substring(2, 3).toInt();
  J4LoopMode = LoopMode.substring(3, 4).toInt();
  J5LoopMode = LoopMode.substring(4, 5).toInt();
  J6LoopMode = LoopMode.substring(5).toInt();

  // -----------------------------
  // Use current position as start
  // -----------------------------
  updatePos();

  float xBeg = xyzuvw_Out[0];
  float yBeg = xyzuvw_Out[1];
  float zBeg = xyzuvw_Out[2];
  // Orientation turns from the current orientation to mid's as a true rotation
  float oriMidDeg[3] = { rx, ry, rz };
  double oriPath[12];
  oriInterpInit(oriPath, &xyzuvw_Out[3], oriMidDeg);

  // -----------------------------
  // Arc geometry from 3 points
  // -----------------------------
  Xvect = xEnd - xMid;
  Yvect = yEnd - yMid;
  Zvect = zEnd - zMid;
  float aDist = sqrt((Xvect * Xvect) + (Yvect * Yvect) + (Zvect * Zvect));

  Xvect = xEnd - xBeg;
  Yvect = yEnd - yBeg;
  Zvect = zEnd - zBeg;
  float bDist = sqrt((Xvect * Xvect) + (Yvect * Yvect) + (Zvect * Zvect));

  Xvect = xMid - xBeg;
  Yvect = yMid - yBeg;
  Zvect = zMid - zBeg;
  float cDist = sqrt((Xvect * Xvect) + (Yvect * Yvect) + (Zvect * Zvect));

  float s = (aDist + bDist + cDist) / 2.0f;
  float areaTerm = s * (s - aDist) * (s - bDist) * (s - cDist);

  if (areaTerm <= 0.0f) {
    Alarm = "{\"msg\":\"error\",\"code\":\"R\"}";
    delay(5);
    Serial.println(Alarm);
      motionFault = true;
    Alarm = "0";
    return;
  }

  float Radius = (aDist * bDist * cDist) / (4.0f * sqrt(areaTerm));

  float BCx = pow(aDist, 2) * (pow(bDist, 2) + pow(cDist, 2) - pow(aDist, 2));
  float BCy = pow(bDist, 2) * (pow(cDist, 2) + pow(aDist, 2) - pow(bDist, 2));
  float BCz = pow(cDist, 2) * (pow(aDist, 2) + pow(bDist, 2) - pow(cDist, 2));

  float denom = BCx + BCy + BCz;
  if (fabs(denom) < 0.000001f) {
    Alarm = "{\"msg\":\"error\",\"code\":\"R\"}";
    delay(5);
    Serial.println(Alarm);
      motionFault = true;
    Alarm = "0";
    return;
  }

  float Px = ((BCx * xBeg) + (BCy * xMid) + (BCz * xEnd)) / denom;
  float Py = ((BCx * yBeg) + (BCy * yMid) + (BCz * yEnd)) / denom;
  float Pz = ((BCx * zBeg) + (BCy * zMid) + (BCz * zEnd)) / denom;

  startVect[0] = xBeg - Px;
  startVect[1] = yBeg - Py;
  startVect[2] = zBeg - Pz;

  float vect_Amag = sqrt(pow(xBeg - Px, 2) + pow(yBeg - Py, 2) + pow(zBeg - Pz, 2));
  float vect_Bmag = sqrt(pow(xMid - Px, 2) + pow(yMid - Py, 2) + pow(zMid - Pz, 2));
  float vect_Cmag = sqrt(pow(xEnd - Px, 2) + pow(yEnd - Py, 2) + pow(zEnd - Pz, 2));

  if (vect_Amag <= 0.0f || vect_Bmag <= 0.0f || vect_Cmag <= 0.0f) {
    Alarm = "{\"msg\":\"error\",\"code\":\"R\"}";
    delay(5);
    Serial.println(Alarm);
      motionFault = true;
    Alarm = "0";
    return;
  }

  float vect_Ax = (xBeg - Px) / vect_Amag;
  float vect_Ay = (yBeg - Py) / vect_Amag;
  float vect_Az = (zBeg - Pz) / vect_Amag;

  float vect_Bx = (xMid - Px) / vect_Bmag;
  float vect_By = (yMid - Py) / vect_Bmag;
  float vect_Bz = (zMid - Pz) / vect_Bmag;

  float vect_Cx = (xEnd - Px) / vect_Cmag;
  float vect_Cy = (yEnd - Py) / vect_Cmag;
  float vect_Cz = (zEnd - Pz) / vect_Cmag;

  float CrossX = (vect_Ay * vect_Bz) - (vect_Az * vect_By);
  float CrossY = (vect_Az * vect_Bx) - (vect_Ax * vect_Bz);
  float CrossZ = (vect_Ax * vect_By) - (vect_Ay * vect_Bx);

  float crossMag = sqrt((CrossX * CrossX) + (CrossY * CrossY) + (CrossZ * CrossZ));
  if (crossMag <= 0.000001f) {
    Alarm = "{\"msg\":\"error\",\"code\":\"R\"}";
    delay(5);
    Serial.println(Alarm);
      motionFault = true;
    Alarm = "0";
    return;
  }

  axis[0] = CrossX / crossMag;
  axis[1] = CrossY / crossMag;
  axis[2] = CrossZ / crossMag;

  float ABdot = (vect_Ax * vect_Bx) + (vect_Ay * vect_By) + (vect_Az * vect_Bz);
  float BCdot = (vect_Bx * vect_Cx) + (vect_By * vect_Cy) + (vect_Bz * vect_Cz);

  ABdot = constrain(ABdot, -1.0f, 1.0f);
  BCdot = constrain(BCdot, -1.0f, 1.0f);

  float ABradians = acos(ABdot);
  float BCradians = acos(BCdot);

  float ABdegrees = degrees(ABradians + BCradians);

  float anglepercent = ABdegrees / 360.0f;
  float circumference = 2.0f * 3.14159265359f * Radius;
  float lineDist = circumference * anglepercent;

  if (lineDist <= 0.0f) {
    sendRobotPos();
    return;
  }

  float wayPts = lineDist / linWayDistSP;
  if (wayPts < 1.0f) wayPts = 1.0f;

  float wayPerc = 1.0f / wayPts;
  float theta_Deg = ABdegrees / wayPts;

  // -----------------------------
  // Speed planning
  // -----------------------------
  int HighStep = (int)(lineDist / 0.05f);
  if (HighStep < 1) HighStep = 1;

  float ACCStep = HighStep * (ACCspd / 100.0f);
  float NORStep = HighStep * ((100.0f - ACCspd - DCCspd) / 100.0f);
  float DCCStep = HighStep * (DCCspd / 100.0f);

  if (SpeedType == "s") {
    speedSP = (SpeedVal * 1000000.0f) * 1.2f;
  } else if (SpeedType == "m") {
    speedSP = ((lineDist / SpeedVal) * 1000000.0f) * 1.2f;
  }

  if (SpeedType == "s" || SpeedType == "m") {
    float zeroStepGap = speedSP / HighStep;
    float zeroACCstepInc = (ACCStep > 0.0f) ? (zeroStepGap * (100.0f / ACCramp)) / ACCStep : 0.0f;
    float zeroACCtime = ((ACCStep)*zeroStepGap) + ((ACCStep - 9.0f) * ((ACCStep) * (zeroACCstepInc / 2.0f)));
    float zeroNORtime = NORStep * zeroStepGap;
    float zeroDCCstepInc = (DCCStep > 0.0f) ? (zeroStepGap * (100.0f / ACCramp)) / DCCStep : 0.0f;
    float zeroDCCtime = ((DCCStep)*zeroStepGap) + ((DCCStep - 9.0f) * ((DCCStep) * (zeroDCCstepInc / 2.0f)));
    float zeroTOTtime = zeroACCtime + zeroNORtime + zeroDCCtime;
    float overclockPerc = speedSP / zeroTOTtime;
    calcStepGap = zeroStepGap * overclockPerc;

    if (calcStepGap <= minSpeedDelay) {
      calcStepGap = minSpeedDelay;
      speedViolation = "1";
    }
  } else {
    calcStepGap = minSpeedDelay / (SpeedVal / 100.0f);
  }

  float calcACCstepInc = (ACCStep > 0.0f) ? (calcStepGap * (100.0f / ACCramp)) / ACCStep : 0.0f;
  float calcDCCstepInc = (DCCStep > 0.0f) ? (calcStepGap * (100.0f / ACCramp)) / DCCStep : 0.0f;
  float calcACCstartDel = (calcACCstepInc * ACCStep) * 2.0f;
  float calcDCCendDel = (calcDCCstepInc * DCCStep) * 2.0f;

  float ACCwayPts = wayPts * (ACCspd / 100.0f);
  float DCCwayPts = wayPts * (DCCspd / 100.0f);

  float ACCwayInc = (ACCwayPts > 0.0f) ? (calcACCstartDel - calcStepGap) / ACCwayPts : 0.0f;
  float DCCwayInc = (DCCwayPts > 0.0f) ? (calcDCCendDel - calcStepGap) / DCCwayPts : 0.0f;

  curDelay = rndTrue ? rndSpeed : calcACCstartDel;

  float cur_deg = theta_Deg;

  resetEncoders();

  for (int i = 0; i <= (int)wayPts - 1; i++) {
    if (estopActive || motionFault) break;
    theta = radians(cur_deg);

    a = cos(theta / 2.0f);
    b = -axis[0] * sin(theta / 2.0f);
    c = -axis[1] * sin(theta / 2.0f);
    d = -axis[2] * sin(theta / 2.0f);

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
    Rotation[0][1] = 2.0f * (bc + ad);
    Rotation[0][2] = 2.0f * (bd - ac);
    Rotation[1][0] = 2.0f * (bc - ad);
    Rotation[1][1] = aa + cc - bb - dd;
    Rotation[1][2] = 2.0f * (cd + ab);
    Rotation[2][0] = 2.0f * (bd + ac);
    Rotation[2][1] = 2.0f * (cd - ab);
    Rotation[2][2] = aa + dd - bb - cc;

    DestPt[0] = (Rotation[0][0] * startVect[0]) + (Rotation[0][1] * startVect[1]) + (Rotation[0][2] * startVect[2]);
    DestPt[1] = (Rotation[1][0] * startVect[0]) + (Rotation[1][1] * startVect[1]) + (Rotation[1][2] * startVect[2]);
    DestPt[2] = (Rotation[2][0] * startVect[0]) + (Rotation[2][1] * startVect[1]) + (Rotation[2][2] * startVect[2]);

    if (rndTrue == true) {
      curDelay = rndSpeed;
    } else if (i <= ACCwayPts) {
      curDelay = curDelay - ACCwayInc;
    } else if (i >= (wayPts - DCCwayPts)) {
      curDelay = curDelay + DCCwayInc;
    } else {
      curDelay = calcStepGap;
    }

    float curWayPerc = wayPerc * i;

    xyzuvw_In[0] = DestPt[0] + Px;
    xyzuvw_In[1] = DestPt[1] + Py;
    xyzuvw_In[2] = DestPt[2] + Pz;
    oriInterpAt(oriPath, curWayPerc, &xyzuvw_In[3]);

    SolveInverseKinematicsGuarded(IK_MAX_WAYPOINT_JUMP_DEG);

    int J1futStepM = (JangleOut[0] + J1axisLimNeg) * J1StepDeg;
    int J2futStepM = (JangleOut[1] + J2axisLimNeg) * J2StepDeg;
    int J3futStepM = (JangleOut[2] + J3axisLimNeg) * J3StepDeg;
    int J4futStepM = (JangleOut[3] + J4axisLimNeg) * J4StepDeg;
    int J5futStepM = (JangleOut[4] + J5axisLimNeg) * J5StepDeg;
    int J6futStepM = (JangleOut[5] + J6axisLimNeg) * J6StepDeg;

    int J1stepDif = J1StepM - J1futStepM;
    int J2stepDif = J2StepM - J2futStepM;
    int J3stepDif = J3StepM - J3futStepM;
    int J4stepDif = J4StepM - J4futStepM;
    int J5stepDif = J5StepM - J5futStepM;
    int J6stepDif = J6StepM - J6futStepM;
    int J7stepDif = 0;
    int J8stepDif = 0;
    int J9stepDif = 0;

    J1dir = (J1stepDif <= 0) ? 1 : 0;
    J2dir = (J2stepDif <= 0) ? 1 : 0;
    J3dir = (J3stepDif <= 0) ? 1 : 0;
    J4dir = (J4stepDif <= 0) ? 1 : 0;
    J5dir = (J5stepDif <= 0) ? 1 : 0;
    J6dir = (J6stepDif <= 0) ? 1 : 0;
    J7dir = J8dir = J9dir = 0;

    J1axisFault = J2axisFault = J3axisFault = J4axisFault = J5axisFault = J6axisFault = 0;

    if ((J1dir == 1 && (J1StepM + J1stepDif > J1StepLim)) || (J1dir == 0 && (J1StepM - J1stepDif < 0))) J1axisFault = 1;
    if ((J2dir == 1 && (J2StepM + J2stepDif > J2StepLim)) || (J2dir == 0 && (J2StepM - J2stepDif < 0))) J2axisFault = 1;
    if ((J3dir == 1 && (J3StepM + J3stepDif > J3StepLim)) || (J3dir == 0 && (J3StepM - J3stepDif < 0))) J3axisFault = 1;
    if ((J4dir == 1 && (J4StepM + J4stepDif > J4StepLim)) || (J4dir == 0 && (J4StepM - J4stepDif < 0))) J4axisFault = 1;
    if ((J5dir == 1 && (J5StepM + J5stepDif > J5StepLim)) || (J5dir == 0 && (J5StepM - J5stepDif < 0))) J5axisFault = 1;
    if ((J6dir == 1 && (J6StepM + J6stepDif > J6StepLim)) || (J6dir == 0 && (J6StepM - J6stepDif < 0))) J6axisFault = 1;

    TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault;

    if (TotalAxisFault == 0 && KinematicError == 0) {
      driveMotorsL(
        abs(J1stepDif), abs(J2stepDif), abs(J3stepDif),
        abs(J4stepDif), abs(J5stepDif), abs(J6stepDif),
        abs(J7stepDif), abs(J8stepDif), abs(J9stepDif),
        J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir,
        curDelay);
      updatePos();
    } else if (KinematicError == 1) {

      Alarm = "{\"msg\":\"error\",\"code\":\"R\"}";
      delay(5);
      Serial.println(Alarm);
      motionFault = true;
      Alarm = "0";
      break;

    } else {

      Alarm = "{\"msg\":\"error\",\"code\":\"L\",\"axes\":[";
      Alarm += String(J1axisFault) + ",";
      Alarm += String(J2axisFault) + ",";
      Alarm += String(J3axisFault) + ",";
      Alarm += String(J4axisFault) + ",";
      Alarm += String(J5axisFault) + ",";
      Alarm += String(J6axisFault) + "]}";

      delay(5);
      Serial.println(Alarm);
      motionFault = true;
      Alarm = "0";
      break;
    }

    cur_deg += theta_Deg;
  }

  checkEncoders();
  if (rndTrue == true && !motionFault && !estopActive) {
    splineResume = true;
  }
  rndTrue = false;
  if (!motionFault) {
    sendRobotPos();
  }
  inData = "";
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//CALLIBRAION ROUTINE
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void calibrateLimits(JsonDocument &doc) {
  JsonArray cal = doc["calibrate"];
  JsonArray off = doc["offsets"];

  int J1req = cal[0] | 0;
  int J2req = cal[1] | 0;
  int J3req = cal[2] | 0;
  int J4req = cal[3] | 0;
  int J5req = cal[4] | 0;
  int J6req = cal[5] | 0;
  int J7req = cal[6] | 0;
  int J8req = cal[7] | 0;
  int J9req = cal[8] | 0;

  float J1calOff = off[0] | 0.0;
  float J2calOff = off[1] | 0.0;
  float J3calOff = off[2] | 0.0;
  float J4calOff = off[3] | 0.0;
  float J5calOff = off[4] | 0.0;
  float J6calOff = off[5] | 0.0;
  float J7calOff = off[6] | 0.0;
  float J8calOff = off[7] | 0.0;
  float J9calOff = off[8] | 0.0;

  float SpeedIn;

  int J1Step = 0;
  int J2Step = 0;
  int J3Step = 0;
  int J4Step = 0;
  int J5Step = 0;
  int J6Step = 0;
  int J7Step = 0;
  int J8Step = 0;
  int J9Step = 0;

  int J1stepCen = 0;
  int J2stepCen = 0;
  int J3stepCen = 0;
  int J4stepCen = 0;
  int J5stepCen = 0;
  int J5step45 = 0;
  int J5step90 = 0;
  int J6stepCen = 0;
  int J7stepCen = 0;
  int J8stepCen = 0;
  int J9stepCen = 0;

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

  bool success;

  SpeedIn = 25;
  success = driveLimit(JStep, SpeedIn);

  if (!success) {
    Serial.println("EA");
    estopActive = true;
    return;
  }

  //Backoff
  backOff(J1req, J2req, J3req, J4req, J5req,
          J6req, J7req, J8req, J9req,
          5,     // speed
          700);  // steps

  //DRIVE TO LIMITS SLOW
  SpeedIn = 2;
  success = driveLimit(JStep, SpeedIn);

  if (!success) {
    Serial.println("EA");
    estopActive = true;
    return;
  }

  // set master steps and center step
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
      J5step45 = (((J5axisLimNeg) + J5calBaseOff + J5calOff) - 45) * J5StepDeg;
      J5step90 = (((J5axisLimNeg) + J5calBaseOff + J5calOff) - 90) * J5StepDeg;
    } else {
      J5StepM = (0 + J5calBaseOff + J5calOff) * J5StepDeg;
      J5stepCen = ((J5axisLimNeg)-J5calBaseOff - J5calOff) * J5StepDeg;
      J5step45 = (((J5axisLimNeg)-J5calBaseOff - J5calOff) + 45) * J5StepDeg;
      J5step90 = (((J5axisLimNeg) + J5calBaseOff + J5calOff) + 90) * J5StepDeg;
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

  // move to center
  J1dir = (J1CalDir) ? 0 : 1;
  J2dir = (J2CalDir) ? 0 : 1;
  J3dir = (J3CalDir) ? 0 : 1;
  J4dir = (J4CalDir) ? 0 : 1;
  J5dir = (J5CalDir) ? 0 : 1;
  J6dir = (J6CalDir) ? 0 : 1;
  J7dir = (J7CalDir) ? 0 : 1;
  J8dir = (J8CalDir) ? 0 : 1;
  J9dir = (J9CalDir) ? 0 : 1;

  float ACCspd = 10;
  float DCCspd = 10;
  String SpeedType = "p";
  float SpeedVal = 100;
  float ACCramp = 50;

  driveMotorsJ(
    J1stepCen, J2stepCen, J3stepCen, J4stepCen, J5step45, J6stepCen, J7stepCen, J8stepCen, J9stepCen,
    J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir,
    SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);

  sendRobotPos();
  inData = "";  // Clear received buffer
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//COMMUNICATIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int32_t modbusQuerry(int slaveID, int MBaddress, int MBval, int function) {
  int32_t result;

  node = ModbusMaster();
  node.begin(slaveID, Serial8);

  if (function == 1) {
    result = node.readCoils(MBaddress, 1);
    return (result == node.ku8MBSuccess) ? node.getResponseBuffer(0) : -1;

  } else if (function == 2) {
    result = node.readDiscreteInputs(MBaddress, 1);
    return (result == node.ku8MBSuccess) ? node.getResponseBuffer(0) : -1;

  } else if (function == 3) {
    result = node.readHoldingRegisters(MBaddress, MBval);
    return (result == node.ku8MBSuccess) ? node.getResponseBuffer(0) : -1;

  } else if (function == 4) {
    result = node.readInputRegisters(MBaddress, MBval);
    return (result == node.ku8MBSuccess) ? node.getResponseBuffer(0) : -1;

  } else if (function == 5) {
    result = node.writeSingleCoil(MBaddress, MBval);
    return (result == node.ku8MBSuccess) ? 1 : -1;

  } else if (function == 6) {
    result = node.writeSingleRegister(MBaddress, MBval);
    return (result == node.ku8MBSuccess) ? 1 : -1;
  }

  return -1;
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//READ DATA
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Drop everything queued for streaming. The command currently in cmdBuffer1 is left to
// finish (it has already been halted if estopActive is set).
void abortStream() {
  splineTrue = false;
  splineResume = false;
  rndTrue = false;
  moveSequence = "";
  cmdBuffer2 = "";
  cmdBuffer3 = "";
  inQueueCount = 0;
}

// Single reply for a soft stop
void finishStop() {
  stopRequested = false;
  sendRobotPosRaw();
}

// {"cmd":"stop"} is handled the moment it is read, even mid-move.
void handleStopRequest() {
  estopActive = true;  // halts the step loops and waypoint loops
  stopRequested = true;
  abortStream();
  if (!cmdExecuting) {
    finishStop();
  }
}

// Move complete lines from inQueue into the 3-slot command buffer
void feedCmdBuffers() {
  while (inQueueCount > 0 && cmdBuffer3 == "") {
    String line = inQueue[0];
    bool queued = inQueueQ[0];
    for (int i = 1; i < inQueueCount; i++) {
      inQueue[i - 1] = inQueue[i];
      inQueueQ[i - 1] = inQueueQ[i];
    }
    inQueueCount--;

    // A streamed move outside spline mode is a leftover from an aborted stream: reject it
    if (queued && !splineTrue) {
      Serial.println("{\"msg\":\"error\",\"code\":\"Q\"}");
      continue;
    }

    cmdBuffer3 = line;
    shiftCMDarray();

    if (queued) {
      sendQueueAck();
    }
  }
}

void processSerial() {
  // Read every complete line that is available (stop requests are handled immediately)
  while (Serial.available() > 0 && inQueueCount < IN_QUEUE_SIZE) {
    char recieved = Serial.read();
    if (recieved == '\r') continue;
    recData += recieved;
    if (recData.length() > 1024) {  // runaway line with no newline
      recData = "";
      continue;
    }
    if (recieved != '\n') continue;

    String line = recData;
    recData = "";
    line.trim();
    if (line.length() == 0) continue;

    JsonDocument peek;
    bool queued = false;
    if (!deserializeJson(peek, line)) {
      const char *pc = peek["cmd"] | "";
      if (strcmp(pc, "stop") == 0) {
        handleStopRequest();
        continue;
      }
      queued = (peek["q"] | 0) != 0;
    }

    inQueue[inQueueCount] = line;
    inQueueQ[inQueueCount] = queued;
    inQueueCount++;
  }

  feedCmdBuffers();
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
  sendRobotPosRaw();
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MAIN
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  // run once:
  Serial.begin(9600);
  load_debug_from_eeprom();
  load_robot_id_from_eeprom();

  // Determine if board is configured
  safeMode = !is_eeprom_initialized() || robot_model == "Unset" || robot_model.length() == 0;

  if (safeMode) {
    DEBUG_PRINTLN("SAFE MODE: Board not configured. Only hello and set_robot_id commands accepted.");
    // Clear command buffers and return — loop() will only process serial commands
    cmdBuffer1 = "";
    cmdBuffer2 = "";
    cmdBuffer3 = "";
    moveSequence = "";
    flag = "";
    rndTrue = false;
    splineTrue = false;
    splineEndReceived = false;
    return;
  }

  // Select and apply the pin map for this robot model (Phase 2)
  if (robot_model == "TST") {
    applyPinMap(&PINS_TST);
    CAP = &CAPS_TST;
  } else {
    applyPinMap(&PINS_AR4);  // AR3 / AR4 family share the AR4 board pinout
    CAP = &CAPS_AR4;
  }

  // Normal hardware initialization (only when configured)
  Serial8.begin(38400);  // Use Serial8 (pins 34 and 35)

  // Initialize Modbus communication
  node.begin(1, Serial8);

  for (int i = 0; i < 9; i++) {
    pinMode(activePins->stepPin[i], OUTPUT);
    pinMode(activePins->dirPin[i], OUTPUT);
    pinMode(activePins->calPin[i], activePins->calPinMode[i]);
    if (activePins->faultPin[i] >= 0) {
      pinMode(activePins->faultPin[i], INPUT_PULLUP);  // behavior added in fault phase
    }
  }

  pinMode(EstopPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EstopPin), EstopProg, LOW);
  if (Estop2Pin >= 0) {
    pinMode(Estop2Pin, INPUT_PULLUP);  // interrupt behavior added in fault phase
  }

  // Construct encoders only when this model has them (capability-gated, Phase 3)
  if (CAP->hasEncoders) {
    J1encPos = new Encoder(activePins->encA[0], activePins->encB[0]);
    J2encPos = new Encoder(activePins->encA[1], activePins->encB[1]);
    J3encPos = new Encoder(activePins->encA[2], activePins->encB[2]);
    J4encPos = new Encoder(activePins->encA[3], activePins->encB[3]);
    J5encPos = new Encoder(activePins->encA[4], activePins->encB[4]);
    J6encPos = new Encoder(activePins->encA[5], activePins->encB[5]);
  }

  for (int i = 0; i < 9; i++) {
    digitalWrite(activePins->stepPin[i], HIGH);
  }

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

  robot_set_AR();
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
    KinematicError = 0;
    debug = "";

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, inData);

    if (err) {
      inData = "";
      cmdBuffer1 = "";
      return;
    }

    const char *cmd = doc["cmd"];
    KinematicError = 0;
    debug = "";


    if (strcmp(cmd, "hello") == 0) {
      handle_hello_command();
      inData = "";
      cmdBuffer1 = "";
      return;
    }


    //-----SET ROBOT ID------------------------------------------------------
    //-----------------------------------------------------------------------
    if (cmd && strcmp(cmd, "set_robot_id") == 0) {
      const char* model   = doc["model"]   | "";
      const char* version = doc["version"] | "";
      const char* board   = doc["board"]   | "";
      const char* serial  = doc["serial"]  | "";
      const char* asset   = doc["asset"]   | "";

      handle_set_robot_id_command(
        String(model),
        String(version),
        String(board),
        String(serial),
        String(asset)
      );

      inData = "";
      cmdBuffer1 = "";
      return;
    }

    // Safe mode: reject all commands except hello and set_robot_id
    if (safeMode) {
      Serial.println("Error: Board in safe mode - configure robot model first");
      inData = "";
      cmdBuffer1 = "";
      return;
    }

    // Streaming bookkeeping for this command
    bool queuedCmd = (doc["q"] | 0) != 0;
    motionFault = false;
    cmdExecuting = true;
    suppressReplies = splineTrue && queuedCmd;

    // A manual (non-queued) command closes any open stream first, so spline mode can
    // never be left on by accident (processSerial() would then consume live-jog input).
    if (splineTrue && !queuedCmd && cmd && strcmp(cmd, "spline_end") != 0 && strcmp(cmd, "spline_start") != 0) {
      abortStream();
    }


    //-----SPLINE START------------------------------------------------------
    //-----------------------------------------------------------------------
    if (cmd && strcmp(cmd, "spline_start") == 0) {
      splineTrue = true;
      delay(5);
      Serial.println("SL");
      moveSequence = "";
      flag = "";
      rndTrue = false;
      splineResume = false;
      splineEndReceived = false;
    }


    //----- SPLINE STOP  ----------------------------------------------------
    //-----------------------------------------------------------------------
    // Executes only after every queued move ahead of it has finished, so this
    // reply means the robot has stopped at the last point.
    if (cmd && strcmp(cmd, "spline_end") == 0) {
      splineTrue = false;
      splineResume = false;
      rndTrue = false;
      splineEndReceived = false;
      delay(5);
      sendRobotPosRaw();
    }

    //-----COMMAND TEST LIMIT SWITCHES---------------------------------------------------
    //-----------------------------------------------------------------------
    if (cmd && strcmp(cmd, "test_limit_switches") == 0) {

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
    if (cmd && strcmp(cmd, "set_encoders") == 0) {
      if (!CAP->hasEncoders) {
        delay(5);
        Serial.println("Error: no encoders on this model");
      } else {
      J1encPos->write(1000);
      J2encPos->write(1000);
      J3encPos->write(1000);
      J4encPos->write(1000);
      J5encPos->write(1000);
      J6encPos->write(1000);
      delay(5);
      Serial.println("Enc_Set_1000");
      }
    }

    //-----COMMAND READ ENCODERS---------------------------------------------------
    //-----------------------------------------------------------------------
    if (cmd && strcmp(cmd, "read_encoders") == 0) {
      if (!CAP->hasEncoders) {
        delay(5);
        Serial.println("Error: no encoders on this model");
      } else {
      J1EncSteps = J1encPos->read();
      J2EncSteps = J2encPos->read();
      J3EncSteps = J3encPos->read();
      J4EncSteps = J4encPos->read();
      J5EncSteps = J5encPos->read();
      J6EncSteps = J6encPos->read();
      String Read = " J1 = " + String(J1EncSteps) + "   J2 = " + String(J2EncSteps) + "   J3 = " + String(J3EncSteps) + "   J4 = " + String(J4EncSteps) + "   J5 = " + String(J5EncSteps) + "   J6 = " + String(J6EncSteps);
      delay(5);
      Serial.println(Read);
      }
    }

    //-----COMMAND REQUEST POSITION---------------------------------------------------
    //-----------------------------------------------------------------------
    if (cmd && strcmp(cmd, "get_position") == 0) {
      //close serial so next command can be read in
      delay(5);
      if (Alarm == "0") {
        sendRobotPos();
      } else {
        Serial.println(Alarm);
      motionFault = true;
        Alarm = "0";
      }
    }



    //-----COMMAND HOME POSITION---------------------------------------------------
    //-----------------------------------------------------------------------

    //For debugging
    if (cmd && strcmp(cmd, "home_position") == 0) {

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
      Serial.println();
    }


    //-----COMMAND CORRECT POSITION---------------------------------------------------
    //-----------------------------------------------------------------------
    if (cmd && strcmp(cmd, "correct_position") == 0) {
      correctRobotPos();
    }

    //-----COMMAND UPDATE PARAMS (JSON)-----------------------------------------
    //-----------------------------------------------------------------------

    if (cmd && strcmp(cmd, "update_params") == 0) {

      JsonArray fields = doc["params"];

      int idx = 0;

      // Tool frame
      Robot_Kin_Tool[0] = fields[idx++].as<float>();                 // TFx
      Robot_Kin_Tool[1] = fields[idx++].as<float>();                 // TFy
      Robot_Kin_Tool[2] = fields[idx++].as<float>();                 // TFz
      Robot_Kin_Tool[3] = fields[idx++].as<float>() * M_PI / 180.0;  // TFrz
      Robot_Kin_Tool[4] = fields[idx++].as<float>() * M_PI / 180.0;  // TFry
      Robot_Kin_Tool[5] = fields[idx++].as<float>() * M_PI / 180.0;  // TFrx

      // Motor directions J1-J9
      J1MotDir = fields[idx++].as<int>();
      J2MotDir = fields[idx++].as<int>();
      J3MotDir = fields[idx++].as<int>();
      J4MotDir = fields[idx++].as<int>();
      J5MotDir = fields[idx++].as<int>();
      J6MotDir = fields[idx++].as<int>();
      J7MotDir = fields[idx++].as<int>();
      J8MotDir = fields[idx++].as<int>();
      J9MotDir = fields[idx++].as<int>();

      // Calibration directions J1-J9
      J1CalDir = fields[idx++].as<int>();
      J2CalDir = fields[idx++].as<int>();
      J3CalDir = fields[idx++].as<int>();
      J4CalDir = fields[idx++].as<int>();
      J5CalDir = fields[idx++].as<int>();
      J6CalDir = fields[idx++].as<int>();
      J7CalDir = fields[idx++].as<int>();
      J8CalDir = fields[idx++].as<int>();
      J9CalDir = fields[idx++].as<int>();

      // Calibration switch polarity J1-J9
      for (int i = 0; i < numJoints; i++) {
        String val = fields[idx++].as<String>();
        val.trim();
        val.toUpperCase();

        if (val == "HIGH") {
          limitSensor[i] = HIGH;
        } else if (val == "LOW") {
          limitSensor[i] = LOW;
        } else {
          Alarm = "{\"msg\":\"error\",\"code\":\"U\"}";
          delay(5);
          Serial.println(Alarm);
      motionFault = true;
          Alarm = "0";
          cmdBuffer1 = "";
          cmdExecuting = false;
          return;
        }
      }

      // Axis limits
      J1axisLimPos = fields[idx++].as<float>();
      J1axisLimNeg = fields[idx++].as<float>();
      J2axisLimPos = fields[idx++].as<float>();
      J2axisLimNeg = fields[idx++].as<float>();
      J3axisLimPos = fields[idx++].as<float>();
      J3axisLimNeg = fields[idx++].as<float>();
      J4axisLimPos = fields[idx++].as<float>();
      J4axisLimNeg = fields[idx++].as<float>();
      J5axisLimPos = fields[idx++].as<float>();
      J5axisLimNeg = fields[idx++].as<float>();
      J6axisLimPos = fields[idx++].as<float>();
      J6axisLimNeg = fields[idx++].as<float>();

      // Steps per degree
      J1StepDeg = fields[idx++].as<float>();
      J2StepDeg = fields[idx++].as<float>();
      J3StepDeg = fields[idx++].as<float>();
      J4StepDeg = fields[idx++].as<float>();
      J5StepDeg = fields[idx++].as<float>();
      J6StepDeg = fields[idx++].as<float>();

      // Encoder multipliers
      J1encMult = fields[idx++].as<float>();
      J2encMult = fields[idx++].as<float>();
      J3encMult = fields[idx++].as<float>();
      J4encMult = fields[idx++].as<float>();
      J5encMult = fields[idx++].as<float>();
      J6encMult = fields[idx++].as<float>();

      // DH theta
      DHparams[0][0] = fields[idx++].as<float>();
      DHparams[1][0] = fields[idx++].as<float>();
      DHparams[2][0] = fields[idx++].as<float>();
      DHparams[3][0] = fields[idx++].as<float>();
      DHparams[4][0] = fields[idx++].as<float>();
      DHparams[5][0] = fields[idx++].as<float>();

      // DH alpha
      DHparams[0][1] = fields[idx++].as<float>();
      DHparams[1][1] = fields[idx++].as<float>();
      DHparams[2][1] = fields[idx++].as<float>();
      DHparams[3][1] = fields[idx++].as<float>();
      DHparams[4][1] = fields[idx++].as<float>();
      DHparams[5][1] = fields[idx++].as<float>();

      // DH d
      DHparams[0][2] = fields[idx++].as<float>();
      DHparams[1][2] = fields[idx++].as<float>();
      DHparams[2][2] = fields[idx++].as<float>();
      DHparams[3][2] = fields[idx++].as<float>();
      DHparams[4][2] = fields[idx++].as<float>();
      DHparams[5][2] = fields[idx++].as<float>();

      // DH a
      DHparams[0][3] = fields[idx++].as<float>();
      DHparams[1][3] = fields[idx++].as<float>();
      DHparams[2][3] = fields[idx++].as<float>();
      DHparams[3][3] = fields[idx++].as<float>();
      DHparams[4][3] = fields[idx++].as<float>();
      DHparams[5][3] = fields[idx++].as<float>();

      // define total axis travel
      J1axisLim = J1axisLimPos + J1axisLimNeg;
      J2axisLim = J2axisLimPos + J2axisLimNeg;
      J3axisLim = J3axisLimPos + J3axisLimNeg;
      J4axisLim = J4axisLimPos + J4axisLimNeg;
      J5axisLim = J5axisLimPos + J5axisLimNeg;
      J6axisLim = J6axisLimPos + J6axisLimNeg;

      // steps full movement of each axis
      J1StepLim = J1axisLim * J1StepDeg;
      J2StepLim = J2axisLim * J2StepDeg;
      J3StepLim = J3axisLim * J3StepDeg;
      J4StepLim = J4axisLim * J4StepDeg;
      J5StepLim = J5axisLim * J5StepDeg;
      J6StepLim = J6axisLim * J6StepDeg;

      // step and axis zero
      J1zeroStep = J1axisLimNeg * J1StepDeg;
      J2zeroStep = J2axisLimNeg * J2StepDeg;
      J3zeroStep = J3axisLimNeg * J3StepDeg;
      J4zeroStep = J4axisLimNeg * J4StepDeg;
      J5zeroStep = J5axisLimNeg * J5StepDeg;
      J6zeroStep = J6axisLimNeg * J6StepDeg;

      robot_set_AR();

      Serial.println("PARAMETERS_UPDATED");

      inData = "";  // Clear received buffer
    }



    //-----COMMAND CALIBRATE EXTERNAL AXIS--------------------------------------
    //-----------------------------------------------------------------------

    if (cmd && strcmp(cmd, "config_ext_axis") == 0) {
      JsonArray j7 = doc["j7"];
      JsonArray j8 = doc["j8"];
      JsonArray j9 = doc["j9"];

      J7length = j7[0];
      J7rot = j7[1];
      J7steps = j7[2];

      J8length = j8[0];
      J8rot = j8[1];
      J8steps = j8[2];

      J9length = j9[0];
      J9rot = j9[1];
      J9steps = j9[2];

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
      Serial.println("Ext_Axes_Calebrated");
    }

    //-----COMMAND ZERO J7---------------------------------------------------
    //-----------------------------------------------------------------------
    if (cmd && strcmp(cmd, "zero_j7") == 0) {
      J7StepM = 0;
      sendRobotPos();
    }

    //-----COMMAND ZERO J8---------------------------------------------------
    //-----------------------------------------------------------------------
    if (cmd && strcmp(cmd, "zero_j8") == 0) {
      J8StepM = 0;
      sendRobotPos();
    }

    //-----COMMAND ZERO J9---------------------------------------------------
    //-----------------------------------------------------------------------
    if (cmd && strcmp(cmd, "zero_j9") == 0) {
      J9StepM = 0;
      sendRobotPos();
    }


    //-----COMMAND TO WAIT TIME---------------------------------------------------
    //-----------------------------------------------------------------------
    if (cmd && strcmp(cmd, "wait_time") == 0) {
      float WaitTime = doc["seconds"] | 0.0;
      int WaitTimeMS = WaitTime * 1000;
      delay(WaitTimeMS);
      Serial.println("WAIT_TIME_DONE");
    }


    //-----COMMAND SET OUTPUT ON---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "ON") {
      int ONstart = inData.indexOf('X');
      int outputNum = inData.substring(ONstart + 1).toInt();
      digitalWrite(outputNum, HIGH);
      delay(5);
      Serial.println();
    }
    //-----COMMAND SET OUTPUT OFF---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "OF") {
      int ONstart = inData.indexOf('X');
      int outputNum = inData.substring(ONstart + 1).toInt();
      digitalWrite(outputNum, LOW);
      delay(5);
      Serial.println();
    }



    //-----MODBUS READ HOLDING REGISTER - FUNCTION 03--------------------------------------------
    //-----------------------------------------------------------------------
    if (cmd && strcmp(cmd, "modbus_read_holding_register") == 0) {
      int slaveID = doc["slave_id"] | 1;
      int address = doc["address"] | 0;
      int count = doc["count"] | 1;

      int32_t result = modbusQuerry(slaveID, address, count, 3);

      if (result == -1) {
        Serial.println("Modbus Error");
      } else {
        Serial.println(result);
      }
    }

    //-----MODBUS READ COIL - FUNCTION 01--------------------------------------------
    //-----------------------------------------------------------------------
    if (cmd && strcmp(cmd, "modbus_read_coil") == 0) {
      int slaveID = doc["slave_id"] | 1;
      int address = doc["address"] | 0;
      int count = doc["count"] | 1;

      int32_t result = modbusQuerry(slaveID, address, count, 1);

      if (result == -1) {
        Serial.println("Modbus Error");
      } else {
        Serial.println(result);
      }
    }

    //-----MODBUS READ INPUT - FUNCTION 02--------------------------------------------
    //-----------------------------------------------------------------------
    if (cmd && strcmp(cmd, "modbus_read_discrete_input") == 0) {
      int slaveID = doc["slave_id"] | 1;
      int address = doc["address"] | 0;
      int count = doc["count"] | 1;

      int32_t result = modbusQuerry(slaveID, address, count, 2);

      if (result == -1) {
        Serial.println("Modbus Error");
      } else {
        Serial.println(result);
      }
    }

    //-----MODBUS READ INPUT REGISTER - FUNCTION 04--------------------------------------------
    //-----------------------------------------------------------------------
    if (cmd && strcmp(cmd, "modbus_read_input_register") == 0) {
      int slaveID = doc["slave_id"] | 1;
      int address = doc["address"] | 0;
      int count = doc["count"] | 1;

      int32_t result = modbusQuerry(slaveID, address, count, 4);

      if (result == -1) {
        Serial.println("Modbus Error");
      } else {
        Serial.println(result);
      }
    }

    //-----MODBUS WRITE SINGLE COIL - FUNCTION 5--------------------------------------------
    //-----------------------------------------------------------------------
    if (cmd && strcmp(cmd, "modbus_write_coil") == 0) {
      int slaveID = doc["slave_id"] | 1;
      int address = doc["address"] | 0;
      int value = doc["value"] | 0;

      int32_t result = modbusQuerry(slaveID, address, value, 5);

      if (result == -1) {
        Serial.println("Modbus Error");
      } else {
        Serial.println("Write Success");
      }
    }

    //-----MODBUS WRITE REGISTER - FUNCTION 6--------------------------------------------
    //-----------------------------------------------------------------------
    if (cmd && strcmp(cmd, "modbus_write_register") == 0) {
      int slaveID = doc["slave_id"] | 1;
      int address = doc["address"] | 0;
      int value = doc["value"] | 0;

      int32_t result = modbusQuerry(slaveID, address, value, 6);

      if (result == -1) {
        Serial.println("Modbus Error");
      } else {
        Serial.println("Write Success");
      }
    }

    //-----QUERRY DRIVE MODBUS--------------------------------------------
    //-----------------------------------------------------------------------
    if (cmd && strcmp(cmd, "modbus_query_test") == 0) {
      uint8_t result;
      int16_t highRegister;

      result = node.readHoldingRegisters(0x1207, 2);

      if (result == node.ku8MBSuccess) {
        highRegister = node.getResponseBuffer(0);
        Serial.println(highRegister);
      } else {
        Serial.println("Modbus error:");
      }

      delay(1000);
    }


    //-----COMMAND TO WAIT MODBUS COIL---------------------------------------------------
    if (cmd && strcmp(cmd, "wait_modbus_coil") == 0) {
      int32_t result = -2;
      int slaveID = doc["slave_id"] | 1;
      int input = doc["address"] | 0;
      int value = doc["expected"] | 0;
      int timeout = doc["timeout"] | 10;

      unsigned long timeoutMillis = timeout * 1000UL;
      unsigned long startTime = millis();

      while ((millis() - startTime < timeoutMillis) && (result != value)) {
        result = modbusQuerry(slaveID, input, 1, 1);
        delay(100);
      }

      if (result == value) {
        Serial.println("Done");
      } else if (result == -1) {
        Serial.println("Modbus Error");
      } else {
        Serial.println("Timeout");
      }
    }

    //-----COMMAND TO WAIT MODBUS HOLDING REGISTER---------------------------------------
    if (cmd && strcmp(cmd, "wait_modbus_holding_register") == 0) {
      int32_t result = -2;
      int slaveID = doc["slave_id"] | 1;
      int address = doc["address"] | 0;
      int value = doc["expected"] | 0;
      int timeout = doc["timeout"] | 10;

      unsigned long timeoutMillis = timeout * 1000UL;
      unsigned long startTime = millis();

      while ((millis() - startTime < timeoutMillis) && (result != value)) {
        result = modbusQuerry(slaveID, address, 1, 3);
        delay(100);
      }

      if (result == value) {
        Serial.println("Done");
      } else if (result == -1) {
        Serial.println("Modbus Error");
      } else {
        Serial.println("Timeout");
      }
    }


    //-----COMMAND SET POSITION---------------------------------------------------
    //-----------------------------------------------------------------------
    if (cmd && strcmp(cmd, "set_position") == 0) {
      JsonArray j = doc["j"];

      J1StepM = (j[0].as<float>() + J1axisLimNeg) * J1StepDeg;
      J2StepM = (j[1].as<float>() + J2axisLimNeg) * J2StepDeg;
      J3StepM = (j[2].as<float>() + J3axisLimNeg) * J3StepDeg;
      J4StepM = (j[3].as<float>() + J4axisLimNeg) * J4StepDeg;
      J5StepM = (j[4].as<float>() + J5axisLimNeg) * J5StepDeg;
      J6StepM = (j[5].as<float>() + J6axisLimNeg) * J6StepDeg;
      J7StepM = (j[6].as<float>() + J7axisLimNeg) * J7StepDeg;
      J8StepM = (j[7].as<float>() + J8axisLimNeg) * J8StepDeg;
      J9StepM = (j[8].as<float>() + J9axisLimNeg) * J9StepDeg;

      sendRobotPos();
    }


    //----- CALIBRATE  ---------------------------------------------------
    //-----------------------------------------------------------------------

    if (cmd && strcmp(cmd, "calibrate") == 0) {
      calibrateLimits(doc);
    }

    if (cmd && strcmp(cmd, "calibrate_stage2") == 0) {
      calibrateLimits(doc);
    }

    //----- MOVE J ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (cmd && strcmp(cmd, "move_j") == 0) {
      moveJ(doc, true, false, false);
    }

    //----- LIVE JOINT JOG  ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (cmd && strcmp(cmd, "live_joint_jog") == 0) {
      liveJointJog(doc);
    }

    //----- LIVE CARTESIAN JOG  ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (cmd && strcmp(cmd, "live_cart_jog") == 0) {
      liveCartJog(doc);
    }

    //----- LIVE TOOL JOG  ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (cmd && strcmp(cmd, "live_tool_jog") == 0) {
      liveToolJog(doc);
    }

    //----- MOVE L COMMAND  ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (cmd && strcmp(cmd, "move_l") == 0) {
      moveL(doc);
    }

    //----- JOG T COMMAND  ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (cmd && strcmp(cmd, "jog_t") == 0) {
      jogToolMove(doc);
    }

    //----- MOVE A COMMAND  ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (cmd && strcmp(cmd, "move_a") == 0) {
      moveA(doc);
    }



    //----- MOVE V ------ VISION OFFSET ----------------------------------
    //-----------------------------------------------------------------------
    if (cmd && strcmp(cmd, "move_vis") == 0) {
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

      JsonArray pose = doc["pose"];
      xyzuvw_In[0] = pose[0];
      xyzuvw_In[1] = pose[1];
      xyzuvw_In[2] = pose[2];
      xyzuvw_In[3] = pose[3];
      xyzuvw_In[4] = pose[4];
      xyzuvw_In[5] = pose[5];
      J7_In = pose[6];
      J8_In = pose[7];
      J9_In = pose[8];

      String SpeedType = doc["spd_type"] | "p";
      float SpeedVal = doc["spd"] | 25.0;
      float ACCspd = doc["acc"] | 10.0;
      float DCCspd = doc["dec"] | 10.0;
      float ACCramp = doc["ramp"] | 20.0;
      WristCon = doc["w"] | "F";
      float VisRot = doc["vis_rot"] | 0.0;
      String LoopMode = doc["loop_mode"] | "000000";
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

      // put tool rotation back where it was
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

        Alarm = "{\"msg\":\"error\",\"code\":\"R\"}";
        delay(5);
        Serial.println(Alarm);
      motionFault = true;
        Alarm = "0";

      } else {

        Alarm = "{\"msg\":\"error\",\"code\":\"L\",\"axes\":[";
        Alarm += String(J1axisFault) + ",";
        Alarm += String(J2axisFault) + ",";
        Alarm += String(J3axisFault) + ",";
        Alarm += String(J4axisFault) + ",";
        Alarm += String(J5axisFault) + ",";
        Alarm += String(J6axisFault) + ",";
        Alarm += String(J7axisFault) + ",";
        Alarm += String(J8axisFault) + ",";
        Alarm += String(J9axisFault) + "]}";

        delay(5);
        Serial.println(Alarm);
      motionFault = true;
        Alarm = "0";
      }

      inData = "";  // Clear received buffer
      ////////MOVE COMPLETE///////////
    }




    //----- MOVE IN JOINTS ROTATION  ---------------------------------------------------
    //-----------------------------------------------------------------------

    if (cmd && strcmp(cmd, "move_joints") == 0) {
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

      JsonArray j = doc["j"];

      float J1Angle = j[0];
      float J2Angle = j[1];
      float J3Angle = j[2];
      float J4Angle = j[3];
      float J5Angle = j[4];
      float J6Angle = j[5];

      J7_In = j[6];
      J8_In = j[7];
      J9_In = j[8];

      const char *spd = doc["spd_type"];
      String SpeedType =
        (strcmp(spd, "seconds") == 0) ? "s" : (strcmp(spd, "mmps") == 0) ? "m"
                                                                         : "p";
      float SpeedVal = doc["spd"];
      float ACCspd = doc["acc"];
      float DCCspd = doc["dec"];
      float ACCramp = doc["ramp"];
      WristCon = doc["wc"].as<String>();
      String LoopMode = doc["loop_mode"].as<String>();

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

        Alarm = "{\"msg\":\"error\",\"code\":\"R\"}";
        delay(5);
        Serial.println(Alarm);
      motionFault = true;
        Alarm = "0";

      } else {

        Alarm = "{\"msg\":\"error\",\"code\":\"L\",\"axes\":[";
        Alarm += String(J1axisFault) + ",";
        Alarm += String(J2axisFault) + ",";
        Alarm += String(J3axisFault) + ",";
        Alarm += String(J4axisFault) + ",";
        Alarm += String(J5axisFault) + ",";
        Alarm += String(J6axisFault) + ",";
        Alarm += String(J7axisFault) + ",";
        Alarm += String(J8axisFault) + ",";
        Alarm += String(J9axisFault) + "]}";

        delay(5);
        Serial.println(Alarm);
      motionFault = true;
        Alarm = "0";
      }

      inData = "";  // Clear recieved buffer
      ////////MOVE COMPLETE///////////
    }



    //----- DELETE PROG FROM SD CARD ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (cmd && strcmp(cmd, "delete_SD_program") == 0) {
      SD.begin(BUILTIN_SDCARD);
      String filename = doc["filename"] | "";
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
    if (cmd && strcmp(cmd, "read_SD_files") == 0) {
      File root;
      SD.begin(BUILTIN_SDCARD);
      root = SD.open("/");
      printDirectory(root, 0);
    }


    //----- WRITE COMMAND TO SD CARD ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (cmd && strcmp(cmd, "write_gcode_move") == 0) {
      SD.begin(BUILTIN_SDCARD);
      String filename = doc["filename"] | "";
      const char *fn = filename.c_str();

      // reconstruct the pose string to write to SD
      const char *spd = doc["spd_type"] | "percent";
      String SpeedType =
        (strcmp(spd, "seconds") == 0) ? "s" : (strcmp(spd, "mmps") == 0) ? "m"
                                                                         : "p";
      float SpeedVal = doc["spd"] | 25.0;
      float ACCspd = doc["acc"] | 10.0;
      float DCCspd = doc["dec"] | 10.0;
      float ACCramp = doc["ramp"] | 20.0;
      String WristCon = doc["w"] | "N";
      String LoopMode = doc["loop_mode"] | "000000";
      JsonArray pose = doc["pose"];

      String info = "X" + String((float)pose[0], 3) + "Y" + String((float)pose[1], 3) + "Z" + String((float)pose[2], 3) + "Rz" + String((float)pose[3], 3) + "Ry" + String((float)pose[4], 3) + "Rx" + String((float)pose[5], 3) + "J7" + String((float)pose[6], 2) + "J8" + String((float)pose[7], 2) + "J9" + String((float)pose[8], 2) + "S" + SpeedType + String(SpeedVal) + "Ac" + String(ACCspd) + "Dc" + String(DCCspd) + "Rm" + String(ACCramp) + "Rnd0" + "W" + WristCon + "Lm" + LoopMode;

      writeSD(fn, info);
      sendRobotPos();
    }


    //----- WRITE PRE-CALC'D MOVE TO SD CARD CURRENTLY NOT USED ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (cmd && strcmp(cmd, "write_gcode_move_precalc") == 0) {
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

      JsonArray pose = doc["pose"];
      xyzuvw_In[0] = pose[0];
      xyzuvw_In[1] = pose[1];
      xyzuvw_In[2] = pose[2];
      xyzuvw_In[3] = pose[3];
      xyzuvw_In[4] = pose[4];
      xyzuvw_In[5] = pose[5];
      J7_In = pose[6];
      J8_In = pose[7];
      J9_In = pose[8];

      String SpeedType = doc["spd_type"] | "p";
      float SpeedVal = doc["spd"] | 25.0;
      float ACCspd = doc["acc"] | 10.0;
      float DCCspd = doc["dec"] | 10.0;
      float ACCramp = doc["ramp"] | 20.0;
      float Rounding = doc["rnd"] | 0.0;
      WristCon = doc["w"] | "F";
      String LoopMode = doc["loop_mode"] | "000000";
      String filename = doc["filename"] | "";

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
        info = "{\"J1\":" + String(abs(J1stepDif)) + ",\"J2\":" + String(abs(J2stepDif)) + ",\"J3\":" + String(abs(J3stepDif)) + ",\"J4\":" + String(abs(J4stepDif)) + ",\"J5\":" + String(abs(J5stepDif)) + ",\"J6\":" + String(abs(J6stepDif)) + ",\"J7\":" + String(abs(J7stepDif)) + ",\"J8\":" + String(abs(J8stepDif)) + ",\"J9\":" + String(abs(J9stepDif)) + ",\"d1\":" + String(J1dir) + ",\"d2\":" + String(J2dir) + ",\"d3\":" + String(J3dir) + ",\"d4\":" + String(J4dir) + ",\"d5\":" + String(J5dir) + ",\"d6\":" + String(J6dir) + ",\"d7\":" + String(J7dir) + ",\"d8\":" + String(J8dir) + ",\"d9\":" + String(J9dir) + ",\"st\":\"" + SpeedType + "\"" + ",\"sp\":" + String(SpeedVal) + ",\"ac\":" + String(ACCspd) + ",\"dc\":" + String(DCCspd) + ",\"rm\":" + String(ACCramp) + "}";
        writeSD(filename, info);
        sendRobotPos();
      } else if (KinematicError == 1) {

        Alarm = "{\"msg\":\"error\",\"code\":\"R\"}";
        delay(5);
        Serial.println(Alarm);
      motionFault = true;
        Alarm = "0";

      } else {

        Alarm = "{\"msg\":\"error\",\"code\":\"L\",\"axes\":[";
        Alarm += String(J1axisFault) + ",";
        Alarm += String(J2axisFault) + ",";
        Alarm += String(J3axisFault) + ",";
        Alarm += String(J4axisFault) + ",";
        Alarm += String(J5axisFault) + ",";
        Alarm += String(J6axisFault) + ",";
        Alarm += String(J7axisFault) + ",";
        Alarm += String(J8axisFault) + ",";
        Alarm += String(J9axisFault) + "]}";

        delay(5);
        Serial.println(Alarm);
      motionFault = true;
        Alarm = "0";
      }

      inData = "";  // Clear received buffer
      ////////MOVE COMPLETE///////////
    }



    //----- PLAY FILE ON SD CARD STEPS - CURRENTLY NOT USED ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (cmd && strcmp(cmd, "play_gcode_file_steps") == 0) {
      File gcFile;
      SD.begin(BUILTIN_SDCARD);
      String filename = doc["filename"] | "";
      const char *fn = filename.c_str();
      gcFile = SD.open(fn);
      if (!gcFile) {
        Alarm = "{\"msg\":\"error\",\"code\":\"G\"}";
        delay(5);
        Serial.println(Alarm);
      motionFault = true;
        Alarm = "0";
        cmdBuffer1 = "";
        cmdExecuting = false;
        return;
      }
      while (gcFile.available() && estopActive == false) {
        String Cmd = gcFile.readStringUntil('\n');
        Cmd.trim();
        JsonDocument lineDoc;
        DeserializationError err = deserializeJson(lineDoc, Cmd);
        if (err) continue;
        int J1step = lineDoc["J1"] | 0;
        int J2step = lineDoc["J2"] | 0;
        int J3step = lineDoc["J3"] | 0;
        int J4step = lineDoc["J4"] | 0;
        int J5step = lineDoc["J5"] | 0;
        int J6step = lineDoc["J6"] | 0;
        int J7step = lineDoc["J7"] | 0;
        int J8step = lineDoc["J8"] | 0;
        int J9step = lineDoc["J9"] | 0;
        int J1dir = lineDoc["d1"] | 0;
        int J2dir = lineDoc["d2"] | 0;
        int J3dir = lineDoc["d3"] | 0;
        int J4dir = lineDoc["d4"] | 0;
        int J5dir = lineDoc["d5"] | 0;
        int J6dir = lineDoc["d6"] | 0;
        int J7dir = lineDoc["d7"] | 0;
        int J8dir = lineDoc["d8"] | 0;
        int J9dir = lineDoc["d9"] | 0;
        String SpeedType = lineDoc["st"] | "p";
        float SpeedVal = lineDoc["sp"] | 25.0;
        float ACCspd = lineDoc["ac"] | 10.0;
        float DCCspd = lineDoc["dc"] | 10.0;
        float ACCramp = lineDoc["rm"] | 20.0;
        driveMotorsG(J1step, J2step, J3step, J4step, J5step, J6step,
                     J7step, J8step, J9step,
                     J1dir, J2dir, J3dir, J4dir, J5dir, J6dir,
                     J7dir, J8dir, J9dir,
                     SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
      }
      gcFile.close();
      sendRobotPos();
    }



    //----- PLAY FILE ON SD CARD ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (cmd && strcmp(cmd, "play_gcode_file") == 0) {
      File gcFile;
      SD.begin(BUILTIN_SDCARD);
      String filename = doc["filename"] | "";
      const char *fn = filename.c_str();
      gcFile = SD.open(fn);
      if (!gcFile) {
        Alarm = "{\"msg\":\"error\",\"code\":\"G\"}";
        delay(5);
        Serial.println(Alarm);
      motionFault = true;
        Alarm = "0";
        cmdBuffer1 = "";
        cmdExecuting = false;
        return;
      }
      while (gcFile.available() && estopActive == false) {
        String Cmd = gcFile.readStringUntil('\n');
        Cmd.trim();

        int xStart = Cmd.indexOf('X');
        int yStart = Cmd.indexOf('Y');
        int zStart = Cmd.indexOf('Z');
        int rzStart = Cmd.indexOf("Rz");
        int ryStart = Cmd.indexOf("Ry");
        int rxStart = Cmd.indexOf("Rx");
        int j7Start = Cmd.indexOf("J7");
        int j8Start = Cmd.indexOf("J8");
        int j9Start = Cmd.indexOf("J9");
        int sStart = Cmd.indexOf('S');
        int acStart = Cmd.indexOf("Ac");
        int dcStart = Cmd.indexOf("Dc");
        int rmStart = Cmd.indexOf("Rm");
        int rndStart = Cmd.indexOf("Rnd");
        int wStart = Cmd.indexOf('W');
        int lmStart = Cmd.indexOf("Lm");

        xyzuvw_In[0] = Cmd.substring(xStart + 1, yStart).toFloat();
        xyzuvw_In[1] = Cmd.substring(yStart + 1, zStart).toFloat();
        xyzuvw_In[2] = Cmd.substring(zStart + 1, rzStart).toFloat();
        xyzuvw_In[3] = Cmd.substring(rzStart + 2, ryStart).toFloat();
        xyzuvw_In[4] = Cmd.substring(ryStart + 2, rxStart).toFloat();
        xyzuvw_In[5] = Cmd.substring(rxStart + 2, j7Start).toFloat();
        J7_In = Cmd.substring(j7Start + 2, j8Start).toFloat();
        J8_In = Cmd.substring(j8Start + 2, j9Start).toFloat();
        J9_In = Cmd.substring(j9Start + 2, sStart).toFloat();

        String SpeedType = Cmd.substring(sStart + 1, sStart + 2);
        float SpeedVal = Cmd.substring(sStart + 2, acStart).toFloat();
        float ACCspd = Cmd.substring(acStart + 2, dcStart).toFloat();
        float DCCspd = Cmd.substring(dcStart + 2, rmStart).toFloat();
        float ACCramp = Cmd.substring(rmStart + 2, rndStart).toFloat();
        WristCon = Cmd.substring(wStart + 1, lmStart);
        String LoopMode = Cmd.substring(lmStart + 2);
        LoopMode.trim();
        J1LoopMode = LoopMode.substring(0, 1).toInt();
        J2LoopMode = LoopMode.substring(1, 2).toInt();
        J3LoopMode = LoopMode.substring(2, 3).toInt();
        J4LoopMode = LoopMode.substring(3, 4).toInt();
        J5LoopMode = LoopMode.substring(4, 5).toInt();
        J6LoopMode = LoopMode.substring(5).toInt();

        SolveInverseKinematics();

        int J1futStepM = (JangleOut[0] + J1axisLimNeg) * J1StepDeg;
        int J2futStepM = (JangleOut[1] + J2axisLimNeg) * J2StepDeg;
        int J3futStepM = (JangleOut[2] + J3axisLimNeg) * J3StepDeg;
        int J4futStepM = (JangleOut[3] + J4axisLimNeg) * J4StepDeg;
        int J5futStepM = (JangleOut[4] + J5axisLimNeg) * J5StepDeg;
        int J6futStepM = (JangleOut[5] + J6axisLimNeg) * J6StepDeg;
        int J7futStepM = (J7_In + J7axisLimNeg) * J7StepDeg;
        int J8futStepM = (J8_In + J8axisLimNeg) * J8StepDeg;
        int J9futStepM = (J9_In + J9axisLimNeg) * J9StepDeg;

        int J1stepDif = J1StepM - J1futStepM;
        int J2stepDif = J2StepM - J2futStepM;
        int J3stepDif = J3StepM - J3futStepM;
        int J4stepDif = J4StepM - J4futStepM;
        int J5stepDif = J5StepM - J5futStepM;
        int J6stepDif = J6StepM - J6futStepM;
        int J7stepDif = J7StepM - J7futStepM;
        int J8stepDif = J8StepM - J8futStepM;
        int J9stepDif = J9StepM - J9futStepM;

        int J1dir = (J1stepDif <= 0) ? 1 : 0;
        int J2dir = (J2stepDif <= 0) ? 1 : 0;
        int J3dir = (J3stepDif <= 0) ? 1 : 0;
        int J4dir = (J4stepDif <= 0) ? 1 : 0;
        int J5dir = (J5stepDif <= 0) ? 1 : 0;
        int J6dir = (J6stepDif <= 0) ? 1 : 0;
        int J7dir = (J7stepDif <= 0) ? 1 : 0;
        int J8dir = (J8stepDif <= 0) ? 1 : 0;
        int J9dir = (J9stepDif <= 0) ? 1 : 0;

        driveMotorsG(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif),
                     abs(J4stepDif), abs(J5stepDif), abs(J6stepDif),
                     abs(J7stepDif), abs(J8stepDif), abs(J9stepDif),
                     J1dir, J2dir, J3dir, J4dir, J5dir, J6dir,
                     J7dir, J8dir, J9dir,
                     SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
      }
      gcFile.close();
      sendRobotPos();
    }



    //----- MOVE C (Circle) ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (strcmp(cmd, "move_c") == 0) {

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

      Alarm = "0";
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

      // JSON parser:
      // move_j already moved the robot to the circle start.
      // move_c only receives center + plane/reference point + orientation/speed data.
      float xBeg = doc["center"][0];
      float yBeg = doc["center"][1];
      float zBeg = doc["center"][2];

      float rxBeg = doc["pose"][0];
      float ryBeg = doc["pose"][1];
      float rzBeg = doc["pose"][2];

      float xEnd = doc["plane"][0];
      float yEnd = doc["plane"][1];
      float zEnd = doc["plane"][2];

      // No track value for Move C
      xyzuvw_In[6] = 0;

      const char *spd = doc["spd_type"] | "percent";

      String SpeedType =
        (strcmp(spd, "seconds") == 0) ? "s" : (strcmp(spd, "mmps") == 0) ? "m"
                                                                         : "p";

      float SpeedVal = doc["spd"];
      float ACCspd = doc["acc"];
      float DCCspd = doc["dec"];
      float ACCramp = doc["ramp"];

      WristCon = doc["w"] | "A";

      String LoopMode = doc["loop_mode"] | "111111";
      LoopMode.trim();

      J1LoopMode = LoopMode.substring(0, 1).toInt();
      J2LoopMode = LoopMode.substring(1, 2).toInt();
      J3LoopMode = LoopMode.substring(2, 3).toInt();
      J4LoopMode = LoopMode.substring(3, 4).toInt();
      J5LoopMode = LoopMode.substring(4, 5).toInt();
      J6LoopMode = LoopMode.substring(5).toInt();

      // Use the robot's current position as the circle start.
      // This assumes the Stage 1 move_j has already completed.
      SolveForwardKinematics();

      float xMid = xyzuvw_Out[0];
      float yMid = xyzuvw_Out[1];
      float zMid = xyzuvw_Out[2];

      //calc vector from start point of circle (current position) to center of circle
      Xvect = xMid - xBeg;
      Yvect = yMid - yBeg;
      Zvect = zMid - zBeg;

      //get radius - distance from center point to start point
      float Radius = pow((pow((Xvect), 2) + pow((Yvect), 2) + pow((Zvect), 2)), .5);

      //set center coordinates of circle
      float Px = xBeg;
      float Py = yBeg;
      float Pz = zBeg;

      //define start vector, current robot position is start of circle
      startVect[0] = (xMid - Px);
      startVect[1] = (yMid - Py);
      startVect[2] = (zMid - Pz);

      //get vectors from center of circle to start and plane/reference point, then normalize
      float vect_Bmag = pow((pow((xMid - Px), 2) + pow((yMid - Py), 2) + pow((zMid - Pz), 2)), .5);
      float vect_Bx = (xMid - Px) / vect_Bmag;
      float vect_By = (yMid - Py) / vect_Bmag;
      float vect_Bz = (zMid - Pz) / vect_Bmag;

      float vect_Cmag = pow((pow((xEnd - Px), 2) + pow((yEnd - Py), 2) + pow((zEnd - Pz), 2)), .5);
      float vect_Cx = (xEnd - Px) / vect_Cmag;
      float vect_Cy = (yEnd - Py) / vect_Cmag;
      float vect_Cz = (zEnd - Pz) / vect_Cmag;

      //get cross product of vectors B and C then apply to axis matrix
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

      //calc way pt angle
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

      //set starting delay
      float curDelay = calcACCstartDel;

      //set starting angle first way pt
      float cur_deg = theta_Deg;

      /////////////////////////////////////
      //loop through waypoints
      ////////////////////////////////////

      resetEncoders();

      for (int i = 1; i <= wayPts; i++) {
        if (estopActive || motionFault) break;

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

        //shift way pts back to original origin and calc kinematics for way pt movement
        xyzuvw_In[0] = (DestPt[0]) + Px;
        xyzuvw_In[1] = (DestPt[1]) + Py;
        xyzuvw_In[2] = (DestPt[2]) + Pz;
        xyzuvw_In[3] = rxBeg;
        xyzuvw_In[4] = ryBeg;
        xyzuvw_In[5] = rzBeg;

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
          updatePos();
        } else if (KinematicError == 1) {

          Alarm = "{\"msg\":\"error\",\"code\":\"R\"}";
          delay(5);
          Serial.println(Alarm);
      motionFault = true;
          Alarm = "0";
          break;

        } else {

          Alarm = "{\"msg\":\"error\",\"code\":\"L\",\"axes\":[";
          Alarm += String(J1axisFault) + ",";
          Alarm += String(J2axisFault) + ",";
          Alarm += String(J3axisFault) + ",";
          Alarm += String(J4axisFault) + ",";
          Alarm += String(J5axisFault) + ",";
          Alarm += String(J6axisFault) + "]}";



          delay(5);
          Serial.println(Alarm);
      motionFault = true;
          Alarm = "0";
          break;
        }

        //increment angle
        cur_deg += theta_Deg;
      }

      checkEncoders();
      if (!motionFault) {
        sendRobotPos();
      }

      inData = "";  // Clear received buffer
      ////////MOVE COMPLETE///////////
    }







    // Streaming bookkeeping: single stop reply, or abort the stream after a fault / E-stop
    suppressReplies = false;
    cmdExecuting = false;
    if (stopRequested) {
      finishStop();
    } else if (splineTrue && queuedCmd && (motionFault || estopActive)) {
      abortStream();
    }

    //shift cmd buffer
    inData = "";
    cmdBuffer1 = "";
    shiftCMDarray();
  }
}
