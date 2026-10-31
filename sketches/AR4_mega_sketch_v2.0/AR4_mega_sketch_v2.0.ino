/*  AR4 Annin Robot Control Software Arduino Mega sketch
    Copyright (c) 2024, Chris Annin
    All rights reserved.

    Version 2.0 JSON command update
    - Converts Mega IO / servo commands to JSON format
    - Matches the updated Nano JSON API behavior
    - Every command returns a newline-terminated response
    - Adds servo gripper detach / auto-detach behavior
    - Preserves Mega pin assignments

    Required library:
    - ArduinoJson
*/

#include <Servo.h>
#include <ArduinoJson.h>

// -----------------------------------------------------------------------------
// Serial command buffer
// -----------------------------------------------------------------------------
String inData;

// -----------------------------------------------------------------------------
// Servo outputs
// Existing Mega design uses A0-A6 for servo headers.
// -----------------------------------------------------------------------------
Servo servo0;
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;

// -----------------------------------------------------------------------------
// IO pins
// Mega version keeps existing input/output assignments.
// Inputs: 0-27
// Outputs: 28-53
// -----------------------------------------------------------------------------
const int Input0 = 0;
const int Input1 = 1;
const int Input2 = 2;
const int Input3 = 3;
const int Input4 = 4;
const int Input5 = 5;
const int Input6 = 6;
const int Input7 = 7;
const int Input8 = 8;
const int Input9 = 9;
const int Input10 = 10;
const int Input11 = 11;
const int Input12 = 12;
const int Input13 = 13;
const int Input14 = 14;
const int Input15 = 15;
const int Input16 = 16;
const int Input17 = 17;
const int Input18 = 18;
const int Input19 = 19;
const int Input20 = 20;
const int Input21 = 21;
const int Input22 = 22;
const int Input23 = 23;
const int Input24 = 24;
const int Input25 = 25;
const int Input26 = 26;
const int Input27 = 27;

const int Output28 = 28;
const int Output29 = 29;
const int Output30 = 30;
const int Output31 = 31;
const int Output32 = 32;
const int Output33 = 33;
const int Output34 = 34;
const int Output35 = 35;
const int Output36 = 36;
const int Output37 = 37;
const int Output38 = 38;
const int Output39 = 39;
const int Output40 = 40;
const int Output41 = 41;
const int Output42 = 42;
const int Output43 = 43;
const int Output44 = 44;
const int Output45 = 45;
const int Output46 = 46;
const int Output47 = 47;
const int Output48 = 48;
const int Output49 = 49;
const int Output50 = 50;
const int Output51 = 51;
const int Output52 = 52;
const int Output53 = 53;

// -----------------------------------------------------------------------------
// ACS712 5A current sensor
// -----------------------------------------------------------------------------
const int ACS_PIN = A7;
const float ACS_VCC = 5.0;
const float ACS_SENS = 0.185;

int acsZeroRaw = 512;

// -----------------------------------------------------------------------------
// Gripper tracking
// -----------------------------------------------------------------------------
float gripperCmd = 20.0;
const float GRIPPER_MIN = 0.0;
const float GRIPPER_MAX = 45.0;

// -----------------------------------------------------------------------------
// Current limit behavior
// -----------------------------------------------------------------------------
const float CURRENT_LIMIT = 0.8;
const unsigned long BACKOFF_INTERVAL = 3000;
unsigned long lastBackoffMs = 0;

// -----------------------------------------------------------------------------
// Wait-input cancellation flag
// -----------------------------------------------------------------------------
bool megaStopRequested = false;

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

void sendError(const char *message) {
  Serial.print(F("Error: "));
  Serial.println(message);
}

bool isValidInputPin(int pin) {
  return pin >= 0 && pin <= 27;
}

bool isValidOutputPin(int pin) {
  return pin >= 28 && pin <= 53;
}

bool isValidServoNum(int num) {
  return num >= 0 && num <= 6;
}

void writeServo(int servoNum, int servoPos) {
  servoPos = constrain(servoPos, 0, 180);

  if (servoNum == 0) {
    if (!servo0.attached()) {
      servo0.attach(A0);
    }

    gripperCmd = constrain((float)servoPos, GRIPPER_MIN, GRIPPER_MAX);
    servo0.write((int)round(gripperCmd));

    if (gripperCmd >= 45) {
      delay(500);
      servo0.detach();
    }

  } else if (servoNum == 1) {
    servo1.write(servoPos);
  } else if (servoNum == 2) {
    servo2.write(servoPos);
  } else if (servoNum == 3) {
    servo3.write(servoPos);
  } else if (servoNum == 4) {
    servo4.write(servoPos);
  } else if (servoNum == 5) {
    servo5.write(servoPos);
  } else if (servoNum == 6) {
    servo6.write(servoPos);
  }
}

float readAcsCurrent() {
  const int samples = 20;
  long sum = 0;

  for (int i = 0; i < samples; i++) {
    sum += analogRead(ACS_PIN);
  }

  float raw = sum / (float)samples;
  float volts = (raw * ACS_VCC) / 1023.0;
  float zeroVolts = (acsZeroRaw * ACS_VCC) / 1023.0;
  float amps = (volts - zeroVolts) / ACS_SENS;

  if (amps < 0) amps = -amps;

  return amps;
}

void gripperBackoff() {
  float amps = readAcsCurrent();
  unsigned long now = millis();

  if (!servo0.attached()) {
    return;
  }

  if (amps > CURRENT_LIMIT) {
    if (now - lastBackoffMs >= BACKOFF_INTERVAL) {
      lastBackoffMs = now;

      if (gripperCmd < GRIPPER_MAX) {
        gripperCmd += 0.25;
        if (gripperCmd > GRIPPER_MAX) gripperCmd = GRIPPER_MAX;
        servo0.write((int)round(gripperCmd));
      }
    }
  } else {
    lastBackoffMs = now;
  }
}

void calibrateCurrentSensor() {
  long sum = 0;
  for (int i = 0; i < 200; i++) {
    sum += analogRead(ACS_PIN);
    delay(2);
  }
  acsZeroRaw = sum / 200;
}

// -----------------------------------------------------------------------------
// JSON command processor
// -----------------------------------------------------------------------------
void processJsonCommand(String rawCommand) {
  rawCommand.trim();

  if (rawCommand.length() == 0) {
    return;
  }

  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, rawCommand);

  if (error) {
    sendError("Invalid JSON");
    return;
  }

  const char *cmd = doc["cmd"] | "";

  // ---------------------------------------------------------------------------
  // hello
  // {"cmd":"hello"}
  // ---------------------------------------------------------------------------
  if (strcmp(cmd, "hello") == 0) {
    Serial.println(F("{\"board\":\"AR4 Mega IO\",\"firmware\":\"2.0\"}"));
    return;
  }

  // ---------------------------------------------------------------------------
  // test_gripper_amps
  // {"cmd":"test_gripper_amps"}
  // ---------------------------------------------------------------------------
  if (strcmp(cmd, "test_gripper_amps") == 0) {
    Serial.println(readAcsCurrent(), 3);
    return;
  }

  // ---------------------------------------------------------------------------
  // servo
  // {"cmd":"servo","num":0,"pos":20}
  // ---------------------------------------------------------------------------
  if (strcmp(cmd, "servo") == 0) {
    int servoNum = doc["num"] | -1;
    int servoPos = doc["pos"] | -1;

    if (!isValidServoNum(servoNum)) {
      sendError("Invalid servo number");
      return;
    }

    if (servoPos < 0 || servoPos > 180) {
      sendError("Invalid servo position");
      return;
    }

    writeServo(servoNum, servoPos);
    Serial.println(F("Servo Actuated"));
    return;
  }

  // ---------------------------------------------------------------------------
  // gripper
  // Convenience alias for servo 0.
  // {"cmd":"gripper","pos":20}
  // ---------------------------------------------------------------------------
  if (strcmp(cmd, "gripper") == 0) {
    int servoPos = doc["pos"] | -1;

    if (servoPos < GRIPPER_MIN || servoPos > GRIPPER_MAX) {
      sendError("Invalid gripper position");
      return;
    }

    writeServo(0, servoPos);
    Serial.println(F("Gripper CMD Executed"));
    return;
  }

  // ---------------------------------------------------------------------------
  // input_read
  // {"cmd":"input_read","pin":2}
  // Returns T for HIGH and F for LOW to preserve current HMI logic.
  // ---------------------------------------------------------------------------
  if (strcmp(cmd, "input_read") == 0) {
    int pin = doc["pin"] | -1;

    if (!isValidInputPin(pin)) {
      sendError("Invalid input pin");
      return;
    }

    if (digitalRead(pin) == HIGH) {
      Serial.println(F("T"));
    } else {
      Serial.println(F("F"));
    }
    return;
  }

  // ---------------------------------------------------------------------------
  // set_output
  // {"cmd":"set_output","pin":28,"state":1}
  // ---------------------------------------------------------------------------
  if (strcmp(cmd, "set_output") == 0) {
    int pin = doc["pin"] | -1;
    int state = doc["state"] | -1;

    if (!isValidOutputPin(pin)) {
      sendError("Invalid output pin");
      return;
    }

    if (!(state == 0 || state == 1)) {
      sendError("Invalid output state");
      return;
    }

    digitalWrite(pin, state == 1 ? HIGH : LOW);
    Serial.println(F("Output Set"));
    return;
  }

  // ---------------------------------------------------------------------------
  // wait_input
  // {"cmd":"wait_input","pin":2,"state":1,"timeout":10}
  // timeout is in seconds.
  // ---------------------------------------------------------------------------
  if (strcmp(cmd, "wait_input") == 0) {
    int pin = doc["pin"] | -1;
    int expectedState = doc["state"] | -1;
    unsigned long timeoutSeconds = doc["timeout"] | 0;

    if (!isValidInputPin(pin)) {
      sendError("Invalid input pin");
      return;
    }

    if (!(expectedState == 0 || expectedState == 1)) {
      sendError("Invalid input state");
      return;
    }

    megaStopRequested = false;
    unsigned long timeoutMillis = timeoutSeconds * 1000UL;
    unsigned long startTime = millis();
    int inputVal = digitalRead(pin);

    while ((millis() - startTime < timeoutMillis) && (inputVal != expectedState)) {
      gripperBackoff();
      inputVal = digitalRead(pin);

      if (Serial.available() > 0) {
        String stopCmd = Serial.readStringUntil('\n');
        stopCmd.trim();

        StaticJsonDocument<128> stopDoc;
        DeserializationError stopError = deserializeJson(stopDoc, stopCmd);

        if (!stopError) {
          const char *stopName = stopDoc["cmd"] | "";
          if (strcmp(stopName, "stop") == 0) {
            megaStopRequested = true;
            break;
          }
        }
      }

      delay(10);
    }

    if (megaStopRequested) {
      Serial.println(F("Mega Stopped"));
    } else if (inputVal == expectedState) {
      Serial.println(F("Wait Complete"));
    } else {
      Serial.println(F("Timeout"));
    }
    return;
  }

  // ---------------------------------------------------------------------------
  // stop
  // {"cmd":"stop"}
  // ---------------------------------------------------------------------------
  if (strcmp(cmd, "stop") == 0) {
    megaStopRequested = true;
    Serial.println(F("Mega Inactive Stopped"));
    return;
  }

  // ---------------------------------------------------------------------------
  // detach
  // {"cmd":"gripper_detach"}
  // ---------------------------------------------------------------------------
  if (strcmp(cmd, "gripper_detach") == 0) {
    servo0.detach();
    Serial.println(F("Servo Gripper Detached"));
    return;
  }

  // ---------------------------------------------------------------------------
  // echo
  // {"cmd":"echo","msg":"test"}
  // ---------------------------------------------------------------------------
  if (strcmp(cmd, "echo") == 0) {
    const char *msg = doc["msg"] | "";
    Serial.println(msg);
    return;
  }

  // Backward-compatible echo alias from earlier sketch.
  if (strcmp(cmd, "test_message") == 0) {
    const char *msg = doc["msg"] | "";
    Serial.println(msg);
    return;
  }

  sendError("Unknown command");
}

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);

  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);
  pinMode(A6, OUTPUT);
  pinMode(A7, INPUT);

  pinMode(A8, INPUT);
  pinMode(A9, INPUT);
  pinMode(A10, INPUT);
  pinMode(A11, INPUT);
  pinMode(A12, INPUT);
  pinMode(A13, INPUT);
  pinMode(A14, INPUT);
  pinMode(A15, INPUT);

  pinMode(Input0, INPUT_PULLUP);
  pinMode(Input1, INPUT_PULLUP);
  pinMode(Input2, INPUT_PULLUP);
  pinMode(Input3, INPUT_PULLUP);
  pinMode(Input4, INPUT_PULLUP);
  pinMode(Input5, INPUT_PULLUP);
  pinMode(Input6, INPUT_PULLUP);
  pinMode(Input7, INPUT_PULLUP);
  pinMode(Input8, INPUT_PULLUP);
  pinMode(Input9, INPUT_PULLUP);
  pinMode(Input10, INPUT_PULLUP);
  pinMode(Input11, INPUT_PULLUP);
  pinMode(Input12, INPUT_PULLUP);
  pinMode(Input13, INPUT_PULLUP);
  pinMode(Input14, INPUT_PULLUP);
  pinMode(Input15, INPUT_PULLUP);
  pinMode(Input16, INPUT_PULLUP);
  pinMode(Input17, INPUT_PULLUP);
  pinMode(Input18, INPUT_PULLUP);
  pinMode(Input19, INPUT_PULLUP);
  pinMode(Input20, INPUT_PULLUP);
  pinMode(Input21, INPUT_PULLUP);
  pinMode(Input22, INPUT_PULLUP);
  pinMode(Input23, INPUT_PULLUP);
  pinMode(Input24, INPUT_PULLUP);
  pinMode(Input25, INPUT_PULLUP);
  pinMode(Input26, INPUT_PULLUP);
  pinMode(Input27, INPUT_PULLUP);

  pinMode(Output28, OUTPUT);
  pinMode(Output29, OUTPUT);
  pinMode(Output30, OUTPUT);
  pinMode(Output31, OUTPUT);
  pinMode(Output32, OUTPUT);
  pinMode(Output33, OUTPUT);
  pinMode(Output34, OUTPUT);
  pinMode(Output35, OUTPUT);
  pinMode(Output36, OUTPUT);
  pinMode(Output37, OUTPUT);
  pinMode(Output38, OUTPUT);
  pinMode(Output39, OUTPUT);
  pinMode(Output40, OUTPUT);
  pinMode(Output41, OUTPUT);
  pinMode(Output42, OUTPUT);
  pinMode(Output43, OUTPUT);
  pinMode(Output44, OUTPUT);
  pinMode(Output45, OUTPUT);
  pinMode(Output46, OUTPUT);
  pinMode(Output47, OUTPUT);
  pinMode(Output48, OUTPUT);
  pinMode(Output49, OUTPUT);
  pinMode(Output50, OUTPUT);
  pinMode(Output51, OUTPUT);
  pinMode(Output52, OUTPUT);
  pinMode(Output53, OUTPUT);

  // Preserve previous startup behavior for first eight Mega outputs.
  digitalWrite(Output28, HIGH);
  digitalWrite(Output29, HIGH);
  digitalWrite(Output30, HIGH);
  digitalWrite(Output31, HIGH);
  digitalWrite(Output32, HIGH);
  digitalWrite(Output33, HIGH);
  digitalWrite(Output34, HIGH);
  digitalWrite(Output35, HIGH);

  servo0.attach(A0);
  servo1.attach(A1);
  servo2.attach(A2);
  servo3.attach(A3);
  servo4.attach(A4);
  servo5.attach(A5);
  servo6.attach(A6);

  servo0.write((int)round(gripperCmd));

  calibrateCurrentSensor();
}

// -----------------------------------------------------------------------------
// Main loop
// -----------------------------------------------------------------------------
void loop() {
  gripperBackoff();

  while (Serial.available() > 0) {
    char received = Serial.read();
    inData += received;

    if (received == '\n') {
      processJsonCommand(inData);
      inData = "";
    }
  }
}
