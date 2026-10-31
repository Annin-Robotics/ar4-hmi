/*  AR4 Annin Robot Control Software Arduino Nano sketch
    Copyright (c) 2024, Chris Annin
    All rights reserved.

    Version 2.0 JSON command update
    - Converts Nano IO / servo commands to JSON format
    - Every command returns a newline-terminated response
    - Preserves existing pin assignments and gripper current backoff behavior

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
// Existing design uses A0-A6 for servo headers.
// Note: On some classic Nano boards A6/A7 are analog-input-only pins. This keeps
// the existing assignment, but servo 6 requires hardware that supports output on A6.
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
// -----------------------------------------------------------------------------
const int Input2 = 2;
const int Input3 = 3;
const int Input4 = 4;
const int Input5 = 5;
const int Input6 = 6;
const int Input7 = 7;

const int Output8 = 8;
const int Output9 = 9;
const int Output10 = 10;
const int Output11 = 11;
const int Output12 = 12;
const int Output13 = 13;

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
bool nanoStopRequested = false;

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

void sendError(const char *message) {
  Serial.print(F("Error: "));
  Serial.println(message);
}

bool isValidInputPin(int pin) {
  return pin >= 2 && pin <= 7;
}

bool isValidOutputPin(int pin) {
  return pin >= 8 && pin <= 13;
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
    Serial.println(F("{\"board\":\"AR4 Nano IO\",\"firmware\":\"2.0\"}"));
    return;
  }

  // ---------------------------------------------------------------------------
  // test_gripper_amps
  // {"cmd":"test_gripper_amps"}
  // Returns current in amps to keep the existing HMI behavior simple.
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
  // {"cmd":"set_output","pin":8,"state":1}
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

    nanoStopRequested = false;
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
            nanoStopRequested = true;
            break;
          }
        }
      }

      delay(10);
    }

    if (nanoStopRequested) {
      Serial.println(F("Nano Stopped"));
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
    nanoStopRequested = true;
    Serial.println(F("Nano Inactive Stopped"));
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

  pinMode(Input2, INPUT_PULLUP);
  pinMode(Input3, INPUT_PULLUP);
  pinMode(Input4, INPUT_PULLUP);
  pinMode(Input5, INPUT_PULLUP);
  pinMode(Input6, INPUT_PULLUP);
  pinMode(Input7, INPUT_PULLUP);

  pinMode(Output8, OUTPUT);
  pinMode(Output9, OUTPUT);
  pinMode(Output10, OUTPUT);
  pinMode(Output11, OUTPUT);
  pinMode(Output12, OUTPUT);
  pinMode(Output13, OUTPUT);

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
