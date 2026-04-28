#include <Arduino.h>
#include <Servo.h>

#define PULSE_WIDTH_MIN 1400
#define PULSE_WIDTH_MAX 1600
#define PULSE_WIDTH_NEUTRAL 1500
// original delta
// #define PULSE_WIDTH_DELTA 400

// reduced delta
#define PULSE_WIDTH_DELTA 100
#define PULSE_WIDTH_MIN PULSE_WIDTH_NEUTRAL - PULSE_WIDTH_DELTA
#define PULSE_WIDTH_MAX PULSE_WIDTH_NEUTRAL + PULSE_WIDTH_DELTA
#define PINS_COUNT 6

const int PINS[PINS_COUNT] = {7, 6, 3, 2, 4, 5};
Servo servos[PINS_COUNT];

String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(115200);
  inputString.reserve(50);

  Serial.println("Initializing thrusters...");
  for (int i = 0; i < PINS_COUNT; i += 1){
      servos[i].attach(PINS[i]);
      servos[i].writeMicroseconds(PULSE_WIDTH_NEUTRAL);  // This sets the thrusters output force to 0 lbf
      Serial.print("Initialized thruster ");
      Serial.print(i);
      Serial.print(" (pin ");
      Serial.print(PINS[i]);
      Serial.println(") to neutral position");
  }
  delay(5000);
  Serial.println("All thrusters initialized successfully");
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}

// Process commands received from serial
void processSerialCommands() {
  if (!stringComplete) {
    return;
  }

  Serial.print("Received command: ");
  Serial.println(inputString);

  if (inputString.length() < 3) {
    Serial.println("Too short");
    return;
  }
 
  int colon_pos = inputString.indexOf(':');
  if (colon_pos <= 0) {
    Serial.println("Invalid command format! Use format: T1:3, D:-2, etc.");
    return;
  }
  
  char cmd_type = inputString.charAt(0);
  switch (cmd_type) {
  case 'T': {
    int value = inputString.substring(colon_pos + 1).toFloat();
    // original from 1100 to 1900
    // float pulse_width = 1500 + 400 * value;

    float pulse_width = PULSE_WIDTH_NEUTRAL + PULSE_WIDTH_DELTA * value;

    if (pulse_width < PULSE_WIDTH_MIN) {
      pulse_width = PULSE_WIDTH_MIN;
    } else if (pulse_width > PULSE_WIDTH_MAX) {
      pulse_width = PULSE_WIDTH_MAX;
    }

    int thruster_index = inputString.substring(1, colon_pos).toInt();

    Serial.print("thruster_index = ");
    Serial.println(thruster_index);
    Serial.print("pin = ");
    Serial.println(PINS[thruster_index]);
    Serial.print("pulse_width = ");
    Serial.println(pulse_width);

    if (pin_index == -1) {
      Serial.println("Invalid Pin");
    } else {
      servos[thruster_index].writeMicroseconds(pulse_width);
    }

    break;
  }
  default:
    Serial.println("Unknown command type!");
    break;
  }

  // Reset for next command
  inputString = "";
  stringComplete = false;
}

void loop() {
  processSerialCommands();
}
