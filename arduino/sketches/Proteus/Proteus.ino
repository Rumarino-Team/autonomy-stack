#include <Arduino.h>
#include <Servo.h>

#define PULSE_WIDTH_MIN 1100
#define PULSE_WIDTH_MAX 1900
#define PULSE_WIDTH_NEUTRAL 1500
#define PINS_COUNT 6

const int PINS_PROTEUS[PINS_COUNT] = {7, 6, 3, 2, 4, 5};
Servo servos[PINS_COUNT];

String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(115200);
  inputString.reserve(50);

  Serial.println("Initializing thrusters...");
  for (int i = 0; i < PINS_COUNT; i += 1){
      servos[i].attach(PINS_PROTEUS[i]);
      servos[i].writeMicroseconds(PULSE_WIDTH_NEUTRAL);  // This sets the thrusters output force to 0 lbf
      Serial.print("Initialized thruster ");
      Serial.print(i);
      Serial.print(" (pin ");
      Serial.print(PINS_PROTEUS[i]);
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
    int pulse_width = inputString.substring(colon_pos + 1).toInt();

    if (pulse_width < PULSE_WIDTH_MIN) {
      pulse_width = PULSE_WIDTH_MIN;
    } else if (pulse_width > PULSE_WIDTH_MAX) {
      pulse_width = PULSE_WIDTH_MAX;
    }

    int pin = inputString.substring(1, colon_pos).toInt();

    Serial.print("pin = ");
    Serial.println(pin);
    Serial.print("pulse_width = ");
    Serial.println(pulse_width);

    int pin_index = -1;
    for (int i = 0; i < PINS_COUNT; i += 1) {
      if (PINS_PROTEUS[i] == pin) {
        pin_index = i;
        break;
      }
    }
    if (pin_index == -1) {
      Serial.println("Invalid Pin");
    } else {
      servos[pin_index].writeMicroseconds(pulse_width);
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
