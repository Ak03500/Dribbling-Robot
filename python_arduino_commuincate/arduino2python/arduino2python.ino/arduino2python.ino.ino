#include <ESP32Servo.h>

// Communication buffers lengths
#define MAX_BUFF_LEN        255
#define CMD_BUFF_LEN        6

// States of the Servo
#define ON_STATE            4
#define OFF_STATE           5
#define CUSTOM_ANGLE_STATE  6 // New state for custom angle input

// Servo configuration
#define SERVO_PIN           25 // Use the appropriate pin for your setup

// Globals
char c; // IN char
char str[CMD_BUFF_LEN];
uint8_t idx = 0; // Reading index

uint8_t state = OFF_STATE; // Default state
Servo myServo;

// Function prototypes
uint8_t interpret(char*);

void setup() {
  // Config serial port
  Serial.begin(115200);

  // Config Servo
  myServo.attach(SERVO_PIN);
}

void loop() {

  // Parse incoming command
  if (Serial.available() > 0) { // There's a command

    c = Serial.read(); // Read one byte

    if (c != '\n') { // Still reading
      str[idx++] = c; // Parse the string byte (char) by byte
    } else { // Done reading
      str[idx] = '\0'; // Convert it to a string

      // Determine nature of the command
      state = interpret(str);

      // Reset reading index
      idx = 0;
    }
  }

  // Main state machine
  int angle;
  switch (state) {
    case ON_STATE:
      myServo.write(180); // Move the servo to the maximum position (sweep)
      delay(2000); // Adjust the delay based on your requirements
      myServo.write(0); // Move the servo back to the minimum position
      delay(2000); // Adjust the delay based on your requirements
      break;

    case OFF_STATE:
      myServo.write(90); // Stop the servo by moving it to the center position
      break;

    case CUSTOM_ANGLE_STATE:
      // Set the servo angle based on the custom input
      angle = atoi(str);
      if (angle >= 0 && angle <= 180) {
        myServo.write(angle);
      } else {
        Serial.println("Invalid angle input");
      }
      state = OFF_STATE; // Change state back to OFF_STATE after setting the custom angle
      break;

    default:
      // Handle other states if needed
      break;
  }
}

uint8_t interpret(char* cmd) {
  if (strcmp(cmd, "on") == 0) {
    return ON_STATE;
  } else if (strcmp(cmd, "off") == 0) {
    return OFF_STATE;
  } else if (isdigit(cmd[0])) { // Check if the first character is a digit
    return CUSTOM_ANGLE_STATE;
  } else {
    Serial.println("UNKNOWN");
    return state; // Don't change anything
  }
}
