#include <ESP32Servo.h>

// Communication buffers lengths
#define MAX_BUFF_LEN        255 
#define CMD_BUFF_LEN        6

// States of the Servo
#define ON_STATE            4
#define OFF_STATE           5
#define CUSTOM_ANGLE_STATE  6 // New state for custom angle input
#define OFF_STATE_WAIT      7
// Servo configuration
#define SERVO_PIN           25 // Use the appropriate pin for your setup

// Globals
char c; // IN char
char str[CMD_BUFF_LEN];
uint8_t idx = 0; // Reading index

uint8_t state = OFF_STATE; // Default state
Servo myServo;
// Time tracking for non-blocking delay
unsigned long start_time;
unsigned long off_state_duration = 200; // Adjust the duration based on your requirements
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
  switch (state) {
    case ON_STATE:
      myServo.write(180); // Move the servo to the maximum position (sweep)
      delay(200); // Adjust the delay based on your requirements
      myServo.write(0); // Move the servo back to the minimum position
      delay(200); // Adjust the delay based on your requirements
      state = OFF_STATE; // Change state back to OFF_STATE after sweeping
      break;

     case OFF_STATE_WAIT:
      // Wait for a certain duration before moving to OFF_STATE
      if (millis() - start_time >= off_state_duration) {
        myServo.write(90); // Stop the servo by moving it to the center position
        state = OFF_STATE; // Change state to OFF_STATE after the delay
      }
      break;

    case OFF_STATE:
      // Do nothing, the servo is already stopped
      break;

    default:
      // Handle other states if needed
      break;
  }
}


uint8_t interpret(char* cmd) {
  if (strcmp(cmd, "sweep") == 0) {
    return ON_STATE;
  } else if (strcmp(cmd, "stop") == 0) {
    return OFF_STATE;
  }  else {
    Serial.println("UNKNOWN");
    return state; // Don't change anything
  }
}