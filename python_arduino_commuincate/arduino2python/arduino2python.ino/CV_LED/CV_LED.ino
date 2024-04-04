// Communication buffers lengths
#define MAX_BUFF_LEN        255 
#define CMD_BUFF_LEN        6

// States of the LED
#define ON_STATE            1
#define OFF_STATE           0

// LED configs
#define LED_PIN             26
#define DEFAULT_DELAY       1000
// Globals
char c; // IN char
char str[CMD_BUFF_LEN];
uint8_t idx = 0; // Reading index

uint8_t state = OFF_STATE; // Default state
int delay_t = DEFAULT_DELAY; // Default blinking delay
unsigned long prev_time;

// Function prototypes
uint8_t interpret(char*);

void setup() {
  // Config serial port
  Serial.begin(115200);
  // Config LED
  pinMode(LED_PIN, OUTPUT);

  // 
  prev_time = millis();
}

void loop() {
  // Parse incoming command
  if(Serial.available() > 0){ // There's a command
    
    c = Serial.read(); // Read one byte
    
    if(c != '\n'){ // Still reading
      str[idx++] = c; // Parse the string byte (char) by byte
    }
    else{ // Done reading
      str[idx] = '\0'; // Convert it to a string
      
      // Determine nature of the command
      state = interpret(str);
      
      // strtol(char*, Ref pointer, Base[Decimal-->10, Hex-->16, ...])
      //delay_t = strtol(str+1, NULL, 10); // str+1 --> exclude the first char
      /* Some input checking could've been done here (like b15f2 --> invalid) */

      // Reset reading index 
      idx = 0;
    }
  }
  else{ // No input from commander
    state = OFF_STATE;
  }

  // Main state machine
  switch (state) {
    case ON_STATE:
      digitalWrite(LED_PIN, HIGH);
      break;

    case OFF_STATE:
      digitalWrite(LED_PIN, LOW);
      break;

    default:
      // Handle other states if needed
      break;
  }
}

uint8_t interpret(char* cmd) {
  if (strcmp(cmd, "sweep") == 0) {
    Serial.println("Received command: sweep");
    return ON_STATE;
  } else if (strcmp(cmd, "stop") == 0) {
    Serial.println("Received command: stop");
    return OFF_STATE;
  } else {
    Serial.println("UNKNOWN");
    return state; // Don't change anything
  }
}


