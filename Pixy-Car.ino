#include <Pixy2.h>

Pixy2 pixy;

// Motor pins configuration
const int enableLeft = 10;  // Enable pin for left motor (PWM)
const int in1 = 9;          // IN1 pin for left motor direction
const int in2 = 8;          // IN2 pin for left motor direction
const int enableRight = 5;  // Enable pin for right motor (PWM)
const int in3 = 7;          // IN3 pin for right motor direction
const int in4 = 6;          // IN4 pin for right motor direction

// Camera parameters
const int PIXY_CENTER_X = 160;  // Center X of Pixy2 camera (320x200 resolution)
const int FRAME_WIDTH = 320;    // Width of Pixy2 frame

// Control parameters - ADJUST THESE FOR YOUR ROBOT
const int SLOW_SPEED = 90;      // Slow movement speed (0-255)
const int NORMAL_SPEED = 100;   // Normal movement speed (0-255)
const int TURN_SPEED = 80;      // Speed for turning (0-255)
const int SCAN_SPEED = 80;      // Speed for scanning/searching (0-255)

// Tracking parameters
const int CENTER_MARGIN = 30;   // Margin around center to consider object centered
const int STOP_SIZE = 4000;     // Object size to stop at (width*height)
const int MIN_VALID_SIZE = 100; // Minimum size to consider a valid detection

// State management variables
enum RobotState {
  SEARCHING,   // Looking for the object
  TRACKING,    // Object found, tracking it
  APPROACHING, // Object centered, moving toward it
  STOPPED      // Object reached, stopped
};

RobotState currentState = SEARCHING;
unsigned long lastStateChangeTime = 0;
unsigned long lastDetectionTime = 0;
int scanDirection = 1;  // 1 = clockwise, -1 = counterclockwise
int searchStage = 0;    // For multi-step search pattern

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Pixy2 Alternative Tracking...");
  
  // Initialize Pixy2
  pixy.init();
  //pixy.setLamp(1, 1); // Turn on both upper (white) and lower (RGB) LEDs
  
  // Configure motor pins
  pinMode(enableLeft, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enableRight, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // Ensure the robot is stopped at startup
  stopMotors();
  
  // Brief delay for initialization
  delay(1000);
}

void loop() {
  // Get blocks from Pixy2
  int blockCount = pixy.ccc.getBlocks();
  bool targetFound = false;
  int targetX = 0;
  int targetSize = 0;
  
  // Process detected blocks
  if (blockCount > 0) {
    // Look for the largest block with signature 1 (assumed to be our target)
    int largestBlockIndex = -1;
    int largestBlockSize = 0;
    
    for (int i = 0; i < blockCount; i++) {
      if (pixy.ccc.blocks[i].m_signature == 1) {
        int currentSize = pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height;
        if (currentSize > largestBlockSize && currentSize >= MIN_VALID_SIZE) {
          largestBlockSize = currentSize;
          largestBlockIndex = i;
        }
      }
    }
    
    // If we found a valid target
    if (largestBlockIndex >= 0) {
      targetFound = true;
      targetX = pixy.ccc.blocks[largestBlockIndex].m_x;
      targetSize = largestBlockSize;
      lastDetectionTime = millis();
      
      // Debug info
      Serial.print("Target: X=");
      Serial.print(targetX);
      Serial.print(" Size=");
      Serial.print(targetSize);
      Serial.print(" State=");
    }
  }
  
  // State machine for robot behavior
  switch (currentState) {
    case SEARCHING:
      if (targetFound) {
        // Object found, start tracking
        changeState(TRACKING);
      } else {
        // Continue searching pattern
        searchForTarget();
      }
      break;
      
    case TRACKING:
      if (!targetFound) {
        // Lost the target, go back to searching
        if (millis() - lastDetectionTime > 1000) {
          changeState(SEARCHING);
        }
      } else if (targetSize > STOP_SIZE) {
        // Target is close enough to stop
        changeState(STOPPED);
      } else if (abs(targetX - PIXY_CENTER_X) <= CENTER_MARGIN) {
        // Target is centered, start approaching
        changeState(APPROACHING);
      } else {
        // Turn to center the target
        turnTowardTarget(targetX);
      }
      break;
      
    case APPROACHING:
      if (!targetFound) {
        // Lost the target while approaching
        if (millis() - lastDetectionTime > 500) {
          changeState(SEARCHING);
        }
      } else if (targetSize > STOP_SIZE) {
        // Target is close enough to stop
        changeState(STOPPED);
      } else if (abs(targetX - PIXY_CENTER_X) > CENTER_MARGIN) {
        // Target moved off-center, go back to tracking
        changeState(TRACKING);
      } else {
        // Move forward toward the target
        approachTarget(targetX, targetSize);
      }
      break;
      
    case STOPPED:
      if (!targetFound || targetSize < STOP_SIZE * 0.8) {
        // Target moved away or lost, resume searching
        changeState(SEARCHING);
      } else {
        // Stay stopped
        stopMotors();
        Serial.println("STOPPED");
      }
      break;
  }
  
  delay(20); // Small delay for stability
}

// Change robot state and record the time
void changeState(RobotState newState) {
  currentState = newState;
  lastStateChangeTime = millis();
  
  // Print state change for debugging
  Serial.print("State changed to: ");
  switch (newState) {
    case SEARCHING: Serial.println("SEARCHING"); break;
    case TRACKING: Serial.println("TRACKING"); break;
    case APPROACHING: Serial.println("APPROACHING"); break;
    case STOPPED: Serial.println("STOPPED"); break;
  }
  
  // Reset search pattern when entering SEARCHING state
  if (newState == SEARCHING) {
    searchStage = 0;
  }
}

// Search for the target using a pattern
void searchForTarget() {
  unsigned long currentTime = millis();
  unsigned long stateTime = currentTime - lastStateChangeTime;
  
  // Implement a multi-stage search pattern
  switch (searchStage) {
    case 0: // First rotate in place to scan 360 degrees
      if (stateTime < 4000) {
        // Rotate in place
        setMotors(-SCAN_SPEED, SCAN_SPEED);
        Serial.println("Search: Rotating 360");
      } else {
        // Move to next search stage
        searchStage = 1;
        lastStateChangeTime = currentTime;
        stopMotors();
        delay(300); // Brief pause between stages
      }
      break;
      
    case 1: // Move forward briefly then scan again
      if (stateTime < 1000) {
        // Move forward
        setMotors(NORMAL_SPEED, NORMAL_SPEED);
        Serial.println("Search: Moving forward");
      } else if (stateTime < 5000) {
        // Rotate in opposite direction
        scanDirection = -scanDirection;
        setMotors(SCAN_SPEED * scanDirection, -SCAN_SPEED * scanDirection);
        Serial.println("Search: Rotating opposite");
      } else {
        // Reset search pattern
        searchStage = 0;
        lastStateChangeTime = currentTime;
        stopMotors();
        delay(300); // Brief pause between cycles
      }
      break;
  }
}

// Turn to center the target in view
void turnTowardTarget(int targetX) {
  // Calculate how far off center the target is
  int errorX = targetX - PIXY_CENTER_X;
  
  // Determine turn direction and speed based on error
  if (errorX > CENTER_MARGIN) {
    // Target is to the right, turn LEFT to center it
    // Right motor forward, left motor backward
    setMotors(-TURN_SPEED, TURN_SPEED);
    Serial.println("Turning LEFT");
  } else if (errorX < -CENTER_MARGIN) {
    // Target is to the left, turn RIGHT to center it
    // Left motor forward, right motor backward
    setMotors(TURN_SPEED, -TURN_SPEED);
    Serial.println("Turning RIGHT");
  } else {
    // Target is roughly centered, stop turning
    stopMotors();
    Serial.println("Centered");
  }
}

// Move toward the centered target
void approachTarget(int targetX, int targetSize) {
  // Calculate approach speed based on distance (size)
  int approachSpeed = map(targetSize, MIN_VALID_SIZE, STOP_SIZE, NORMAL_SPEED, SLOW_SPEED);
  approachSpeed = constrain(approachSpeed, SLOW_SPEED, NORMAL_SPEED);
  
  // Small steering adjustment to keep centered while approaching
  int errorX = targetX - PIXY_CENTER_X;
  int steeringAdjustment = map(errorX, -CENTER_MARGIN, CENTER_MARGIN, -20, 20);
  
  int leftSpeed = approachSpeed - steeringAdjustment;
  int rightSpeed = approachSpeed + steeringAdjustment;
  
  // Ensure minimum motor speed
  leftSpeed = constrain(leftSpeed, SLOW_SPEED, NORMAL_SPEED);
  rightSpeed = constrain(rightSpeed, SLOW_SPEED, NORMAL_SPEED);
  
  setMotors(leftSpeed, rightSpeed);
  Serial.print("Approaching: L=");
  Serial.print(leftSpeed);
  Serial.print(" R=");
  Serial.println(rightSpeed);
}

// Set motor speeds and directions
void setMotors(int leftSpeed, int rightSpeed) {
  // Set left motor direction
  if (leftSpeed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (leftSpeed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  
  // Set right motor direction
  if (rightSpeed > 0) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } else if (rightSpeed < 0) {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  } else {
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }
  
  // Set motor speeds
  analogWrite(enableLeft, abs(leftSpeed));
  analogWrite(enableRight, abs(rightSpeed));
}

// Stop both motors
void stopMotors() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enableLeft, 0);
  analogWrite(enableRight, 0);
}