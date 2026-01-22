/*
 * ============================================
 * ARDUINO SOCCER PINBALL GAME
 * ============================================
 *
 * HARDWARE SETUP:
 * - 4 Servo Motors (SG90 blue servos)
 * - 2 Ultrasonic Sensors (HC-SR04)
 * - 2 Push Buttons
 *
 * CONFIGURATION:
 * LEFT SIDE:
 *   - Bottom: Flipper servo
 *   - Top: Scanner servo with ultrasonic mounted on it
 * RIGHT SIDE:
 *   - Bottom: Flipper servo
 *   - Top: Scanner servo with ultrasonic mounted on it
 *
 * SERVO WIRE COLORS:
 *   Yellow = Signal (to Arduino pin)
 *   Red = Power (5V)
 *   Brown = Ground (GND)
 */

#include <Servo.h>

// ============================================
// PIN DEFINITIONS
// ============================================

// BUTTONS
const int BUTTON_LEFT = 2;   // Left flipper button
const int BUTTON_RIGHT = 3;  // Right flipper button

// FLIPPER SERVOS
const int SERVO_FLIPPER_LEFT = 9;   // Bottom left servo
const int SERVO_FLIPPER_RIGHT = 10; // Bottom right servo

// SCANNER SERVOS (rotate ultrasonic sensors)
const int SERVO_SCANNER_LEFT = 11;  // Top left servo
const int SERVO_SCANNER_RIGHT = 5;  // Top right servo

// ULTRASONIC SENSOR 1 (Left goal - mounted on left scanner servo)
const int TRIG_LEFT = 4;
const int ECHO_LEFT = 6;

// ULTRASONIC SENSOR 2 (Right goal - mounted on right scanner servo)
const int TRIG_RIGHT = 7;
const int ECHO_RIGHT = 8;

// OPTIONAL: Buzzer
const int BUZZER_PIN = 13;

// ============================================
// SERVO OBJECTS
// ============================================
Servo flipperLeft;
Servo flipperRight;
Servo scannerLeft;
Servo scannerRight;

// ============================================
// GAME VARIABLES
// ============================================

// Scores
int scorePlayer1 = 0;  // Left player
int scorePlayer2 = 0;  // Right player

// CALIBRATED REST POSITIONS (from your testing!)
const int D9_REST = 20;    // Bottom left servo
const int D10_REST = 160;  // Bottom right servo
const int D11_REST = 150;  // Top left servo
const int D5_REST = 80;    // Top right servo

// Goal detection
const int GOAL_DISTANCE_CM = 20;
bool goalLeftTriggered = false;
bool goalRightTriggered = false;

// Timing for goal cooldown
unsigned long lastGoalLeft = 0;
unsigned long lastGoalRight = 0;
const unsigned long GOAL_COOLDOWN = 3000; // 3 seconds

// Button state
bool buttonLeftPressed = false;
bool buttonRightPressed = false;

// ============================================
// SETUP
// ============================================
void setup() {
  // Serial for debugging
  Serial.begin(9600);
  Serial.println(F("========================================"));
  Serial.println(F("   ARDUINO SOCCER PINBALL GAME"));
  Serial.println(F("========================================"));
  Serial.println(F("CALIBRATED REST POSITIONS:"));
  Serial.println(F("  D9:  20°  | D10: 160°"));
  Serial.println(F("  D11: 150° | D5:  80°"));
  Serial.println(F(""));
  Serial.println(F("LEFT button = Open D9 + D10"));
  Serial.println(F("RIGHT button = Open D5 + D11"));
  Serial.println(F("First to 5 goals wins!"));
  Serial.println(F("========================================\n"));
 
  // Button pins
  pinMode(BUTTON_LEFT, INPUT_PULLUP);
  pinMode(BUTTON_RIGHT, INPUT_PULLUP);
 
  // Ultrasonic pins
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
 
  // Buzzer (optional)
  pinMode(BUZZER_PIN, OUTPUT);
 
  // Attach servos
  flipperLeft.attach(SERVO_FLIPPER_LEFT);
  flipperRight.attach(SERVO_FLIPPER_RIGHT);
  scannerLeft.attach(SERVO_SCANNER_LEFT);
  scannerRight.attach(SERVO_SCANNER_RIGHT);
 
  // Set initial servo positions - CALIBRATED REST POSITIONS
  flipperLeft.write(D9_REST);    // D9 - 50° (calibrated)
  flipperRight.write(D10_REST);  // D10 - 130° (calibrated)
  scannerLeft.write(D11_REST);   // D11 - 150° (calibrated)
  scannerRight.write(D5_REST);   // D5 - 80° (calibrated)
 
  // Startup beep
  tone(BUZZER_PIN, 1000, 200);
  delay(250);
  tone(BUZZER_PIN, 1200, 200);
 
  delay(1000);
  Serial.println(F("Game started! Press any button to play!\n"));
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
  // Handle button inputs
  handleButtons();
 
  // Check for goals
  checkGoals();
 
  delay(10);
}

// ============================================
// BUTTON HANDLING
// ============================================
void handleButtons() {
  // Check if RIGHT button is pressed
  if (digitalRead(BUTTON_RIGHT) == LOW) {
    // RIGHT button controls D11 and D5 - OPEN from REST
    scannerLeft.write(D11_REST+225);    
    scannerRight.write(D5_REST-225);    
    buttonRightPressed = true;
  } else {
    // RIGHT button released - return to CALIBRATED REST
    scannerLeft.write(D11_REST);    
    scannerRight.write(D5_REST);    
    buttonRightPressed = false;
  }
 
  // Check if LEFT button is pressed
  if (digitalRead(BUTTON_LEFT) == LOW) {
    // LEFT button controls D9 and D10 - OPEN from REST
    flipperLeft.write(180-D9_REST);    
    flipperRight.write(180-D10_REST);  
    buttonLeftPressed = true;
  } else {
    // LEFT button released - return to CALIBRATED REST
    flipperLeft.write(D9_REST);    
    flipperRight.write(D10_REST);  
    buttonLeftPressed = false;
  }
}


// ============================================
// GOAL DETECTION
// ============================================
void checkGoals() {
  unsigned long currentTime = millis();
 
  // Check left goal
  if (currentTime - lastGoalLeft > GOAL_COOLDOWN) {
    float distLeft = measureDistance(TRIG_LEFT, ECHO_LEFT);
   
    if (distLeft > 0 && distLeft < GOAL_DISTANCE_CM) {
      if (!goalLeftTriggered) {
        scorePlayer2++;  // Player 2 scores on left goal
        goalLeftTriggered = true;
        lastGoalLeft = currentTime;
        announceGoal(2);
      }
    } else {
      goalLeftTriggered = false;
    }
  }
 
  // Check right goal
  if (currentTime - lastGoalRight > GOAL_COOLDOWN) {
    float distRight = measureDistance(TRIG_RIGHT, ECHO_RIGHT);
   
    if (distRight > 0 && distRight < GOAL_DISTANCE_CM) {
      if (!goalRightTriggered) {
        scorePlayer1++;  // Player 1 scores on right goal
        goalRightTriggered = true;
        lastGoalRight = currentTime;
        announceGoal(1);
      }
    } else {
      goalRightTriggered = false;
    }
  }
}

// ============================================
// ULTRASONIC DISTANCE MEASUREMENT
// ============================================
float measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
 
  long duration = pulseIn(echoPin, HIGH, 30000);
  float distance = (duration * 0.0343) / 2;
 
  return distance;
}

// ============================================
// GOAL ANNOUNCEMENT
// ============================================
void announceGoal(int player) {
  Serial.println(F("\n*** GOAL!!! ***"));
  Serial.print(F("Player "));
  Serial.print(player);
  Serial.println(F(" scores!"));
  Serial.print(F("Score: Player 1 = "));
  Serial.print(scorePlayer1);
  Serial.print(F(" | Player 2 = "));
  Serial.println(scorePlayer2);
  Serial.println();
 
  // Goal sound
  for (int i = 0; i < 3; i++) {
    tone(BUZZER_PIN, 1500, 100);
    delay(150);
  }
 
  // Check for winner
  if (scorePlayer1 >= 5) {
    announceWinner(1);
  } else if (scorePlayer2 >= 5) {
    announceWinner(2);
  }
}

// ============================================
// WINNER ANNOUNCEMENT
// ============================================
void announceWinner(int player) {
  Serial.println(F("\n========================================"));
  Serial.print(F("       PLAYER "));
  Serial.print(player);
  Serial.println(F(" WINS!!!"));
  Serial.println(F("========================================\n"));
 
  // Victory melody
  int melody[] = {523, 587, 659, 698, 784, 880, 988, 1047};
  for (int i = 0; i < 8; i++) {
    tone(BUZZER_PIN, melody[i], 150);
    delay(170);
  }
 
  // Reset scores
  scorePlayer1 = 0;
  scorePlayer2 = 0;
 
  delay(2000);
  Serial.println(F("New game starting...\n"));
}
