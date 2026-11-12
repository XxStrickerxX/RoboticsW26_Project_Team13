// Team 13 - Robotics EDPT-1009 - Milestone 03
// Open-loop servo test: Arduino Uno + PCA9685 servo driver + 5 servos.
//
// What it does (once, after reset or power-on):
//   1. Moves every servo to HOME (90 deg = arm straight up), one after the other.
//   2. Moves each servo on its own: HOME -> HOME+RANGE -> HOME-RANGE -> HOME, slowly.
//   3. Prints every step on the Serial Monitor (9600 baud).
// Open loop: the servos get commands only; nothing is measured or fed back.
//
// Needs the library "Adafruit PWM Servo Driver Library" (Arduino IDE: Tools > Manage Libraries).

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ------------------------- settings (edit here) -------------------------
const int NUM_SERVOS = 5;
const int CHANNEL[NUM_SERVOS] = {0, 1, 2, 3, 4};          // PCA9685 channel of each servo
const char* NAME[NUM_SERVOS] = {
  "Joint 1 waist (MG996R)",
  "Joint 2 shoulder (MG996R)",
  "Joint 3 elbow (MG996R)",
  "Joint 4 gripper rotation (MG90S)",
  "Gripper clamp (MG90S)"
};

const int HOME_DEG = 90;        // 90 deg = straight-up pose (q = 0)
const int RANGE_DEG = 30;       // test moves HOME +/- RANGE (60 to 120 deg)
const int STEP_DELAY_MS = 20;   // 1 degree every 20 ms (slow, easy to see on video)

const int PULSE_MIN_US = 500;   // pulse width at 0 deg   (typical for MG996R / MG90S)
const int PULSE_MAX_US = 2500;  // pulse width at 180 deg
// -------------------------------------------------------------------------

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);   // default I2C address
int currentDeg[NUM_SERVOS];

// Send one servo to an angle (0..180 deg).
void writeServo(int i, int deg) {
  deg = constrain(deg, 0, 180);
  long pulseUs = map(deg, 0, 180, PULSE_MIN_US, PULSE_MAX_US);
  long ticks = pulseUs * 4096L / 20000L;     // 50 Hz -> 20000 us period, 4096 ticks
  pca.setPWM(CHANNEL[i], 0, ticks);
  currentDeg[i] = deg;
}

// Move one servo to the target angle, 1 degree at a time.
void moveSlowly(int i, int target) {
  Serial.print("  ");
  Serial.print(NAME[i]);
  Serial.print(": ");
  Serial.print(currentDeg[i]);
  Serial.print(" -> ");
  Serial.print(target);
  Serial.println(" deg");
  while (currentDeg[i] != target) {
    if (currentDeg[i] < target) {
      writeServo(i, currentDeg[i] + 1);
    } else {
      writeServo(i, currentDeg[i] - 1);
    }
    delay(STEP_DELAY_MS);
  }
}

void setup() {
  Serial.begin(9600);
  pca.begin();
  pca.setOscillatorFrequency(27000000);
  pca.setPWMFreq(50);                         // standard servo frequency
  delay(10);

  Serial.println("Team 13 - M3 open-loop servo test");
  Serial.println("Step 1: all servos to HOME (90 deg)");
  for (int i = 0; i < NUM_SERVOS; i++) {
    writeServo(i, HOME_DEG);
    Serial.print("  ");
    Serial.print(NAME[i]);
    Serial.println(" -> 90 deg");
    delay(500);                               // one at a time, to limit the current peak
  }
  delay(2000);

  Serial.println("Step 2: move each servo on its own");
  for (int i = 0; i < NUM_SERVOS; i++) {
    moveSlowly(i, HOME_DEG + RANGE_DEG);
    delay(500);
    moveSlowly(i, HOME_DEG - RANGE_DEG);
    delay(500);
    moveSlowly(i, HOME_DEG);
    delay(1000);
  }

  Serial.println("Test finished: all servos at HOME. Press RESET to run it again.");
}

void loop() {
  // nothing: the test runs once in setup()
}
