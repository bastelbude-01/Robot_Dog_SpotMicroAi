
#include <Wire.h>

#include <Adafruit_PWMServoDriver.h>


// Creat object to represent PCA9685 at default I2C address
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);

// Define maximum and minimum number of "ticks" for the servo motors
// Range from 0 to 4095
// This determines the pulse width

#define SERVOMIN  80  // Minimum value
#define SERVOMAX  600  // Maximum value

// Define servo motor connections (expand as required)
#define SER0  0       // Schulter  FRS  FRONT RECHTS
#define SER1  1       // Knie      FRK
#define SER2  2       // Fuss      FRF

#define SER3  3       // Schulter  FLS  FRONT LINKS
#define SER4  4       // Knie      FLK
#define SER5  5       // Fuss      FLF

#define SER6  6       // Schulter  HRS  HECK RECHTS
#define SER7  7       // Knie      HRK
#define SER8  8       // Fuss      HRF

#define SER9  9       // Schulter  HLS  HECK LINKS
#define SER10  10     // Knie      HLK
#define SER11  11     // Fuss      HLF

// Variables for int without value
int angle_r, angle_l, pwm0, pwm1, pwm2, pwm3, pwm4, pwm5, pwm6, pwm7, pwm8, pwm9, pwm10, pwm11;


int angle_right(int r) {
  return angle_r = map( r, 0, 180, SERVOMIN, SERVOMAX);

}
int angle_left(int l) {
  return angle_l = map( l, 180, 0, SERVOMIN, SERVOMAX);

}

void setup() {

  // Serial monitor setup
  Serial.begin(115200);

  // Print to monitor
  Serial.println("PCA9685 Servo Test");

  // Initialize PCA9685
  pca9685.begin();

  // Set PWM Frequency to 50Hz
  pca9685.setPWMFreq(50);

  int schulter_r = angle_right(77);
  int schulter_l = angle_left(100);

  int knie_r = angle_right(70);
  int knie_l = angle_left(100);

  int fuss_r = angle_right(100);
  int fuss_l = angle_left(120);

  pca9685.setPWM(SER0, 0, schulter_r);
  pca9685.setPWM(SER3, 0, schulter_l);
  pca9685.setPWM(SER6, 0, schulter_r);
  pca9685.setPWM(SER9, 0, schulter_l);

  pca9685.setPWM(SER1, 0, knie_r);
  pca9685.setPWM(SER4, 0, knie_l);
  pca9685.setPWM(SER7, 0, knie_r);
  pca9685.setPWM(SER10, 0, knie_l);

  pca9685.setPWM(SER2, 0, fuss_r);
  pca9685.setPWM(SER5, 0, fuss_l);
  pca9685.setPWM(SER8, 0, fuss_r);
  pca9685.setPWM(SER11, 0, fuss_l);

}

void push_up() {


  // Move Motor 0 from 0 to 180 degrees
  for (int posDegrees = 75; posDegrees <= 105; posDegrees++) {

    // Determine PWM pulse width
    pwm1 = map(posDegrees, 0, 180, SERVOMIN, SERVOMAX);
    pwm4 = map(posDegrees, 0, 180, SERVOMIN, SERVOMAX);
    pwm7 = map(posDegrees, 0, 180, SERVOMIN, SERVOMAX);
    pwm10 = map(posDegrees, 0, 180, SERVOMIN, SERVOMAX);
    // Write to PCA9685
    pca9685.setPWM(SER1, 0, pwm1);
    pca9685.setPWM(SER4, 0, pwm4);
    pca9685.setPWM(SER7, 0, pwm7);
    pca9685.setPWM(SER10, 0, pwm10);
    delay(30);
  }

  // Move Motor 1 from 180 to 0 degrees
  for (int posDegrees = 105; posDegrees >= 75; posDegrees--) {

    // Determine PWM pulse width
    pwm1 = map(posDegrees, 0, 180, SERVOMIN, SERVOMAX);
    pwm4 = map(posDegrees, 0, 180, SERVOMIN, SERVOMAX);
    pwm7 = map(posDegrees, 0, 180, SERVOMIN, SERVOMAX);
    pwm10 = map(posDegrees, 0, 180, SERVOMIN, SERVOMAX);
    // Write to PCA9685
    pca9685.setPWM(SER1, 0, pwm1);
    pca9685.setPWM(SER4, 0, pwm4);
    pca9685.setPWM(SER7, 0, pwm7);
    pca9685.setPWM(SER10, 0, pwm10);
    delay(30);
  }
}

void push_() {


  // Move Motor 0 from 0 to 180 degrees
  for (int posDegrees = 75; posDegrees <= 105; posDegrees++) {

    // Determine PWM pulse width
    pwm2 = map(posDegrees, 0, 180, SERVOMIN, SERVOMAX);
    pwm5 = map(posDegrees, 180, 0, SERVOMIN, SERVOMAX);
    pwm8 = map(posDegrees, 0, 180, SERVOMIN, SERVOMAX);
    pwm11 = map(posDegrees, 180, 0, SERVOMIN, SERVOMAX);
    // Write to PCA9685
    pca9685.setPWM(SER2, 0, pwm2);
    pca9685.setPWM(SER5, 0, pwm5);
    pca9685.setPWM(SER8, 0, pwm8);
    pca9685.setPWM(SER11, 0, pwm11);
    delay(30);
  }

  // Move Motor 1 from 180 to 0 degrees
  for (int posDegrees = 105; posDegrees >= 75; posDegrees--) {

    // Determine PWM pulse width
    pwm2 = map(posDegrees, 0, 180, SERVOMIN, SERVOMAX);
    pwm5 = map(posDegrees, 180, 0, SERVOMIN, SERVOMAX);
    pwm8 = map(posDegrees, 0, 180, SERVOMIN, SERVOMAX);
    pwm11 = map(posDegrees, 180, 0, SERVOMIN, SERVOMAX);
    // Write to PCA9685
    pca9685.setPWM(SER2, 0, pwm2);
    pca9685.setPWM(SER5, 0, pwm5);
    pca9685.setPWM(SER8, 0, pwm8);
    pca9685.setPWM(SER11, 0, pwm11);
    delay(30);
  }
}

void sweep() {


  // Move Motor 0 from 0 to 180 degrees
  for (int posDegrees = 55; posDegrees <= 105; posDegrees++) {

    // Determine PWM pulse width
    pwm0 = map(posDegrees, 0, 180, SERVOMIN, SERVOMAX);
    pwm3 = map(posDegrees, 0, 180, SERVOMIN, SERVOMAX);
    pwm6 = map(posDegrees, 0, 180, SERVOMIN, SERVOMAX);
    pwm9 = map(posDegrees, 0, 180, SERVOMIN, SERVOMAX);
    // Write to PCA9685
    pca9685.setPWM(SER0, 0, pwm0);
    pca9685.setPWM(SER3, 0, pwm3);
    pca9685.setPWM(SER6, 0, pwm6);
    pca9685.setPWM(SER9, 0, pwm9);
    delay(30);
  }
  push_();
  push_up();

  // Move Motor 1 from 180 to 0 degrees
  for (int posDegrees = 105; posDegrees >= 55; posDegrees--) {

    // Determine PWM pulse width
    pwm0 = map(posDegrees, 0, 180, SERVOMIN, SERVOMAX);
    pwm3 = map(posDegrees, 0, 180, SERVOMIN, SERVOMAX);
    pwm6 = map(posDegrees, 0, 180, SERVOMIN, SERVOMAX);
    pwm9 = map(posDegrees, 0, 180, SERVOMIN, SERVOMAX);
    // Write to PCA9685
    pca9685.setPWM(SER0, 0, pwm0);
    pca9685.setPWM(SER3, 0, pwm3);
    pca9685.setPWM(SER6, 0, pwm6);
    pca9685.setPWM(SER9, 0, pwm9);
    delay(30);
  }
}

void loop() {
  
  Serial.print(" sweep ");
  sweep();

}
