
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

// Variables for Servo Motor positions (expand as required)
int pwm0, pwm1, pwm2, pwm3, pwm4, pwm5, pwm6, pwm7, pwm8, pwm9, pwm10, pwm11, pwm_;


void setup() {

  int start = map(90, 0, 180, SERVOMIN, SERVOMAX);
  pca9685.setPWM(SER0, 0, start);
  pca9685.setPWM(SER1, 0, start);
  pca9685.setPWM(SER2, 0, start);
  pca9685.setPWM(SER3, 0, start);
  pca9685.setPWM(SER4, 0, start);
  pca9685.setPWM(SER5, 0, start);
  pca9685.setPWM(SER6, 0, start);
  pca9685.setPWM(SER7, 0, start);
  pca9685.setPWM(SER8, 0, start);
  pca9685.setPWM(SER9, 0, start);
  pca9685.setPWM(SER10, 0, start);
  pca9685.setPWM(SER11, 0, start);

  // Serial monitor setup
  Serial.begin(115200);

  // Print to monitor
  Serial.println("PCA9685 Servo Test");

  // Initialize PCA9685
  pca9685.begin();

  // Set PWM Frequency to 50Hz
  pca9685.setPWMFreq(50);

}

void sweep() {


  // Move Motor from 0 to 180 degrees
  for (int posDegrees = 60; posDegrees <= 115; posDegrees++) {

    // Determine PWM pulse width
    pwm_ = map(posDegrees, 0, 180, SERVOMIN, SERVOMAX);
    // Write to PCA9685
    pca9685.setPWM(SER0, 0, pwm_);
    pca9685.setPWM(SER3, 0, pwm_);
    pca9685.setPWM(SER6, 0, pwm_);
    pca9685.setPWM(SER9, 0, pwm_);
    // Print to serial monitor
    Serial.print("Motor = ");
    Serial.println(posDegrees);
    delay(30);
  }



  // Move Motor 0 from 180 to 0 degrees
  for (int posDegrees = 115; posDegrees >= 60; posDegrees--) {

    // Determine PWM pulse width
    pwm_ = map(posDegrees, 0, 180, SERVOMIN, SERVOMAX);
    // Write to PCA9685
    pca9685.setPWM(SER0, 0, pwm_);
    pca9685.setPWM(SER3, 0, pwm_);
    pca9685.setPWM(SER6, 0, pwm_);
    pca9685.setPWM(SER9, 0, pwm_);
    // Print to serial monitor
    Serial.print("Motor = ");
    Serial.println(posDegrees);
    delay(30);
  }
}


void loop() {
  sweep();
  


}
