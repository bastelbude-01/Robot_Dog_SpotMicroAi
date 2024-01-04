#include <Arduino.h>
#include <Wire.h>
#include <FastLED.h>


#define NUM_LEDS 10
#define DATA_PIN 23

CRGB leds[10];



// Define pins for the ultrasonic sensor
//Front Right
const int e_FR = 36;
const int t_FR = 19;
//Front Left
const int e_FL = 39;
const int t_FL = 18;
// Rechte Seite
const int e_RH = 34;
const int t_RH = 5;
// Linke Seite
const int e_LH = 35;
const int t_LH = 17;
// Heck
const int e_HE = 32;
const int t_HE = 16;

// Function prototype for reading sensor data
float readSensorData();

void setup() {
  // Begin serial communication at 115200 baud rate
  Serial.begin(115200);

  FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS);

  // Set echoPin as input and trigPin as output
  pinMode(e_FR, INPUT);
  pinMode(t_FR, OUTPUT);

  pinMode(e_FL, INPUT);
  pinMode(t_FL, OUTPUT);

  pinMode(e_RH, INPUT);
  pinMode(t_RH, OUTPUT);

  pinMode(e_LH, INPUT);
  pinMode(t_LH, OUTPUT);

  pinMode(e_HE, INPUT);
  pinMode(t_HE, OUTPUT);
  // Print sensor information to the serial monitor
  Serial.println("Ultrasonic sensor:");
}

void setLeftGroupColor(CRGB *ledArray, int r, int g, int b ) {
  for (int i = 0; i < 4; i++) {
    ledArray[i].setRGB(r, g, b);
  }
}
void setRightGroupColor(CRGB *ledArray, int r, int g, int b ) {
  for (int i = 5; i < 10; i++) {
    ledArray[i].setRGB(r, g, b);
  }
}

void optic(){
  float distance_FR = readSensorData(e_FR, t_FR);
  float distance_FL = readSensorData(e_FL, t_FL);
  float distance_RH = readSensorData(e_RH, t_RH);
  float distance_LH = readSensorData(e_LH, t_LH);
  float distance_HE = readSensorData(e_HE, t_HE);

  if (distance_FR < 15)
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      setLeftGroupColor(leds, 255, 0, 0); // RED
      setRightGroupColor(leds, 0, 255, 0); // RED
    }
  }
  if (distance_FL < 15)
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      setLeftGroupColor(leds, 0, 255, 0); // RED
      setRightGroupColor(leds, 255, 0, 0); // RED
    }
  }
  if (distance_RH < 15)
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      setLeftGroupColor(leds, 0, 0, 255); 
      setRightGroupColor(leds, 0, 0, 255);
    }
  }
  if (distance_LH < 15)
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      setLeftGroupColor(leds, 0, 255, 255); // RED
      setRightGroupColor(leds, 0, 255, 255); // RED
    }
  }
  if (distance_FR < 30 && distance_FL < 30  )
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      setLeftGroupColor(leds, 0, 255, 0); // RED
      setRightGroupColor(leds, 0, 255, 0); // RED
    }
  }
  FastLED.setBrightness(20);
  FastLED.show();
  }



void loop() {

  optic();

  // Read distance from the ultrasonic sensor
  /*

  Serial.print("FR : ");
  Serial.print(distance_FR);
  Serial.print(" cm; ");
  Serial.print("FL : ");
  Serial.print(distance_FL);
  Serial.print(" cm; ");
  Serial.print("RH : ");
  Serial.print(distance_RH);
  Serial.print(" cm; ");
  Serial.print("LH : ");
  Serial.print(distance_LH);
  Serial.print(" cm; ");
  Serial.print("HECK : ");
  Serial.print(distance_HE);
  Serial.println(" cm");
  Serial.println("");

  
  // Delay between readings
  delay(200);
  */
  
}

// Function to read data from the ultrasonic sensor
float readSensorData(int echo, int trigger) {
  // Trigger a low signal before sending a high signal
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  // Send a 10-microsecond high signal to the trigPin
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  // Return to low signal
  digitalWrite(trigger, LOW);

  // Measure the duration of the high signal on the echoPin
  unsigned long microsecond = pulseIn(echo, HIGH);

  // Calculate the distance using the speed of sound (29.00µs per centimeter)
  float distance = microsecond / 29.00 / 2;

  // Return the calculated distance
  return distance;
}
