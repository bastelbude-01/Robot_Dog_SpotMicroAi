#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <FastLED.h>


Adafruit_MPU6050 mpu;
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

float xtreshhold = 0.0;
float ytreshhold = 0.0;
float ztreshhold = 0.0;

#define NUM_LEDS 10
#define DATA_PIN 23

CRGB leds[NUM_LEDS];

void rgb_angle() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  if (a.acceleration.x <= -2.5) // DOWN
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i].setRGB(255, 0, 0); // RED
    }
  }

  else if (a.acceleration.x >= 2.5) // UP
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i].setRGB(0, 255, 0); // GREEN
    }
  }

  else if (a.acceleration.y <= -2.5) // LEFT
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i].setRGB(0, 0, 255); // BLUE
    }
  }

  else if (a.acceleration.y >= 2.5) // RIGHT
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i].setRGB(255, 255, 0); // YELLOW
    }
  }

  else if (g.gyro.z <= -2.5) // FRONT
  {

    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i].setRGB(0, 255, 255); // cyan
    }
  }

  FastLED.setBrightness(20);
  FastLED.show();
}

void setup() {

  Serial.begin(115200);
  // while (!Serial);
  Serial.println("MPU6050 OLED demo");

  if (!mpu.begin()) {
    Serial.println("Sensor init failed");
    while (1)
      yield();
  }
  Serial.println("Found a MPU-6050 sensor");

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }
  FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS);
  display.display();
  delay(250); // Pause for 2 seconds
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setRotation(0);
}


void loop() {
  rgb_angle();

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  display.clearDisplay();
  display.setCursor(0, 0);

  Serial.print("Accelerometer ");
  Serial.print("X: ");
  Serial.print(a.acceleration.x, 1);
  Serial.print(" m/s^2, ");
  Serial.print("Y: ");
  Serial.print(a.acceleration.y, 1);
  Serial.print(" m/s^2, ");
  Serial.print("Z: ");
  Serial.print(a.acceleration.z, 1);
  Serial.println(" m/s^2");

  display.println("Accelerometer - m/s^2");
  display.print(a.acceleration.x, 1);
  display.print(", ");
  display.print(a.acceleration.y, 1);
  display.print(", ");
  display.print(a.acceleration.z, 1);
  display.println("");

  Serial.print("Gyroscope ");
  Serial.print("X: ");
  Serial.print(g.gyro.x, 1);
  Serial.print(" rps, ");
  Serial.print("Y: ");
  Serial.print(g.gyro.y, 1);
  Serial.print(" rps, ");
  Serial.print("Z: ");
  Serial.print(g.gyro.z, 1);
  Serial.println(" rps");

  display.println("Gyroscope - rps");
  display.print(g.gyro.x, 1);
  display.print(", ");
  display.print(g.gyro.y, 1);
  display.print(", ");
  display.print(g.gyro.z, 1);
  display.println("");

  display.display();
  delay(10);
}
