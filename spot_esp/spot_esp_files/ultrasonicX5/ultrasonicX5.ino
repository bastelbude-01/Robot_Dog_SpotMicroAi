// Define pins for the ultrasonic sensor
//Front Right
const int e_FR = 36;
const int t_FR = 19;

const int e_FL = 39;
const int t_FL = 18;

const int e_RH = 34;
const int t_RH = 5;

const int e_LH = 35;
const int t_LH = 17;

const int e_HE = 32;
const int t_HE = 16;

// Function prototype for reading sensor data
float readSensorData();

void setup() {
  // Begin serial communication at 115200 baud rate
  Serial.begin(115200);
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

void loop() {
  // Read distance from the ultrasonic sensor
  float distance_FR = readSensorData(e_FR,t_FR);
  float distance_FL = readSensorData(e_FL,t_FL);
  float distance_RH = readSensorData(e_RH,t_RH);
  float distance_LH = readSensorData(e_LH,t_LH);
  float distance_HE = readSensorData(e_HE,t_HE);
  // Print the measured distance to the serial monitor
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
