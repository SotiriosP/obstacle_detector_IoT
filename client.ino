/*
The system consists of an Arduino MKR1000 device serving as the client and an Arduino MKR1000 device acting like a server. 
The first board is part of a wearable technology attached to a knee or a wrist band. 
This wearable technology is equipped with two ultrasonic sensors, two corresponding vibration motor modules,
one accelerometer and a push button.

*/

#include <MPU6050.h>
#include <SPI.h>
#include <WiFi101.h>
#include <Wire.h>

MPU6050 mpu;

char ssid[] = "test";  //Character array that stores the name of the Wi-Fi network that the MKR1000 board will connect to
char pass[] = "test";  //Character array that stores the password of the Wi-Fi network that the MKR1000 board will connect to
int keyIndex = 0;      //Network key Index number

int status = WL_IDLE_STATUS;  //holds the Wi-Fi connection's current state
WiFiServer server(80);        //Wi-FiServer class object that represents a server that is configured to listen on port 80


#define leftEchoPin 2     // attach pin  Arduino to pin Echo of HC-SR04
#define leftTrigPin 3     //attach pin  Arduino to pin Trig of HC-SR04
#define rightBuzzPin 4    //attach pin  to buzzer Left
#define leftBuzzPin 5     //attach pin  to buzzer Left
#define distThreshold 30  //20 cm equals danger
#define rightTrigPin 0
#define rightEchoPin 1

const int buttonPin = 7;  // Type int used as input pin for the button
const int vibePin = 8;    // Type int used as input pin for the button vibe
const int ledPin = 6;     // Type int used as input pin for the LED
int buttonState = 1;      // set initial button state
int vibeState = 1;        // set initial vibe state

//defines variables
long durationLeft;   //Variable type long used to store the duration of the left HC-SR04 sensor sound wave travel
long durationRight;  // Variable type long used to store the duration of the right HC-SR04 sensor sound wave travel
int distanceLeft;    //Variable type int used to store the distance of the left HC-SR04 sensor sound wave travel
int distanceRight;   //Variable type long used to store the distance of the right HC-SR04 sensor sound wave travel

// Used to connect to the destination IP address ISY, which is an instance of the class IPAddress.
WiFiClient client;

IPAddress ISY(172, 20, 10, 14);  //Set my target IP address

const int MPU_addr = 0x68;  // Constant of type int that is used to store the I2C address of the MPU-6050 accelerometer

int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;  //Variables of type int16_t and are used to store the raw data read from the accelerometer

float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;  //Variables of type float and are used to store the acceleration and angular velocity values calculated from the raw data

boolean fall = false;      //Boolean variable that stores if a fall has occurred
boolean trigger1 = false;  //Boolean variable that stores if first trigger (lower threshold) has occurred
boolean trigger2 = false;  //Boolean variable that stores if second trigger (upper threshold) has occurred
boolean trigger3 = false;  //Boolean variable that stores if third trigger (orientation change) has occurred
byte trigger1count = 0;    //Byte type variable that stores the counts past since trigger 1 was set true; thus activated
byte trigger2count = 0;    //Byte type variable that stores the counts past since trigger 2 was set true; thus activated
byte trigger3count = 0;    //Byte type variable that stores the counts past since trigger 3 was set true; thus activated

int angleChange = 0;  //Type int variable that stores the angle change

void send_event(const char *event);
const char *host = "maker.ifttt.com";     //Constant of type char * used to store the host address for the IFTTT web service
const char *privateKey = "generatedkey";  //Constant of type char * used to store the private key for the IFTTT web service

void setup() {
  pinMode(rightTrigPin, OUTPUT);  //right trigPin as an OUTPUT
  pinMode(leftTrigPin, OUTPUT);   //lefttrigPin as an OUTPUT
  pinMode(leftEchoPin, INPUT);    //left echoPin as an INPUT
  pinMode(rightEchoPin, INPUT);   //right echoPin as an INPUT
  pinMode(leftBuzzPin, OUTPUT);   //leftBuzzPin as an OUTPUT
  pinMode(rightBuzzPin, OUTPUT);  //rightBuzzPin as an OUTPUT
  pinMode(ledPin, OUTPUT);        //LED pin as an OUTPUT
  pinMode(buttonPin, INPUT);      //button pin as an INPUT
  pinMode(vibePin, INPUT);        //vibe sensor pin an an INPUT


  Serial.begin(9600);  //9600 of baudrate speed
  Serial.println("Arduino MKR1000");

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     //launches MPU-6050
  Wire.endTransmission(true);

  //test for the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");

    while (true)
      ;
  }

  //attempt to establish wifi connection
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);

    status = WiFi.begin(ssid, pass);
    //delay
    delayMicroseconds(1000);
  }

  //SSID info
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  //IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  //received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");

  Serial.println("Initializing: MPU6050");

  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("Unable to locate MPU6050");
    //delay
    delayMicroseconds(500);
  }
}

//Main
void loop() {

  mpu_read();

  ax = (AcX - 2050) / 16384.00;
  ay = (AcY - 77) / 16384.00;
  az = (AcZ - 1947) / 16384.00;
  gx = (GyX + 270) / 131.07;
  gy = (GyY - 351) / 131.07;
  gz = (GyZ + 136) / 131.07;

  //Amplitude calculation for all axis
  float raw_amplitude = pow(pow(ax, 2) + pow(ay, 2) + pow(az, 2), 0.5);
  int amplitude = raw_amplitude * 10;  // Mulitiplied by 10 bcz values are between 0 to 1
  Serial.println(amplitude);

  //First trigger
  if (amplitude <= 4 && trigger2 == false) {  //First threshold: Rapid move breaks the lowest threshold (0.4g)

    trigger1 = true;  //First trigger activated
    Serial.println("TRIGGER 1 ACTIVATED");
  }

  //Second trigger
  if (trigger1 == true) {
    trigger1count++;
    if (amplitude >= 5) {  //Second threshold: Rapid move breaks the medium threshold (3g)

      trigger2 = true;
      Serial.println("TRIGGER 2 ACTIVATED");
      trigger1 = false;
      trigger1count = 0;
    }
  }

  //Third trigger
  if (trigger2 == true) {
    trigger2count++;
    angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);
    Serial.println(angleChange);
    if (angleChange >= 10 && angleChange <= 400) {  //Third threshold: Orientations change by 80-100 degrees
      trigger3 = true;
      trigger2 = false;
      trigger2count = 0;
      Serial.println(angleChange);
      Serial.println("TRIGGER 3 ACTIVATED");
    }
  }
  if (trigger3 == true) {
    trigger3count++;
    if (trigger3count >= 2) {
      angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);
      Serial.println(angleChange);
      if ((angleChange >= 0) && (angleChange <= 20)) {  //Check if Orientation stays between 0-10 degrees => Confirmed fall
        fall = true;
        trigger3 = false;
        trigger3count = 0;
        Serial.println(angleChange);
      } else {  //False Alarm => User stood up
        trigger3 = false;
        trigger3count = 0;
        Serial.println("TRIGGER 3 DEACTIVATED");
      }
    }
  }
  if (fall == true) {  //Fall detected
    Serial.println("FALL DETECTED");
    send_event("Fall Detection");
    fall = false;
  }
  if (trigger2count >= 6) {
    trigger2 = false;
    trigger2count = 0;
    Serial.println("TRIGGER 2 DECACTIVATED");
  }
  if (trigger1count >= 6) {
    trigger1 = false;
    trigger1count = 0;
    Serial.println("TRIGGER 1 DECACTIVATED");
  }
  delayMicroseconds(100);

  // Clears the trigPin
  digitalWrite(rightTrigPin, LOW);
  delayMicroseconds(2);

  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(rightTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(rightTrigPin, LOW);

  //Returns duration of the sound wave travel in microseconds
  durationRight = pulseIn(rightEchoPin, HIGH);

  // Clears the trigPin
  digitalWrite(leftTrigPin, LOW);
  delayMicroseconds(2);

  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(leftTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(leftTrigPin, LOW);

  //Returns duration of the sound wave travel in microseconds
  durationLeft = pulseIn(leftEchoPin, HIGH);

  //Calculating distances
  distanceLeft = durationLeft * 0.034 / 2;  //Divided by 2, (back and forth)
  distanceRight = durationRight * 0.034 / 2;

  if (distanceLeft < distThreshold) {
    digitalWrite(leftBuzzPin, HIGH);
  }  //Trigger leftBuzzer
  else {
    digitalWrite(leftBuzzPin, LOW);
  }  //Turn off leftBuzzer

  if (distanceRight < distThreshold) {
    digitalWrite(rightBuzzPin, HIGH);
  }  //Trigger  rightBuzzer
  else {
    digitalWrite(rightBuzzPin, LOW);
  }  //Turn of rightBuzzer

  // Displays the distance on the Serial Monitor
  Serial.print("Distance Left: ");
  Serial.print(distanceLeft);
  Serial.println(" cm");

  Serial.print("Distance Right: ");
  Serial.print(distanceRight);
  Serial.println(" cm");


  // check for BUTTON state change
  if (digitalRead(buttonPin) == HIGH && buttonState == 0) {
    digitalWrite(ledPin, HIGH);  // Turn on led
    Serial.println("MKR server LED on");
    buttonState = 1;      // Set button closed
    sendButtonChangeH();  // http GET request => button to high
  }
  if (digitalRead(buttonPin) == LOW && buttonState == 1) {
    digitalWrite(ledPin, LOW);  // turn off led
    Serial.println("MKR server LED off");
    buttonState = 0;      // Set button open
    sendButtonChangeL();  // http GET request => button to low
  }


  // check for VIBRATION state change. Same routine as before.
  if (digitalRead(vibePin) == HIGH && vibeState == 0) {
    digitalWrite(ledPin, HIGH);
    Serial.println("MKR vibe on");
    vibeState = 1;
    sendVibeChangeH();
  }

  if (digitalRead(vibePin) == LOW && vibeState == 1) {
    digitalWrite(ledPin, LOW);
    Serial.println("MKR vibe off");
    vibeState = 0;
  }


  Vector rawAccel = mpu.readRawAccel();
  Vector normAccel = mpu.readNormalizeAccel();

  Serial.print(" Xnorm = ");
  Serial.print(normAccel.XAxis);
  Serial.print(" Ynorm = ");
  Serial.print(normAccel.YAxis);
  Serial.print(" Znorm = ");
  Serial.println(normAccel.ZAxis);


  delayMicroseconds(1000);
}

//Sent http GET request when sensor state changes
void sendButtonChangeL() {

  if (client.connect(ISY, 80)) {

    Serial.println("Connected");

    //HTTP request to set state variable to 1
    client.println(" HTTP/1.1");
    client.println("Host: 172.20.10.14");  //MKR server's IP Address
    client.println("GET /L-btn");          //This sends GET request. You can also go to your browser at
                                           //the server's IP address and see available links to perform the same actions as
                                           //the client MKR buttons
    client.println("Content-Type: text/html");
    client.print("Client Button State = ");
    client.println(buttonState);

    client.println("Authorization: Basic xxxxx");  //send authorization header
    client.println();

    //Print variable change
    Serial.print("Variable Changed to ");
    Serial.println(buttonState);

    listenToClient();  //Listen for MKR server response to previous request
  } else {
    Serial.println("Connection Failed, send button change L");
  }
}
void sendButtonChangeH() {

  if (client.connect(ISY, 80)) {  //check if connection to MKR server is made

    Serial.println("Connected");

    //Make a HTTP request to set state variable 1
    client.println(" HTTP/1.1");
    client.println("Host: 172.20.10.14");
    client.println("GET /H-btn");
    client.println("Content-Type: text/html");
    client.print("Client Button State = ");
    client.println(buttonState);

    client.println("Authorization: Basic xxxxx");  //send authorization header
    client.println();

    //Print variable change
    Serial.print("Variable Changed to ");
    Serial.println(buttonState);

    listenToClient();  //Listen for MKR server response to previous request
  } else {
    Serial.println("Connection Failed H");
  }
}

void sendVibeChangeH() {

  if (client.connect(ISY, 80)) {  //check if connection to MKR server is made

    Serial.println("Connected");

    //Make a HTTP request to set state variable 1
    client.println(" HTTP/1.1");
    client.println("Host: 172.20.10.14");
    client.println("GET /H-vibe");
    client.println("Content-Type: text/html");
    client.print("Client Vibration State = ");
    client.println(vibeState);

    client.println("Authorization: Basic xxxxx");  //Send authorization header
    client.println();

    //variable change
    Serial.print("Variable Changed to ");
    Serial.println(vibeState);

    listenToClient();  //Listen for MKR server response to previous request
  } else {
    Serial.println("Connection Failed,den sindethike");
  }
}

// listen for and print response from MKR server
void listenToClient() {

  unsigned long startTime = millis();
  bool received = false;

  while ((millis() - startTime < 5000) && !received) {  // listen for 5 seconds
    while (client.available()) {
      received = true;
      char c = client.read();
      Serial.write(c);
    }
  }
  client.stop();
  Serial.println();
}

void mpu_read() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);  // request a total of 14 registers

  AcX = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void send_event(const char *event) {
  Serial.print("Connecting to ");
  Serial.println(host);
  WiFiClient client;
  const int httpPort = 80;
  if (!client.connect(host, httpPort)) {
    Serial.println("Connection failed");
    return;
  }
  
  String url = "/trigger/";
  url += event;
  url += "/with/key/";
  url += privateKey;
  Serial.print("Requesting URL: ");
  Serial.println(url);
  client.print(String("GET ") + url + " HTTP/1.1\r\n" + "Host: " + host + "\r\n" + "Connection: close\r\n\r\n");
  while (client.connected()) {
    if (client.available()) {
      String line = client.readStringUntil('\r');
      Serial.print(line);
    } else {
      delayMicroseconds(50);
    };
  }
  Serial.println();
  Serial.println("closing connection");
  client.stop();
}
