/*
The secondary system consists of an Arduino MKR1000 board which is connected with a led light and a vibration motor. 
Its purpose is to receive signals from the primary system and transform them into actions accordingly, e.g. flashing the led
to indicate a possible fall.
*/

#include <WiFi101.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <WiFiSSLClient.h>
#include <WiFiUdp.h>
#include <SPI.h>
#include <WiFi101.h>

char ssid[] = "test";  // char array that stores the name of the Wi-Fi network that the device should connect to
char pass[] = "test";  // variable is a char array and it stores the password of the Wi-Fi network
int keyIndex = 0;      // key index number as an integer  (needed only for WEP networks)
int ledpin = 6;        //pin number of the LED
int vibepin = 2;       //pin number of the Vibration pin
bool val = true;

int status = WL_IDLE_STATUS;  //integer variable that holds the Wi-Fi connection's current state
WiFiServer server(80);        //Wi-FiServer class object that represents a server that is configured to listen on port 80

void setup() {
  Serial.begin(9600);  // initialize serial communication
  Serial.print("Start Serial ");
  pinMode(ledpin, OUTPUT);   // set the LED pin mode
  pinMode(vibepin, OUTPUT);  // set the LED pin mode

  // Check for shield
  Serial.print("WiFi101 shield: ");
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("NOT PRESENT");
    return;
  }
  Serial.println("DETECTED");
  // Connection attempt to the Wi-Fi
  while (status != WL_CONNECTED) {
    digitalWrite(ledpin, LOW);
    digitalWrite(vibepin, LOW);
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(ssid);  // print the SSID of the network

    status = WiFi.begin(ssid, pass);
    // delay for connection purposes
    delay(100);
  }
  server.begin();     // Launch the web server on port 80
  printWifiStatus();  // Successfull connection status
  digitalWrite(ledpin, LOW);
  digitalWrite(vibepin, LOW);
}
void loop() {
  WiFiClient client = server.available();  // listen for incoming clients and create a client object
  if (client) {
    Serial.println("new client");  //// print message to indicate new client connection

    String currentLine = "";  // create a string to store incoming data from client

    while (client.connected()) {  // Continuously loop while the client is connected
      if (client.available()) {   // check if there is data available from the client
        char c = client.read();   // read a byte of data
        Serial.write(c);          //  write the data to the serial monitor
        if (c == '\n') {

          // If the current line is blank, it means the end of the client HTTP request
          // Send the HTTP response to the client

          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code
            //and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // Write the content of the HTTP response
            client.print("Click <a href=\"/H-btn\">here</a> turn the built in LED on<br>");
            client.print("Click <a href=\"/L-btn\">here</a> turn the built in LED off<br>");
            client.print("Repeatedly click <a href=\"/H-vibe\">here</a>  to step through fading of pin 5 LED<br>");

            // End the HTTP response with another blank line
            client.println();
            // break
            break;
          } else {  // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // append it to the end of the currentLine
        }

        // Check the client request
        // If the request was to turn on the built-in LED (GET /H-btn)
        if (currentLine.endsWith("GET /H-btn")) {
          digitalWrite(ledpin, HIGH);  // Turn on the built-in LED
          digitalWrite(vibepin, HIGH);
        }
        If the request was to turn off the built - in LED(GET / L - btn) if (currentLine.endsWith("GET /L-btn")) {
          digitalWrite(ledpin, LOW);  // Turn the LED off
          digitalWrite(vibepin, LOW);
        }
      }
    }
    // Close the connection with the client
    client.stop();
    Serial.println("client disonnected");
  }
}

// This function is used to print the WiFi status information
void printWifiStatus() {
  // SSID of the network:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // Received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");

  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}
