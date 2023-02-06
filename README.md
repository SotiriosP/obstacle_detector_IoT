# obstacle_detector_IoT
**Obstacle detector and warning system for the visually impaired**

-- --
The system consists of an Arduino MKR1000 device serving as the client and an Arduino MKR1000 device acting like a server. The first board is part of a wearable technology attached to a knee or a wrist band. This wearable technology is equipped with two ultrasonic sensors and two corresponding vibration motor modules.

To begin with, the MKR1000 device, which is a microcontroller board, gets connected to a Wi-Fi network and then continuously sends GET requests to the remote server, according to what its sensors have captured. The server can then perform actions based on the GET request it receives.

The client uses the ultrasonic sensors, which are used to measure the distance of an object. The ultrasonic sensors use a trigger pin and an echo pin to send out a sound wave and then measure the time it takes for the sound wave to bounce back. This information is used to calculate the distance to the object. 

Furthermore, the wearable system is equipped with an emergency push button switch. When the emergency button is pressed by the blind person, a selected emergency contact is informed of the disabled person’s status and a red flashing light is enabled, that indicates that they are in danger.

In addition, the wearable system will be equipped with a 3-axis accelerometer and gyroscope, which is a device capable of detecting changes in motion in the form of accelerations. The program uses an algorithm to detect a fall, which is triggered by a combination of data from the MPU6050 sensor. If the visually impaired person falls on the ground, the program sends a GET request to the server device, and also sends an event to IFTTT (If This Then That) service. The GET request is used to change a server state variable, so the server can perform an action based on the state of the client device. The event sent to IFTTT can be used to trigger an action, such as sending an urgent email in our case.

Last but not least, the wearable system is equipped with a Light Intensity Detection Sensor, which can detect the presence of light. In case of darkness, a light will be activated for the blind person to be visible to their surroundings; Thus, minimizing the possibility of a serious accident, especially while crossing the road.

-- --
For the proposed system, the components used are the following:

Arduino MKR1000 board (client)

Arduino MKR1000 board (server)

2 Ultrasonic sensors

2 Vibration motors

1 Accelerometer

1 Push button

1 Light sensor

1 Led light

1 Mobile device

Jumper wires

Resistors

Breadboard

-- --
More specifically, the detailed system architecture consists of:

1.	MKR1000 client device: This device runs the main program and is responsible for connecting to the Wi-Fi network, collecting data from sensors connected to it, such as the MPU6050 sensor and ultrasonic sensors, analyzing the data to detect a fall and sending GET requests to the server device.
2.	UNO server device: This device acts as a server and receives GET requests from the client device. It has a state variable that can be changed based on the GET request received. The server can then perform actions based on the state of the variable, such enabling light switches and forwarding emails to the emergency contact.
3.	Wi-Fi network: The client device connects to this network to send GET requests to the server device. The network provides a communication channel between the client and server devices.
4.	Sensors: The system uses the MPU6050 sensor, ultrasonic sensors and light sensor to collect data. The data is used to detect multiple types of data and specifically to detect obstacles and a possible fall.
5.	Motors: The system uses the vibration motors and push buttons, to perform actions and specifically inform the visually impaired person of nearby obstacles and give them the opportunity to ask for help by their emergency contacts.
6.	Algorithm: The program uses an algorithm to detect obstacles and a possible fall, which is triggered by a combination of data from the MPU6050 sensor, the ultrasonic sensors and the light sensor. The algorithm analyzes the data and performs the needed.
7.	IFTTT: The program also sends an event to IFTTT (If This Then That) service to trigger an action based on the event. The action can be anything that IFTTT supports, such as sending an email.

-- --
