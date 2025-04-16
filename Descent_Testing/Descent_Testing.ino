//libraries to include
#include <WiFi.h>
#include <WebServer.h>                                
#include <WebSocketsServer.h>                     
#include <ArduinoJson.h>    
#include <ESP32Servo.h>
#include <ADXL345.h>
#include <Wire.h>

//define pins for ESP32
#define BUZZER      27  //buzzer pin
#define INT_LED     2   //on board LED pin (blue)
#define EXT_LED     4   //external LED pin (green)
#define SERVO_1     16  //servo 1 signal pin
#define SERVO_2     17  //servo 2 signal pin

//create servo objects
Servo servo_1;
Servo servo_2;

//create accelerometer object
ADXL345 adxl;

// SSID and password of Wifi connection:
const char *ssid = "Aerospace PW";
const char *password = "";

int startPos1, startPos2;
int relPos1, relPos2;
bool launch = false;

// The String below "webpage" contains the complete HTML code that is sent to the client whenever someone connects to the webserver
String webpage = "<!DOCTYPE html><html><head><title>Page Title</title></head><body style='background-color: #EEEEEE;'><span style='color: #003366;'><h1>Descent Testing</h1><p><label for='servo1start'>Servo 1 initial position(0 to 180):</label><input id='servo1start' type='number' name='servo1start' value='0' min='0' max='180'/><p><label for='servo1final'>Servo 1 release position(0 to 180):</label><input id='servo1final' type='number' name='servo1final' value='180' min='0' max='180'/><p><label for='servo2start'>Servo 2 initial position(0 to 180):</label><input id='servo2start' type='number' name='servo2start' value='0' min='0' max='180'/><p><label for='servo2final'>Servo 2 release position(0 to 180):</label><input id='servo2final' type='number' name='servo2final' value='180' min='0' max='180'/><p><p><button type='button' id='BTN_INIT'>Initialize</button></p><button type='button' id='BTN_TEST'>Test Release</button></p><p><button type='button' id='BTN_LAUNCH'>Ready for Launch!</button></p></span></body><script> var Socket; document.getElementById('BTN_INIT').addEventListener('click', button_initilize); document.getElementById('BTN_TEST').addEventListener('click', button_test); document.getElementById('BTN_LAUNCH').addEventListener('click', button_launch); function init() { Socket = new WebSocket('ws://' + window.location.hostname + ':81/'); Socket.onmessage = function(event) { processCommand(event); }; } function button_initilize() { var msg = {button: 0,servo1i: parseInt(document.getElementById('servo1start').value),servo1f: parseInt(document.getElementById('servo1final').value),servo2i: parseInt(document.getElementById('servo2start').value),servo2f: parseInt(document.getElementById('servo2final').value)};Socket.send(JSON.stringify(msg)); } function button_test() { var msg = {button: 1,servo1i: parseInt(document.getElementById('servo1start').value),servo1f: parseInt(document.getElementById('servo1final').value),servo2i: parseInt(document.getElementById('servo2start').value),servo2f: parseInt(document.getElementById('servo2final').value)};Socket.send(JSON.stringify(msg)); } function button_launch() { var msg = {button: 2,servo1i: parseInt(document.getElementById('servo1start').value),servo1f: parseInt(document.getElementById('servo1final').value),servo2i: parseInt(document.getElementById('servo2start').value),servo2f: parseInt(document.getElementById('servo2final').value)};Socket.send(JSON.stringify(msg)); } function processCommand(event) {var obj = JSON.parse(event.data);document.getElementById('altitude').innerHTML = obj.altitude; console.log(obj.altitude); } window.onload = function(event) { init(); }</script></html>";

unsigned long timer;

// Initialization of webserver and websocket
WebServer server(80);                                 // the server uses port 80 (standard port for websites
WebSocketsServer webSocket = WebSocketsServer(81);    // the websocket uses port 81 (standard port for websockets

void setup() {
  Serial.begin(115200);
 
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: http://");
  Serial.println(IP);

  server.on("/", []() {                               // define here wat the webserver needs to do
    server.send(200, "text/html", webpage);           //    -> it needs to send out the HTML string "webpage" to the client
  });
  server.begin();
  
  webSocket.begin();                                  // start websocket
  webSocket.onEvent(webSocketEvent);                  // define a callback function -> what does the ESP32 need to do when an event from the websocket is received? -> run function "webSocketEvent()"

  init_servos();    //intitilize the servos
  init_accel();     //initilize the accelerometer
}

void loop() {
  server.handleClient();                              // Needed for the webserver to handle all clients
  webSocket.loop();                                   // Update function for the webSockets 

  //get new accelerometer data
  double xyz[3];
	double ax,ay,az;
	adxl.getAcceleration(xyz);
	ax = xyz[0];
	ay = xyz[1];
	az = xyz[2];

  float totalAccel = abs(ax + ay + az);
  // if ((millis() - timer) >= 100){
  //   Serial.println(totalAccel);
  //   timer = millis();
  // }

  if (launch && totalAccel < 0.01){
    //turn on LEDs
    digitalWrite(INT_LED, HIGH);
    digitalWrite(EXT_LED, HIGH);
    tone(BUZZER, 1000);

    //move servos
    servo_1.write(relPos1);
    servo_2.write(relPos2);

    delay(6000);

    //turn off buzzer
    noTone(BUZZER);
    launch = false;

  }
  else if (launch){
    //blink LED
    digitalWrite(EXT_LED, millis() % 1000 < 50 ? HIGH : LOW);
    digitalWrite(INT_LED, millis() % 1000 < 50 ? HIGH : LOW);
  }
  else{
    //turn off LEDs
    digitalWrite(INT_LED, LOW);
    digitalWrite(EXT_LED, LOW);
  }
}

void webSocketEvent(byte num, WStype_t type, uint8_t * payload, size_t length) {      // the parameters of this callback function are always the same -> num: id of the client who send the event, type: type of message, payload: actual data sent and length: length of payload
  switch (type) {                                     // switch on the type of information sent
    case WStype_DISCONNECTED:                         // if a client is disconnected, then type == WStype_DISCONNECTED
      Serial.println("Client " + String(num) + " disconnected");
      break;
    case WStype_CONNECTED:                            // if a client is connected, then type == WStype_CONNECTED
      Serial.println("Client " + String(num) + " connected");
      // optionally you can add code here what to do when connected
      break;
    case WStype_TEXT:                                 // if a client has sent data, then type == WStype_TEXT
      // try to decipher the JSON string received
      StaticJsonDocument<200> doc;                    // create a JSON container
      DeserializationError error = deserializeJson(doc, payload);
      if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        return;
      }
      else {
        //move values from JSON string to variables
        int button = doc["button"];
        startPos1 = doc["servo1i"];
        relPos1 = doc["servo1f"];
        startPos2 = doc["servo2i"];
        relPos2 = doc["servo1f"];

        //remove values less than 0 and greater than 180
        if (startPos1 > 180){
          startPos1 = 180;
        }
        else if (startPos1 < 0){
          startPos1 = 0;
        }
        if (relPos1 > 180){
          relPos1 = 180;
        }
        else if (relPos1 < 0){
          relPos1 = 0;
        }
        if (startPos2 > 180){
          startPos2 = 180;
        }
        else if (startPos2 < 0){
          startPos2 = 0;
        }
        if (relPos2 > 180){
          relPos2 = 180;
        }
        else if (relPos2 < 0){
          relPos2 = 0;
        }
        
        //print values
        Serial.print(button);
        Serial.print("\t");
        Serial.print(startPos1);
        Serial.print("\t");
        Serial.print(relPos1);
        Serial.print("\t");
        Serial.print(startPos2);
        Serial.print("\t");
        Serial.println(relPos2);

        if (button == 0){  //set servos to starting positions
          servo_1.write(startPos1);
          servo_2.write(startPos2);
          launch = false;
        }
        else if (button == 1){  //set servos to release positions
          servo_1.write(relPos1);
          servo_2.write(relPos2);
          launch = false;
        }
        else if (button == 2){  //
          servo_1.write(startPos1);
          servo_2.write(startPos2);
          launch = true;
        }
      }
      break;
  }
}

void init_accel(){  //initilize the accelerometer
  adxl.powerOn();

  //check if accelerometer is connected
  if (!adxl.status){
    Serial.println("Error connecting to accelerometer");
  }

    //set activity/ inactivity thresholds (0-255)
  adxl.setActivityThreshold(75); //62.5mg per increment
  adxl.setInactivityThreshold(75); //62.5mg per increment
  adxl.setTimeInactivity(10); // how many seconds of no activity is inactive?
 
  //look of activity movement on this axes - 1 == on; 0 == off 
  adxl.setActivityX(1);
  adxl.setActivityY(1);
  adxl.setActivityZ(1);
 
  //look of inactivity movement on this axes - 1 == on; 0 == off
  adxl.setInactivityX(1);
  adxl.setInactivityY(1);
  adxl.setInactivityZ(1);
 
  //look of tap movement on this axes - 1 == on; 0 == off
  adxl.setTapDetectionOnX(0);
  adxl.setTapDetectionOnY(0);
  adxl.setTapDetectionOnZ(1);
 
  //set values for what is a tap, and what is a double tap (0-255)
  adxl.setTapThreshold(50); //62.5mg per increment
  adxl.setTapDuration(15); //625us per increment
  adxl.setDoubleTapLatency(80); //1.25ms per increment
  adxl.setDoubleTapWindow(200); //1.25ms per increment
 
  //set values for what is considered freefall (0-255)
  adxl.setFreeFallThreshold(7); //(5 - 9) recommended - 62.5mg per increment
  adxl.setFreeFallDuration(45); //(20 - 70) recommended - 5ms per increment
 
  //setting all interrupts to take place on int pin 1
  //I had issues with int pin 2, was unable to reset it
  adxl.setInterruptMapping( ADXL345_INT_SINGLE_TAP_BIT,   ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_DOUBLE_TAP_BIT,   ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_FREE_FALL_BIT,    ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_ACTIVITY_BIT,     ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_INACTIVITY_BIT,   ADXL345_INT1_PIN );
 
  //register interrupt actions - 1 == on; 0 == off  
  adxl.setInterrupt( ADXL345_INT_SINGLE_TAP_BIT, 1);
  adxl.setInterrupt( ADXL345_INT_DOUBLE_TAP_BIT, 1);
  adxl.setInterrupt( ADXL345_INT_FREE_FALL_BIT,  1);
  adxl.setInterrupt( ADXL345_INT_ACTIVITY_BIT,   1);
  adxl.setInterrupt( ADXL345_INT_INACTIVITY_BIT, 1);

  Serial.println("accelerometer ready.");  
}

void init_servos(){  //attach and initilize servos
  Serial.print("Initilizing servos...");
  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

  //set servo pwm frequency
	servo_1.setPeriodHertz(50);
  servo_2.setPeriodHertz(50);
  
  // attaches servos to assigned pins
  servo_1.attach(SERVO_1, 650, 2600);
  servo_2.attach(SERVO_2, 650, 2600);

  //initialize servos to starting positions
  servo_1.write(startPos1);  
  servo_2.write(startPos2);
  delay(1000);

  //notify the serial monitor
  Serial.println("servos initilized.");
}