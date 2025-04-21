#include "ESP32.h"
#include "Wire.h"
#include "BluetoothSerial.h"
#include "SPI.h"
#include "SD.h"
#include "ESP32Servo.h"
#include "ADXL345.h"
#include "SimpleKalmanFilter.h"
#include "BMP280.h"
#include "Adafruit_FRAM_SPI.h"

BluetoothSerial SerialBT;
ADXL345 adxl;
BMP280 bmp280;
Adafruit_FRAM_SPI fram = Adafruit_FRAM_SPI(FRAM_CS);

//starting values for Kalman filtering of altitude
SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.1);

//create servo objects
Servo servo_1;
Servo servo_2;

//increase memory allocated to loop
SET_LOOP_TASK_STACK_SIZE(1024 * 32);

void setup() {
  Serial.begin(115200);

  //set pin modes
  pinMode(INT_LED, OUTPUT);
  pinMode(EXT_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(CAM_PWR, OUTPUT);
  pinMode(CAM_MODE, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  pinMode(FRAM_CS, OUTPUT);
  
  //start bluetooth connection
  SerialBT.begin("Your Team Name Here..."); //bluetooth device name for your team's rocket
  
  startup(); //play the startup chime

  //wait for device to connect
  Serial.print("Waiting to connect...");
  while(!SerialBT.connected()){  
    //fast flashes on led indicates waiting for connection
    digitalWrite(INT_LED, millis() % 100 < 50 ? HIGH : LOW);
    digitalWrite(EXT_LED, millis() % 100 < 50 ? HIGH : LOW);
  }

  //turn on the led to indicate the connection was established
  digitalWrite(INT_LED, HIGH);  
  digitalWrite(EXT_LED, HIGH);
  Serial.println("Bluetooth connection established.");
  SerialBT.println("Connection established.");

  init_SD();        //initalize the SD card file
  init_FRAM();      //initialize the FRAM
  init_accel();     //initilize the accelerometer
  init_alt();       //initilize the altimeter
  init_servos();    //intitilize the servos
  init_camera();    //initialize the camera

  //enable writing to the FRAM
  fram.writeEnable(true); 
  resetEof();
  addr = OFFSET;

  //signal ready for launch
  Serial.println("Initilization complete...  Ready for launch!");
  SerialBT.print("Initilization complete... ");
  delay(1000);
  SerialBT.print("Ready for launch!");
  beep();

  start = millis();  //start the timer for the log
}

void loop() {
  //get new data from accelerometer & multiply by 9.8 m/s/s
  double xyz[3];
	adxl.getAcceleration(xyz);
  //removes 1g offset for vertical axis
	accelX = xyz[0] * 9.8 - Xoffset;
	accelY = xyz[1] * 9.8 - Yoffset;
	accelZ = xyz[2] * 9.8 - Zoffset;  

  //get new pressure reading from altimeter and filter data
  altitude = pressureKalmanFilter.updateEstimate(44330 * (1.0 - pow(bmp280.getPressure() / basePressure, 0.1903)));

  //calculate time since last loop and vertical velocity
  float loop_time = (millis() - lastTime) * 0.001;  //time for each loop in seconds
  float vertVel = (altitude - last_altitude) / loop_time;  //vertical velocity based on altitude m/s

  last_altitude = altitude;  //update previous altitude
  lastTime = millis();  //update loop timer

  //calculate running average altitude
  avg_altitude = 0.99 * avg_altitude + 0.01 * altitude;
  
  //send new data to the log every 100ms
  if ((millis() - dataTimer) >= 100){
    telemetry();  //default is: time, state, altitude, and accelX
    dataTimer = millis();
  }
  
  //capture image every 30 seconds to keep the camera awake 
  cam_awake();

  //blink the LED 
  digitalWrite(EXT_LED, millis() % 1000 < 50 ? HIGH : LOW);

  //rocket states for flight
  switch(state){
    case 0:  //rocket is waiting on the launchpad
      //launch detection using acceleration or altitude threshold
      if (accelX > launch_accel || altitude > launch_alt){
        SerialBT.println("Launch detected");
        Serial.println("Launch detected");
        state ++;  //advance to the ascending state
      }
      break;

    case 1:  //rocket is ascending
      //check if apogee reached
      if (vertVel <= 0.5){
        //release parachute
        servo_1.write(relPos1);
        servo_2.write(relPos2);
        SerialBT.println("Paracute release");
        Serial.println("Parachute release");
        state ++;  //advance to the descent state
      }
      break;

    case 2:  //rocket is descending
      camera();  //capture pictures every 3 seconds

      //check if current altitude is close to running average
      if (abs(avg_altitude - altitude) < 0.1){
        state ++;  //advance to the ground state
      }
      break;

    case 3:  //rocket is on the ground
      SerialBT.println("Touchdown");
      Serial.println("Touchdown");

      //power off camera
      cam_off();

      //write FRAM data to SD card
      eof = getEof();
      log_SD(eof);

      //flash LED and play sound to aid recovery
      while (1){
        digitalWrite(INT_LED, millis() % 500 < 50 ? HIGH : LOW);
        digitalWrite(EXT_LED, millis() % 500 < 50 ? HIGH : LOW);

        if (millis() % 500 < 50){
          tone(BUZZER, 1500, 500);
        }
        else {
          noTone(BUZZER);
        }
      }
      break;
  }
}





