void beep() {  //plays 500Hz tone for 100ms
    tone(BUZZER, 500, 100);  //(pin, frequency, duration)
}

void init_SD() {  //initialze the SD card and set log file name
  char buffer[12];

  //activate the SD Card
  digitalWrite(FRAM_CS, HIGH);  //high is inactive
  digitalWrite(SD_CS, LOW);  //low is active
  
  if (!SD.begin()) {
    Serial.println("SD card failed, or not present");
    SerialBT.println("SD card failed, or not present");
    error();  //call error notification
  }

  //check to see if the file already exists
  itoa(file_num, buffer, 10); //convert int file number to charStr
  strcat(file_name, buffer);
  strcat(file_name, file_ext);
  strcat(file_name, "\0");  //concatenate file name as charStr

  while (SD.exists(file_name)) {  
    Serial.print(file_name);
    Serial.println(" already exists");
    file_num ++;
    strcpy(file_name, "/log");  //reset the file name
    itoa(file_num, buffer, 10); //convert int to charStr
    strcat(file_name, buffer);
    strcat(file_name, file_ext);
    strcat(file_name, "\0");  //concatenate file name as charStr
  }
  Serial.println(file_name);  //print the current file name
  SerialBT.println(file_name);
  File dataFile = SD.open(file_name, FILE_WRITE);

  //check if the file is open
  if (dataFile) {
    //print the headers for the file
    dataFile.println("time,state,altitude,acceleration");  
    dataFile.close();
  }

  // if the file isn't open, pop up an error:
  else {
    Serial.print("Error opening ");
    Serial.println(file_name);
    error();  //call error notification
  }
  
  //notify the serial monitor
  Serial.println("SD card initialized.");  
  SerialBT.println("SD card initialized.");

  //activate the FRAM
  digitalWrite(FRAM_CS, LOW);  //low is active
  digitalWrite(SD_CS, HIGH);  //high is inactive
}

void init_FRAM(){  //initialize the FRAM chip
  //activate the FRAM
  digitalWrite(FRAM_CS, LOW);  //low is active
  digitalWrite(SD_CS, HIGH);  //high is inactive

  if (!fram.begin()) {
    Serial.println("FRAM failed to connect");
    SerialBT.println("FRAM failed to connect");
    error();  //call error notification
  }

  //Check for data existing on the FRAM chip
  uint32_t eof = getEof();
  if (eof != 0) {  //the first three bytes represent an address if there is data to write
    Serial.println("Existing data found on FRAM");
    SerialBT.println("Existing data found on FRAM");

    log_SD(eof);  //write the existing data to the log file
    strcpy(file_name, "/log");  //reset the file name for a new log
    init_SD();  //initilize a new log file
  }

  //notify the serial monitor
  Serial.println("FRAM initialized.");  
  SerialBT.println("FRAM initialized.");
}

uint32_t getEof() {
  //read four bytes starting from address 0x00
  uint8_t byte1 = fram.read8(0x00);
  uint8_t byte2 = fram.read8(0x01);
  uint8_t byte3 = fram.read8(0x02);
  uint8_t byte4 = fram.read8(0x03);

  return (byte4 << 24) | (byte3 << 16) | (byte2 << 8) | byte1;
}

void setEof(uint32_t value) {
  // Write the bytes of the integer to consecutive memory locations
  fram.write8(0x00, (uint8_t)(value & 0xFF));          
  fram.write8(0x01, (uint8_t)((value >> 8) & 0xFF));  
  fram.write8(0x02, (uint8_t)((value >> 16) & 0xFF)); 
  fram.write8(0x03, (uint8_t)((value >> 24) & 0xFF));
}

void resetEof() {
  setEof(0);
}

void init_alt(){  //initialize altimeter
  bmp280.begin();

  //check if altimeter is connected
  if(!bmp280.eStatusErrDeviceNotDetected){
    Serial.println("Error connecting to altimeter.");
    SerialBT.println("Error connecting to altimeter.");
    error();
  }

  //set sampling and filter parameters
  bmp280.setCtrlMeasSamplingPress(BMP280::eSampling_X2);
  bmp280.setConfigFilter(BMP280::eConfigFilter_off);
  delay(1000);
  
  //set base altitude 
  Serial.print("Calibrating altimeter...");
  SerialBT.print("Calibrating altimeter...");

  //collect 100 samples and average
  for (int i = 0; i <= 99; i++){
    basePressure += bmp280.getPressure();
    delay(20);
  }
  basePressure = basePressure / 100;

  //notify the serial monitor
  Serial.println("altimeter calibrated.");  
  SerialBT.println("altimeter calibrated.");
}

void init_accel(){  //initilize the accelerometer
  adxl.powerOn();

  //check if accelerometer is connected
  if (!adxl.status){
    Serial.println("Error connecting to accelerometer");
    SerialBT.println("Error connecting to accelerometer");
    error();
  }

  Serial.print("Calibrating accelerometer...");
  SerialBT.print("Calibrating accelerometer...");

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

  //collect 100 samples and average
  for (int i = 0; i < 100; i++){
    double xyz[3];
    adxl.getAcceleration(xyz);
    Xoffset += xyz[0] * 9.8;
    Yoffset += xyz[1] * 9.8;
    Zoffset += xyz[2] * 9.8;
    delay(10);
  }
  Xoffset /= 100;
  Yoffset /= 100;
  Zoffset /= 100;

  Serial.println("accelerometer ready.");  
  SerialBT.println("accelerometer ready.");
}

void error(){  //play tone and flash LED to indicate fault
  Serial.println("Error!  Something went wrong.");
  SerialBT.println("Error!  Something went wrong.");
  while(1){
    digitalWrite(INT_LED, HIGH);
    digitalWrite(EXT_LED, LOW);
    tone(BUZZER, 500, 1000);
    noTone(BUZZER);
    delay(200);

    digitalWrite(INT_LED, LOW);
    digitalWrite(EXT_LED, HIGH);
    tone(BUZZER, 1500, 1000);
    noTone(BUZZER);
    delay(200);
  }
}

void init_servos(){  //attach and initilize servos
  Serial.print("Initilizing servos...");
  SerialBT.print("Initilizing servos...");
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
  SerialBT.println("servos initilized.");
}

void init_camera(){  //initialize the camera
  Serial.print("Initializing camera..."); 
  SerialBT.print("Initializing camera...");

  //set the mode pin to high to allow the camera to power on
  digitalWrite(CAM_MODE, HIGH);  //high is "not pressed"

  //Turn on camera
  digitalWrite(CAM_PWR, HIGH);  //high is "pressed"
  delay(2000);
  digitalWrite(CAM_PWR, LOW);  //low is "not pressed"
  delay(3000);

  //activate the mode button twice for picture mode
  digitalWrite(CAM_MODE, LOW);  //low is "pressed"
  delay(100);
  digitalWrite(CAM_MODE, HIGH);  //high is "not pressed"
  delay(1000);
  digitalWrite(CAM_MODE, LOW);
  delay(100);
  digitalWrite(CAM_MODE, HIGH);
  delay(1000);

  //notify the serial monitor
  Serial.println("camera initilized.");
  SerialBT.println("camera initilized.");
}

void telemetry(){  //log flight data to FRAM
  float t = (millis() - start) / 1000;  //current time from end of setup/callibration
  uint8_t buffer[4];  //4 byte buffer for floats

  //write the current time to FRAM
  memcpy(buffer, (void *)&t, 4); 
  fram.write(addr, buffer, 4);
  Serial.print(t);
  addr = addr + 4;

  //write the state to FRAM
  memcpy(buffer, (void *)&state, 4); 
  fram.write(addr, buffer, 4);
  Serial.print("\t");
  Serial.print(state);
  addr = addr + 4;

  //write the altitude to FRAM
  memcpy(buffer, (void *)&altitude, 4); 
  fram.write(addr, buffer, 4);
  Serial.print("\t");
  Serial.print(altitude);
  addr = addr + 4;
  
  //write the acceleration to FRAM
  memcpy(buffer, (void *)&accelX, 4); 
  fram.write(addr, buffer, 4);
  Serial.print("\t");
  Serial.println(accelX);
  addr = addr + 4;

  //write EOF location
  setEof(addr);

  //print data to the serial monitor
  if (dataStream){
    String data = "";

    data += String(t, 3);  //time (seconds) with 3 decimals
    data += "/t";
    data += String(state);  //rocket state
    data += "/t";
    data += String(altitude, 2);  //altitude (meters) with 2 decimals 
    data += "/t";
    data += String(accelX, 2);  //acceleration (m/s/s) with 2 decimals

    Serial.println(data); 
    SerialBT.println(data);
  }
}
  
void log_SD(uint32_t eof){
  float value;
  uint8_t buffer[4];  //4 bytes for float
  String dataString;

  fram.writeEnable(false); //disable writing to the FRAM
  addr = OFFSET;  //reset the address to the start
  
  //Write sensor data to log file
  while (addr < eof){
    dataString = "";  //clear the string to write

    //fast flashes to indicate data is being written
    digitalWrite(INT_LED, millis() % 100 < 50 ? HIGH : LOW);

    //activate the FRAM
    digitalWrite(FRAM_CS, LOW);  //low is active
    digitalWrite(SD_CS, HIGH);  //high is inactive

    //read time data to send to SD log
    fram.read(addr, buffer, 4);
    memcpy((void *)&value, buffer, 4);
    dataString += String(value, 4);
    dataString += ",";
    addr = addr + 4;  //next 4 bytes

    //read state data to send to SD log
    fram.read(addr, buffer, 4);
    memcpy((void *)&value, buffer, 4);
    dataString += String(value, 4);
    dataString += ",";
    addr = addr + 4;  //next 4 bytes

    //read altitude data to send to SD log
    fram.read(addr, buffer, 4);
    memcpy((void *)&value, buffer, 4);
    dataString += String(value, 4);
    dataString += ",";
    addr = addr + 4;  //next 4 bytes

    //read acceleration data to send to SD log
    fram.read(addr, buffer, 4);
    memcpy((void *)&value, buffer, 4);
    dataString += String(value, 4);
    addr = addr + 4;  //next 4 bytes

    //activate the SD Card
    digitalWrite(FRAM_CS, HIGH);  //high is inactive
    digitalWrite(SD_CS, LOW);  //low is active

    File dataFile = SD.open(file_name, FILE_APPEND);

    // if the file is available, write to it
    if (dataFile) {
      dataFile.println(dataString);
      dataFile.close();
    }
    else {
      Serial.println("Failed to wtite to SD Log.");
      SerialBT.println("Failed to write to SD Log.");
      error();
    }
  }
  
  //activate the FRAM
  digitalWrite(FRAM_CS, LOW);  //low is active
  digitalWrite(SD_CS, HIGH);  //high is inactive
  
  fram.writeEnable(true); //enable writing to the FRAM
  resetEof();  //reset the first 4 bytes for memory address

  //notify the serial monitor
  Serial.println("Finished writing all data to SD card ");
  SerialBT.println("Finished writing all data to SD card ");
  Serial.println(file_name);
  SerialBT.println(file_name);
}

void startup(){  //play a simple series of tones to indicate that the initialization process has started
  tone(BUZZER, 294);  //D4
  delay(100);
  noTone(BUZZER);
  delay(10);
  
  tone(BUZZER, 330);  //E4
  delay(100);
  noTone(BUZZER);
  delay(10);

  tone(BUZZER, 370);  //F4#
  delay(100);
  noTone(BUZZER);
  delay(10);

  tone(BUZZER, 392);  //G4
  delay(400);
  noTone(BUZZER);
}

void camera(){  //activate the camera to capture images every 3 seconds
  if (!shutter && (millis() - camTimer) >= 3000){
    //activate the power button to take a picture
    digitalWrite(CAM_PWR, HIGH);  
    shutterTimer = millis();
    shutter = true;
    Serial.println("picture");
    SerialBT.println("picture");
  }
  else if (shutter && (millis() - shutterTimer) >= 100){
    //release the power button to finish
    digitalWrite(CAM_PWR, LOW);  
    shutter = false;
    camTimer = millis();
  }
}

void cam_awake(){  //capture image every 30 seconds to keep the camera awake while waiting for launch
  if (((millis() - start) % 30000) < 20){
    camera();  //call to activate the camera
  }
  else if (shutter){
    camera();  //call to check if the task was complete
  }
}

void cam_off(){  //hold power button high for 6 seconds to power off the camera
  if (!shutter){
    digitalWrite(CAM_PWR, HIGH);  
    shutterTimer = millis();
    shutter = true;
  }
  else if (shutter && (millis() - shutterTimer) >= 6000){
    //release power button
    digitalWrite(CAM_PWR, LOW);  
  }  
}