//define pins for the ESP32
#define BUZZER      27  //buzzer pin
#define INT_LED     2   //on board LED pin (blue)
#define EXT_LED     4   //external LED pin (green)
#define SD_CS       5   //chip select for SD card
#define FRAM_CS     25  //chip select for FRAM
#define CAM_PWR     13  //power button for camera (red)
#define CAM_MODE    14  //mode button for camera (white)
#define SERVO_1     16  //servo 1 signal pin
#define SERVO_2     17  //servo 2 signal pin

#define OFFSET 4 //bytes 0-3 are reserved for EOF address

/*other pins used
SDA     21
SCL     22
MISO    19
MOSI    23
SCLK    18
SD_CS   5
FRAM_CS 25
*/

//define starting positions for servos
#define startPos1 90
#define startPos2 90
#define relPos1 0
#define relPos2 180

//define parameters for launch/recovery
#define launch_accel 10 //acceleration in m/s/s to detect launch
#define launch_alt 20  //altitude in meters for fail safe detection of launch

float basePressure; // baseline pressure (Pa) at the launchpad
float altitude;  //calculated and filtered altitude above launchpad in meters
float last_altitude;  //store previous altitude reading for apogee detection
float avg_altitude;  //running average altitude for landing detection

float accelX, accelY, accelZ;  //acceleration readings
float Xoffset, Yoffset, Zoffset;  //acceleration offsets from setup  

unsigned long start;  //start time for the datalog
char file_name[12] = "/log";  //file name string
int file_num = 1;  //starting value for file number
const char file_ext[5] = ".csv";  //file extension string
uint32_t addr; //memory address for FRAM
uint32_t eof;  //memory address for the end of log file

byte state = 0;  //tracks state of rocket launch
unsigned long dataTimer, camTimer, shutterTimer, lastTime; 
bool shutter;  //true when image capture is active

bool dataStream = true;  //true sends data to serial monitor and BT Serial

