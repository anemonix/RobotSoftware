/*
 * Libraries and files
 */
#include <SPI.h>
#include <WiFi101.h>
#include "arduino_secrets.h" 

/*
 * Constants
 */
#define leftIR 12
#define rightIR 11
#define leftMotor 10
#define rightMotor 9
#define baseline 80
#define radius 25
#define turnRatio 75

/*
 * Change SSID and password accordingly in header file
 */
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;   
int status = WL_IDLE_STATUS;
IPAddress ip(192, 168, 1, 23);

struct velTheta
{
  double velocity;
  double theta;
};
velTheta mainData = {0.0 , 0.0};
WiFiClient client;

float centralAngle = 90/turnRatio;
float arcLength = radius * deg2rad(centralAngle);
float z = 0, deltaZ = arcLength/baseline;
float x = 0, anglePerTickX = (baseline/2) * sin(deltaZ), deltaX;
float y = 0, anglePerTickY = (baseline/2 ) * (1-cos(deltaZ) ), deltaY;

/*
 * Set up accessPoint, open port, IRs, IMU and motor to default state
 */
void setup() 
{
    accessPoint();
    openPort();
    setIR();
    imuSetup();
    motorSetup();
}

void loop() 
{
  // put your main code here, to run repeatedly:
  /*
   * ax, ay, az = getIMU()
   * a = checkUDP() non blocking
   * velocity, theta = parse(a)
   * setSpeed(velocity)
   * setDir(Theta)
   * ML, MR <-- make globals
   * If(pickedUp) ML = MR = 0 ;; check az
   * setMotor(ML, MR)
   */
   
}

/*
 * Set up microcontroller to connec to an AP
 */
void accessPoint()
{
  WiFi.setPins(8,7,4,2);
  Serial.begin(9600);
  while (!Serial) 
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  if (WiFi.status() == WL_NO_SHIELD) 
  {
    Serial.println("WiFi shield not present");
    while (true);
  }
  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) 
  {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(10000);
  }
  Serial.println("Connected to wifi");
}

void openPort()
{
  /*
   * open port of uc for sbc?
   */
}

//void setupDebug(Serial input)
{
  
//}

void imuSetup()
{
  /*
   * set up imu
   */
}
/*
 * Set IR as input pullup and handle interrupts
 */
void setIR()
{
  pinMode(leftIR, INPUT_PULLUP);
  pinMode(rightIR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(leftIR), leftInterrupt, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(rightIR), rightInterrupt, CHANGE); 
}

/*
 * Set pin modes and initialize speed to zero
 */
void motorSetup()
{
  pinMode(leftMotor, OUTPUT);
  pinMode(rightMotor, OUTPUT);
}

void initStruct()
{
  //motor value
  //udp buffer input
  //odometry
}
 /*
  * no need to do IMU yet, need more info
  */
int checkIMU()
{
  /*
   * check imu for updates, return 1 if active 0 if not
   */
}

int[] getIMU()
{
  /*
   * return ax, ay, az
   */
}

void checkUDP()
{
  /*
   * check if UDP message has been sent
   * make sure non-blocking
   */
}

void leftInterrupt()
{
  deltaX = ((anglePerTickX * cos(deltaZ)) + (anglePerTickY * sin(deltaZ)));
  deltaY = ((anglePerTickX * sin(deltaZ)) + (anglePerTickY * cos(deltaZ)));
 
  x += deltaX;
  y -= deltaY;
  z += deltaZ;
}

void rightInterrupt()
{
 
  deltaX = ((anglePerTickX * cos(deltaZ)) + (anglePerTickY * sin(deltaZ)));
  deltaY = ((anglePerTickX * sin(deltaZ)) + (anglePerTickY * cos(deltaZ)));
  
  x += deltaX;
  y += deltaY;
  z -= deltaZ;
}
