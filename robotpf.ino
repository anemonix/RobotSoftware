/*
 * Libraries and files needed
 */
#include <math.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi101.h>
#include <WiFiUdp.h>
#include "arduino_secrets.h"
#include <LSM303.h>

/*
 * Constants
 */
#define leftIRSensor 5
#define rightIRSensor 6
#define leftMotorForward 12
#define leftMotorBackward 11
#define rightMotorForward 10
#define rightMotorBackward 9
#define baseline 70
#define wheelRadius 17.5
#define BUFFSIZE 1000
#define SPEEDKP 4
#define PHIKP 1.5
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;   
int status = WL_IDLE_STATUS;
IPAddress ip(192, 168, 1, 23);
char receiveBuffer[BUFFSIZE];
LSM303 compass;
WiFiUDP udp;
unsigned int localPort = 8313;
float circRobot = 2*PI*baseline;
float circWheel = 2*PI*wheelRadius;
float tickWheelDistance=circWheel/(75.8*2);
float tickPhiRad = tickWheelDistance/baseline;
float deltaX = (baseline/2) * sin( tickPhiRad );
float deltaY = (baseline/2) - ((baseline/2)*cos(tickPhiRad));
float xGlobal = 0, yGlobal = 0, phiGlobalRad = 0, phiGlobalDeg = 0;
int tickCountLeft = 0, tickCountRight = 0;
float velLeft = 0, velRight = 0;
float errorLeft = 0, errorRight = 0, errorPhi = 0;
float velocityLeft = 0, velocityRight = 0, omega = 0;
float leftMotorSet = 0, rightMotorSet = 0;
bool turnLeft = false;
float startingHeadingDeg, startingHeadingRad;
float relativeHeadingDeg, relativeHeadingRad;
float currentHeadingRad, averageHeadingRad, errorHeadingRad;
float currentHeadingDeg, averageHeadingDeg = 0, errorHeadingDeg, absDiff;
float verticalAcceleration = 0;
int t1 = 0, t2 = 0;

/*
 * Structs
 */
typedef struct cmdPacket{
  double vel;
  double phi;
  int mode;
} cmdPacket;

cmdPacket _cmdPacket;

typedef struct rtnPacket{
  double x;
  double y;
  double head;
} rtnPacket;

rtnPacket _rtnPacket;


/*
 * Set up Motor pins and set them all to 0
 */
void setupPins()
{
  
  pinMode(leftMotorForward, OUTPUT);
  pinMode(leftMotorBackward, OUTPUT);
  pinMode(rightMotorForward, OUTPUT);
  pinMode(rightMotorBackward, OUTPUT);
  analogWrite(leftMotorForward, 0);
  analogWrite(leftMotorBackward, 0);
  analogWrite(rightMotorForward, 0);
  analogWrite(rightMotorBackward, 0);
  pinMode(leftIRSensor, INPUT_PULLUP);
  pinMode(rightIRSensor, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(leftIRSensor), leftInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(rightIRSensor), rightInterrupt, RISING);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

/*
 * Sets motor to 0 and robot to face north with 0 velocity
 */
void setup()
{
  Serial.begin(9600);
  setupIMU();
  setupPins();
  setupWiFi();
  setupPort();

  analogWrite(leftMotorForward, 0);
  analogWrite(leftMotorBackward, 0);
  analogWrite(rightMotorForward, 0);
  analogWrite(rightMotorBackward, 0);

  _cmdPacket.vel = 0;
  _cmdPacket.phi = 270;
  _cmdPacket.mode = 2;
  
}


/*
 * Set up wifi connection
 */
void setupWiFi()
{
  WiFi.setPins(8,7,4,2);
  WiFi.begin(ssid,pass);
  delay(10000);
}

/*
 * Set up local port
 */
void setupPort()
{
  udp.begin(localPort);
}

/*
 * Setting up the IMU (calibration values and all)
 */
void setupIMU()
{
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>){-4848, -2881, -1357};
  compass.m_max = (LSM303::vector<int16_t>){+1136, +2758, +3524};
  startingHeadingDeg = compass.heading();
}

/*
 * Checks constantly for updates and send info to left/right motors
 */
void loop()
{
  updateIMU();
  checkUDP();
  rightMotorKP(_cmdPacket.vel);
  leftMotorKP(_cmdPacket.vel);
  phiKP(_cmdPacket.phi);
  timeUpdate();
}

/*
 * Count ticks in left wheel to calculate distance
 */
void leftInterrupt()
{
  tickCountLeft += 1;
  phiGlobalRad -= tickPhiRad;
  phiGlobalDeg = phiGlobalRad * 180.0 / PI;

  xGlobal += ((deltaX * cos(phiGlobalRad)) + deltaY*sin(phiGlobalRad));
  yGlobal += ((deltaX * sin(phiGlobalRad)) + deltaY*cos(phiGlobalRad));
}

/*
 * Count ticks in right wheel to calculate distance
 */
void rightInterrupt()
{
  tickCountRight += 1;
  phiGlobalRad += tickPhiRad;
  phiGlobalDeg = phiGlobalRad * 180.0 / PI;

  xGlobal += ((deltaX * cos(phiGlobalRad)) + deltaY*sin(phiGlobalRad));
  yGlobal += ((deltaX * sin(phiGlobalRad)) + deltaY*cos(phiGlobalRad));
}

/*
 * Adjust left motor
 */
void leftMotorKP( double speedLeft )
{
  velLeft = tickCountLeft/(2*75.8);
  tickCountLeft = 0;
  errorLeft = speedLeft - velLeft;
  velocityLeft = SPEEDKP * errorLeft;
}

/*
 * Adjust right motor
 */
void rightMotorKP(double speedRight)
{
  velRight = tickCountRight/(2*75.8);
  tickCountRight = 0;
  errorRight = speedRight - velRight;
  velocityRight = SPEEDKP * errorRight;
}

/*
 * Adjusts phi angle heading
 */
void phiKP(double phiDesiredDeg)
{
  averageHeadingDeg = currentHeadingDeg;
  }
  errorHeadingDeg = phiDesiredDeg - averageHeadingDeg;

  if(0 == phiDesiredDeg)
  {
    if (averageHeadingDeg >= 0 && averageHeadingDeg < 30)
      turnLeft = true;
    else
    {
      errorHeadingDeg = phiDesiredDeg - ( 360 - averageHeadingDeg );
      turnLeft = false;
    }
    absDiff = 180 - abs( abs( phiDesiredDeg - averageHeadingDeg ) - 180 );

    if (absDiff < 20) 
      errorHeadingDeg = 0;
  }
  else
  {
    if(errorHeadingDeg >= 0)
      turnLeft = true;
    else
      turnLeft = false;
  }
  omega = PHIKP * errorHeadingDeg;
}

/*
 * Allows us to update our motor contorl value every 20 ms (50Hz)
 */
void timeUpdate()
{
  t1 = millis(); 
  if(t1 - t2 >= 20)
  {
    updateMotors();
    t2 = t1;
  }
}

/*
 * Handle pick up case = motors stop
 */
void pickup()
{
  digitalWrite(LED_BUILTIN, HIGH);
  analogWrite(leftMotorForward, 0);
  analogWrite(leftMotorBackward, 0);
  analogWrite(rightMotorForward, 0);
  analogWrite(rightMotorBackward, 0);
  while(true);
}

/*
 * Update IMU and check whether pick up or not
 */
void updateIMU()
{
  compass.read();
  currentHeadingDeg = compass.heading();
  currentHeadingRad = currentHeadingDeg*PI/180.0;

  relativeHeadingDeg = currentHeadingDeg - startingHeadingDeg;
  relativeHeadingRad = relativeHeadingDeg * PI/180.0;
  if(compass.a.z>4000)
    pickup();
  
}

/*
 * send response and update global positioning
 */
void sendResponse()
{
  _rtnPacket.x = xGlobal;
  _rtnPacket.y = yGlobal;
  _rtnPacket.head = averageHeadingDeg;
  udp.beginPacket(udp.remoteIP(), udp.remotePort());
  char trasmitBuffer[BUFFSIZE] = {0};
  memcpy(trasmitBuffer, &_rtnPacket, sizeof(rtnPacket));
  udp.write(trasmitBuffer, sizeof(rtnPacket));
  udp.endPacket();
}

/*
 * read inputs from UDP and decipher which state is needed
 */
void readPacket()
{
  int len = udp.read(receiveBuffer, BUFFSIZE);
  if (len>0)
  {
    memcpy(&_cmdPacket, receiveBuffer, sizeof(cmdPacket));
    switch (_cmdPacket.mode)
    {
      case 0:
        Serial.println(_cmdPacket.vel);
        Serial.println(_cmdPacket.phi);
        sendResponse();
        break;
      case 1:
        Serial.println( _cmdPacket.vel );
        sendResponse();
        break;
      case 2:
        Serial.println(_cmdPacket.vel);
        Serial.println(_cmdPacket.phi);
        sendResponse();
        break;
      case 3:
        pickup();
        break;
    }
  }
}

/*
 * Checks UDP inputs to see what command need
 */
void checkUDP()
{
  int packetSize = udp.parsePacket();
  if(packetSize)
  {
    readPacket();
  }
}

void updateMotors()
{
  if(_cmdPacket.mode == 2)
  {
    if(!turnLeft)
    {
      rightMotorSet = (abs(omega*baseline))/(2*wheelRadius);
      leftMotorSet = 0;
    }
    else
    {
      rightMotorSet = 0;
      leftMotorSet = (abs(omega*baseline))/(2*wheelRadius);
    }
  }
  else
  {
    rightMotorSet = 2 * velocityRight;
    leftMotorSet = 2 * velocityLeft;
  }

  if( leftMotorSet > 100) leftMotorSet = 100;
  else if ( leftMotorSet < 0 ) leftMotorSet = 0;

  if( rightMotorSet > 100) rightMotorSet = 100;
  else if ( rightMotorSet < 0 ) rightMotorSet = 0;

  analogWrite(leftMotorForward, leftMotorSet);
  analogWrite(leftMotorBackward, 0);
  analogWrite(rightMotorForward, rightMotorSet);
  analogWrite(rightMotorBackward, 0);
}
