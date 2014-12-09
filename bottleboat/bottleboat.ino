
//  Servo = 2  GPS = 4(read) Compass = A4(Data) A5(CLK) Motor = 8

#include "Wire.h"
#include "I2Cdev.h"
#include "HMC5883L.h"
//#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Servo.h>
//#include "softare_uart.c"

HMC5883L mag;
int16_t mx, my, mz;

TinyGPSPlus gps;
//SoftwareSerial ss(4, 3);  // Reading from 4
int const gpsPin = 4;
int const gpsBaud = 4800;

Servo rudder;

// Angle variables
float boatDir = 5; // Direction of the boat from the compass
float marginOerror = 15;
float desiredHeading = 0;
float resdeg = 0;   // The degrees off course
double distanceKm = 0.01;

// GPS variables
float currentLat = 0;
float currentLong = 0;
float waypointLat = 52.41122;
float waypointLong = -4.08785;
int sat = 0;

//52.41122 - 52.41129 - 52.41150
//-4.08785 - -4.08766 - -4.08765

long timer = 0;
bool run = true;
int motorstatus = LOW;

int waypointCounter = 0;
float coordinates[2][7] = {{52.41122, 52.41129, 52.41150, 52.41122, 52.41129, 52.41150, 52.41122},  //Lat
                           {-4.08785, -4.08766, -4.08765, -4.08785, -4.08766, -4.08765, -4.08785}}; //Lon

//Pin
//int ledright = 2;
//int ledleft = 13;
int motorPin = 8;



// bits per second, measured in microseconds.
#define HALF_BIT_4800 104;
#define FULL_BIT_4800 208;





unsigned char uart_rx(int pin, int baud) {
    unsigned char rec = 0;
    // double pause = 0.0;
    // double half_pause = 0.0;
    int i = 0;

    // this is a bit pointless for now. we're only reading the GPS at 4800bps.
    // but might want to read it quicker in future...
    /* switch(baud) {
        case 4800:
        default:
            pause = FULL_BIT_4800;
            half_pause = HALF_BIT_4800;
            break;
    } */

    double pause = FULL_BIT_4800;
    double half_pause = HALF_BIT_4800;

    start_again:
        rec = 0;
        while(digitalRead(pin) == LOW);  // wait for line to be high
        while(digitalRead(pin) == HIGH);  // wait for line to fall (start bit)
        delayMicroseconds(half_pause);

        if(digitalRead(pin) != LOW) goto start_again;
        delayMicroseconds(pause);

        if(digitalRead(pin) == HIGH) rec += 1;
        delayMicroseconds(pause);

        if(digitalRead(pin) == HIGH) rec += 2;
        delayMicroseconds(pause);

        if(digitalRead(pin) == HIGH) rec += 4;
        delayMicroseconds(pause);

        if(digitalRead(pin) == HIGH) rec += 8;
        delayMicroseconds(pause);

        if(digitalRead(pin) == HIGH) rec += 16;
        delayMicroseconds(pause);

        if(digitalRead(pin) == HIGH) rec += 32;
        delayMicroseconds(pause);

        if(digitalRead(pin) == HIGH) rec += 64;
        delayMicroseconds(pause);

        if(digitalRead(pin) == HIGH) rec += 128;
        delayMicroseconds(pause);

        if(digitalRead(pin) != HIGH) goto start_again;

    return rec;
}


void uart_tx(char data, int baud, int pin) {
    /* double pause = 0;

    switch(baud) {
        case 4800:
        default:
            pause = FULL_BIT_4800;
            break;
    } */

    double pause = FULL_BIT_4800;

    digitalWrite(pin, LOW);  // transmit start bit
    delayMicroseconds(pause);

    if(data & 1) digitalWrite(pin, HIGH);
    else digitalWrite(pin, LOW);
    delayMicroseconds(pause);

    if(data & 2) digitalWrite(pin, HIGH);
    else digitalWrite(pin, LOW);
    delayMicroseconds(pause);

    if(data & 4) digitalWrite(pin, HIGH);
    else digitalWrite(pin, LOW);
    delayMicroseconds(pause);

    if(data & 8) digitalWrite(pin, HIGH);
    else digitalWrite(pin, LOW);
    delayMicroseconds(pause);

    if(data & 16) digitalWrite(pin, HIGH);
    else digitalWrite(pin, LOW);
    delayMicroseconds(pause);

    if(data & 32) digitalWrite(pin, HIGH);
    else digitalWrite(pin, LOW);
    delayMicroseconds(pause);

    if(data & 64) digitalWrite(pin, HIGH);
    else digitalWrite(pin, LOW);
    delayMicroseconds(pause);

    if(data & 128) digitalWrite(pin, HIGH);
    else digitalWrite(pin, LOW);
    delayMicroseconds(pause);

    digitalWrite(pin, HIGH); // transmit stop bit
    delayMicroseconds(pause);
}


void uart_str(char *str, int baud, int pin) {
    int i;
    for(i=0; i<strlen(str); i++) {
        uart_tx(str[i], baud, pin);
    }
}




void setup()
{
  //ss.begin(4800);
  Wire.begin();
  Serial.begin(115200);
  
  pinMode(motorPin, OUTPUT);
  pinMode(gpsPin, INPUT);
  
  rudder.attach(2);
  
  Serial.println("Initializing I2C devices...");
  mag.initialize();
  
  Serial.println("Testing device connections...");
  Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");
  
  rudder.write(90);
}

void loop()
{
  //readserial();
  
  if (run)
  {
    if (motorstatus == LOW)
      motor("ON");
      
    getCoordinates();
    turnRudder(turnDir());
    
    
    if (distanceKm < 0.020)// we will try to get within 20meters of the target.
    {
       waypointCounter ++;
       waypointLat = coordinates[0][waypointCounter];
       waypointLong = coordinates[1][waypointCounter];
       Serial.print("\n\nNext waypoint: ");
       Serial.print(waypointLat);
       Serial.print(" , ");
       Serial.print(waypointLong);
    }
  }
  
}

//Reading Serial
void readserial()
{
  int serialVal = 0;
  while (millis() < timer || !run)
  {
    serialVal = 0;
    
    while (!Serial.available())
    {
      if (millis() > timer && run)
      {
        return;
      }
    }
    
    serialVal = Serial.parseInt();
    Serial.println(serialVal);
    switch (serialVal)
    {
      case 1:
      {
        turnRudder("Left");
        Serial.println("Left");
        break;
      }
      case 2:
      {
        turnRudder("Right");
        break;
      }
      case 3:
      {
        turnRudder("NT");
        break;
      }
      case 4:
      {
        if (motorstatus == LOW)
          motor("ON");
        else
          motor("OFF");
        break;
      }
      case 5:
      {
        if (!run)
          run = true;
        else
          run = false;
        break;
      }
      default:
        break;
    }
  }
}


void motor(String motorPower)
{
  if (motorPower == "ON")
    {digitalWrite(motorPin, HIGH); motorstatus = HIGH;Serial.println("Motor ON");}
  else if (motorPower == "OFF")
    {digitalWrite(motorPin, LOW); motorstatus = LOW;Serial.println("Motor OFF");}
}


void turnRudder(String rudderDir)
{
  if (rudderDir == "Left")
    {rudder.write(50);
    Serial.println("Left");}
  else if (rudderDir == "Right")
    {rudder.write(130);
    Serial.println("Right");}
  else if (rudderDir == "NT")
    {rudder.write(90);
    Serial.println("Not Turning");}
}

String turnDir()
{
  // read raw heading measurements from device
  mag.getHeading(&mx, &my, &mz);
  
  // To calculate heading in degrees. 0 degree indicates North
  float heading = atan2(my, mx);
  if(heading < 0)
    heading += 2 * M_PI;
  boatDir = (heading * 180/M_PI);
  
  desiredHeading = findBearing();
  
  Serial.print("\nCurrent Heading: ");
  Serial.print(boatDir);
  Serial.print("\nDesired Heading: ");
  Serial.print(desiredHeading);
  Serial.print("\nBearing Difference: ");
  Serial.print(boatDir - desiredHeading);
  
  
  
  
  int BearingDifference = boatDir - desiredHeading;
	
  if (abs(BearingDifference) < marginOerror) {
      return("NT");
  }
	
//  /*Initial quick check for 180 degree turn 
//  (direction is arbitrary in this case)*/
//  if (BearingDifference == 180) {
//      return("Right");//could be turn left
//  }
  //Which way should the boat turn
  //we could add in by how much to turn if needed
  if (BearingDifference > 0 && BearingDifference <= 180) {
    return("Left");
  } else if (BearingDifference > (-360) && BearingDifference < (-180)) {
    return("Left");
  } else if (BearingDifference >= (-180) && BearingDifference < 0) {
    return("Right");
  } else if (BearingDifference > 180 && BearingDifference < 360) {
    return("Right");
  } else {
    Serial.println("Error: variable out of range");
  }
}


//Find angle between coordinates

float findBearing()
{
  float pi = 3.1416;
  float bearing = 0;
  
  float dLat = waypointLat - currentLat;
  
  float dLong = waypointLong - currentLong;
  
  float meanLat = currentLat + (dLat/2);
  
  float departure = dLong * cos((meanLat * pi) / 180);
  
  float theta = atan(departure / dLat);
  
  theta = (theta * 180) / pi;  //Converting from radians to degrees
  
  if (dLat >= 0)
  {
     if(dLong >= 0)
     {
        bearing = theta; 
     }
     else
     {
        bearing = 360 + theta;
     }
  }
  else
  {
      bearing = 180 + theta; 
  }
  
  return bearing;
}

void getCoordinates()
{
  char gps_string[255];
  int i=0;
  gps_string[0]=0;
  char gps_char;
  boolean started=false;
  while(i<254) {
    gps_char = uart_rx(gpsPin,gpsBaud);
    if(gps_char=='$')
    {
      started=true;
    }
    if(started)
    {
      gps_string[i] = gps_char;
      i++;
      if(gps_char=='\r')
      {
        break;
      }
    }
  }
  gps_string[i] = 0;
  if(gps_string[1]=='G'&&gps_string[2]=='P'&&gps_string[3]=='R'&&gps_string[4]=='M'&&gps_string[5]=='C')
  {
    for(i=0;i<strlen(gps_string);i++)
    {
      gps.encode(gps_string[i]);
    }
  }
  
  if (gps.location.isValid())
  {
    currentLat = gps.location.lat(), 6;
    currentLong = gps.location.lng(), 6;
    
    // Distance between boat and waypoint
    distanceKm = gps.distanceBetween(currentLat, currentLong, waypointLat, waypointLong) / 1000.0;
    Serial.print("\ndistanceKm = ");
    Serial.println(distanceKm);
    
    Serial.print(currentLat);
    Serial.print(F(","));
    Serial.print(currentLong);
  }
  else
  {
    Serial.print(F("INVALID"));
  }
  
//  bool newData = false;
//  while (!newData)
//  {
//    unsigned long chars;
//    unsigned short sentences, failed;
//  
//    // For one second we parse GPS data and report some key values
//    for (unsigned long start = millis(); millis() - start < 1000;)
//    {
//      // THIS IS THE OLD WAY TO READ FROM GPS WITH INTERRUPTS
////      while (ss.available())
////      {
////        char c = ss.read();
////        // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
////        if (gps.encode(c)) // Did a new valid sentence come in?
////          newData = true;
////      }
//
//      //NEW LIBRARY
//      for(int i = 0; i<256; i++)
//      {
//        char c = uart_rx(gpsPin,gpsBaud);
//        Serial.write(c); // uncomment this line if you want to see the GPS data flowing
//        if (gps.encode == c) // Did a new valid sentence come in?
//          {newData = true;Serial.println("TRUE");break;}
//      }
//    }
//  
//    if (newData)
//    {
//      if (gps.location.isValid())
//      {
//        float flat, flon;
//        flat = gps.location.lat(), 6;
//        flon = gps.location.lng(), 6;
//        
//        Serial.print("\n\n");
//        Serial.print(flat);
//        Serial.print(" , ");
//        Serial.print(flon);
//        
//        // Distance between boat and waypoint
//        distanceKm = gps.distanceBetween(gps.location.lat(), gps.location.lng(), waypointLat, waypointLong) / 1000.0;
//        Serial.print("\ndistanceKm = ");
//        Serial.println(distanceKm);
//      }
//      else
//      {
//        Serial.print("INVALID");
//      }
//    }
//  }
}
