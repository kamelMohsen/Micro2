#include <SoftwareSerial.h>
#include <Servo.h> 
//////////////////////////////////////////Defining constants for car ports//////////////////////////////////////////
#define enA 5
#define in1 4
#define in2 9
#define enB 6
#define in3 10 
#define in4 7 
#define servoPin 11 
//////////////////////////////////////////Defining constants for Ultra Sonic Sensor//////////////////////////////////////////
#define trigger 12
#define echo 13
//////////////////////////////////////////Defining constant for Reciever pin//////////////////////////////////////////
#define RX 2
//////////////////////////////////////////UltraSonic sensor Vars//////////////////////////////////////////
long time_taken ;
long dist;
long cm;
//////////////////////////////////////////Motors speed//////////////////////////////////////////
int motorSpeedA = 0;
int motorSpeedB = 0;
int yAxis=0;
int xAxis=0;
int servoAngle=90;
Servo Servo1;
//////////////////////////////////////////RX recieved data//////////////////////////////////////////
int data[2];
SoftwareSerial BTSerial(2, 3);
//////////////////////////////////////////Setup Function//////////////////////////////////////////
void setup() {

 
//////////////////////////////////////////Motor ports setup//////////////////////////////////////////        
      Servo1.attach(servoPin);
      pinMode(enA, OUTPUT);
      pinMode(enB, OUTPUT);
      pinMode(in1, OUTPUT);
      pinMode(in2, OUTPUT);
      pinMode(in3, OUTPUT);
      pinMode(in4, OUTPUT);
//////////////////////////////////////////UltraSonic sensor pins setup//////////////////////////////////////////
      pinMode(trigger, OUTPUT);
      pinMode(echo, INPUT);
      Serial.begin(9600);
      BTSerial.begin(38400);
}
//////////////////////////////////////////Loop Function//////////////////////////////////////////
void loop() {
//////////////////////////////////////////Serial printing of the data array//////////////////////////////////////////

//////////////////////////////////////////Recieving the Accelerometer data//////////////////////////////////////////
if(BTSerial.available() > 0)
 { 
    // Checks whether data is comming from the serial port
  do{
   while(BTSerial.read()!=255)
   {
   BTSerial.read();
   }
   data[0]=BTSerial.read();
   data[1]=BTSerial.read();
  }while(data[0]==255||data[1]==255);
    // Reads the data from the serial port
 }
   Serial.print("yAxis F/B : ");
   Serial.print(data[0]);
   Serial.print("xAxis R/L : ");
   Serial.print(data[1]);
   Serial.print(" Motor Speed: ");
   Serial.print(motorSpeedA);
   Serial.println();

      yAxis=data[0];
      xAxis=data[1];  
      servoAngle = map(xAxis,255,0,-40,220);
      Servo1.write(servoAngle);
     
//////////////////////////////////////////Moving//////////////////////////////////////////
if (yAxis < 100) 
{
//////////////////////////////////////////Set Motor A backward//////////////////////////////////////////
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
//////////////////////////////////////////Set Motor B backward//////////////////////////////////////////
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
//////////////////////////////////////////Convert the declining Y-axis readings for going backward////////////////////////////////////////// 
      motorSpeedA = map(yAxis, 0,100 , 0, 300);
      motorSpeedB = map(yAxis, 0,100 , 0, 300);
  }
  else if (yAxis > 156) 
  {
//////////////////////////////////////////Set Motor A forward//////////////////////////////////////////
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
//////////////////////////////////////////Set Motor B forward//////////////////////////////////////////
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
//////////////////////////////////////////Convert the increasing Y-axis readings for going forward//////////////////////////////////////////
      motorSpeedA = map(yAxis, 156, 255, 0, 300);
      motorSpeedB = map(yAxis, 156, 255, 0, 300);
  }
//////////////////////////////////////////If No Send data stays in middle the motors are not moving//////////////////////////////////////////
  else 
  {
      motorSpeedA = 0;
      motorSpeedB = 0;
  }
//////////////////////////////////////////X-axis used for left and right control//////////////////////////////////////////
  if (xAxis > 156) 
  {
//////////////////////////////////////////Convert the declining X-axis//////////////////////////////////////////
      int xMapped = map(xAxis, 0,100 , 0, 300);
//////////////////////////////////////////Move to left - decrease left motor speed, increase right motor speed//////////////////////////////////////////
      motorSpeedA = motorSpeedA - xMapped;
      motorSpeedB = motorSpeedB + xMapped;
//////////////////////////////////////////Confine the range from 0 to 255//////////////////////////////////////////
    if (motorSpeedA < 0) 
    {
      motorSpeedA = 0;
    }
    if (motorSpeedB > 255) 
    {
      motorSpeedB = 255;
    }
  }
  if (xAxis < 100)
  {
//////////////////////////////////////////Convert the increasing X-axis readings//////////////////////////////////////////
    int xMapped = map(xAxis, 156, 255, 0, 300);
//////////////////////////////////////////Move right - decrease right motor speed, increase left motor speed//////////////////////////////////////////
    motorSpeedA = motorSpeedA + xMapped;
    motorSpeedB = motorSpeedB - xMapped;
//////////////////////////////////////////Confine the range from 0 to 255//////////////////////////////////////////
    if (motorSpeedA > 255) 
    {
      motorSpeedA = 255;
    }
    if (motorSpeedB < 0) {
      motorSpeedB = 0;
    }
  }
//////////////////////////////////////////Prevent buzzing at low speeds////////////////////////////////////////// 
  if (motorSpeedA < 70) 
  {
      motorSpeedA = 0;
  }
  if (motorSpeedB < 70) 
  {
      motorSpeedB = 0;
  }
//////////////////////////////////////////UltraSonic stop before hitting


      
      cm=calculate_distance(trigger,echo);
      
if(cm<50 && (yAxis>156 || xAxis>156 || xAxis<100))
{
      motorSpeedA=0; 
      motorSpeedB =0;
}



        analogWrite(enA, motorSpeedA ); // Send PWM signal to motor A
    
        analogWrite(enB,  motorSpeedB); // Send PWM signal to motor B 
}

//////////////////////////////////////////functions used//////////////////////////////////////////
float calculate_distance(int trig,int ech)
{
      digitalWrite(trig, LOW);
      delayMicroseconds(2);
      digitalWrite(trig, HIGH);
      delayMicroseconds(10);
      digitalWrite(trig, LOW);
      
      time_taken = pulseIn(ech, HIGH);
      dist= time_taken*0.034/2;
      return dist;
}
