/*
UIA-AGV-SLAM Commissioning Code
This is a test code for testing the following:
- Pin Assignment
- ROS subscriber from teleop_twist_keyboard
- Basic Motor Movement with ROS
- Emergency Stop
- Mapping ROS velocity (linear X(m/s) and angular Z(rad/s)) to Arduino Due PWM (229 - 0) (10% - 100%) 

ProStrain Technologies
By Zharif Zubaidi
*/
//ROS Setup
#define USE_USBCON //Need to be used for Arduino Due with ROS
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;
std_msgs::Float32 veltemp_msg;

//Variable Declaration
double linearXVel=0, angularZVel=0;
float linearXVelArd=0, angularZVelArd=0;

//Functions - Start//

//ROS Related Functions
//Velocity subscriber callback function
void cmd_vel_callback(const geometry_msgs::Twist& twist){
  linearXVel = twist.linear.x;
  angularZVel = twist.angular.z;
  nh.loginfo("Arduino received twist");
}

//Arduino Related Functions
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//Functions - Stop//

//ROS Publisher and Subcriber Node Declaration
//Publisher
ros::Publisher vel_pub("velocity", &veltemp_msg);
//Subscriber
ros::Subscriber<geometry_msgs::Twist>cmd_vel_sub("cmd_vel",cmd_vel_callback);

//Pin Assignment
//Digital Input Pins
int LSR_Out1 = 42; int LSR_Out2 = 41; int LSR_Out3 = 40;                //Digital Input Lidar
int SW_MODE = 38; int DOCK_STATUS = 37;                                 //Digital Input General
int MAG_D1 = 66; int MAG_D2 = 67; int MAG_D3 = 68; int MAG_D4 = 69; int MAG_D5 = 49; int MAG_D6 = 48; int MAG_D7 = 47; int MAG_D8 = 32; //Digital Input Magnetic Sensor

#define LH_ENA 44
#define LH_ENB 43
#define RH_ENA 46
#define RH_ENB 45
#define ES_SIG 39

//Analog Input
int IR1 = A0; int IR2 = A1; int IR3 = A2; int CSENS = A3; //J2 - A0,A1,A2,A3,A4,A5
int NTC1 = 62; int NTC2 = 63; int BAT_LVL = 64;           //J3 - 60,61,62,63,64,65 (A6-A11)

//Digital Output Pins
int BR_SIG = 29; int RH_D2 = 28 ; int RH_D3 = 27; int LH_D2 = 26; int LH_D3 = 25;  int ES_LED = 24;                                //Digital Output
int RH_D1 = 7; int LH_D1 = 8; int LED_FR_RH = 9; int LED_FR_LH = 10 ; int LED_RR_RH = 11 ;int LED_RR_LH = 12; int DOCK_ACT = 13;   //Digital Output (PWM)

int count = 0;

volatile unsigned int encoder_RH = 0;
volatile unsigned int encoder_LH = 0;

//Setup Function - Start//

void setup() {
  //ROS Node Initiate
  Serial.begin(57600);
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.advertise(vel_pub);
  
  //Arduino Setup
  pinMode(LH_ENA, INPUT);  pinMode(LH_ENB, INPUT);pinMode(RH_ENA, INPUT);pinMode(RH_ENB, INPUT);pinMode(SW_MODE,INPUT); pinMode(ES_SIG,INPUT);
  digitalWrite(LH_ENA, HIGH); digitalWrite(LH_ENB, HIGH); digitalWrite(RH_ENA, HIGH);digitalWrite(RH_ENB, HIGH); digitalWrite(SW_MODE,HIGH); digitalWrite(ES_SIG,HIGH);
  
  pinMode(BR_SIG,OUTPUT);pinMode(LH_D2,OUTPUT);pinMode(LH_D3,OUTPUT);pinMode(RH_D2,OUTPUT);pinMode(RH_D3,OUTPUT);
  pinMode(LH_D1, OUTPUT);pinMode(RH_D1, OUTPUT);pinMode(DOCK_ACT, OUTPUT);

  analogWrite(RH_D1,229);analogWrite(LH_D1,229);digitalWrite(BR_SIG,HIGH);
 
  digitalWrite(RH_D2,LOW);digitalWrite(RH_D3,LOW);
  digitalWrite(LH_D2,LOW);digitalWrite(LH_D3,LOW);
  digitalWrite(BR_SIG,HIGH);digitalWrite(DOCK_ACT,HIGH); //BR_SIG High == Release Brake, DOCK_ACT High == Not Dock
}

//Setup Function - End//


//Loop/Main Function - Start//

void loop() {
  nh.spinOnce();
  //Convert X linear velocity (m/s) to PWM
  float linearXVelf=float(linearXVel);
  
  float angularZVelf=float(angularZVel);

  //Original conversion code
  /*linearXVelArd=mapfloat(linearXVelf,0,1,229,0);
  linearXVelArd=round(linearXVelArd);
  int linearXVelArdInt=linearXVelArd;
  
  angularZVelArd=mapfloat(angularZVelf,0,1,229,0);
  angularZVelArd=round(angularZVelArd);
  int angularZVelArdInt=angularZVelArd;*/
 
 if(digitalRead(ES_SIG) == HIGH){ 
 //Move forward
   if(linearXVelf>0 && angularZVelf==0){
    
     linearXVelArd=mapfloat(linearXVelf,0,1,229,0);
     linearXVelArd=round(linearXVelArd);
     int linearXVelArdInt=linearXVelArd;
      
     digitalWrite(RH_D2,HIGH); //D2 Enable Motor
     digitalWrite(LH_D2,HIGH);
     digitalWrite(RH_D3,LOW);  //D3 - Direction Forward - Note that RH is inverse compare to LH
     digitalWrite(LH_D3,HIGH); //Forward
     analogWrite(RH_D1,linearXVelArdInt); //D1 PWM aka motor speed
     analogWrite(LH_D1,linearXVelArdInt);
   }
   else if(linearXVelf<0 && angularZVelf==0){
     linearXVelf=abs(linearXVelf);
     linearXVelArd=mapfloat(linearXVelf,0,1,229,0);
     linearXVelArd=round(linearXVelArd);
     int linearXVelArdInt=linearXVelArd;
     
     digitalWrite(RH_D2,HIGH);  //D2 Enable Motor
     digitalWrite(LH_D2,HIGH);
     digitalWrite(RH_D3,HIGH);  //D3 - Direction Reverse - Note that RH is inverse compare to LH
     digitalWrite(LH_D3,LOW);   //Reverse
     analogWrite(RH_D1,linearXVelArdInt); //D1 PWM aka motor speed
     analogWrite(LH_D1,linearXVelArdInt);
   }
   else if(linearXVelf==0 && angularZVelf>0){
     angularZVelArd=mapfloat(angularZVelf,0,2,229,0); //CCW
     angularZVelArd=round(angularZVelArd);
     int angularZVelArdInt=angularZVelArd;
     
     digitalWrite(RH_D2,HIGH);  //D2 Enable Motor
     digitalWrite(LH_D2,HIGH);
     digitalWrite(RH_D3,LOW);  //D3 - Direction Reverse - Note that RH is inverse compare to LH
     digitalWrite(LH_D3,LOW);  //Reverse
     analogWrite(RH_D1,angularZVelArdInt); //D1 PWM aka motor speed
     analogWrite(LH_D1,angularZVelArdInt);
   }
   else if(linearXVelf==0 && angularZVelf<0){
     angularZVelf=abs(angularZVelf); //CW
     angularZVelArd=mapfloat(angularZVelf,0,2,229,0);
     angularZVelArd=round(angularZVelArd);
     int angularZVelArdInt=angularZVelArd;
     
     digitalWrite(RH_D2,HIGH);  //D2 Enable Motor
     digitalWrite(LH_D2,HIGH);
     digitalWrite(RH_D3,HIGH);   //D3 - Direction Reverse - Note that RH is inverse compare to LH
     digitalWrite(LH_D3,HIGH);   //Reverse
     analogWrite(RH_D1,angularZVelArdInt); //D1 PWM aka motor speed
     analogWrite(LH_D1,angularZVelArdInt);
   }      
   else{
    //Stop all motors
    digitalWrite(LH_D2,LOW);
    digitalWrite(RH_D2,LOW);
    digitalWrite(LH_D3,HIGH);
    digitalWrite(RH_D3,LOW);
    analogWrite(RH_D1,229);
    analogWrite(LH_D1,229);   
   }
   
 }
 else {                        
    //Stop all motors
    digitalWrite(LH_D2,LOW);
    digitalWrite(RH_D2,LOW);
    digitalWrite(LH_D3,HIGH);
    digitalWrite(RH_D3,LOW);
    analogWrite(RH_D1,229);
    analogWrite(LH_D1,229);
  }
  
 veltemp_msg.data=angularZVelf;
 vel_pub.publish(&veltemp_msg);
 delay(10);        // delay in between reads for stability
}

//Loop/Main Function - End//  
