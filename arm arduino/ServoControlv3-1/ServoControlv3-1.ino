
#if (ARDUINO >= 100) 
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
//#define USE_USBCON



#include "Wire.h"
#include "Arduino.h"
#include "ArduinoHardware.h"

#include <Servo.h>
#include <SCServo.h>
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <ros/time.h>


SMS_STS sms;
SMS_STS sts;

Servo myservo; 


u16 Speed[3];
byte ACC[3];

u16 Speed1[3];
byte ACC1[3];

uint8_t ID_SMS[3] = {0,1,2};
uint8_t ID_STS[2] = {0,1};
uint8_t rxPacket[4];
int16_t Posi_sms[3];
int16_t Posi_sms1[3];
int16_t Posi_sms2[3];
int16_t Posi_sts[3];
int16_t Posi_sts1[3];
int16_t Posi_sts2[3];
int16_t Posi_servo;



int counteropen = 0;
int counterclose = 0;



ros::NodeHandle  nh;

std_msgs::Float64MultiArray motorPositionMsg;
ros::Publisher motorPositionPublisher("motor_position", &motorPositionMsg);

void servo_cb( const sensor_msgs::JointState& arm_step){
   
   //read joint_01
   Posi_sms2[0] = -(radiansToPosition(arm_step.position[0]))+ Posi_sms1[0];
  
   //read joint_02
   Posi_sms2[1] = -(radiansToPosition(arm_step.position[1]))+ Posi_sms1[1];
  
   //read joint_03
   Posi_sms2[2] = -(radiansToPosition(arm_step.position[2]))+ Posi_sms1[2];
   
   //read joint_04
   Posi_sts2[0] = (radiansToPosition(arm_step.position[3]))+ Posi_sts1[0];
   
   //read joint_05
   //Posi_sts2[1] = -(radiansToPosition(arm_step.position[4]))+ Posi_sts1[1];
   //Posi_servo = arm_step.position[4]+60;
    
  
   if(arm_step.position[5]< -0.01 || arm_step.position[5]> 0.01 ){
    Posi_sts2[2] =-1200;
    counteropen++;
    counterclose = 0;

   }
   else{
    Posi_sts2[2] =-2895;
    counteropen = 0;
    counterclose++;

   }
   
   sms.SyncWritePosEx(ID_SMS,3, Posi_sms2, Speed, ACC);
   sts.SyncWritePosEx(ID_STS,1, Posi_sts2, Speed1, ACC1);
   //myservo.write(Posi_servo);
   
   if(counteropen==1){
    sts.WritePosEx(1, Posi_sts2[2], 800, 100);
      
   }

   if(counterclose==1){
    sts.WritePosEx(1, Posi_sts2[2], 800, 100);
      
   }

   Serial3.println("read values");
   Serial3.println(Posi_sms1[0]);
   Serial3.println(Posi_sms1[1]);
   Serial3.println(Posi_sms1[2]);
   Serial3.println(Posi_sts1[0]);
   Serial3.println(Posi_sts1[1]);
   Serial3.println(Posi_sts1[2]);

   Serial3.println("joint state values");
   Serial3.println(arm_step.position[0]);
   Serial3.println(arm_step.position[1]);
   Serial3.println(arm_step.position[2]);
   Serial3.println(arm_step.position[3]);
   Serial3.println(arm_step.position[4]);
   //Serial3.println(Posi_sts2[2]);
   //Serial3.println("blah");


   //motor feedback
   getposition();
   motorPositionMsg.data[0] = positionToRadians(-(Posi_sms[0]-Posi_sms1[0]));
   motorPositionMsg.data[1] = positionToRadians(-(Posi_sms[1]-Posi_sms1[1]));
   motorPositionMsg.data[2] = positionToRadians(-(Posi_sms[2]-Posi_sms1[2]));
   motorPositionMsg.data[3] = positionToRadians(Posi_sts[0]-Posi_sts1[0]);
   motorPositionMsg.data[4] = positionToRadians(-(Posi_sts[1]-Posi_sts1[1]));
   motorPositionPublisher.publish(&motorPositionMsg);
 
}



void getposition(){
  //read current motor positions
  sms.syncReadPacketTx(ID_SMS, sizeof(ID_SMS), SMS_STS_PRESENT_POSITION_L, sizeof(rxPacket));
  Posi_sts[0] = sts.ReadPos(0);
  //Posi_sts[1] = sts.ReadPos(1);
  //Posi_sts[2] = sts.ReadPos(2);
 
  int8_t i;
  for(i=0; i<sizeof(ID_SMS); i++){
    if(!sms.syncReadPacketRx(ID_SMS[i], rxPacket)){
     Serial.print("ID:");
     Serial.println(ID_SMS[i]);
     Serial.println("sync read error!");
     continue;
    }
    
    Posi_sms[i] = sms.syncReadRxPacketToWrod(15);

  }
 
  delay(10);
}


ros::Subscriber<sensor_msgs::JointState> sub("joint_states", servo_cb);



void setup(){
  //Serial.begin(57600);
  Serial1.begin(115200);
  Serial2.begin(1000000);
  Serial3.begin(57600);
  sms.pSerial = &Serial1;
  sts.pSerial = &Serial2;
  myservo.attach(9);
  sms.syncReadBegin(sizeof(ID_SMS), sizeof(rxPacket), 5);
  ID_SMS[0] = 0;
  ID_SMS[1] = 1;
  ID_SMS[2] = 2;
  ID_STS[0] = 0;
  ID_STS[1] = 1;
  Speed[0] = 800;
  Speed[1] = 800;
  Speed[2] = 800;
  ACC[0] = 40;
  ACC[1] = 40;
  ACC[2] = 40;
  Speed1[0] = 1500;
  Speed1[1] = 1500;
  ACC1[0] = 40;
  ACC1[1] = 40;
  delay(10);
 
  getposition();
  Posi_sms1[0]=Posi_sms[0];
  Posi_sms1[1]=Posi_sms[1];
  Posi_sms1[2]=Posi_sms[2];
  Posi_sts1[0]=Posi_sts[0];
  //Posi_sts1[1]=Posi_sts[1];
  //Posi_sts1[2]=Posi_sts[2];
  delay(10);

  //read joint01
  if(2343<Posi_sms1[0] && Posi_sms1[0]<2363){
    Posi_sms1[0] = Posi_sms1[0] ;
  }
  else{
    sms.WritePosEx(0,2353 , 5, 5);
    Posi_sms1[0] = 2353;
  }

  //read joint02
  if(1035<Posi_sms1[1] && Posi_sms1[1]<1055){
    Posi_sms1[1] = Posi_sms1[1] ;
  }
  else{
    sms.WritePosEx(1,1045 , 5, 5);
    Posi_sms1[1] = 1040;
  }

  //read joint03
  if(1040<Posi_sms1[2] && Posi_sms1[2]<1060){
    Posi_sms1[2] = Posi_sms1[2] ;
  }
  else{
    sms.WritePosEx(2,1050 , 5, 5);
    Posi_sms1[2] = 1050;
  }

  //read joint04
  if(1690<Posi_sts1[0] && Posi_sts1[0]<1710){
    Posi_sts1[0] = Posi_sts1[0] ;
  }
  else{
    sts.WritePosEx(0,1700 , 1000, 40);
    Posi_sts1[0] = 1700;
  }

  //read joint05
  //if(2150<Posi_sts1[1] && Posi_sts1[1]<2170){
  Posi_servo = 60 ;
  myservo.write(Posi_servo);
  //}
  //else{
    //sts.WritePosEx(1,2160 , 1000, 40);
    //Posi_sts1[1] = 2190;
  //}

  //sts.WritePosEx(1, -1200, 140, 1200);

  nh.getHardware()->setBaud(128000);
  nh.initNode();
  nh.subscribe(sub);

  nh.advertise(motorPositionPublisher);
  motorPositionMsg.data_length = 6;
  motorPositionMsg.data = new float[6];
  
  
  //std_msgs::MultiArrayLayout layout;
  //std_msgs::MultiArrayDimension dim;
  //dim.size = 3;
  //dim.stride = 1;
  //layout.dim = new std_msgs::MultiArrayDimension[1];  // allocate memory for dim pointer
  //layout.dim[0] = dim;  // assign dim to the first element of dim pointer
  //layout.dim[0].label = "joint_positions"; // assign a label to the first dimension
  //motorPositionMsg.layout = layout;

  //motorPositionMsg.data_length = layout.dim[0].size;
  
  Serial3.println("done");

   
}

void loop(){
  nh.spinOnce();
  //Serial3.println("blah");
  //delay(10);


}

double radiansToPosition(float position_radians)
{
  //convert radians to position
  position_radians = round(map(((position_radians)*(57.2958)),0,360,0,4095));
  return position_radians;

}

double positionToRadians(float radians_position)
{
  //convert radians to position
  radians_position = (map(radians_position,0,4095,0,360))/57.2958;
  return radians_position;

}
