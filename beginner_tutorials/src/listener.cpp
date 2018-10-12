#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "beginner_tutorials/rs232.h"
#include "beginner_tutorials/rs232.c"

#define Start 0xAA
#define Address 0x7F
#define ReturnType 0x00
#define Clean 0x00
#define Reserve 0x00
#define End 0x55 
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void TwistToMotors();
void Sendmessage(float Rrpm,float Lrpm);
int myabs( int a );

char senddata[16];
int vx,wz;
float Lrpm,Rrpm;
float ticks_since_target;
float timeout_ticks;
float motor_rpm_r, motor_rpm_l;
double w;
double rate;
double Dimeter;
float dx,dy,dr;
int motor_seq,motor_old_seq;


void cmd_vel_Callback(const geometry_msgs::Twist& vel)
{
    dx = vel.linear.x;
    dy = vel.linear.y;
    dr = vel.angular.z;
	TwistToMotors(); 
}

void TwistToMotors()
{
    float right,left;
    float vel_data[2];
    float motor_rpm_vx, motor_rpm_theta;
    motor_old_seq = motor_seq;
    w = 0.302;
    rate = 20;
    timeout_ticks = 2;
    Dimeter = 0.127;
    // prevent agv receive weird 1.0 command from cmd_vel
    //if (dr == 1.0){
    //    dr = 0.001;
    //}
    right = ( 1.0 * dx ) + (dr * w /2);
    left = ( 1.0 * dx ) - (dr * w /2);
    motor_rpm_vx = ( 1.0 * dx )*rate/(Dimeter/2)*60/(2*3.1416);
    if((motor_rpm_vx !=0) && (myabs(motor_rpm_vx)<100)){
        if(motor_rpm_vx >0){
            motor_rpm_vx = 100;
        }
        else{
            motor_rpm_vx = -100;
        }
    }
    motor_rpm_theta=(dr * w /2)*rate/(Dimeter/2)*60/(2*3.1416);
    motor_rpm_r = motor_rpm_vx+ motor_rpm_theta;
    motor_rpm_l = motor_rpm_vx- motor_rpm_theta;
    if (myabs(motor_rpm_r)<100|| myabs(motor_rpm_l)<100){
        if( dx==0){
            if(dr>0){
                motor_rpm_r=100;
                motor_rpm_l=-100;
            }else if (dr<0){
                motor_rpm_r=-100;
                motor_rpm_l=100; 
            }else{
                motor_rpm_r=0;
                motor_rpm_l=0;
            }
        }
        else if(dx>0){
            if (myabs(motor_rpm_r)<100){
                motor_rpm_r =100;
            }
            if (myabs(motor_rpm_l)<100){
                motor_rpm_l =100;
            }
        }
        else{
            if(myabs(motor_rpm_r)<100){
                motor_rpm_r =-100;
            }
            if(myabs(motor_rpm_l)<100){
                motor_rpm_l =-100;
            }
        }
    }
        vel_data[0] = motor_rpm_r;
        vel_data[1] = motor_rpm_l;
        Sendmessage(motor_rpm_r,motor_rpm_l);
    ticks_since_target += 1;

}

int myabs( int a ){
    if ( a < 0 ){
        return -a;
        }
        return a;
        }

void Sendmessage(float Rrpm,float Lrpm)
{
unsigned char sendData[16];
unsigned int tmpCRC;
int motor1,motor2;
   
    sendData[0] = Start;
    sendData[1] = Address;
    sendData[2] = ReturnType;
    sendData[3] = Clean;
    sendData[4] = Reserve;
    sendData[5]  = 0x01;//motor1Sevro ON
    sendData[6] = 0x01;//motor2Sevro ON
if (Rrpm>0){sendData[7] = 0x00;}else{sendData[7] = 0x01;}
if (Lrpm>0){sendData[8] = 0x01;}else{sendData[8] = 0x00;}
   motor1 =  abs(Rrpm);
   motor2 =  abs(Lrpm); 
    
    sendData[9] = (motor1>>8);//motor1speedH
    sendData[10] = (motor1 & 0xFF);//motor1speedL
    sendData[11] = (motor2>>8);//motor2speedH
    sendData[12] = (motor2 & 0xFF);//motor2speedL
    sendData[13] = End;
    tmpCRC = CRC_Verify(sendData, 14);
    sendData[14] = (tmpCRC & 0xFF);
    sendData[15] = (tmpCRC>>8);
    int i;
    for (i=0;i<16;i++)
    {
		RS232_SendByte(0, sendData[i]);
    }
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n("~");
  ros::Subscriber sub = n.subscribe("cmd_vel", 1000, cmd_vel_Callback);
  ros::Rate loop_rate(10);

 int i=0,
      cport_nr=0,        /* /dev/ttyS0 (COM1 on windows) */
      bdrate=38400; 
 char mode[]={'8','N','1',0},str[2][512];
 strcpy(str[0], "The quick brown fox jumped over the lazy grey dog.\n");
 //	strcpy(data[0], "Gogogogo.\n");
 strcpy(str[1], "Happy serial programming!\n");

 if(RS232_OpenComport(cport_nr, bdrate, mode))
 {
   printf("Can not open comport\n");
   return(0);
 }
senddata[0] = 0x30;
senddata[1] = 0x31;
senddata[2] = 0x32;
senddata[3] = 0x33;

	
	
 while (ros::ok()) {

	//RS232_cputs(cport_nr, vx.str());
 	//printf("sent: %s\n", data[0]);
	loop_rate.sleep();
        ros::spinOnce();
 }
return 0;
}
