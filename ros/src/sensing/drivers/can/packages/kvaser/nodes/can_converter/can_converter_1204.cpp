#include <ros/ros.h>
#include "kvaser/CANPacket.h"
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#define threshold 2

float ndt_pose_x;
float ndt_pose_y;

// setup publisher
ros::Publisher DSRC_flag;
std_msgs::Bool DSRC;
void chatterCallback1(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  ndt_pose_x =msg->pose.position.x;
  ndt_pose_y =msg->pose.position.y;
}

float CalDistance(float x, float y, float a, float b){
	return sqrt(pow(x - a, 2) + pow(y - b, 2));
}

void reset(){
  DSRC.data = false;
}

void chatterCallback(const kvaser::CANPacket::ConstPtr& msg)
{
  unsigned short w;
  static int enc_sum;
  short diff;
  static short steer,shift,speed2,speed3,brake;
  static char  speed;
  static short enc,enc_p,enc_diff;
  static short wheel1,wheel2,wheel3,wheel4;
  static short gyro,accy,accz;
  static char accx; 
  static FILE* log_fp;

  int changed=0;

  if(msg->id==501){
    double distance=CalDistance(ndt_pose_x,ndt_pose_y, 6804.45361215, -8222.22599349);
    fprintf(stderr, "501: dis = %f \n",distance);
    if(distance<threshold){
       fprintf(stderr, "Distance < Threshold \n");  
       fprintf(stderr, "msg_data = %d, %d \n",msg->dat[4],msg->dat[4] & 128);  

      if( (msg->dat[4] & 128) || (msg->dat[4] & 32) ){
        fprintf(stderr, "Red or Yellow\n");    
        DSRC.data = true;

    	}
      else{
        DSRC.data = false;

      }
      DSRC_flag.publish(DSRC);


    }

  }

  // else if(msg->id==524){
  //   if(CalDistance(ndt_pose_x,ndt_pose_y, 6763.3843876, -8182.76986069)<threshold){
  //   	if(  (msg->dat[4] & 0x80==0x80) || (msg->dat[4]&0x40==0x40) ){
  //       DSRC.data=true;
        
  //   	}

  //   }
  // }

  // else if(msg->id==553){
  //   if(CalDistance(ndt_pose_x,ndt_pose_y, 6763.3843876, -8182.76986069)<threshold){
  //   	if(  (msg->dat[4]& 0x80==0x80) || (msg->dat[4]&0x40==0x40) ){
  //       DSRC.data=true;
  //   	}

  //   }
  // }

  // else if(msg->id==563){
  //   if(CalDistance(ndt_pose_x,ndt_pose_y, 6759.14468672, -8258.82854876)<threshold){
  //   	if(  (msg->dat[4]& 0x80==0x80) || (msg->dat[4]&0x40==0x40) ){
  //       DSRC.data=true;
  
  //   	}

  //   }
  // }

  // else if(msg->id==573){
  //   if(CalDistance(ndt_pose_x,ndt_pose_y, 6778.19726562, -8299.4765625)<threshold){
  //   	if(  msg->dat[4]& 0x10!=0x10 ){
  //       DSRC.data=true;

  //   	}

  //   }
  // }

  // else if(msg->id==511){
  //   if(CalDistance(ndt_pose_x,ndt_pose_y, 6821.26513672, -8258.06738281)<threshold){
  //   	if(  (msg->dat[4]& 0x80==0x80) || (msg->dat[4]&0x40==0x40) ){
  //       DSRC.data=true;
 
  //   	}

  //   }
  // }
  // else{
  //   reset();
  // } 
  // DSRC_flag.publish(DSRC);
  // ros::Duration(0.1).sleep();


  // if(msg->id==0x24){
  //   w=msg->dat[0]*256+msg->dat[1];
  //   gyro=w;
  //   w=msg->dat[2]*256+msg->dat[3];
  //   accx=w;
  //   w=msg->dat[4]*256+msg->dat[5];
  //   accy=w;
  //   w=msg->dat[7];
  //   accz=w;
  //   changed=1;
  // }
  // if(msg->id==0x25){
  //   w=msg->dat[0]*4096+msg->dat[1]*16;
  //   steer=w;
  //   steer=steer/16;
  //   changed=1;
  // }

  // if(msg->id==0xaa){
  //   w=msg->dat[0]*256+msg->dat[1];
  //   wheel1=w;
  //   w=msg->dat[2]*256+msg->dat[3];
  //   wheel2=w;
  //   w=msg->dat[4]*256+msg->dat[5];
  //   wheel3=w;
  //   w=msg->dat[6]*256+msg->dat[7];
  //   wheel4=w;
  //   changed=1;
  // }
  // if(msg->id==0xb4){
  //   w=msg->dat[5]*256+msg->dat[6];
  //   speed3=w;
  //   changed=1;
  // }
  // if(msg->id==0x224){
  //   w=msg->dat[4]*256+msg->dat[5];
  //   brake=w;
  //   changed=1;
  // }
  // if(msg->id==0x127){
  //   shift=msg->dat[3];
  //   speed=msg->dat[4];
  //   changed=1;
  // }
  // if(msg->id==0x230){
  //   w=msg->dat[0]*256+msg->dat[1];
  //   enc_p=enc;
  //   enc=w;
  //   diff=enc-enc_p;
  //   enc_diff=diff;
  //   enc_sum+=diff;
  //   changed=1;
  // }
  // if(changed){
  //   if(!log_fp)log_fp=fopen("/tmp/can_log","w");
  //   fprintf(log_fp,"%f %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
	 //    msg->header.stamp.toSec(),msg->time,steer,shift,speed,speed2,speed3,enc_sum,enc_diff,brake,
	 //    wheel1,wheel2,wheel3,wheel4,
	 //    accx,accy,accz,gyro);
  // }  
}



int main (int argc, char *argv[]){
  kvaser::CANPacket candat;
DSRC.data = false;

  ros::init(argc, argv, "can_converter");
  ros::NodeHandle n;
  DSRC_flag = n.advertise<std_msgs::Bool>("/DSRC_vel_stop", 1);

  ros::Subscriber sub = n.subscribe("can_raw", 1000, chatterCallback);
  ros::Subscriber sub1 = n.subscribe("ndt_pose", 1000, chatterCallback1);

  ros::spin();
}
