#include <ros/ros.h>
#include "autoware_msgs/CANPacket.h"
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#define threshold 4

float ndt_pose_x;
float ndt_pose_y;
int traffic_501;
int traffic_524;
int traffic_553;
int traffic_563;
int traffic_573;
int traffic_511;

double traffic_light_1_x = 6804.45361215;
double traffic_light_1_y = -8222.22599349;

double traffic_light_2_x = 6763.84015291;
double traffic_light_2_y = -8182.08874345;

double traffic_light_3_x =  6739.08039691;
double traffic_light_3_y = -8217.62607547;

double traffic_light_4_x = 6758.37597656;
double traffic_light_4_y = -8257.54199219;

double traffic_light_5_x = 6778.03417969;
double traffic_light_5_y =  -8299.61035156;

double traffic_light_6_x = 6821.38427734;
double traffic_light_6_y = -8258.15527344;

double end_x = 6812.08789062;
double end_y = -8234.3984375;
    


// setup publisher
ros::Publisher DSRC_flag;
std_msgs::Bool DSRC;
void chatterCallback1(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  ndt_pose_x = msg->pose.position.x;
  ndt_pose_y = msg->pose.position.y;
}

float CalDistance(float x, float y, float a, float b){
	return sqrt(pow(x - a, 2) + pow(y - b, 2));
}

void reset(){
  DSRC.data = false;
}

void chatterCallback(const autoware_msgs::CANPacket::ConstPtr& msg)
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

  int changed = 0;
  if(msg->id == 501){
    traffic_501 = msg->dat[4];
      // fprintf(stderr, "%d\n", traffic_501);
      fprintf(stderr, "traffic_501\n");

    // double distance=CalDistance(ndt_pose_x,ndt_pose_y, traffic_light_1_x, traffic_light_1_y);
    // fprintf(stderr, "501: dis = %f \n",distance);
    // if(distance<threshold){
    //    fprintf(stderr, "Distance < Threshold \n");  
    //    fprintf(stderr, "msg_data = %d, %d \n",msg->dat[4],msg->dat[4] & 128);  

    //   if( (msg->dat[4] & 128) || (msg->dat[4] & 64) ){
    //     fprintf(stderr, "Red or Yellow\n");    
    //     DSRC.data = true;

    // 	}
    //   else{
    //     DSRC.data = false;

    //   }
    //   DSRC_flag.publish(DSRC);
    // }

  }

  else if(msg->id == 524){
    traffic_524 = msg->dat[4];
    fprintf(stderr, "traffic_524\n");

  }

  else if(msg->id == 553){
    traffic_553 = msg->dat[4];
    fprintf(stderr, "traffic_553\n");
  }

  else if(msg->id == 563){
    traffic_563 = msg->dat[4];
    fprintf(stderr, "traffic_563\n");
  }

  else if(msg->id == 573){
    traffic_573 = msg->dat[4];
    fprintf(stderr, "traffic_573\n");
  }

  else if(msg->id == 511){
    traffic_511 = msg->dat[4];
    fprintf(stderr, "traffic_511\n");
  }
  else{
    //reset();
  } 

}
void* thread_Decision(void* args){
  while(1){
  double distance_1 = CalDistance(ndt_pose_x,ndt_pose_y, traffic_light_1_x, traffic_light_1_y);
  double distance_2 = CalDistance(ndt_pose_x,ndt_pose_y, traffic_light_2_x, traffic_light_2_y);
  double distance_3 = CalDistance(ndt_pose_x,ndt_pose_y, traffic_light_3_x, traffic_light_3_y);
  double distance_4 = CalDistance(ndt_pose_x,ndt_pose_y, traffic_light_4_x, traffic_light_4_y);
  double distance_5 = CalDistance(ndt_pose_x,ndt_pose_y, traffic_light_5_x, traffic_light_5_y);
  double distance_6 = CalDistance(ndt_pose_x,ndt_pose_y, traffic_light_6_x, traffic_light_6_y);
  double distance_end = CalDistance(ndt_pose_x,ndt_pose_y, end_x, end_y);

  bool Red_1 = traffic_501 & 128;
  // fprintf(stderr, "%d\n", traffic_501);
  // if(Red_1)
  //   fprintf(stderr, "true\n" );
  // else
  //   fprintf(stderr, "false\n" );
  bool Red_2 = traffic_524 & 128;
  bool Red_3 = traffic_553 & 128;
  bool Red_4 = traffic_563 & 128;
  bool Red_5=traffic_573 & 128;
  bool Red_6 = traffic_511 & 128;
  bool Yellow_1 = traffic_501 & 64;
  bool Yellow_2 = traffic_524 & 64;
  bool Yellow_3 = traffic_553 & 64;
  bool Yellow_4 = traffic_563 & 64;
  bool Yellow_5=traffic_573 & 64;
  bool Yellow_6 = traffic_511 & 64;
  bool GreenLeft_5 = traffic_573 & 16;
  fprintf(stderr, "%d\n", traffic_573);



  if((distance_1 < threshold && (Red_1 || Yellow_1) ) || (distance_2 < threshold && (Red_2 || Yellow_2)) || (distance_3 < threshold && (Red_3 || Yellow_3)) || (distance_4 < threshold && (Red_4 || Yellow_4)) || (distance_5 < threshold && (Red_5 || Yellow_5)) || (distance_6 < threshold && (Red_6 || Yellow_6)))
  {
    DSRC.data = true;
    fprintf(stderr, "Condition 1\n");
  }
  else if(distance_end < threshold)
  {
    DSRC.data = true;
    fprintf(stderr, "Condition 2\n");

  }
  else
  {
    DSRC.data = false;
    fprintf(stderr, "Condition 3\n");

  }
  DSRC_flag.publish(DSRC);
  ros::Duration(0.1).sleep();
  }
}


int main (int argc, char *argv[]){
  autoware_msgs::CANPacket candat;
  DSRC.data = false;

  ros::init(argc, argv, "can_converter");
  ros::NodeHandle n;
  DSRC_flag = n.advertise<std_msgs::Bool>("/DSRC_vel_stop", 1);

  ros::Subscriber sub = n.subscribe("can_raw", 1000, chatterCallback);
  ros::Subscriber sub1 = n.subscribe("ndt_pose", 1000, chatterCallback1);

  pthread_t thread;
  pthread_create(&thread, NULL, thread_Decision, NULL);
  ros::spin();
  pthread_cancel(thread);

  
}
