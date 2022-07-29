#include <ros/ros.h>
#include <fulanghua_action/testAction.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>


void chargingFunction(){
    printf("charging action here\n");
}


typedef actionlib::SimpleActionServer<fulanghua_action::testAction> Server;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "task_server");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  std::string cmd_vel_;
  private_nh.param("cmd_vel_posture", cmd_vel_, std::string("cmd_vel_posture_"));
  Server server(nh, "action", false);
  server.start();
  ros::Publisher twist_pub;
  twist_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_,1000);
  ros::Time start_time;
  ros::Rate loop_rate(20);
  fulanghua_action::testGoalConstPtr current_goal;
  while (ros::ok())
  {
    if (server.isNewGoalAvailable())
    {
      current_goal = server.acceptNewGoal();
      start_time = ros::Time::now();
      printf("Update Goal\n");
    }
    if (server.isActive())
    {
      if (server.isPreemptRequested())
      {
        server.setPreempted();
        printf("Preempt Goal\n");
      }
      else
      {
        geometry_msgs::Twist twist;
        if (start_time + ros::Duration(current_goal->duration) < ros::Time::now())
        {
          server.setSucceeded();
          // server.setAborted();
          
        }
        else
        {
          fulanghua_action::testFeedback feedback;
          feedback.rate = (ros::Time::now() - start_time).toSec() / current_goal->duration;
          server.publishFeedback(feedback);
          // printf("Active: publish feedback id:%i\n", current_goal->task_id);
          std::cout << "received command: " << current_goal->command <<std::endl;
          // printf("Active: publish result id:%i\n", current_goal->task_id);
          if(current_goal->command =="lookup"){
              printf("look up\n");
              twist.linear.x = 1;
          }
          else if (current_goal->command =="lookdown"){
              printf("look down\n");
              twist.linear.x = -1;
          }
          else if (current_goal->command =="lookleft"){
              printf("look left\n");
              twist.angular.z = 1;
          }
          else if (current_goal->command =="lookright"){
            printf("look right\n");
            twist.angular.z = -1;
          }
          else if (current_goal->command =="stop"){
            printf("stop\n");
            twist.linear.x = 0;
            twist.angular.z = 0;
          }
          else if (current_goal->command =="charge"){
            chargingFunction();  
          }
          else if (current_goal->command =="takephoto"){
             printf("take photo\n");
          }
          else if (current_goal->command =="videostream"){
            printf(" watch video\n");
          }
          printf("cmd_x_vel = %f\n", twist.linear.x);
          printf("cmd_z_vel = %f\n", twist.angular.z);
          twist_pub.publish(twist);
        }
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}