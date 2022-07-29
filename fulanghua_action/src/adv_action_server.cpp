#include <ros/ros.h>
#include <fulanghua_action/testAction.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

class SpecialMove{
  public:


    SpecialMove(){
          ros::NodeHandle private_nh("~");
          private_nh.param("cmd_vel_posture", cmd_vel_, std::string("cmd_vel_posture_"));
          private_nh.param("dist_err", dist_err, std::string("0.8"));
    }
    void chargingFunction(){
          printf("charging action here\n");
    }


    void  coordinate_callback(const geometry_msgs::Point& point){
      
        const double wx = dest.x;
        const double wy = dest.y;
        const double rx = point.x;
        const double ry = point.x;
        const double dist = std::sqrt(std::pow(wx - rx, 2) + std::pow(wy - ry, 2));
        // dist < stof(dist_err);
    }

    ros::NodeHandle nh;
    tf::TransformListener tf_listener_;
    std::string cmd_vel_,dist_err;
    ros::Publisher twist_pub; 
    ros::Subscrive robot_coordinate_sub;

};



typedef actionlib::SimpleActionServer<fulanghua_action::testAction> Server;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "task_server");
  SpecialMove SpM;
  Server server(SpM.nh, "action", false);
  server.start();
  ros::Time start_time;
  ros::Rate loop_rate(20);
  SpM.twist_pub = SpM.nh.advertise<geometry_msgs::Twist>(SpM.cmd_vel_,1000);
  SpM.robot_coordinate_sub = SpM.nh.subscribe("robot_coordniate", 1000 &SpecialMove::coordinate_callback, &SpM);
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
            SpM.chargingFunction();  
          }
          else if (current_goal->command =="takephoto"){
             printf("take photo\n");
          }
          else if (current_goal->command =="videostream"){
            printf(" watch video\n");
          }
          printf("cmd_x_vel = %f\n", twist.linear.x);
          printf("cmd_z_vel = %f\n", twist.angular.z);
          SpM.twist_pub.publish(twist);
        }
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}