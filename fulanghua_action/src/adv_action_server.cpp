#include <ros/ros.h>
#include <fulanghua_action/testAction.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "orne_waypoints_msgs/Waypoint.h"

class SpecialMove{
  public:


    SpecialMove(){
          ros::NodeHandle private_nh("~");
          private_nh.param("cmd_vel_posture", cmd_vel_, std::string("cmd_vel_posture_"));
          private_nh.param("robot_frame", robot_frame_, std::string("base_link"));
          private_nh.param("world_frame", world_frame_, std::string("map"));
    }
    void chargingFunction(){
          printf("charging action here\n");
    }

    bool onNavigationPoint(const orne_waypoints_msgs::Waypoint &dest, double dist_err = 0.8){
        tf::StampedTransform robot_gl = getRobotPosGL();

        const double wx = dest.x;
        const double wy = dest.y;
        const double rx = robot_gl.getOrigin().x();
        const double ry = robot_gl.getOrigin().y();
        const double dist = std::sqrt(std::pow(wx - rx, 2) + std::pow(wy - ry, 2));

        return dist < dist_err;
    }

    tf::StampedTransform getRobotPosGL(){
        tf::StampedTransform robot_gl;
        geometry_msgs::Point pt;
        try{
            tf_listener_.lookupTransform(world_frame_, robot_frame_, ros::Time(0.0), robot_gl);
            pt.x = robot_gl.getOrigin().x();
            pt.y = robot_gl.getOrigin().y();
            robot_coordinate.publish(pt);
        }catch(tf::TransformException &e){
            ROS_WARN_STREAM("tf::TransformException: " << e.what());
        }
        return robot_gl;
    }

    ros::NodeHandle nh;
    tf::TransformListener tf_listener_;
    std::string robot_frame_, world_frame_,cmd_vel_;
    ros::Publisher twist_pub =nh.advertise<geometry_msgs::Twist>(cmd_vel_,1000);
    ros::Publisher robot_coordinate = nh.advertise<geometry_msgs::Point>("robot_coordniate",1000);
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