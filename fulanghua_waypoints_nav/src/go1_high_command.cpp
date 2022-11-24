#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <fulanghua_action/special_moveAction.h>
#include <camera_action/camera_pkgAction.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "orne_waypoints_msgs/Pose.h"
#include <sound_play/SoundRequest.h>
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"
#include "unitree_ros_msgs/HighCmd.h"

class Command{
    public:
        Command():
        server(nh, "go1_command", false);
        {
            ros::NodeHandle private_nh("~"); 
            go1_ros_cmd_pub=nh.advertise<geometry_msgs::Twist>("high_cmd", 10);
        }

    //Server
    Server server; //make a server


    //Publishers
    ros::NodeHandle nh;
    ros::Publisher go1_ros_cmd_pub; 
    //Subscribers

    //Services

    //Topics

    //Variables
    unitree_ros_msgs::Highcmd go1_cmd;
    double target; // target angle(radian)
    std::string cmd[3] = {"standup", "sitdown", "sidestep"}

};

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");
  Command go1_cmd;
  ros::Rate rate(10.0); // set the rate 
  ros::Time start_time;
  fulanghua_action::special_moveGoalConstPtr current_goal; // instance of a goal
  server.start(); //start the server
  while (ros::ok()){
      if(server.isNewGoalAvailable()){
          current_goal = server.acceptNewGoal();
          start_time = ros::Time::now();
      }
      if(server.isActive()){
        if(server.isPreemptRequested()){
          server.setPreempted(); // cancel the goal
          ROS_WARN("Preemmpt Goal\n");
        }else{
          if(start_time + ros::Duration(current_goal->duration) < ros::Time::now()){
            server.setAborted(); // abort it
          }
          else{
            ROS_INFO("go1 starts move");
            fulanghua_action::special_moveFeedback feedback; // set the feeback
            feedback.rate = (ros::Time::now() - start_time).toSec() / current_goal->duration; // decide the rate of feedback
            server.publishFeedback(feedback); //publish the feedback
            if (current_goal->commnd == go1_cmd.cmd[0]){
                ROS_INFO("Go1 standing up");
            }else if (current_goal->commnd == go1_cmd.cmd[1]){
                ROS_INFO("Go1 sitting down");
            }else if (current_goal->commnd == go1_cmd.cmd[2]){
                ROS_INFO("Go1 side stepping");
            }
          }
        }
      }
      ros::spinOnce();
      rate.sleep();
  }
  return 0;
}