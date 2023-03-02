#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <fulanghua_action/special_moveAction.h>
#include <camera_action/camera_pkgAction.h>
#include <actionlib/server/simple_action_server.h>

#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"
#include <unitree_legged_msgs/HighCmd.h>



typedef actionlib::SimpleActionServer<fulanghua_action::special_moveAction> Server;


void initialize(unitree_legged_msgs::HighCmd &cmd){
            cmd.head[0] = 0xFE;
            cmd.head[1] = 0xEF;
            cmd.levelFlag = 0x00;
            cmd.mode = 0;
            cmd.gaitType = 0;
            cmd.speedLevel = 0;
            cmd.footRaiseHeight = 0;
            cmd.bodyHeight = 0;
            cmd.euler[0] = 0;
            cmd.euler[1] = 0;
            cmd.euler[2] = 0;
            cmd.velocity[0] = 0.0f;
            cmd.velocity[1] = 0.0f;
            cmd.yawSpeed = 0.0f;
            cmd.reserve = 0;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "go1_high_command");
  ros::NodeHandle nh;
  ros::Rate rate(10.0); // set the rate 
  ros::Publisher go1_ros_cmd_pub; 
  Server server(nh, "go1_command", false); //make a server
  unitree_legged_msgs::HighCmd high_cmd_ros;
  std::string cmd[11] = {"standup", "sitdown", "rightsidestep", 
  "leftsidestep", "dump", "moveforward", "movebackward", 
  "lookleft", "lookright", "lookup", "lookdown"};
  std::string _high_cmd;
  double yaw_value,picth_value;
  ros::Time start_time;
  ros::NodeHandle private_nh("~"); 
  private_nh.param("high_cmd", _high_cmd, std::string("high_cmd"));
  private_nh.param("yaw_value", yaw_value, 0.3);
  private_nh.param("high_cmd", picth_value, 0.4);
  go1_ros_cmd_pub=nh.advertise<unitree_legged_msgs::HighCmd>(_high_cmd, 10);
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
            ROS_INFO("Go1 starts move");
            fulanghua_action::special_moveFeedback feedback; // set the feeback
            feedback.rate = (ros::Time::now() - start_time).toSec() / current_goal->duration; // decide the rate of feedback
            server.publishFeedback(feedback); //publish the feedback
            //initialize high cmd
            initialize(high_cmd_ros);

            
            if (current_goal->command == cmd[0]){
                ROS_INFO("Go1 standing up");
                high_cmd_ros.mode = 6;
            }else if (current_goal->command == cmd[1]){
                ROS_INFO("Go1 sitting down");
                high_cmd_ros.mode = 5;
            }else if (current_goal->command == cmd[2]){
                ROS_INFO("Go1 right side stepping");
                high_cmd_ros.mode = 2;
                high_cmd_ros.gaitType = 1;
                high_cmd_ros.velocity[1] = -0.112f;
            }else if (current_goal->command == cmd[3]){
                ROS_INFO("Go1 left side stepping");
                high_cmd_ros.mode = 2;
                high_cmd_ros.gaitType = 1;
                high_cmd_ros.velocity[1] = 0.112f;
            }else if (current_goal->command == cmd[4]){
                ROS_INFO("Go1 dumping");
                high_cmd_ros.mode = 7;
            }else if (current_goal->command == cmd[5]){
                ROS_INFO("Go1 moving forward");
                high_cmd_ros.mode = 2;
                high_cmd_ros.gaitType = 1;
                high_cmd_ros.velocity[0] = 0.112f;
            }else if (current_goal->command == cmd[6]){
                ROS_INFO("Go1 moving backword");
                high_cmd_ros.mode = 2;
                high_cmd_ros.gaitType = 1;
                high_cmd_ros.velocity[0] = -0.112f;
            }else if (current_goal->command == cmd[7]){
                ROS_INFO("Go1 looking right");
                high_cmd_ros.mode = 1;
                high_cmd_ros.euler[2] = yaw_value;
            }else if (current_goal->command == cmd[8]){
                ROS_INFO("Go1 looking left");
                high_cmd_ros.mode = 1;
                high_cmd_ros.euler[2] = -yaw_value;
            }else if (current_goal->command == cmd[9]){
                ROS_INFO("Go1 looking up");
                high_cmd_ros.mode = 1;
                high_cmd_ros.gaitType = 1;
                high_cmd_ros.euler[1] = picth_value;
            }else if (current_goal->command == cmd[10]){
                ROS_INFO("Go1 looking down");
                high_cmd_ros.mode = 1;
                high_cmd_ros.gaitType = 1;
                high_cmd_ros.velocity[1] = -picth_value;
            }


            while(start_time + ros::Duration(current_goal->duration) -ros::Duration(0.5) > ros::Time::now()){
              go1_ros_cmd_pub.publish(high_cmd_ros);
              rate.sleep();
            }
            initialize(high_cmd_ros);
            go1_ros_cmd_pub.publish(high_cmd_ros);
            server.setSucceeded();
            ROS_INFO("Go1 command finished");
          }
        }
      }
      ros::spinOnce();
      rate.sleep();
  }
  return 0;
}
