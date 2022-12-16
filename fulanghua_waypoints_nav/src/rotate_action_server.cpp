#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <fulanghua_action/special_moveAction.h>
#include <actionlib/server/simple_action_server.h>
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
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle nh;
  Server server(nh, "rotate", false); //make a server

  double target; // target angle(radian)
  std::string _high_cmd; // target cmd_vel
  ros::Rate rate(10.0); // set the rate 
  ros::NodeHandle private_nh("~"); 
  private_nh.param("high_cmd", _high_cmd, std::string("high_cmd"));
  unitree_legged_msgs::HighCmd high_cmd_ros;
  ros::Publisher go1_ros_cmd_pub;
  go1_ros_cmd_pub=nh.advertise<unitree_legged_msgs::HighCmd>(_high_cmd, 10);
  ros::Time start_time;
  tf::TransformListener listener;
  tf::StampedTransform target_transform;
  fulanghua_action::special_moveGoalConstPtr current_goal; // instance of a goal
  double previous = 0.0;
  bool inverseAngle = false;
  server.start(); //start the server
  while (ros::ok()){
      if(server.isNewGoalAvailable()){
          current_goal = server.acceptNewGoal();
          start_time = ros::Time::now();
          while (ros::ok()){
            try{
                listener.lookupTransform("/base_link", "/odom", ros::Time(0), target_transform);
                break;
            }
            catch (tf::TransformException &ex) {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
         }
          //get the target angle (rotate left)
          target = current_goal->angle*3.14/180 + atan2(target_transform.getOrigin().y(),
                                  target_transform.getOrigin().x());
          if (target > 3.14){
            inverseAngle = true;
            target -= 6.28;
          }else if (target <-3.14){
            inverseAngle = true;
            target += 6.28;
          }
      }
      if(server.isActive()){
        if(server.isPreemptRequested()){
          initialize(high_cmd_ros);
          go1_ros_cmd_pub.publish(high_cmd_ros);
          server.setPreempted(); // cancel the goal
          ROS_WARN("Preemmpt Goal\n");
        }else{
          if(start_time + ros::Duration(current_goal->duration) < ros::Time::now()){
            initialize(high_cmd_ros);
            go1_ros_cmd_pub.publish(high_cmd_ros);
            server.setAborted(); // abort it
            ROS_WARN("Aborted Goal\n");
          }
          else{
            ROS_INFO("start rotating");
            initialize(high_cmd_ros);
            fulanghua_action::special_moveFeedback feedback; // set the feeback
            feedback.rate = (ros::Time::now() - start_time).toSec() / current_goal->duration; // decide the rate of feedback
            server.publishFeedback(feedback); //publish the feedback
             tf::StampedTransform current_transform;
             geometry_msgs::Point pt;
             geometry_msgs::Twist vel_msg;
            try{
              listener.lookupTransform("/base_link", "/odom", ros::Time(0), current_transform);
              pt.x = current_transform.getOrigin().x();
              pt.y = current_transform.getOrigin().y();
              double current = atan2(pt.y,pt.x);
              printf("target: %lf\n",target);
              printf("current: %lf\n",current);
              

              /*since we have 0 ~ 180 and -180 ~0, sometimes the degree changed to an unexpected value 
              such as - 3.13 to 3.13. This happens due to the avobe reason
              To avoid this problem, adjusting statement is put below.
              */
              if(std::abs(current - previous)>6){
                printf("Overshoot!");
                current *=-1;
                inverseAngle = false;
              }
              double diff = target -current;
              printf("diff_from target to current_position: %lf\n",diff);
              high_cmd_ros.mode = 2;
              high_cmd_ros.gaitType = 1;
              high_cmd_ros.yawSpeed = 0.112f;
              if (diff > 0){
                high_cmd_ros.yawSpeed *= -1;
              }
              if (std::abs(diff)>5)
                high_cmd_ros.yawSpeed *= -1;

              go1_ros_cmd_pub.publish(high_cmd_ros);
              if(std::abs(diff)<0.05 ){
                initialize(high_cmd_ros);
                go1_ros_cmd_pub.publish(high_cmd_ros);
                server.setSucceeded();
                ROS_INFO("Succeeded it!");
              }
              previous = current;
            }
            catch (tf::TransformException &ex) {
              ROS_ERROR("%s",ex.what());
              ros::Duration(1.0).sleep();
              continue;
            }
          }
        }
      }
      ros::spinOnce();
      rate.sleep();
  }
  return 0;
};