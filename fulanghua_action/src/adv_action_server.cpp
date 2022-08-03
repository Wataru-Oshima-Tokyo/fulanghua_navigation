#include <ros/ros.h>
#include <fulanghua_action/special_moveAction.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "orne_waypoints_msgs/Pose.h"

typedef actionlib::SimpleActionServer<fulanghua_action::special_moveAction> Server;

class SpecialMove{
  public:


    SpecialMove():
      server(nh, "action", false)  
    {
          ros::NodeHandle private_nh("~");
          private_nh.param("cmd_vel_posture", cmd_vel_posture, std::string("cmd_vel_posture_"));
          private_nh.param("cmd_vel", cmd_vel_, std::string("cmd_vel"));
          private_nh.param("max_vel", max_vel_, std::string("0.4"));
          private_nh.param("min_vel", min_vel_, std::string("0.1"));
          max_vel = std::stod(max_vel_);
          min_vel = std::stod(min_vel_);
          private_nh.param("dist_err", _dist_err, std::string("0.8"));
          server.start();
    }

    void coordinate_callback(const geometry_msgs::Point& point){
        rx = point.x;
        ry = point.y;
    }

    void chargingFunction(){
          printf("charging action here\n");
    }

    void P2P_move(const orne_waypoints_msgs::Pose &dest){
      if (!onNavigationPoint(dest)){
          twist_move_pub.publish(twist);
      }else{
          initial = true;
          t=0;
          server.setPreempted();
      }
    }

    bool onNavigationPoint(const orne_waypoints_msgs::Pose &dest, double dist_err = 0.8){
        const double wx = dest.position.x;
        const double wy = dest.position.y;
        const double dist = std::sqrt(std::pow(wx - rx, 2) + std::pow(wy - ry, 2));
        //get the angle the target from the current position
        double angle = std::atan2((wy-ry),(wx-rx)); 
        // printf("angle prev = %f\n", angle);
        // if(std::abs(wy-ry)>std::abs(wx-rx)){
          // if(angle>0)
          //   angle = radian_90 - angle;
          // else 
          //   angle = -(radian_90+angle);
        // }
        if(initial){
          // direction.orientation = tf::createQuaternionMsgFromYaw(angle);
          
          // origin = dist;
          original_angle =angle;
          // steering = direction.orientation - angle;
          initial = false;

        }
        // if(original_angle>0){
        //   steering = original_angle - angle;
        // }else{
        //   steering = original_angle + angle;
        // }
        steering = original_angle - angle;
        //rn I only consider the x coordinate for determing the velocity
        double temp=0;
        double diff = std::abs(dist-prev_location);
        if (t!=0){
          //PD
          if(diff ==0){
              diff = prev_diff;
          }else{
              prev_diff = diff;
          }
          velocity_x = Kp* std::abs(dist) - Kv * diff/interval;
          //P
          // velocity_x = Kp* std::abs(dist);
          temp = velocity_x;
          velocity_x = std::min(max_vel,velocity_x);
          velocity_x = std::max(min_vel, velocity_x);

        }else{
          velocity_x =0.4;
        }
        printf("dx: %f\n", (dist-prev_location));
        prev_location = dist;
        // twist.linear.x = velocity_x;
        twist.linear.x = velocity_x;
        twist.angular.z = -steering*0.3;
        printf("cmd_vel_x = %f\n", velocity_x);
        printf("calculated velocity %f\n", temp);
        printf("dist = %f\n", dist);
        
        // // printf("ry = %f\n", ry);
        // // printf("rx = %f\n", rx);
        // // printf("wr-ry = %f\n", std::abs(wy-ry));
        // // printf("wx - rx = %f\n", std::abs(wx - rx));
        // printf("angle = %f\n", angle);
        // // printf("orientation.x = %f\n", direction.orientation.x);
        // // printf("orientation.y = %f\n", direction.orientation.y);
        // printf("steering   = %f\n", steering);
        // printf("orientation.w = %f\n", direction.orientation.w);
        t++;
        return dist < dist_err;
    }

    ros::NodeHandle nh;
    tf::TransformListener tf_listener_;
    std::string cmd_vel_, _dist_err,cmd_vel_posture;
    ros::Publisher twist_move_pub, twist_postgure_pub; 
    ros::Subscriber robot_coordinate_sub;
    geometry_msgs::Twist twist;
    actionlib::SimpleActionServer<fulanghua_action::special_moveAction> server;
    const double hz =20;
  private:
    const double Kp = 0.5;
    const double Kv = 0.4775;
    orne_waypoints_msgs::Pose direction;
    double velocity_x;
    double rx, ry;
    const double radian_90 = 1.5708;
    const double interval =1/hz;
    bool initial = true;
    double steering;
    double original_angle;
    double prev_location = 0;
    std::string max_vel_;
    std::string min_vel_;
    double max_vel =0;
    double min_vel =0;
    double t =0;
    double prev_diff=0;
    
};





int main(int argc, char** argv)
{
  ros::init(argc, argv, "task_server");
  SpecialMove SpM;
  ros::Time start_time;
  ros::Rate loop_rate(SpM.hz);
  // Server server;
  SpM.twist_postgure_pub = SpM.nh.advertise<geometry_msgs::Twist>(SpM.cmd_vel_posture,1000);
  SpM.twist_move_pub = SpM.nh.advertise<geometry_msgs::Twist>(SpM.cmd_vel_,1000);
  SpM.robot_coordinate_sub = SpM.nh.subscribe("robot_coordinate", 1000, &SpecialMove::coordinate_callback, &SpM);
  fulanghua_action::special_moveGoalConstPtr current_goal;
  while (ros::ok())
  {
    if (SpM.server.isNewGoalAvailable())
    {
      current_goal = SpM.server.acceptNewGoal();
      start_time = ros::Time::now();
      printf("Update Goal\n");
    }
    if (SpM.server.isActive())
    {
      if (SpM.server.isPreemptRequested())
      {
        SpM.server.setPreempted();
        printf("Preempt Goal\n");
      }
      else
      {
        geometry_msgs::Twist twist;
        if (start_time + ros::Duration(current_goal->duration) < ros::Time::now())
        {
          SpM.server.setSucceeded();
          // server.setAborted();
          
        }
        else
        {
          fulanghua_action::special_moveFeedback feedback;
          feedback.rate = (ros::Time::now() - start_time).toSec() / current_goal->duration;
          SpM.server.publishFeedback(feedback);
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
          else if (current_goal->command =="p2p"){
            SpM.P2P_move(current_goal->wp);  
          }
          else if (current_goal->command =="takephoto"){
             printf("take photo\n");
          }
          else if (current_goal->command =="videostream"){
            printf(" watch video\n");
          }
          // printf("cmd_x_vel = %f\n", twist.linear.x);
          // printf("cmd_z_vel = %f\n", twist.angular.z);
          // SpM.twist_pub.publish(twist);
        }
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}