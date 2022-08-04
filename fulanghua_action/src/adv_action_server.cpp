#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <fulanghua_action/special_moveAction.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "orne_waypoints_msgs/Pose.h"
#include <sound_play/SoundRequestAction.h>
#include <sound_play/SoundRequest.h>
typedef actionlib::SimpleActionServer<fulanghua_action::special_moveAction> Server;

class SpecialMove{
  public:


    SpecialMove():
      server(nh, "action", false),
      sound_client("sound_play", true),
      rate_(2)
    {
          ros::NodeHandle private_nh("~");
          private_nh.param("cmd_vel_posture", cmd_vel_posture, std::string("cmd_vel_posture_"));
          private_nh.param("cmd_vel", cmd_vel_, std::string("cmd_vel"));
          private_nh.param("max_vel", max_vel_, std::string("0.4"));
          private_nh.param("min_vel", min_vel_, std::string("0.1"));
          private_nh.param("dist_err", _dist_err, std::string("0.8"));
          private_nh.param("voice_path", voice_path, std::string(""));
          max_vel = std::stod(max_vel_);
          min_vel = std::stod(min_vel_);
          dist_err = std::stod(_dist_err);
          server.start();
    }
    void odom_callback(const nav_msgs::Odometry& odom){
        _odom = odom;
    }
    void coordinate_callback(const geometry_msgs::Point& point){
        rx = point.x;
        ry = point.y;
    }
    void speaking_function(std::string& sound_fle_name){
        speak_start = true;
        if (sound_client.isServerConnected())
        {
            sound_play::SoundRequestGoal goal;
            sound_play::SoundRequest sr;
            goal.sound_request.sound = sr.PLAY_FILE;
            goal.sound_request.command = sr.PLAY_ONCE;
            goal.sound_request.volume = 1.0;
            goal.sound_request.arg = voice_path + sound_fle_name + ".wav";
            std::cout <<"sound file name:" << sound_fle_name;
            sound_client.sendGoal(goal);
            actionlib::SimpleClientGoalState state = sound_client.getState();
            while(state !=actionlib::SimpleClientGoalState::PREEMPTED || state !=actionlib::SimpleClientGoalState::SUCCEEDED){
                state = sound_client.getState();
                rate_.sleep();
            }
            printf("Voice Action finished\n");
            server.setPreempted();
            printf("Preempt Goal\n");
        }   

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
          printf("Preempt Goal\n");
      }
    }

    bool onNavigationPoint(const orne_waypoints_msgs::Pose &dest){
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
          initial_odom =_odom;
          // steering = direction.orientation - angle;
          initial = false;
        }
        double odom_diff =(initial_odom.pose.pose.orientation.z - _odom.pose.pose.orientation.z);
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
          velocity_x = Kp* std::abs(dist);// - Kv * diff/interval;
          //P
          // velocity_x = Kp* std::abs(dist);
          temp = velocity_x;
          velocity_x = std::min(max_vel,velocity_x);
          velocity_x = std::max(min_vel,velocity_x);

        }else{
          velocity_x =0.4;
        }
        printf("dx: %f\n", (dist-prev_location));
        prev_location = dist;
        // twist.linear.x = velocity_x;
        twist.linear.x = velocity_x;
        twist.angular.z = 3.33232 * -odom_diff + 0.32467; // Calculated by linear regression
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
    ros::Subscriber robot_coordinate_sub, odom_sub;
    geometry_msgs::Twist twist;
    nav_msgs::Odometry _odom, initial_odom;;
    actionlib::SimpleActionServer<fulanghua_action::special_moveAction> server;
    actionlib::SimpleActionClient<sound_play::SoundRequestAction> sound_client;
    ros::Rate rate_;
    const double hz =20;
    bool speak_start = false;
  private:
    const double Kp = 0.5;
    const double Kv = 0.2865;
    orne_waypoints_msgs::Pose direction;
    double velocity_x;
    double rx, ry;
    double dist_err = 0;
    const double radian_90 = 1.5708;
    const double interval =1/hz;
    bool initial = true;
    double steering;
    double original_angle;
    double prev_location = 0;
    std::string max_vel_;
    std::string min_vel_;
    std::string voice_path;
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
  SpM.odom_sub = SpM.nh.subscribe("odom", 1000, &SpecialMove::odom_callback, &SpM);
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
            if(!SpM.speak_start){
              std::string str = "mg400";
              SpM.speaking_function(str);
            }
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