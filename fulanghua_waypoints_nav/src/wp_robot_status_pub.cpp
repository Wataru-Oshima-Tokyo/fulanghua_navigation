#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "orne_waypoints_msgs/Pose.h"
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int8.h>
#include "fulanghua_msg/WpRobotStatus.h"

class WpRobotStatusPub{
    public:
        WpRobotStatusPub():
        rate_(10)
        {
            position_sub_ = nh.subscribe("/amcl_pose", 1000, &WpRobotStatusPub::position_callback, this);
            waypoint_sub_ = nh.subscribe("/next_waypoint", 1000, &WpRobotStatusPub::waypoint_callback, this);
            wp_robot_status_pub_ = nh.advertise<fulanghua_msg::WpRobotStatus>("/wp_robot_status", 10);
        }

        void position_callback(const geometry_msgs::PoseWithCovarianceStamped &msg){
            wpRobotStatus.currentCoordinate = msg.pose.pose;
        }

        void waypoint_callback(const orne_waypoints_msgs::Pose &msg){
            wpRobotStatus.nextWaypoint.position.x = msg.position.x;
            wpRobotStatus.nextWaypoint.position.y = msg.position.y;
            wpRobotStatus.nextWaypoint.position.z = msg.position.z;
            wpRobotStatus.nextWaypoint.orientation.x = msg.orientation.x;
            wpRobotStatus.nextWaypoint.orientation.y = msg.orientation.y;
            wpRobotStatus.nextWaypoint.orientation.z = msg.orientation.z;
            wpRobotStatus.nextWaypoint.orientation.w = msg.orientation.w;
        }

        void publish_wp_robot(){
            while(ros::ok()){
                rate_.sleep();
                ros::spinOnce();
                wp_robot_status_pub_.publish(wpRobotStatus);
            }
        }
    
    private:
        ros::NodeHandle nh;
        ros::Subscriber position_sub_, waypoint_sub_;
        ros::Publisher wp_robot_status_pub_;
        fulanghua_msg::WpRobotStatus wpRobotStatus;
        ros::Rate rate_;
 };
 
int main(int argc, char** argv){
    ros::init(argc, argv, "wp_robot_status");
    WpRobotStatusPub wp_robot;
    wp_robot.publish_wp_robot();
}