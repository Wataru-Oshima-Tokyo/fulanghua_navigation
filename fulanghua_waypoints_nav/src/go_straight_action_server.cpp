#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <fulanghua_action/special_moveAction.h>
#include <actionlib/server/simple_action_server.h>
typedef actionlib::SimpleActionServer<fulanghua_action::special_moveAction> Server;

int main(int argc, char** argv){
    ros::init(argc, argv, "go_straight");

    ros::NodeHandle nh;
    Server server(nh, "go_straight", false);

    double Kp;
    double target;
    std::string cmd_vel_;
    ros::Rate rate(10.0);
    ros::NodeHandle pnh("~");
    pnh.param("cmd_vel", cmd_vel_, std::string("/cmd_vel"));
    pnh.param("Kp", Kp, 0.8);
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_, 10);
    ros::Time start_time;
    tf::TransformListener listener;
    tf::StampedTransform start_transform;
    fulanghua_action::special_moveGoalConstPtr current_goal;
    geometry_msgs::Point pt_start;
    server.start();

    while(ros::ok()){
        if(server.isNewGoalAvailable()){
            current_goal = server.acceptNewGoal();
            start_time = ros::Time::now();
            while(ros::ok()){
                try{
                    listener.lookupTransform("/base_link", "/odom", ros::Time(0), start_transform);
                    break;
                }catch(tf::TransformException &ex){
                    ROS_ERROR("%s", ex.what());
                    ros::Duration(1.0).sleep();
                    continue;
                }
            }
            target = current_goal->distance;
            pt_start.x = start_transform.getOrigin().x();
            ROS_INFO("target_distance:%0.3f", target);
        }
        if(server.isActive()){
            if(server.isPreemptRequested()){
                server.setPreempted();
                ROS_WARN("Preempt Goal\n");
            }else{
                if(start_time + ros::Duration(current_goal->duration) < ros::Time::now()){
                    server.setAborted();
                }else{
                    ROS_INFO("start going straight");
                    fulanghua_action::special_moveFeedback feedback;
                    feedback.rate = (ros::Time::now() - start_time).toSec() / current_goal->duration;
                    server.publishFeedback(feedback);
                    tf::StampedTransform current_transform;
                    geometry_msgs::Point pt_current;
                    geometry_msgs::Twist twist;
                    try{
                        listener.lookupTransform("/base_link", "odom", ros::Time(0), current_transform);
                        pt_current.x = current_transform.getOrigin().x();
                        double distance = abs(pt_current.x - pt_start.x);
                        double diff = target - distance;
                        printf("target:%lf\n", target);
                        printf("current position:%lf\n", distance);
                        printf("difference:%lf\n", diff);

                        twist.linear.x = Kp*diff;
                        cmd_vel_pub.publish(twist);
                        if(abs(diff) < 0.01){
                            geometry_msgs::Twist finish;
                            cmd_vel_pub.publish(finish);
                            server.setSucceeded();
                            ROS_INFO("Succeeded it!");
                        }
                        if(abs(diff) > 10){
                            server.setAborted();
                        }
                    }catch(tf::TransformException &ex){
                        ROS_ERROR("%s", ex.what());
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
}