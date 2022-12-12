#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <fulanghua_action/special_moveAction.h>
typedef actionlib::SimpleActionServer<fulanghua_action::special_moveAction> Server;


int main(int argc, char** argv){
    ros::NodeHandle nh;
    actionlib::SimpleActionClient<fulanghua_action::special_moveAction> ar_align_client;
}




