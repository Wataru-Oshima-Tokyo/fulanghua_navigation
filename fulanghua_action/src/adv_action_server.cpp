#include <ros/ros.h>
#include <fulanghua_action/testAction.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
typedef actionlib::SimpleActionServer<fulanghua_action::testAction> Server;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "task_server");
  ros::NodeHandle nh;
  Server server(nh, "task", false);
  server.start();

  ros::Time start_time;
  ros::Rate loop_rate(2);
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
          printf("Active: publish result id:%i\n", current_goal->task_id);
          switch (current_goal->command)
          {
          case 'i':
            printf("look up\n");
            break;
          case ',':
            printf("look down\n");
            break;
          case 'j':
            printf("look left\n");
            break;
          case 'l':
            printf("look right\n");
            break;
          case 'k':
            printf("return normal\n");
            break;
          case 't':
            printf("take photo\n");
            break;
          case 'w':
            printf(" watch video\n");
            break;
          default:
            break;
          }
        }
        else
        {
          fulanghua_action::testFeedback feedback;
          feedback.rate = (ros::Time::now() - start_time).toSec() / current_goal->duration;
          server.publishFeedback(feedback);
          printf("Active: publish feedback id:%i\n", current_goal->task_id);
        }
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}