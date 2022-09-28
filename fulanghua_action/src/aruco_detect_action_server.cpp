#include <ros/ros.h>
 // Include opencv2

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

// Include CvBridge, Image Transport, Image msg
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <fulanghua_action/special_moveAction.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<fulanghua_action::special_moveAction> Server;
struct timespec timer_start, timer_stop;
double fstart, fstop;

class ADJUST_POSITION{
    public:
        ADJUST_POSITION():
        rate(10),
        server(nh, "ar_detect", false)
        {
            ros::NodeHandle private_nh("~"); 
            private_nh.param("Kp", Kp, 0.5);
            private_nh.param("image_topic", IMAGE_TOPIC, std::string("/camera/color/image_raw"));
            private_nh.param("cmd_vel", CMD_VEL_TOPIC, std::string("/cmd_vel"));
            private_nh.param("offset_fixed_x", fixed_x, -0.075);
            private_nh.param("offset_fixed_y", fixed_y, -0.04);
            private_nh.param("offset_fixed_z", fixed_z, 0.37);
            private_nh.param("calibration_path", CALIBRATION, std::string(""));
            image_sub = nh.subscribe(IMAGE_TOPIC, 1000, &ADJUST_POSITION::image_callback,this);
            cmd_vel_pub =nh.advertise<geometry_msgs::Twist>(CMD_VEL_TOPIC, 10);
            cv::FileStorage fs;
            fs.open(CALIBRATION, cv::FileStorage::READ); 
            if (!fs.isOpened())
            {
                std::cout << "Failed to open " << CALIBRATION << std::endl;
            }
            fs["camera_matrix"] >> camera_matrix;
            fs["distortion_coefficients"] >> dist_coeffs;
            std::cout << "camera_matrix\n" << camera_matrix << std::endl;
            std::cout << "\ndist coeffs\n" << dist_coeffs << std::endl;

                    server.start(); //start the server
                }

        void image_callback(const sensor_msgs::ImageConstPtr& msg){
            std_msgs::Header msg_header = msg->header;
            std::string frame_id = msg_header.frame_id.c_str();
            // ROS_INFO_STREAM("New Image from " << frame_id);

            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            src = cv_ptr->image;
        }


        bool adjustPosition(double &x, double &y, double &z, double &ang){
            geometry_msgs::Twist twist;    
            threshold_z = 0.02;
            offset_x = (double)fixed_x - x;
            offset_y = (double)fixed_y - y;
            offset_z = (double)fixed_z - z;
            ROS_INFO("start approching");
            double move_y = Kp*offset_y;
            double move_z = Kp*offset_z;
            twist.linear.x = -move_z; // depth
            twist.angular.z = -move_y*15; // horizontal
            // twist.linear.y = move_x; // horizontal 
            // twist.linear.z = move_y; // vertical
            ROS_INFO("\nOffset_x: %lf, Offset_y: %lf, Offset_z: %lf\n", offset_x, offset_y, offset_z);
            ROS_INFO("\nlinear.x: %lf, linear.y: %lf, linear.z: %lf\n", twist.linear.x, twist.linear.y, twist.linear.z);
            cmd_vel_pub.publish(twist);
            clock_gettime(CLOCK_MONOTONIC, &timer_start); fstart=(double)timer_start.tv_sec + ((double)timer_start.tv_nsec/1000000000.0);

            if(std::abs(offset_z)<=threshold_z){
                    Done_z = true;
            }
            if(Done_z){
              Done_z = false;
              return true;
            }else
              return false;
        }

        void put_commnets(double &x, double &y, double &z, double &_angle){
              vector_to_marker.str(std::string());
              vector_to_marker << std::setprecision(4)
                                << "x: " << std::setw(8) << z;
              cv::putText(src, vector_to_marker.str(),
                        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1,
                        cv::Scalar(0, 252, 124), 1, CV_AVX);

            vector_to_marker.str(std::string());
            vector_to_marker << std::setprecision(4)
                                << "y: " << std::setw(8) << y;
            cv::putText(src, vector_to_marker.str(),
                        cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 1,
                        cv::Scalar(0, 252, 124), 1, CV_AVX);

            vector_to_marker.str(std::string());
            vector_to_marker << std::setprecision(4)
                                << "z: " << std::setw(8) << z;
            cv::putText(src, vector_to_marker.str(),
                        cv::Point(10, 150), cv::FONT_HERSHEY_SIMPLEX, 1,
                        cv::Scalar(0, 252, 124), 1, CV_AVX);
            
            vector_to_marker.str(std::string());
            vector_to_marker << std::setprecision(4)
                                << "angle: " << std::setw(8) << _angle;
            cv::putText(src, vector_to_marker.str(),
                        cv::Point(10, 210), cv::FONT_HERSHEY_SIMPLEX, 1,
                        cv::Scalar(0, 252, 124), 1, CV_AVX);

      }



    
    ros::NodeHandle nh;

    //publishers and  subscribers



    actionlib::SimpleActionServer<fulanghua_action::special_moveAction> server;//make a server
    cv::Mat src,camera_matrix, dist_coeffs;
    double Kp; // proportional coefficient
    double target; // target angle(radian)
    double offset_x =0; double offset_y=0; double offset_z=0;
    bool Done_x = false; bool Done_y = false; bool Done_z = false; bool Done_r= false;
    double fixed_x,fixed_y, fixed_z;
    double threshold_x, threshold_y, threshold_z;
    std::string CMD_VEL_TOPIC; // target cmd_vel
    std::string IMAGE_TOPIC; // image topic 
    ros::Rate rate; // set the rate 
    
    ros::Time start_time;
    
    fulanghua_action::special_moveGoalConstPtr current_goal; // instance of a goal
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50); 
    std::vector<std::vector<cv::Point2f> > corners;
    std::vector<int> ids;
    std::ostringstream vector_to_marker;
    

  private:
      ros::Publisher cmd_vel_pub;
      ros::Subscriber image_sub;
      std::string CALIBRATION;


};



int main(int argc, char** argv){
  ros::init(argc, argv, "aruco_detect_server");
  ADJUST_POSITION adj;

  while (ros::ok()){
      if(adj.server.isNewGoalAvailable()){
          adj.current_goal = adj.server.acceptNewGoal();
          adj.start_time = ros::Time::now();
      }
      if(adj.server.isActive()){
        if(adj.server.isPreemptRequested()){
          adj.server.setPreempted(); // cancel the goal
          ROS_WARN("Preemmpt Goal\n");
        }else{
          if(adj.start_time + ros::Duration(adj.current_goal->duration) < ros::Time::now()){
            adj.server.setAborted(); // abort it
          }
          else{
            if (!adj.src.empty()){
              cv::imshow("src", adj.src);
              ROS_INFO("detected");
              cv::aruco::detectMarkers(adj.src, adj.dictionary, adj.corners, adj.ids);
              ROS_INFO("detect Markers is ok");
              std::vector<cv::Vec3d> rvecs, tvecs;
              cv::aruco::estimatePoseSingleMarkers(adj.corners, 0.05, adj.camera_matrix, adj.dist_coeffs, rvecs, tvecs);
              ROS_INFO("estimate pose is ok");
              if (adj.ids.size()>0){
                  cv::drawFrameAxes(adj.src, adj.camera_matrix, adj.dist_coeffs, rvecs[0], tvecs[0], 0.1);
                  double _angle = rvecs[0](2)*180/M_PI;
                  //putting texst on src
                  adj.put_commnets(tvecs[0](0), tvecs[0](1), tvecs[0](2), _angle);

                  if(adj.adjustPosition(tvecs[0](0), tvecs[0](1), tvecs[0](2), _angle)){
                    adj.server.setSucceeded();
                    ROS_INFO("Succeeded it!");
                    cv::destroyAllWindows();
                  }
                 
              }
              
            }

          }
        }
      }
      ros::spinOnce();
      adj.rate.sleep();
  }
  return 0;
};