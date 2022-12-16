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
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionServer<fulanghua_action::special_moveAction> Server;
struct timespec timer_start, timer_stop;
double fstart, fstop;

class ADJUST_POSITION{
    public:
        ADJUST_POSITION():
        rate(10),
        server(nh, "ar_align", false),
        rotate_client("rotate", true),
        go_straight_client("go_straight", true)
        {
            ros::NodeHandle private_nh("~"); 
            private_nh.param("Kp", Kp, 0.03);
            private_nh.param("Ki", Ki, 0.003);
            private_nh.param("Kpang", Kpang, 1.5);
            private_nh.param("image_topic", IMAGE_TOPIC, std::string("/camera/rgb/image_raw"));
            private_nh.param("cmd_vel", CMD_VEL_TOPIC, std::string("/cmd_vel"));
            private_nh.param("adjust_speed", adjust_speed, 0.01);
            private_nh.param("holonomic", holonomic_, false);
            private_nh.param("offset_fixed_x", fixed_x, 0.35);
            private_nh.param("offset_fixed_y", fixed_y, 0.00);
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
            w = src.size().width;
            h = src.size().height;
            // cv::line(src, cv::Point(w/3,0), cv::Point(w/3,h), cv::Scalar(255, 0, 0), thickness, cv::LINE_8);
            // cv::line(src, cv::Point(w/2,0), cv::Point(w/2,h), cv::Scalar(255, 255, 0), thickness, cv::LINE_8);
        }


        //rotate the robot until the marker comes to the center of the camera image
        bool lookAMarker(double &y){
          double error = y;
          if(abs(error) > 0.01){
            ROS_INFO("error:%lf",error);
            double ang_s = Kpang * error;
            twist.angular.z = abs(ang_s) > 1.0 ? 1.0 * ang_s / abs(ang_s) : ang_s;
            twist.linear.x = 0;
            cmd_vel_pub.publish(twist);
            return false;
          }else{
            return true;
          }
        }

        //align the direction of the robot before approaching to the marker.
        bool alignRobot(double &x, double &y, double &ang, bool align){
          if(align){
            if(lookAMarker(y)){
              if (_counter>40){
                std::sort(angle_array.begin(), angle_array.end());
                angle = angle_array[angle_array.size()/2-1] * 0.6; // calculate median of marker's angle and time 0.6.             
                _counter = -1;
                return false;
              }else if (_counter>=0){
                angle_array.push_back(ang);
                _counter++;
                return false;
              }else if(abs(angle) > 3){
                double direction = angle > 0 ? 90 - angle : -90 - angle;
                rotate_action(direction);
                double theta = 3.14 * angle /180;
                double distance = abs(x * sin(theta));
                go_straight_action(distance);
                return false;
              }else{
                return true;
              }
            }else{
              return false;
            }
          }else{
            return true;
          }
        }


        //approach to the marker adjusting pisition
        bool adjustPosition(double &x, double &y, double &z, double &ang){
            Done_x = false;
            double t = (ros::Time::now() - start_time).toSec();
            threshold_x = (2 + 0.05*t) * 0.001;//0.002;

            //robot(x, y, z) <-> aruco(z, x, y)
            offset_x = z - (double)fixed_x;
            offset_y = (double)fixed_y - x;
            offset_z = (double)fixed_z - y;
            ROS_INFO("start approching");
            offset_I += offset_x;
            double move_x = Kp * offset_x + Ki * offset_I;
            double rotate_z = Kpang * offset_y;
            ROS_INFO("move x: %lf,\n", move_x);
            twist.linear.x = move_x;
            twist.angular.z = rotate_z; // horizontal
            clock_gettime(CLOCK_MONOTONIC, &timer_start); fstart=(double)timer_start.tv_sec + ((double)timer_start.tv_nsec/1000000000.0);
            
            //if offset is smaller than threshold, position adjastment has done
            if(std::abs(offset_x)<=threshold_x){
              double target = 180;
              rotate_action(target);
              Done_x = true;
            }

            //getting the angle after position adjastment has done
            if(Done_x){
              // Done_z = false;
              //move it to the center
              double angle=0;
              offset_I = 0;
              return true;
            }else{
              if(holonomic_){
                ROS_INFO("Offset_z: %lf,\n", offset_z);
                ROS_INFO("Linear_y: %lf,\n", twist.linear.y);
                ROS_INFO("threshold: %lf,\n", threshold_z);
                ROS_INFO("Agnle: %lf,\n", ang);
              }else{
                ROS_INFO("Offset_x: %lf,\n", offset_x);
                ROS_INFO("linear x: %lf,\n", twist.linear.x);
                ROS_INFO("threshold: %lf,\n", threshold_z);
                ROS_INFO("Agnle: %lf,\n", ang);
              }
                cmd_vel_pub.publish(twist);
              return false;
            }
              
        }


        //rotate angle_ degree
        bool rotate_action(double &angle_){
          bool state = true;//
          if (rotate_client.isServerConnected()){
            fulanghua_action::special_moveGoal current_goal;
            current_goal.duration = 20;
            current_goal.angle = angle_;
            for (int i=0;i<5;i++){
              state = true;  
              rotate_client.sendGoal(current_goal);
              actionlib::SimpleClientGoalState client_state = rotate_client.getState();
              while(client_state !=actionlib::SimpleClientGoalState::SUCCEEDED){
                client_state = rotate_client.getState();
                if (client_state == actionlib::SimpleClientGoalState::PREEMPTED || client_state == actionlib::SimpleClientGoalState::ABORTED){
                  ROS_WARN("failed %d times\n", i+1);
                  state = false;
                  break;
                }
                ros::Duration(0.1).sleep();
              }
              if (state){
                break;
              }
            }
          }
        }


        //go straight x[m] ahead.
         bool go_straight_action(double &x){
          bool state = true;
          if (rotate_client.isServerConnected()){
            fulanghua_action::special_moveGoal current_goal;
            current_goal.duration = 20;
            current_goal.distance = x;
            for (int i=0;i<5;i++){
              state = true;  
              go_straight_client.sendGoal(current_goal);
              actionlib::SimpleClientGoalState client_state = go_straight_client.getState();
              while(client_state !=actionlib::SimpleClientGoalState::SUCCEEDED){
                client_state = go_straight_client.getState();
                if (client_state == actionlib::SimpleClientGoalState::PREEMPTED || client_state == actionlib::SimpleClientGoalState::ABORTED){
                  ROS_WARN("failed %d times\n", i+1);
                  state = false;
                  break;
                }
                ros::Duration(0.1).sleep();
              }
              if (state){
                break;
              }
            }
          }
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
    actionlib::SimpleActionClient<fulanghua_action::special_moveAction> rotate_client; //for rotation client
    actionlib::SimpleActionClient<fulanghua_action::special_moveAction> go_straight_client;// for go straight client
    cv::Mat src,camera_matrix, dist_coeffs;
    double offset_I = 0;
    double Kp = 0; // proportional coefficient
    double Ki = 0;
    double Kpang = 0;
    double Kv; // derivative coefficient
    double target; // target angle(radian)
    double offset_x =0; double offset_y=0; double offset_z=0;
    bool Done_x = false; bool Done_y = false; bool Done_z = false; bool Done_r= false;
    double fixed_x,fixed_y, fixed_z;
    double threshold_x, threshold_y, threshold_z;
    double c_x= 0.0; double c_y=0.0; double w=0.0; double h=0.0; //in case the marker is on the edge
    double adjust_speed;
    double _counter=0; double angle; std::vector<double> angle_array;
    
    int thickness =2;
    std::string CMD_VEL_TOPIC; // target cmd_vel
    std::string IMAGE_TOPIC; // image topic 
    ros::Rate rate; // set the rate 
    geometry_msgs::Twist twist; 
    ros::Time start_time;
    
    fulanghua_action::special_moveGoalConstPtr current_goal; // instance of a goal
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250); 
    std::vector<std::vector<cv::Point2f> > corners;
    std::vector<int> ids;
    std::ostringstream vector_to_marker;
    

  private:
      ros::Publisher cmd_vel_pub;
      ros::Subscriber image_sub;
      std::string CALIBRATION;
      bool holonomic_;

};



int main(int argc, char** argv){
  ros::init(argc, argv, "aruco_align_server");
  ADJUST_POSITION adj;
  bool visualize = true;
  bool align = true;
  double y = 0;
  while (ros::ok()){
      if(adj.server.isNewGoalAvailable()){
          adj.current_goal = adj.server.acceptNewGoal();
          adj.start_time = ros::Time::now();
          adj._counter=0;
          visualize = true;
          adj.Done_z=false;
      }
      if(adj.server.isActive()){
        if(adj.server.isPreemptRequested()){
          adj.server.setPreempted(); // cancel the goal
          ROS_WARN("AR align: Preemmpt Goal\n");
        }else{
          if(adj.start_time + ros::Duration(adj.current_goal->duration) < ros::Time::now()){
            adj.server.setAborted(); // abort it
          }
          else{
            if (!adj.src.empty()){        
              cv::aruco::detectMarkers(adj.src, adj.dictionary, adj.corners, adj.ids); //detecting a marker
              std::vector<cv::Vec3d> rvecs, tvecs;
              cv::aruco::estimatePoseSingleMarkers(adj.corners, 0.05, adj.camera_matrix, adj.dist_coeffs, rvecs, tvecs); //gettting x,y,z and angle
              if (adj.ids.size()>0 && adj.ids[0] == 114){
                  y = tvecs[0](1);
                  cv::drawFrameAxes(adj.src, adj.camera_matrix, adj.dist_coeffs, rvecs[0], tvecs[0], 0.1); //drawing them on the marker
                  double _angle = rvecs[0](2)*180/M_PI;
                  adj.put_commnets(tvecs[0](0), tvecs[0](1), tvecs[0](2), _angle);//putting texst on src
                 
                  if(adj.alignRobot(tvecs[0](0), tvecs[0](1), _angle, align)){
                    align = false;
                    if(adj.adjustPosition(tvecs[0](0), tvecs[0](1), tvecs[0](2), _angle)){ //adjusting the positiion of the mobile robot
                      adj.server.setSucceeded();
                      ROS_INFO("AR align: Succeeded!");
                      visualize = false;
                      }
                  }
                  }else{
                    adj.lookAMarker(y);
                  }
              cv::imshow("src", adj.src);
              cv::waitKey(3); 
              if (!visualize)
                cv::destroyAllWindows();
            }
          }
        }
      }
      ros::spinOnce();
      adj.rate.sleep();
  }
  return 0;
};
