/**
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: align_prior_landmark_node.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-04-12 10:21:40
  * @last_modified_date: 2019-04-17 14:16:41
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <iostream>
#include <fstream>
#include <memory>
#include <ros/ros.h>
#include <cmath>
#include <csignal>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <dynamic_reconfigure/server.h>
#include <laser_detection/AlignLandmarkConfig.h>

//CODE

class LandmarkAligner
{
  public:
    class PriorLandmark : public tf::Vector3
    {
      public:
        PriorLandmark() = default;
        PriorLandmark(double x, double y, double theta)
          : tf::Vector3(x, y, 0.),
            x_align_(x),
            y_align_(y),
            theta_align_(theta) {ROS_INFO("X: %f, Y: %f, Theta: %f", getX(), getY(), theta_align_); };

      //private:
        double x_align_;
        double y_align_;
        double theta_align_;
    };

  public:
    LandmarkAligner() = default;
    LandmarkAligner(int argc, char** argv);
    //LandmarkAligner()
    void addPriorLandmark(const PriorLandmark& landmark);
    void readLandmarkFromFile(const std::string& file_path);
    void dynamic_params_callback(laser_detection::AlignLandmarkConfig& config, uint32_t level);
    void addVisualizationMark(const tf::StampedTransform& tf);
    void run();

  private:
    tf::StampedTransform tf_to_prior_;
    std::shared_ptr<tf::TransformBroadcaster> ptr_tf_;
    std::shared_ptr<ros::NodeHandle> ptr_nh_;
    std::shared_ptr<ros::Rate> ptr_rate_;
    ros::Publisher prior_landmarks_pub_;

    std::vector<PriorLandmark> prior_landmarks_;
    //visualization_msgs::Marker marker_;
    dynamic_reconfigure::Server<laser_detection::AlignLandmarkConfig>::CallbackType callback_func_;
    std::shared_ptr<dynamic_reconfigure::Server<laser_detection::AlignLandmarkConfig>> ptr_server_;
};

visualization_msgs::Marker marker_saver;
tf::StampedTransform tf_info;
void handler(int signum)
{
  //tf::StampedTransform tf_info;
  //tf::TransformListener tf_listener;
  //try
  //{
  //  ros::Time now = ros::Time::now();
  //  tf_listener.waitForTransform("map", "prior_landmark", now, ros::Duration(1));
  //  tf_listener.lookupTransform("map", "prior_landmark", now, tf_info);
  //}
  //catch (tf::TransformException ex)
  //{
  //  ROS_ERROR("%s", ex.what());
  //}
  ROS_INFO("%s is received, Terminating the node...",  strsignal(signum));
  auto points = marker_saver.points;
  std::ofstream file_out;
  file_out.open("/home/aliben/prior_landmark_align_manual.txt");
  ROS_INFO_STREAM("Save align landmark");
  int i=0;
  for(const auto& point:points)
  {
    auto tf_point = tf_info * tf::Vector3(point.x, point.y, 0);
    if(i==0)
    {
      ROS_INFO_STREAM("X: " << tf_point.getX() << ", Y:" << tf_point.getY());
      file_out << tf_point.getX() << " " << tf_point.getY() << " ";
      file_out << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0<< std::endl;
      i = 1;
      continue;
    }
    else if(i==1)
    {
      i=0;
    }
  }
  file_out.close();
  ros::shutdown();
  exit(signum);
};

void LandmarkAligner::addVisualizationMark(const tf::StampedTransform& tf)
{
  visualization_msgs::Marker marker;
  //marker.header.frame_id = tf.header.frame_id;
  //marker.header.stamp = tf.header.stamp;
  marker.header.frame_id = tf.child_frame_id_;
  marker.header.stamp = tf.stamp_;
  marker.ns = "prior";
  marker.id = 1;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.color.g = 0.5;
  marker.color.b = 0.3;
  marker.color.a = 1.0;
  marker.scale.x = 0.25;
  marker.scale.y = 0.25;

  for(const auto& landmark:prior_landmarks_)
  {
    geometry_msgs::PointStamped point;
    point.header.stamp = marker.header.stamp;
    point.header.frame_id = marker.header.frame_id;
    point.point.x = landmark.getX();
    point.point.y = landmark.getY();
    point.point.z = 0.;
    marker.points.emplace_back(point.point);
    point.point.z = 2.;
    marker.points.emplace_back(point.point);
  }
  marker_saver = marker;
  prior_landmarks_pub_.publish(marker);
}

void LandmarkAligner::run()
{
  ROS_INFO("Spinning node");
  tf::Transform tf(tf::Quaternion::getIdentity(), tf::Vector3(0., 0.,0.));
  tf::StampedTransform tf_stamped(tf, ros::Time::now(), "map",  "prior_landmark");
  tf_to_prior_ = tf_stamped;
  ptr_tf_->sendTransform(tf_to_prior_);
  addVisualizationMark(tf_stamped);
  while(ros::ok())
  {
    //tf_to_prior_.stamp_ = ros::Time::now();
    //ptr_tf_->sendTransform(tf_to_prior_);
    ros::spinOnce();
  }
}

LandmarkAligner::LandmarkAligner(int argc, char** argv)
{
  ros::init(argc, argv, "LandmarkAligner");
  ptr_nh_ = std::make_shared<ros::NodeHandle>();
  ptr_tf_ = std::make_shared<tf::TransformBroadcaster>();
  ptr_rate_ = std::make_shared<ros::Rate>(100);
  ptr_tf_ = std::make_shared<tf::TransformBroadcaster>();
  prior_landmarks_pub_ = ptr_nh_->advertise<visualization_msgs::Marker>("prior_landmarks", 10);
  callback_func_ = boost::bind(&LandmarkAligner::dynamic_params_callback, this, _1, _2);
  ptr_server_ = std::make_shared<dynamic_reconfigure::Server<laser_detection::AlignLandmarkConfig>>();
  ptr_server_->setCallback(callback_func_);
  signal(SIGINT, handler);
  
  if(argc < 2)
  {
    ROS_ERROR("Less params");
    exit(1);
  }

  std::string file_path = std::string(argv[1]);
  readLandmarkFromFile(file_path);
}

void LandmarkAligner::readLandmarkFromFile(const std::string& file_path)
{
  std::ifstream file_in;
  file_in.open(file_path);
  ROS_INFO("HERE");
  if(file_in.is_open() == false)
  {
    ROS_ERROR_STREAM("File " << file_path << " can't not be opened, please check.");
    exit(1);
  }
  ROS_INFO_STREAM("File " << file_path << " opened");
  //std::vector<Point> prior_landmarks;
  while(file_in.eof() == false)
  {
    double x,y;
    file_in >> x >> y;
    if(file_in.good() ==false)
    {
      break;
    }
    //prior_landmarks.emplace_back(Point(x,y,0));
    addPriorLandmark(PriorLandmark(x ,y, 0.));
  }
  file_in.close();
}

void LandmarkAligner::addPriorLandmark(const PriorLandmark& landmark)
{
  prior_landmarks_.emplace_back(landmark);
}

void LandmarkAligner::dynamic_params_callback(laser_detection::AlignLandmarkConfig& config, uint32_t level)
{
  // Calculation TF and tf landmark and display
  auto x = config.x;
  auto y = config.y;
  //auto theta = double(config.theta / M_PI * 180.0);
  auto theta =  config.theta;
  ROS_INFO("Reconfigure Request: x: %f, y: %f, theta: %f", x, y, theta);
  auto rad_theta = theta/180.0*M_PI;
  tf::Quaternion quat;
  tf::Vector3 translation(x, y, 0.0);
  quat.setRPY(0.0, 0.0, rad_theta);
  //tf::Vector3 translation(x*std::cos(rad_theta)-y*std::sin(rad_theta),
  //      x*std::sin(rad_theta)+y*std::cos(rad_theta), 0.0);
  //quat.setRPY(0.0, 0.0, rad_theta);

  tf::Transform tf(quat, translation);
  //const std::string frame_id("prior_landmark");
  //const std::string child_id("map");
  const std::string frame_id("map");
  const std::string child_id("prior_landmark");
  tf::StampedTransform tf_stamped(tf, ros::Time::now(), frame_id, child_id);
  tf_to_prior_ = tf_stamped;

  //for(auto& landmark:prior_landmarks_)
  //{
  //  //auto align_point = landmark.rotate(tf::Vector3(0, 0, 1), rad_theta);
  //  //ROS_INFO_STREAM("Before: " << align_point.getX() << ", " << align_point.getY());
  //  //align_point = tf_stamped * align_point;
  //  //ROS_INFO_STREAM("After: " << align_point.getX() << ", " << align_point.getY());
  //  //landmark.x_align_ = align_point.getX();
  //  //landmark.y_align_ = align_point.getY();
  //  //landmark = tf_stamped * landmark;
  //}
  ptr_tf_->sendTransform(tf_stamped);
  tf_info = tf_stamped;
  addVisualizationMark(tf_stamped);
}

int main(int argc, char** argv)
{
  //ros::init(argc, argv, "AlignPriorLandmark");
  //ros::NodeHandle nh;
  //ros::Publisher prior_landmark_pub = nh.advertise<visualization_msgs::Marker>("PriorLandmark");
  
  //==========================================================
  //std::ifstream file_in;
  //file_in.open(argv[1]);
  //if(file_in.is_open() == false)
  //{
  //  ROS_ERROR_STREAM("File " << argv[1] << " can't not be opened, please check.");
  //  exit(1);
  //}
  //ROS_INFO_STREAM("File " << argv[1] << " opened");
  //std::vector<Point> prior_landmarks;
  //while(file_in.eof() == false)
  //{
  //  double x,y;
  //  file_in >> x >> y;
  //  if(file_in.good() ==false)
  //  {
  //    break;
  //  }
  //  prior_landmarks.emplace_back(Point(x,y,0));
  //}

  //==========================================================
  //dynamic_reconfigure::Server<laser_detection::AlignLandmarkConfig> server;
  //dynamic_reconfigure::Server<laser_detection::AlignLandmarkConfig>::CallbackType callback_func;
  //callback_func = boost::bind(&dynamic_params_callback, _1, _2);
  //server.setCallback(callback_func);

  //ROS_INFO("Spinning node");
  //ros::spin();
  
  LandmarkAligner landmark_aligner(argc, argv);
  landmark_aligner.run();
  return 0;
}
