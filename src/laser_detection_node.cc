/**
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: laser_detection.cc
  * @version: v0.0.1
  * @author: kwhu@visionnav.com
  * @create_date: 2019-03-08 08:55:04
  * @last_modified_date: 2019-04-18 08:47:12
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <laser_detection/landmark_detector.hh>
//#include <ros/ros.h>
//#include <tf/transform_listener.h>
//#include <nav_msgs/Odometry.h>
//#include <sensor_msgs/LaserScan.h>
//#include <visualization_msgs/Marker.h>
//#include <csignal>
//#include <memory>
//#include <fstream>
//
////CODE
//struct Landmark : public std::enable_shared_from_this<Landmark>
//{
//  public: 
//    using Ptr = std::shared_ptr<Landmark>;
//
//    Landmark() = default;
//    Landmark(double x, double y, double z)
//      :ptr_attached_landmark_(nullptr),
//       position_(x, y, z),
//       position_tf_(x, y, z),
//       position_mean_(x, y, z),
//       quat_(0, 0, 0, 1)
//    {};
//    void transform(tf::StampedTransform& tf)
//    {
//      position_tf_ =  tf * position_;
//    };
//
//    void update()
//    {
//      auto size_cluster = cluster_landmarks_.size();
//      tf::Vector3 average_position(0.0,0.0,0.0);
//      double xy_average = 0.0;
//      for(const auto& ptr_landmark:cluster_landmarks_)
//      {
//        average_position += ptr_landmark->position_;
//        xy_average += ptr_landmark->position_.getX() * ptr_landmark->position_.getY();
//      }
//      average_position /= double(size_cluster);
//      xy_average /= double(size_cluster);
//      position_mean_ = average_position;
//      double var_x = 0;
//      double var_y = 0;
//      double var_xy = 0;
//      for(const auto& ptr_landmark:cluster_landmarks_)
//      {
//        auto error_x = ptr_landmark->position_.getX() - position_mean_.getX();
//        auto error_y = ptr_landmark->position_.getY() - position_mean_.getY();
//        var_x += std::pow(error_x, 2);
//        var_y += std::pow(error_y, 2);
//      }
//      var_x /= double(size_cluster);
//      var_y /= double(size_cluster);
//      var_xy = xy_average - position_mean_.getX()*position_mean_.getY();
//      covariance[0] = var_x;
//      covariance[1] = var_xy;
//      covariance[2] = var_xy;
//      covariance[3] = var_y;
//
//      ROS_WARN_STREAM("Landmark " << this->id_  << ": prior: ("
//                                  << this->position_.getX() << "," << this->position_.getY() << ")"
//                                  );
//      ROS_WARN_STREAM("Landmark " << this->id_  << ": Mean: ("
//                                  << this->position_mean_.getX() << "," << this->position_mean_.getY() << ")"
//                                  );
//      ROS_WARN_STREAM("Landmark " << this->id_  << ": Var: (" << covariance[0]
//                                                        << ", " << covariance[1]
//                                                        << ", " << covariance[2]
//                                                        << ", " << covariance[3] << ")"
//            );
//    }
//    
//    void addClusterLandmark(Landmark::Ptr& ptr_landmark)
//    { 
//      ++num_observation_;
//      cluster_landmarks_.emplace_back(ptr_landmark);
//      update();
//    }
//
//  public:
//    std::vector<Landmark::Ptr> cluster_landmarks_;
//    Landmark::Ptr ptr_attached_landmark_;
//    tf::Vector3 position_;
//    tf::Vector3 position_tf_;
//    tf::Vector3 position_mean_;
//    double covariance[4];
//    tf::Quaternion quat_;
//    size_t id_;
//    size_t num_observation_;
//    static size_t count_landmark;
//};
//
//std::vector<Landmark::Ptr> landmarks_save;
//void handler(int signum);
//size_t Landmark::count_landmark = 0;
//
//class LandmarkDetector
//{
//  public:
//    friend class ReflectionMarker;
//    using Ptr = std::shared_ptr<LandmarkDetector>;
//    LandmarkDetector() = default;
//    LandmarkDetector(double threshold, const std::string& prior_landmark_filepath)
//      : threshold_(threshold)
//    {
//      readPriorLandmark(prior_landmark_filepath);
//    };
//    bool newPotentialLandmark(double x, double y, double z);
//    bool newPotentialLandmark(const geometry_msgs::PointStamped& point)
//    {
//      return newPotentialLandmark(point.point.x, point.point.y, point.point.z);
//    }
//    bool isAttached(const Landmark::Ptr& ptr_landmark,
//                    const Landmark::Ptr& ptr_potential_landmark);
//
//    void readPriorLandmark(const std::string& prior_landmark_filepath);
//    const std::vector<Landmark::Ptr> getLandmarks()
//    { return landmarks_; }
//
//  private:
//    std::vector<Landmark::Ptr> landmarks_;
//    double threshold_{0.1};
//};
//
//void LandmarkDetector::readPriorLandmark(const std::string& prior_landmark_filepath)
//{
//  std::ifstream file_in(prior_landmark_filepath);
//  if(!file_in)
//  {
//    ROS_FATAL_STREAM("No such file calling" << prior_landmark_filepath);
//    exit(-1);
//  }
//  if(file_in.is_open() == false)
//  {
//    ROS_FATAL_STREAM("File: " << prior_landmark_filepath << " open failed. Please check.");
//    exit(1);
//  }
//  landmarks_.clear();
//  while(!file_in.eof())
//  {
//    double x,y;
//    file_in >> x >> y;
//    ROS_INFO_STREAM("X: " << x << ", Y: " << y);
//    if(file_in.good() == false)
//    {
//      break;
//    }
//    Landmark::Ptr ptr_prior_landmark= std::make_shared<Landmark>(x, y, 0.);
//    ptr_prior_landmark->id_ = Landmark::count_landmark++;
//    landmarks_.emplace_back(ptr_prior_landmark);
//    //ROS_INFO_STREAM("ID: " << ptr_prior_landmark->id_);
//  }
//  ROS_INFO_STREAM("Size of prior: " << landmarks_.size());
//  landmarks_save = landmarks_;
//  //exit(0);
//}
//
//bool LandmarkDetector::isAttached(const Landmark::Ptr& ptr_landmark,
//                                  const Landmark::Ptr& ptr_potential_landmark)
//{
//  auto error = ptr_landmark->position_.distance(ptr_potential_landmark->position_);
//  if(error < threshold_)
//  {
//    return true;
//  }
//  return false;
//}
//
//bool LandmarkDetector::newPotentialLandmark(double x, double y, double z)
//{
//  Landmark::Ptr ptr_potential_landmark = std::make_shared<Landmark>(x, y, z);
//  ptr_potential_landmark->id_ = Landmark::count_landmark;
//  bool attached_flag = false;
//  if(landmarks_.size() == 0)
//  {
//    landmarks_.emplace_back(ptr_potential_landmark);
//    return true;
//  }
//  else
//  {
//    for(auto& ptr_landmark:landmarks_)
//    {
//      if(ptr_landmark == nullptr)
//      {
//        continue;
//      }
//      if(isAttached(ptr_landmark, ptr_potential_landmark) == true)
//      {
//        ptr_potential_landmark->id_ = ptr_landmark->id_;
//        ptr_landmark->addClusterLandmark(ptr_potential_landmark);
//        ++ptr_landmark->num_observation_;
//        attached_flag = true;
//      }
//    }
//    if(attached_flag == false)
//    {
//      //Landmark::count_landmark++;
//      //ptr_potential_landmark->id_ = Landmark::count_landmark;
//      //landmarks_.emplace_back(ptr_potential_landmark);
//    }
//  }
//  landmarks_save = landmarks_;
//  return attached_flag;
//}
//
////=======================LandmarkDetector============
////===================================================
//class ReflectionMarker
//{
//  public:
//    ReflectionMarker() = default;
//    ReflectionMarker(int argc, char** argv);
//    ~ReflectionMarker() = default;
//  public:
//    void drawMarker();
//    void detectMarker(const sensor_msgs::LaserScan::ConstPtr& ptr_scan_data);
//    inline void run()
//    {
//      marker_prior_.header.frame_id = "map";
//      marker_prior_.header.stamp = ros::Time::now();
//      marker_prior_.ns = "prior_";
//      marker_prior_.action = visualization_msgs::Marker::ADD;
//      marker_prior_.id = 1;
//      marker_prior_.type = visualization_msgs::Marker::LINE_LIST;
//      marker_prior_.color.g = 1.0f;
//      marker_prior_.color.r = 0.6f;
//
//      marker_prior_.color.a = 1.0;
//      marker_prior_.scale.x = 0.1;
//      geometry_msgs::PointStamped prior_point;
//      auto prior_landmarks = ptr_detector_->getLandmarks();
//      for(const auto& ptr_landmark:prior_landmarks)
//      {
//        prior_point.point.x = ptr_landmark->position_tf_.getX();
//        prior_point.point.y = ptr_landmark->position_tf_.getY();
//        prior_point.point.z = 0.0;
//        marker_prior_.points.emplace_back(prior_point.point);
//
//        prior_point.point.z = 3.0;
//        marker_prior_.points.emplace_back(prior_point.point);
//      }
//      //ROS_INFO_STREAM("Size of prior: " << marker_prior_.points.size());
//      //ROS_INFO_STREAM("Size of prior: " << prior_landmarks.size());
//      //marker_prior
//      while(ros::ok())
//      {
//        ros::spinOnce();
//        drawMarker();
//        //try
//        //{
//        //  tf::StampedTransform tmp_tf;
//        //  listener_->lookupTransform("laser", "map", ros::Time(0), tmp_tf);
//        //}
//        //catch (tf::TransformException ex)
//        //{
//        //  ROS_ERROR("%s", ex.what());
//        //  ros::Duration(0.1).sleep();
//        //}
//      }
//    }
//
//  private:
//    std::shared_ptr<ros::NodeHandle> ptr_nh_;
//    ros::Publisher marker_pub_;
//    ros::Publisher marker_map_pub_;
//    ros::Publisher marker_prior_pub_;
//    //ros::Publisher all_marker_pub_;
//    ros::Subscriber laser_sub_;
//    std::shared_ptr<ros::Rate> ptr_rate_;
//    visualization_msgs::Marker marker_;
//    visualization_msgs::Marker marker_map_;
//    visualization_msgs::Marker marker_prior_;
//    //std::vector<visualization_msgs::Marker> all_marker_list_;
//    std::shared_ptr<tf::TransformListener> listener_;
//    LandmarkDetector::Ptr ptr_detector_;
//};
//
//ReflectionMarker::ReflectionMarker(int argc, char** argv)
//{
//  ros::init(argc, argv, "ReflectionMarker");
//  ptr_nh_ = std::make_shared<ros::NodeHandle>();
//  marker_pub_ = ptr_nh_->advertise<visualization_msgs::Marker>("reflection_mark", 10);
//  marker_map_pub_ = ptr_nh_->advertise<visualization_msgs::Marker>("reflection_mark_map", 10);
//  marker_prior_pub_ = ptr_nh_->advertise<visualization_msgs::Marker>("prior_mark", 10);
//  //all_marker_pub_ = ptr_nh_->advertise<visualization_msgs::Marker>("all_reflection_mark", 10);
//  laser_sub_ = ptr_nh_->subscribe("scan", 10, &ReflectionMarker::detectMarker, this);
//  listener_ = std::make_shared<tf::TransformListener>();
//  ptr_rate_ = std::make_shared<ros::Rate>(100);
//  ptr_detector_ = std::make_shared<LandmarkDetector>(0.2, std::string(argv[1]));
//  signal(SIGINT, handler);
//  //signal(SIGQUIT, handler);
//}
//
//void ReflectionMarker::detectMarker(const sensor_msgs::LaserScan::ConstPtr& ptr_scan_data)
//{
//  tf::StampedTransform tf_info;
//  try
//  {
//    //listener_->waitForTransform("/laser", "/map", ros::Time(0), ros::Duration(0.1));
//    listener_->lookupTransform("/map", "/laser", ros::Time(0), tf_info);
//  }
//  catch (tf::TransformException ex)
//  {
//    ROS_ERROR("%s", ex.what());
//    return;
//    //ros::Duration(1.0).sleep();
//  }
//
//  auto intensities = ptr_scan_data->intensities;
//  auto ranges = ptr_scan_data->ranges;
//  auto angle_increment = ptr_scan_data->angle_increment;
//  auto min_angle = ptr_scan_data->angle_min;
//  //marker_list_.clear();
//  //marker_list_map_.clear();
//
//  visualization_msgs::Marker marker;
//  static visualization_msgs::Marker marker_map;
//
//  marker.header.frame_id = "laser";
//  marker.header.stamp = ros::Time::now();
//  marker.ns = "reflection";
//  marker.id = 1;
//  marker.type = visualization_msgs::Marker::LINE_LIST;
//  marker.color.g = 1.0f;
//  marker.color.a = 1.0;
//  marker.scale.x = 0.01;
//
//  marker_map.header.frame_id = "map";
//  marker_map.header.stamp = ros::Time::now();
//  marker_map.ns = "reflection";
//  marker_map.action = visualization_msgs::Marker::ADD;
//  marker_map.id = 1;
//  marker_map.type = visualization_msgs::Marker::LINE_LIST;
//  marker_map.color.b = 1.0f;
//  marker_map.color.a = 1.0;
//  marker_map.scale.x = 0.01;
//  for(size_t i=0; i<intensities.size(); ++i)
//  {
//    if(intensities[i] > 800 && ranges[i] > 1.5 && ranges[i] < 6)
//    {
//      //ROS_INFO_STREAM("Range: " <<  ranges[i] << " Intensity: " << intensities[i]);
//      auto angle = i*angle_increment + min_angle;
//      auto y = ranges[i] * sin(angle);
//      auto x = ranges[i] * cos(angle);
//      tf::Vector3 point(x, y, 0.0);
//      auto point_map = tf_info * point;
//      geometry_msgs::PointStamped p;
//      geometry_msgs::PointStamped p_map;
//      p.header.stamp = ptr_scan_data->header.stamp;
//      p.header.frame_id = ptr_scan_data->header.frame_id;
//      p.point.x = x;
//      p.point.y = y;
//      p.point.z = 0.0;
//      p_map.header.stamp = ptr_scan_data->header.stamp;
//      p_map.header.frame_id = "map";
//      p_map.header.frame_id = "laser";
//      p_map.point.x = point_map.getX();
//      p_map.point.y = point_map.getY();
//      p_map.point.z = 0.0;
//      if( ptr_detector_->newPotentialLandmark(p_map) == true)
//      {
//        // Add to display
//        //ROS_WARN_STREAM("Laser: " << x << "," << y
//        //                <<" Map: " << point_map.getX() << ", " << point_map.getY());
//        marker.points.emplace_back(p.point);
//        marker_map.points.emplace_back(p_map.point);
//        p.point.z = 3.0;
//        p_map.point.z = 3.0;
//        marker.points.emplace_back(p.point);
//        marker_map.points.emplace_back(p_map.point);
//      }
//      else
//      {
//        continue;
//      }
//    }
//  }
//  if(marker.points.size() > 0)
//  {
//    marker_ = marker;
//    marker_map_ = marker_map;
//    ROS_WARN_STREAM("Size: " << marker_map_.points.size());
//  }
//  // if(update)
//  // {
//  //   update_landmark_save;
//  // }
//}
//
//void ReflectionMarker::drawMarker()
//{
//  marker_pub_.publish(marker_);
//  marker_map_pub_.publish(marker_map_);
//  marker_prior_pub_.publish(marker_prior_);
//  //ROS_WARN_STREAM("Detected Landmarks: " << ptr_detector_->getLandmarks().size());
//}
//
//std::shared_ptr<ReflectionMarker> ptr_reflection_mark;
//void handler(int signum)
//{
//  ROS_INFO("%s is received, Terminating the node...", strsignal(signum));
//  std::ofstream file_out;
//  std::string prefix(getenv("HOME"));
//  file_out.open(prefix+"/poster_landmark.txt");
//  ROS_INFO_STREAM("Save align landmarks");
//  ROS_INFO_STREAM("Size of landmark: " << landmarks_save.size());
//  for(auto& ptr_landmark:landmarks_save)
//  {
//    if(ptr_landmark == nullptr)
//    {
//      ROS_ERROR_STREAM("Nullptr ptr_landmark");
//      continue;
//    }
//    file_out << ptr_landmark->position_mean_.getX() << " "<< ptr_landmark->position_mean_.getY() << std::endl;
//  }
//  file_out.close();
//  ROS_WARN_STREAM("Close file");
//  ROS_WARN_STREAM("Shutdown ROS node");
//  ros::shutdown();
//  exit(signum);
//}

int main(int argc, char** argv)
{
  //ReflectionMarker reflection_mark(argc, argv);
  auto ptr_reflection_mark = std::make_shared<ReflectionMarker>(argc, argv);
  ptr_reflection_mark->run_update_map();
  //ptr_reflection_mark->run_detect_attached_submap();
  return 0;
}
