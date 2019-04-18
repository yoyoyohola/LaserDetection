/**
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: landmark_detector.cc
  * @version: v0.0.1
  * @author: kwhu@visionnav.com
  * @create_date: 2019-04-17 09:04:02
  * @last_modified_date: 2019-04-18 13:00:36
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <laser_detection/landmark_detector.hh>

//CODE

size_t Landmark::count_landmark = 0;
std::vector<Landmark::Ptr> landmarks_save;
cartographer_ros_msgs::SubmapList submap_final_update;

ReflectionMarker::ReflectionMarker(int argc, char** argv)
{
  ros::init(argc, argv, "ReflectionMarker");
  ptr_nh_ = std::make_shared<ros::NodeHandle>();
  //all_marker_pub_ = ptr_nh_->advertise<visualization_msgs::Marker>("all_reflection_mark", 10);
  listener_ = std::make_shared<tf::TransformListener>();
  ptr_rate_ = std::make_shared<ros::Rate>(100);
  ptr_detector_ = std::make_shared<LandmarkDetector>(0.1, argv);
  //signal(SIGQUIT, handler);
}

void ReflectionMarker::detectLandmarkCallback(const sensor_msgs::LaserScan::ConstPtr& ptr_scan_data)
{
  tf::StampedTransform tf_info;
  try
  {
    listener_->lookupTransform("/map", "/laser", ros::Time(0), tf_info);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }
  auto detected_landmarks = ptr_detector_->detectMarker(ptr_scan_data, tf_info);
  drawLandmark(detected_landmarks);
}

void ReflectionMarker::detectMarkerCallback(const sensor_msgs::LaserScan::ConstPtr& ptr_scan_data)
{
  tf::StampedTransform  tf_info;
  try
  {
    //listener_->waitForTransform("/laser", "/map", ros::Time(0), ros::Duration(0.1));
    listener_->lookupTransform("/map", "/laser", ros::Time(0), tf_info);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
    //ros::Duration(1.0).sleep();
  }

  auto intensities = ptr_scan_data->intensities;
  auto ranges = ptr_scan_data->ranges;
  auto angle_increment = ptr_scan_data->angle_increment;
  auto min_angle = ptr_scan_data->angle_min;
  //marker_list_.clear();
  //marker_list_map_.clear();

  visualization_msgs::Marker marker;
  static visualization_msgs::Marker marker_map;

  marker.header.frame_id = "laser";
  marker.header.stamp = ros::Time::now();
  marker.ns = "reflection";
  marker.id = 1;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.color.g = 1.0f;
  marker.color.a = 1.0;
  marker.scale.x = 0.01;

  marker_map.header.frame_id = "map";
  marker_map.header.stamp = ros::Time::now();
  marker_map.ns = "reflection";
  marker_map.action = visualization_msgs::Marker::ADD;
  marker_map.id = 1;
  marker_map.type = visualization_msgs::Marker::LINE_LIST;
  marker_map.color.b = 1.0f;
  marker_map.color.a = 1.0;
  marker_map.scale.x = 0.01;
  for(size_t i=0; i<intensities.size(); ++i)
  {
    if(intensities[i] > 800 && ranges[i] > 1.5 && ranges[i] < 12)
    {
      //ROS_INFO_STREAM("Range: " <<  ranges[i] << " Intensity: " << intensities[i]);
      auto angle = i*angle_increment + min_angle;
      auto y = ranges[i] * sin(angle);
      auto x = ranges[i] * cos(angle);
      tf::Vector3 point(x, y, 0.0);
      auto point_map = tf_info * point;
      geometry_msgs::PointStamped p;
      geometry_msgs::PointStamped p_map;
      p.header.stamp = ptr_scan_data->header.stamp;
      p.header.frame_id = ptr_scan_data->header.frame_id;
      p.point.x = x;
      p.point.y = y;
      p.point.z = 0.0;
      p_map.header.stamp = ptr_scan_data->header.stamp;
      p_map.header.frame_id = "map";
      p_map.header.frame_id = "laser";
      p_map.point.x = point_map.getX();
      p_map.point.y = point_map.getY();
      p_map.point.z = 0.0;
      if( ptr_detector_->newPotentialLandmark(p_map) == true)
      {
        // Add to display
        //ROS_WARN_STREAM("Laser: " << x << "," << y
        //                <<" Map: " << point_map.getX() << ", " << point_map.getY());
        marker.points.emplace_back(p.point);
        marker_map.points.emplace_back(p_map.point);
        p.point.z = 3.0;
        p_map.point.z = 3.0;
        marker.points.emplace_back(p.point);
        marker_map.points.emplace_back(p_map.point);
      }
      else
      {
        continue;
      }
    }
  }
  if(marker.points.size() > 0)
  {
    marker_ = marker;
    marker_map_ = marker_map;
    ROS_WARN_STREAM("Size: " << marker_map_.points.size());
  }
  // if(update)
  // {
  //   update_landmark_save;
  // }
}

void ReflectionMarker::detectMarkerWithSubmapCallback(const sensor_msgs::LaserScan::ConstPtr& ptr_scan_data)
{
  tf::StampedTransform tf_info;
  try
  {
    //listener_->waitForTransform("/laser", "/map", ros::Time(0), ros::Duration(0.1));
    listener_->lookupTransform("/map", "/laser", ros::Time(0), tf_info);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
    //ros::Duration(1.0).sleep();
  }

  //ROS_INFO_STREAM("Target map: FrameID->" << tf_info.frame_id_ << " child_id->" << tf_info.child_frame_id_);
  //ROS_INFO_STREAM("Position" << tf_info.getOrigin().getX()
  //                           << " " << tf_info.getOrigin().getY()
  //                           << " " << 0.0);
  auto intensities = ptr_scan_data->intensities;
  auto ranges = ptr_scan_data->ranges;
  auto angle_increment = ptr_scan_data->angle_increment;
  auto min_angle = ptr_scan_data->angle_min;
  //marker_list_.clear();
  //marker_list_map_.clear();

  visualization_msgs::Marker marker;
  static visualization_msgs::Marker marker_map;

  marker.header.frame_id = "laser";
  marker.header.stamp = ros::Time::now();
  marker.ns = "reflection";
  marker.id = 1;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.color.g = 1.0f;
  marker.color.a = 1.0;
  marker.scale.x = 0.01;

  marker_map.header.frame_id = "map";
  marker_map.header.stamp = ros::Time::now();
  marker_map.ns = "reflection";
  marker_map.action = visualization_msgs::Marker::ADD;
  marker_map.id = 1;
  marker_map.type = visualization_msgs::Marker::LINE_LIST;
  marker_map.color.b = 1.0f;
  marker_map.color.a = 1.0;
  marker_map.scale.x = 0.01;
  for(size_t i=0; i<intensities.size(); ++i)
  {
    if(intensities[i] > 800 && ranges[i] > 1.5 && ranges[i] < 12)
    {
      //ROS_INFO_STREAM("Range: " <<  ranges[i] << " Intensity: " << intensities[i]);
      auto angle = i*angle_increment + min_angle;
      auto y = ranges[i] * sin(angle);
      auto x = ranges[i] * cos(angle);
      tf::Vector3 point(x, y, 0.0);
      auto point_map = tf_info * point;
      geometry_msgs::PointStamped p;
      geometry_msgs::PointStamped p_map;
      p.header.stamp = ptr_scan_data->header.stamp;
      p.header.frame_id = ptr_scan_data->header.frame_id;
      p.point.x = x;
      p.point.y = y;
      p.point.z = 0.0;
      p_map.header.stamp = ptr_scan_data->header.stamp;
      p_map.header.frame_id = "map";
      p_map.header.frame_id = "laser";
      p_map.point.x = point_map.getX();
      p_map.point.y = point_map.getY();
      p_map.point.z = 0.0;
      if( ptr_detector_->newPotentialLandmarkWithSubmap(x,
                                                        y,
                                                        0.0,
                                                        tf_info) == true)
      {
        // Add to display
        //ROS_WARN_STREAM("Laser: " << x << "," << y
        //                <<" Map: " << point_map.getX() << ", " << point_map.getY());
        marker.points.emplace_back(p.point);
        marker_map.points.emplace_back(p_map.point);
        p.point.z = 3.0;
        p_map.point.z = 3.0;
        marker.points.emplace_back(p.point);
        marker_map.points.emplace_back(p_map.point);
      }
      else
      {
        continue;
      }
    }
  }
  if(marker.points.size() > 0)
  {
    marker_ = marker;
    marker_map_ = marker_map;
    ROS_WARN_STREAM("Size: " << marker_map_.points.size());
  }
  // if(update)
  // {
  //   update_landmark_save;
  // }
}

void ReflectionMarker::publishMarker()
{
  marker_pub_.publish(marker_);
  marker_map_pub_.publish(marker_map_);
  marker_prior_pub_.publish(marker_prior_);
}

void ReflectionMarker::drawPriorLandmark()
{
  marker_prior_.header.frame_id = "map";
  marker_prior_.header.stamp = ros::Time::now();
  marker_prior_.ns = "prior_";
  marker_prior_.action = visualization_msgs::Marker::ADD;
  marker_prior_.id = 1;
  marker_prior_.type = visualization_msgs::Marker::LINE_LIST;
  marker_prior_.color.g = 1.0f;
  marker_prior_.color.r = 0.6f;

  marker_prior_.color.a = 1.0;
  marker_prior_.scale.x = 0.1;
  geometry_msgs::PointStamped prior_point;
  auto prior_landmarks = ptr_detector_->getLandmarks();
  for(const auto& ptr_landmark:prior_landmarks)
  {
    prior_point.point.x = ptr_landmark->position_tf_.getX();
    prior_point.point.y = ptr_landmark->position_tf_.getY();
    prior_point.point.z = 0.0;
    marker_prior_.points.emplace_back(prior_point.point);

    prior_point.point.z = 3.0;
    marker_prior_.points.emplace_back(prior_point.point);
  }
}

std::vector<Landmark::Ptr> LandmarkDetector::detectMarker(const sensor_msgs::LaserScan::ConstPtr& ptr_scan_data, const tf::StampedTransform& tf_info)
{
  auto intensities = ptr_scan_data->intensities;
  auto ranges = ptr_scan_data->ranges;
  auto angle_increment = ptr_scan_data->angle_increment;
  auto min_angle = ptr_scan_data->angle_min;
  std::vector<Landmark::Ptr> detected_landmarks;

  for(auto& ptr_landmark: landmarks_)
  {
    ptr_landmark->reset();
  }

  for(size_t i=0; i<intensities.size(); ++i)
  {
    if(intensities[i] > 800 && ranges[i] > 1.5 && ranges[i] < 12)
    {
      ROS_INFO_STREAM("Range: " <<  ranges[i] << " Intensity: " << intensities[i]);
      auto angle = i*angle_increment + min_angle;
      auto y = ranges[i] * sin(angle);
      auto x = ranges[i] * cos(angle);
      tf::Vector3 point(x, y, 0.0);
      //auto point_map = tf_info * point;

      //newPotentialLandmark(x, y, 0.0, tf_info);

      // newPotentialLandmark, if detected, save it with TF(From laser to the newest submap)
      newPotentialLandmarkWithSubmap(x, y, 0.0, tf_info);
    }
  }

  for(auto& ptr_landmark: landmarks_)
  {
    if(ptr_landmark->is_detected_ == true)
    {
      ptr_landmark->update();
      ptr_landmark->transform(tf_info, ptr_landmark->position_mean_);
      detected_landmarks.emplace_back(ptr_landmark);
    }
  }
  return detected_landmarks;
}

void ReflectionMarker::drawLandmark(const std::vector<Landmark::Ptr>& detected_landmarks)
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = "laser";
  marker.header.stamp = ros::Time::now();
  marker.ns = "reflection";
  marker.id = 1;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.color.g = 1.0f;
  marker.color.b = 0.7f;
  marker.color.r = 0.5f;
  marker.color.a = 1.0;
  marker.scale.x = 0.09;
  
  for(const auto& ptr_landmark:detected_landmarks)
  {
    geometry_msgs::PointStamped p;
    p.point.x = ptr_landmark->position_mean_.getX();
    p.point.y = ptr_landmark->position_mean_.getY();
    p.point.z = 0.0f;
    marker.points.emplace_back(p.point);
    p.point.z = 2.0f;
    marker.points.emplace_back(p.point);
  }

  if(marker.points.size() > 0)
  {
    marker_detected_ = marker;
  }
  ROS_INFO_STREAM("Detected landmark: " << detected_landmarks.size());
}


//bool LandmarkDetector::detectLandmark(const Landmark::Ptr& ptr_potential_landmark)
//{
//  for(auto& ptr_landmark:landmarks_)
//  {
//    if(ptr_landmark == nullptr)
//    {
//      continue;
//    }
//    if(isAttached(ptr_landmark, ptr_potential_landmark) == true)
//    {
//      if(ptr_landmark->is_detected_ == false)
//      {
//        ptr_landmark->is_detected_ = true;
//      }
//      return true;
//    }
//  }
//  return false;
//}

void LandmarkDetector::updateSubmapList(const cartographer_ros_msgs::SubmapList::ConstPtr& ptr_submap_list)
{
  submap_list_ = *ptr_submap_list;
  submap_final_update = submap_list_;
}
void LandmarkDetector::readLandmark(const std::string& prior_landmark_filepath)
{
  ROS_INFO_STREAM("Landmark: " << prior_landmark_filepath);
  std::ifstream file_in(prior_landmark_filepath);
  if(!file_in)
  {
    ROS_FATAL_STREAM("No such file calling" << prior_landmark_filepath);
    exit(-1);
  }
  if(file_in.is_open() == false)
  {
    ROS_FATAL_STREAM("File: " << prior_landmark_filepath << " open failed. Please check.");
    exit(1);
  }
  landmarks_.clear();
  while(!file_in.eof())
  {
    double x,y;
    file_in >> x >> y;
    ROS_INFO_STREAM("X: " << x << ", Y: " << y);
    if(file_in.good() == false)
    {
      break;
    }
    Landmark::Ptr ptr_prior_landmark= std::make_shared<Landmark>(x, y, 0.);
    ptr_prior_landmark->id_ = Landmark::count_landmark++;
    for(int i=0;i<4;++i)
    {
      file_in >> ptr_prior_landmark->covariance[i];
    }
    landmarks_.emplace_back(ptr_prior_landmark);
    //ROS_INFO_STREAM("ID: " << ptr_prior_landmark->id_);
  }
  ROS_INFO_STREAM("Size of prior: " << landmarks_.size());
  landmarks_save = landmarks_;
  //exit(0);
}


bool LandmarkDetector::isAttached(const Landmark::Ptr& ptr_landmark,
                                  const Landmark::Ptr& ptr_potential_landmark)
{
  auto error = ptr_landmark->position_tf_.distance(ptr_potential_landmark->position_tf_);
  ROS_INFO_STREAM("Error: " << error << " threshold: " << threshold_);
  if(error < threshold_)
  {
    return true;
  }
  return false;
}

bool LandmarkDetector::newPotentialLandmark(double x, double y, double z)
{
  Landmark::Ptr ptr_potential_landmark = std::make_shared<Landmark>(x, y, z);
  ptr_potential_landmark->id_ = Landmark::count_landmark;
  bool attached_flag = false;
  if(landmarks_.size() == 0)
  {
    landmarks_.emplace_back(ptr_potential_landmark);
    return true;
  }
  else
  {
    for(auto& ptr_landmark:landmarks_)
    {
      if(ptr_landmark == nullptr)
      {
        ROS_ERROR_STREAM("Nullptr landmark");
        continue;
      }
      if(isAttached(ptr_landmark, ptr_potential_landmark) == true)
      {
        ROS_INFO_STREAM("Detected");
        ptr_potential_landmark->id_ = ptr_landmark->id_;
        ptr_landmark->is_detected_ = true;
        ptr_landmark->addClusterLandmark(ptr_potential_landmark);
        ++ptr_landmark->num_observation_;
        attached_flag = true;
      }
    }
    if(attached_flag == false)
    {
      //Landmark::count_landmark++;
      //ptr_potential_landmark->id_ = Landmark::count_landmark;
      //landmarks_.emplace_back(ptr_potential_landmark);
    }
  }
  landmarks_save = landmarks_;
  return attached_flag;
}

bool LandmarkDetector::newPotentialLandmark(double x, double y, double z, const tf::StampedTransform& tf_info)
{
  Landmark::Ptr ptr_potential_landmark = std::make_shared<Landmark>(x, y, z);
  ptr_potential_landmark->transform(tf_info);
  ptr_potential_landmark->id_ = Landmark::count_landmark;
  //ROS_INFO_STREAM("Laser x: " << x << " y: " << y);
  //ROS_INFO_STREAM("Map x: " << ptr_potential_landmark->position_tf_.getX()
  //                << " y: " << ptr_potential_landmark->position_tf_.getY());
  bool attached_flag = false;
  if(landmarks_.size() == 0)
  {
    landmarks_.emplace_back(ptr_potential_landmark);
    return true;
  }
  else
  {
    for(auto& ptr_landmark:landmarks_)
    {
      if(ptr_landmark == nullptr)
      {
        continue;
      }
      if(isAttached(ptr_landmark, ptr_potential_landmark) == true)
      {
        ROS_INFO_STREAM("Detected");
        ptr_potential_landmark->id_ = ptr_landmark->id_;
        ptr_landmark->is_detected_ = true;
        ptr_landmark->addClusterLandmark(ptr_potential_landmark);
        ++ptr_landmark->num_observation_;
        attached_flag = true;
      }
    }
    if(attached_flag == false)
    {
      //Landmark::count_landmark++;
      //ptr_potential_landmark->id_ = Landmark::count_landmark;
      //landmarks_.emplace_back(ptr_potential_landmark);
    }
  }
  landmarks_save = landmarks_;
  return attached_flag;
}

bool LandmarkDetector::newPotentialLandmarkWithSubmap(double x, double y, double z, const tf::StampedTransform& tf_info)
{
  Landmark::Ptr ptr_potential_landmark = std::make_shared<Landmark>(x, y, z);
  ptr_potential_landmark->transform(tf_info);
  ptr_potential_landmark->id_ = Landmark::count_landmark;
  auto submap_size = submap_list_.submap.size();
  //ROS_INFO_STREAM("Laser x: " << x << " y: " << y);
  //ROS_INFO_STREAM("Map x: " << ptr_potential_landmark->position_tf_.getX()
  //                << " y: " << ptr_potential_landmark->position_tf_.getY());
  bool attached_flag = false;
  if(landmarks_.size() == 0)
  {
    landmarks_.emplace_back(ptr_potential_landmark);
    return true;
  }
  else
  {
    for(auto& ptr_landmark:landmarks_)
    {
      if(ptr_landmark == nullptr)
      {
        continue;
      }
      if(isAttached(ptr_landmark, ptr_potential_landmark) == true)
      {
        ROS_INFO_STREAM("Detected");
        ptr_potential_landmark->id_ = ptr_landmark->id_;
        ptr_landmark->is_detected_ = true;
        ptr_landmark->addClusterLandmark(ptr_potential_landmark);
        ++ptr_landmark->num_observation_;
        attached_flag = true;

        auto index_submap_attached = submap_size - 1;
        if(index_submap_attached >= 0)
        {
          // Prepare Submap info
          auto origin = submap_list_.submap[index_submap_attached].pose.position;
          auto quaternion = submap_list_.submap[index_submap_attached].pose.orientation;
          tf::Vector3 ori(origin.x, origin.y, origin.z);
          tf::Quaternion quat(quaternion.x,quaternion.y,quaternion.z,quaternion.w);
          tf::Transform tf_submap_map(quat, ori);
          auto pos_landmark_at_submap = tf_submap_map.inverse() * tf_info * ptr_potential_landmark->position_;
          ptr_potential_landmark->submap_index_ = index_submap_attached;
          ptr_potential_landmark->position_submap_ = pos_landmark_at_submap;
          
        }
      }
    }
    if(attached_flag == false)
    {
      //Landmark::count_landmark++;
      //ptr_potential_landmark->id_ = Landmark::count_landmark;
      //landmarks_.emplace_back(ptr_potential_landmark);
    }
  }
  landmarks_save = landmarks_;
  return attached_flag;
}

void handler(int signum)
{
  ROS_INFO("%s is received, Terminating the node...", strsignal(signum));
  std::ofstream file_out;
  std::string prefix(getenv("HOME"));
  file_out.open(prefix+"/poster_landmark.txt");
  ROS_INFO_STREAM("Save align landmarks");
  ROS_INFO_STREAM("Size of landmark: " << landmarks_save.size());
  for(auto& ptr_landmark:landmarks_save)
  {
    if(ptr_landmark == nullptr)
    {
      ROS_ERROR_STREAM("Nullptr ptr_landmark");
      continue;
    }
    file_out << ptr_landmark->position_mean_.getX() << " "<< ptr_landmark->position_mean_.getY() << " ";
    file_out << ptr_landmark->covariance[0] << " "
             << ptr_landmark->covariance[1] << " "
             << ptr_landmark->covariance[2] << " "
             << ptr_landmark->covariance[3] << std::endl;
  }
  file_out.close();
  ROS_WARN_STREAM("Close file");
  ROS_WARN_STREAM("Shutdown ROS node");
  ros::shutdown();
  exit(signum);
}


void handler_submap(int signum)
{
  ROS_INFO("%s is received, Terminating the node...", strsignal(signum));
  std::ofstream file_out;
  std::string prefix(getenv("HOME"));
  file_out.open(prefix+"/poster_landmark.txt");
  ROS_INFO_STREAM("Save align landmarks(submap)");
  ROS_INFO_STREAM("Size of landmark: " << landmarks_save.size());

  for(auto& ptr_landmark:landmarks_save)
  {
    if(ptr_landmark == nullptr)
    {
      ROS_ERROR_STREAM("Nullptr ptr_landmark");
      continue;
    }
    //ptr_landmark->position_mean_ = tf::Vector3(0.0, 0.0, 0.0);
    auto position_mean = tf::Vector3(0.0, 0.0, 0.0);
    auto cluster_size = ptr_landmark->cluster_landmarks_.size();
    for(auto ptr_cluster_landmark: ptr_landmark->cluster_landmarks_)
    {
      auto submap_id = ptr_cluster_landmark->submap_index_;
      auto origin = submap_final_update.submap[submap_id].pose.position;
      auto quaternion = submap_final_update.submap[submap_id].pose.orientation;
      auto pos_at_submap = ptr_cluster_landmark->position_submap_;
      tf::Vector3 ori(origin.x, origin.y, origin.z);
      tf::Quaternion quat(quaternion.x,quaternion.y,quaternion.z,quaternion.w);
      tf::Transform tf_submap_map(quat, ori);
      ptr_cluster_landmark->position_ = tf_submap_map * pos_at_submap;
      ptr_cluster_landmark->position_tf_ = tf_submap_map * pos_at_submap;
    }
    // Get position_mean_ after optimization with submap
    ptr_landmark->update();
    file_out << ptr_landmark->position_mean_.getX() << " "<< ptr_landmark->position_mean_.getY() << " ";
    file_out << ptr_landmark->covariance[0] << " "
             << ptr_landmark->covariance[1] << " "
             << ptr_landmark->covariance[2] << " "
             << ptr_landmark->covariance[3] << std::endl;
  }
  //for(auto& ptr_landmark:landmarks_save)
  //{
  //  if(ptr_landmark == nullptr)
  //  {
  //    ROS_ERROR_STREAM("Nullptr ptr_landmark");
  //    continue;
  //  }
  //  file_out << ptr_landmark->position_mean_.getX() << " "<< ptr_landmark->position_mean_.getY() << " ";
  //  file_out << ptr_landmark->covariance[0] << " "
  //           << ptr_landmark->covariance[1] << " "
  //           << ptr_landmark->covariance[2] << " "
  //           << ptr_landmark->covariance[3] << std::endl;
  //}
  file_out.close();
  ROS_WARN_STREAM("Close file");
  ROS_WARN_STREAM("Shutdown ROS node");
  ros::shutdown();
  exit(signum);
}
