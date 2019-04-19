#ifndef __LANDMARKDETECTOR_HH__
#define __LANDMARKDETECTOR_HH__
/**-----------------------------------------------
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: landmarkDetector.hh
  * @version: v0.0.1
  * @author: kwhu@visionnav.com
  * @create_date: 2019-04-16 16:54:09
  * @last_modified_date: 2019-04-19 13:26:13
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

// Header include
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <cartographer_ros_msgs/SubmapList.h>
#include <csignal>
#include <memory>
#include <fstream>

// Declaration
class Landmark : public std::enable_shared_from_this<Landmark>
{
  public:  
    using Ptr = std::shared_ptr<Landmark>;

    Landmark() = default;
    Landmark(double x, double y, double z)
      :ptr_attached_landmark_(nullptr),
       position_(x, y, z),
       position_tf_(x, y, z),
       position_map_(x, y, z),
       position_mean_(x, y, z),
       position_submap_(x, y, z),
       quat_(0, 0, 0, 1),
       is_detected_(false)
    {};

    inline static Landmark::Ptr CreateLandmark(double x, double y, double z)
    {
      auto ptr_landmark = std::make_shared<Landmark>(x, y, z);
      ptr_landmark->id_ = Landmark::count_landmark++;
    }
    inline void transform(const tf::StampedTransform& tf)
    {
      //position_tf_ =  tf * position_;
      this->transform(tf, position_);
    };

    inline void transform(const tf::StampedTransform& tf, const tf::Vector3& position)
    {
      position_tf_ =  tf * position;
    }

    inline void reset()
    {
      is_detected_ = false;
      position_mean_ = position_;
    }

    void update()
    {
      auto size_cluster = cluster_landmarks_.size();
      tf::Vector3 average_position(0.0,0.0,0.0);
      double xy_average = 0.0;
      for(const auto& ptr_landmark:cluster_landmarks_)
      {
        average_position += ptr_landmark->position_;
        xy_average += ptr_landmark->position_.getX() * ptr_landmark->position_.getY();
      }
      average_position /= double(size_cluster);
      xy_average /= double(size_cluster);
      position_mean_ = average_position;
      double var_x = 0;
      double var_y = 0;
      double var_xy = 0;
      for(const auto& ptr_landmark:cluster_landmarks_)
      {
        auto error_x = ptr_landmark->position_.getX() - position_mean_.getX();
        auto error_y = ptr_landmark->position_.getY() - position_mean_.getY();
        var_x += std::pow(error_x, 2);
        var_y += std::pow(error_y, 2);
      }
      var_x /= double(size_cluster);
      var_y /= double(size_cluster);
      var_xy = xy_average - position_mean_.getX()*position_mean_.getY();
      covariance[0] = var_x;
      covariance[1] = var_xy;
      covariance[2] = var_xy;
      covariance[3] = var_y;

      ROS_WARN_STREAM("Landmark " << this->id_  << ": prior: ("
                                  << this->position_.getX() << "," << this->position_.getY() << ")"
                                  );
      ROS_WARN_STREAM("Landmark " << this->id_  << ": Mean: ("
                                  << this->position_mean_.getX() << "," << this->position_mean_.getY() << ")"
                                  );
      ROS_WARN_STREAM("Landmark " << this->id_  << ": Var: (" << covariance[0]
                                                        << ", " << covariance[1]
                                                        << ", " << covariance[2]
                                                        << ", " << covariance[3] << ")"
            );
    }
    
    void addClusterLandmark(Landmark::Ptr& ptr_landmark)
    { 
      ++num_observation_;
      cluster_landmarks_.emplace_back(ptr_landmark);
      //update();
    }

    inline const tf::Vector3& getPositionAtMap()
    {
      return position_tf_;
    }

    inline const tf::Vector3& getPositionAtLaser()
    {
      return position_;
    }

  public:
    std::vector<Landmark::Ptr> cluster_landmarks_;
    Landmark::Ptr ptr_attached_landmark_;
    tf::Vector3 position_;
    tf::Vector3 position_tf_;
    tf::Vector3 position_map_;
    tf::Vector3 position_mean_;
    tf::Vector3 position_submap_;
    double covariance[4];
    tf::Quaternion quat_;
    size_t id_;
    size_t num_observation_;
    static size_t count_landmark;
    bool is_detected_;
    int submap_index_;
};

void handler(int signum);
void handler_submap(int signum);

class LandmarkDetector
{
  public:
    friend class ReflectionMarker;
    using Ptr = std::shared_ptr<LandmarkDetector>;
    LandmarkDetector() = default;
    LandmarkDetector(double threshold, char** argv)
      : threshold_(threshold)
    {
      std::string landmark_filepath = std::string(argv[1]);
      readLandmark(landmark_filepath);
    };
    bool newPotentialLandmarkWithSubmap(double x, double y, double z, const tf::StampedTransform& tf_info);
    bool newPotentialLandmark(double x, double y, double z);
    bool newPotentialLandmark(double x, double y, double z, const tf::StampedTransform& tf_info);
    bool newPotentialLandmark(const geometry_msgs::PointStamped& point)
    {
      return newPotentialLandmark(point.point.x, point.point.y, point.point.z);
    }

    bool newPotentialLandmark(Landmark::Ptr& ptr_potential_landmark);

    bool isAttached(const Landmark::Ptr& ptr_landmark,
                    const Landmark::Ptr& ptr_potential_landmark);

    //bool detectLandmark(const Landmark::Ptr& ptr_potential_landmark);

    void readLandmark(const std::string& prior_landmark_filepath);
    const std::vector<Landmark::Ptr> getLandmarks()
    { return landmarks_; }
    void updateSubmapList(const cartographer_ros_msgs::SubmapList::ConstPtr& ptr_submap_list);

    std::vector<Landmark::Ptr> detectLandmark(const sensor_msgs::LaserScan::ConstPtr& ptr_scan_data, const tf::StampedTransform& tf_info);

  private:
    std::vector<Landmark::Ptr> landmarks_;
    cartographer_ros_msgs::SubmapList submap_list_;
    double threshold_{0.1};
};


//=======================LandmarkDetector============
//===================================================
class ReflectionMarker
{
  public:
    ReflectionMarker() = default;
    ReflectionMarker(int argc, char** argv);
    ~ReflectionMarker() = default;
  public:
    void publishMarker();
    void drawLandmark(const std::vector<Landmark::Ptr>& detected_landmarks, const tf::StampedTransform& tf_info);
    void drawPriorLandmark();
    void detectMarkerCallback(const sensor_msgs::LaserScan::ConstPtr& ptr_scan_data);
    void detectMarkerWithSubmapCallback(const sensor_msgs::LaserScan::ConstPtr& ptr_scan_data);
    //std::vector<Landmark::Ptr> detectLandmark(const sensor_msgs::LaserScan::ConstPtr& ptr_scan_data, const tf::StampedTransform& tf_info);
    void detectLandmarkCallback(const sensor_msgs::LaserScan::ConstPtr& ptr_scan_data);
    void updateLaserScan(const sensor_msgs::LaserScan::ConstPtr& ptr_scan_data)
    {
      ptr_scan_data_ = ptr_scan_data;
    }
    void updateSubmapCallback(const cartographer_ros_msgs::SubmapList::ConstPtr& ptr_submap_list)
    {
      //ROS_INFO_STREAM("Update submap_list");
      //submap_list_ = *ptr_submap_list;
      ptr_detector_->updateSubmapList(ptr_submap_list);
      //ROS_INFO_STREAM("SubmapList size: " << ptr_submap_list->submap.size());
    }

    inline void run_update_map()
    {
      tf::StampedTransform tf_info;
      try
      {
        listener_->lookupTransform("/map", "/laser", ros::Time(0), tf_info);
        //ROS_INFO_STREAM("Target map: FrameID->" << tf_info.frame_id_);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
      }
      signal(SIGINT, handler_submap);
      ptr_scan_data_ = nullptr;
      //laser_sub_ = ptr_nh_->subscribe("scan", 2, &ReflectionMarker::updateLaserScan, this);
      //laser_sub_ = ptr_nh_->subscribe("scan", 2, &ReflectionMarker::detectMarkerCallback, this);
      laser_sub_ = ptr_nh_->subscribe("scan", 2, &ReflectionMarker::detectMarkerWithSubmapCallback, this);
      submap_list_sub_ = ptr_nh_->subscribe("submap_list", 1, &ReflectionMarker::updateSubmapCallback, this);
      marker_pub_ = ptr_nh_->advertise<visualization_msgs::Marker>("reflection_mark", 2);
      marker_map_pub_ = ptr_nh_->advertise<visualization_msgs::Marker>("reflection_mark_map", 10);
      marker_prior_pub_ = ptr_nh_->advertise<visualization_msgs::Marker>("prior_mark", 2);

      drawPriorLandmark();
      while(ros::ok())
      {
        ros::spinOnce();
        //detectMarkerWithSubmapCallback(ptr_scan_data_);
        publishMarker();
      }
    }

    inline void run_generate_poster_landmark()
    {
      tf::StampedTransform tf_info;
      try
      {
        listener_->lookupTransform("/map", "/laser", ros::Time(0), tf_info);
        //ROS_INFO_STREAM("Target map: FrameID->" << tf_info.frame_id_);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
      }
      signal(SIGINT, handler);
      //laser_sub_ = ptr_nh_->subscribe("scan", 10, &ReflectionMarker::detectMarkerWithSubmapCallback, this);
      laser_sub_ = ptr_nh_->subscribe("scan", 2, &ReflectionMarker::detectMarkerCallback, this);
      submap_list_sub_ = ptr_nh_->subscribe("submap_list", 1, &ReflectionMarker::updateSubmapCallback, this);
      marker_pub_ = ptr_nh_->advertise<visualization_msgs::Marker>("reflection_mark", 2);
      marker_map_pub_ = ptr_nh_->advertise<visualization_msgs::Marker>("reflection_mark_map", 10);
      marker_prior_pub_ = ptr_nh_->advertise<visualization_msgs::Marker>("prior_mark", 2);

      drawPriorLandmark();
      while(ros::ok())
      {
        ros::spinOnce();
        publishMarker();
      }
    }

    inline void run_detect_landmark()
    {
      tf::StampedTransform tf_info;
      try
      {
        listener_->lookupTransform("/map", "/laser", ros::Time(0), tf_info);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
      }
      laser_sub_ = ptr_nh_->subscribe("scan", 10, &ReflectionMarker::detectLandmarkCallback, this);
      marker_prior_pub_ = ptr_nh_->advertise<visualization_msgs::Marker>("prior_marker", 10);
      marker_detected_pub_ = ptr_nh_->advertise<visualization_msgs::Marker>("detected_marker", 10);
      drawPriorLandmark();
      while(ros::ok())
      {
        ros::spinOnce();
        marker_detected_pub_.publish(marker_detected_);
        marker_prior_pub_.publish(marker_prior_);
      }
    }

    inline void run_detect_attached_submap()
    {
      submap_list_sub_ = ptr_nh_->subscribe("submap_list", 1, &ReflectionMarker::updateSubmapCallback, this);
      while(ros::ok())
      {
        ros::spinOnce();
      }
    }

  private:
    std::shared_ptr<ros::NodeHandle> ptr_nh_;
    ros::Publisher marker_pub_;
    ros::Publisher marker_map_pub_;
    ros::Publisher marker_prior_pub_;
    ros::Publisher marker_detected_pub_;
    //ros::Publisher all_marker_pub_;
    ros::Subscriber laser_sub_;
    ros::Subscriber submap_list_sub_;
    std::shared_ptr<ros::Rate> ptr_rate_;
    visualization_msgs::Marker marker_;
    visualization_msgs::Marker marker_map_;
    visualization_msgs::Marker marker_prior_;
    visualization_msgs::Marker marker_detected_;
    //std::vector<visualization_msgs::Marker> all_marker_list_;
    std::shared_ptr<tf::TransformListener> listener_;
    LandmarkDetector::Ptr ptr_detector_;
    sensor_msgs::LaserScan::ConstPtr ptr_scan_data_;
};



//int main(int argc, char** argv)
//{
//  //ReflectionMarker reflection_mark(argc, argv);
//  ptr_reflection_mark = std::make_shared<ReflectionMarker>(argc, argv);
//  ptr_reflection_mark->run();
//  return 0;
//}
#endif // __LANDMARKDETECTOR_HH__
