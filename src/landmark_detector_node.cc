/**
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: landmark_detector_node.cc
  * @version: v0.0.1
  * @author: kwhu@visionnav.com
  * @create_date: 2019-04-17 09:17:13
  * @last_modified_date: 2019-04-17 11:58:06
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <laser_detection/landmark_detector.hh>

//CODE
int main(int argc, char** argv)
{
  auto ptr_reflection_mark = std::make_shared<ReflectionMarker>(argc, argv);
  ptr_reflection_mark->run_detect_landmark();
  return 0;
}
