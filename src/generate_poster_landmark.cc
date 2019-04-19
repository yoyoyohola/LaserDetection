/**
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: laser_detection.cc
  * @version: v0.0.1
  * @author: kwhu@visionnav.com
  * @create_date: 2019-03-08 08:55:04
  * @last_modified_date: 2019-04-18 13:40:00
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <laser_detection/landmark_detector.hh>

int main(int argc, char** argv)
{
  auto ptr_reflection_mark = std::make_shared<ReflectionMarker>(argc, argv);
  ptr_reflection_mark->run_generate_poster_landmark();
  return 0;
}
