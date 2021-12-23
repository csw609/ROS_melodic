#ifndef GOAL_FUNCTION_H
#define GOAL_FUNCTION_H

#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include <fstream>
#include <random>

namespace goal {
  void make_goal(geometry_msgs::PoseStamped *goal);

  int random_make(int min, int max);
};

#endif // GOAL_FUNCTION_H
