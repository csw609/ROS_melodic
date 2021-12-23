#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
#include <iostream>

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  tf::TransformListener listener;
  std::string test;
  std::cin >> test;
  ROS_INFO("%s", test.c_str());
  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/map_path", "/base_footprint",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("");
    ROS_INFO("frame id : ", transform.frame_id_.c_str());
    ROS_INFO("chile frame id : ", transform.child_frame_id_.c_str());
    ROS_INFO("translastion");
    ROS_INFO("x : %lf", transform.getOrigin().x());
    ROS_INFO("y : %lf", transform.getOrigin().y());
    ROS_INFO("z : %lf", transform.getOrigin().z());
    ROS_INFO("");
    ROS_INFO("rotation");
    ROS_INFO("x : %lf", transform.getRotation().x());
    ROS_INFO("y : %lf", transform.getRotation().y());
    ROS_INFO("z : %lf", transform.getRotation().z());
    ROS_INFO("w : %lf", transform.getRotation().w());
    ROS_INFO("");

    rate.sleep();
  }
  return 0;
}
