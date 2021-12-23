#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include <iostream>
#include <fstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "goal_pub");
  ros::NodeHandle nh;

  //ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
  ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1000);
  //ros::Subscriber map_sub = nh.subscribe("map", 1000, map_call);

  ros::Rate loop_rate(10);
  std::ifstream f;

  geometry_msgs::PoseStamped msg;
  geometry_msgs::PoseStamped tmp;
  while (ros::ok())
  {
    tmp = msg;
    std::ifstream f;
    std_msgs::String t;

    std::string group = ros::this_node::getNamespace();

    if(group == "/"){
      f.open("/home/csw/catkin_ws/src/multi_robot_ros/src/test");
    }
    else{
      f.open("/home/csw/catkin_ws/src/multi_robot_ros/src" + group);
    }
    std::string text1, text2;



    getline(f,text1);
    msg.pose.position.x = std::stod(text1);

    getline(f,text2);
    msg.pose.position.y = std::stod(text2);
    msg.pose.position.z = 0;

    msg.pose.orientation.w = 0.5;
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;

    msg.header.frame_id = "map";
    msg.header.seq = 1; //count
    msg.header.stamp.sec = 0;
    msg.header.stamp.nsec = 0;

    if(tmp != msg){
      t.data = group;
      ROS_INFO("%s",t.data.c_str());
      t.data = text1;
      ROS_INFO("%s",t.data.c_str());
      t.data = text2;
      ROS_INFO("%s",t.data.c_str());
      goal_pub.publish(msg);
    }

    f.close();
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
