#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/OccupancyGrid.h"
#include <iostream>
#include <fstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_pub");
  ros::NodeHandle nh;

  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 1000);
  ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    //path set
    nav_msgs::Path path;

    path.header.frame_id = "base_footprint";
    path.header.seq = 1;
    path.header.stamp.sec = 0;
    path.header.stamp.nsec = 0;

    std::vector<geometry_msgs::PoseStamped> path_con;
    geometry_msgs::PoseStamped path_ele;

    path_ele.pose.position.x = 1;
    path_ele.pose.position.y = 1;
    path_ele.pose.position.z = 0;

    path_ele.pose.orientation.w = 0.5;
    path_ele.pose.orientation.x = 0.0;
    path_ele.pose.orientation.y = 0.0;
    path_ele.pose.orientation.z = 0.0;

    path_con.push_back(path_ele);

    path_ele.pose.position.x = 3;
    path_ele.pose.position.y = 3;
    path_ele.pose.position.z = 0;

    path_ele.pose.orientation.w = 0.5;
    path_ele.pose.orientation.x = 0.0;
    path_ele.pose.orientation.y = 0.0;
    path_ele.pose.orientation.z = 0.0;

    path_con.push_back(path_ele);

    path.poses.resize(path_con.size());

    for(unsigned int i=0; i < path_con.size(); i++){
      path.poses[i] = path_con[i];
    }
    //file open
    std::ifstream yaml;
    std::ifstream pgm;

    std::string addr =  "/home/csw/catkin_ws/src/map/test";

    yaml.open(addr + ".yaml");
    pgm.open(addr + ".pgm");
    std::string str1,str2;



    std_msgs::String t;

    //image type
    getline(pgm,str1);
    t.data = str1;
    ROS_INFO("%s",t.data.c_str());

    //image width height
    getline(pgm,str1);
    unsigned long tmp = 0;
    for(unsigned long i = 0; i < str1.size() ; i++) {
      if(str1[i] == ' '){
        tmp = i;
        break;
      }
    }
    std::string width_str, height_str;
    for(unsigned long i = 0; i < tmp ; i++) {
      width_str = width_str + str1[i];
    }
    for(unsigned long i = tmp + 1; i < str1.size(); i++) {
      height_str = height_str + str1[i];
    }
    int width, height, max;

    t.data = width_str;
    ROS_INFO("%s",t.data.c_str());
    t.data = height_str;
    ROS_INFO("%s",t.data.c_str());

    width = std::stoi(width_str);
    height = std::stoi(height_str);


    //image max value
    getline(pgm,str1);
    t.data = str1;
    max = std::stoi(str1);
    ROS_INFO("%s",t.data.c_str());

    //map data set
    nav_msgs::OccupancyGrid map;

    map.header.seq = 1;
    map.header.stamp.sec = 0;
    map.header.stamp.nsec = 0;
    map.header.frame_id = "map";

    map.info.width = (unsigned int)width;
    map.info.height = (unsigned int)height;
    map.info.resolution = (float)0.050;
    map.info.origin.position.x = 0;
    map.info.origin.position.y = 0;
    map.info.origin.position.z = 0;

    int num = width * height;
    int8_t *map_data;

    map_data = new int8_t[(unsigned long)num];
    map.data.resize((unsigned int)num);
    for(unsigned int i = 0; i < (unsigned int)num; i++){
      pgm >> map_data[i];
      map.data[i] = map_data[i];
    }

    map_pub.publish(map);
    path_pub.publish(path);


    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
