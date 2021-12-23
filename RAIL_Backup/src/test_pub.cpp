#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "visualization_msgs/MarkerArray.h"
#include <queue>
#include <algorithm>
#include <fstream>
#include <random>

int count = 0;
nav_msgs::OccupancyGrid map;
nav_msgs::OccupancyGrid map2;
geometry_msgs::PointStamped p1;
geometry_msgs::PointStamped p2;
geometry_msgs::PointStamped p3;
geometry_msgs::PointStamped p4;

const int paddingSize = 11;

bool mapReceive = false;

struct compare{
    bool operator()(std::pair<int, std::pair<int, int>> a, std::pair<int, std::pair<int, int>> b){
        return a.first > b.first;
    }
};

bool cost_compare(int tx, int ty, double *cost, double *cost_e,const int size, bool* track_e){
  int ix, iy;
  int half = (int)(size / 2);

//boundary check
  for(int j = -half; j <= half; j++){
    ix = tx + j;
    iy = ty + half;
    if(ix > 0 && ix < map.info.width && iy > 0 && iy < map.info.height){//range consider
      if(track_e[iy*map.info.width + ix] && std::abs(cost[ty*map.info.width + tx] - cost_e[iy*map.info.width + ix]) < (size / 2 * 1.414) ){//cost difference check
        return true; //if cost difference is small && is track
        // ROS_INFO("cost : %lf  cost_e ; %lf", cost[ty*map.info.width + tx],cost_e[iy*map.info.width + ix] );
      }
    }
    }
  for(int j = -half; j <= half; j++){
    ix = tx + j;
    iy = ty - half;
    if(ix > 0 && ix < map.info.width && iy > 0 && iy < map.info.height){//range consider
      if(track_e[iy*map.info.width + ix] && std::abs(cost[ty*map.info.width + tx] - cost_e[iy*map.info.width + ix]) < (size / 2 * 1.414) ){//cost difference check
        return true; //if cost difference is small && is track
        // ROS_INFO("cost : %lf  cost_e ; %lf", cost[ty*map.info.width + tx],cost_e[iy*map.info.width + ix] );
      }
    }
  }
  for(int j = -half + 1; j < half; j++){
    ix = tx - half;
    iy = ty + j;
    if(ix > 0 && ix < map.info.width && iy > 0 && iy < map.info.height){//range consider
      if(track_e[iy*map.info.width + ix] && std::abs(cost[ty*map.info.width + tx] - cost_e[iy*map.info.width + ix]) < (size / 2 * 1.414) ){//cost difference check
        return true; //if cost difference is small && is track
        // ROS_INFO("cost : %lf  cost_e ; %lf", cost[ty*map.info.width + tx],cost_e[iy*map.info.width + ix] );
      }
    }
  }
  for(int j = -half + 1; j < half; j++){
    ix = tx + half;
    iy = ty + j;
    if(ix > 0 && ix < map.info.width && iy > 0 && iy < map.info.height){//range consider
      if(track_e[iy*map.info.width + ix] && std::abs(cost[ty*map.info.width + tx] - cost_e[iy*map.info.width + ix]) < (size / 2 * 1.414) ){//cost difference check
        return true; //if cost difference is small && is track
        // ROS_INFO("cost : %lf  cost_e ; %lf", cost[ty*map.info.width + tx],cost_e[iy*map.info.width + ix] );
      }
    }
  }

  return false;
}

void trackPadding(bool *track,int x, int y,const int size, unsigned int width, unsigned height){
  int ix, iy;
  int half = (int)(size / 2);
  //track padding
  for(int i = -half; i <= half; i++){
    for(int j = -half; j <= half; j++){
      ix = x + j;
      iy = y + i;
      if(ix > 0 && ix < (int)width && iy > 0 && iy < (int)height){
        track[iy*width + ix] = true;
      }
    }
  }
}

void point_sub(const geometry_msgs::PointStamped p)
{
  ROS_INFO("%f", p.point.x);
  switch(count){
  case 0:
    p1 = p;
    break;
  case 1:
    p2 = p;
    break;
  case 2:
    p3 = p;
    break;
  case 3:
    p4 = p;
    break;

  }
  count++;
  ROS_INFO("count : %d", count);
}

void map_call(nav_msgs::OccupancyGrid m){
  map = m;
  map2 = m;

  ROS_INFO("map catch!");
  mapReceive = true;
}

//delete this later
//unsigned int scell, ecell;
//

bool path_planning(std::vector<geometry_msgs::PoseStamped> &path_con,const nav_msgs::OccupancyGrid &map,const geometry_msgs::PointStamped &s,const geometry_msgs::PointStamped &e
                   ,double *cost, double *cost_e, bool *track, bool *track_e){
  //map information
  unsigned int width = map.info.width;
  unsigned int height = map.info.height;
  float unit = 1.0;
  //coordinate calculate
  float ratio = unit / map.info.resolution;

  //start point , end point
  unsigned int sx, sy, ex, ey;
  sx = (unsigned int)((s.point.x - map.info.origin.position.x) * ratio);
  sy = (unsigned int)((s.point.y - map.info.origin.position.y) * ratio);
  ex = (unsigned int)((e.point.x - map.info.origin.position.x) * ratio);
  ey = (unsigned int)((e.point.y - map.info.origin.position.y) * ratio);

  //A*

  //variable init
  bool **visited;
  std::pair<int,int> **ways;
  visited = new bool*[width];

  ways = new std::pair<int,int>*[width];
  for(unsigned int i = 0; i < width; i++){
    visited[i] = new  bool[height];
    ways[i] = new std::pair<int,int>[height];
    for(unsigned int j = 0; j < height; j++){
      visited[i][j] = false;
      cost[i*height + j] = 987654321.0;//cost init
      track[i*height + j] = false;
      ways[i][j] = {987654321,987654321};

    }
  }

  //123
  //456
  //789
  int array_x[8] = {-1, 1, -1,  1, 0, -1, 1,  0};//1 3 7 9 2 8 6 4 <= direction order
  int array_y[8] = { 1, 1, -1, -1, 1,  0, 0, -1};

  //search begin
  std::priority_queue<std::pair<int, std::pair<int, int>>,std::vector<std::pair<int, std::pair<int, int>>>, compare> q;

  q.push({0,{sx,sy}}); //start point insert
  cost[sy*width + sx] = 0;    //start point cost zero

  while(true){
    if(q.empty()){
      return false;
    }

    int tx = q.top().second.first;
    int ty = q.top().second.second;

    q.pop();
    //ROS_INFO("%lf", cost[ty*width + tx]);
    if(tx == (int)ex && ty == (int)ey){//if reach end point
      //make path
      break;
    }


    //close point search need!!!!!!!!!!!!!!!!!!!!1
    //if(track_e[ty*width + tx] && cost_compare(tx,ty,cost,cost_e,paddingSize)){
    if(cost_compare(tx,ty,cost,cost_e,paddingSize,track_e)){
      //ROS_INFO("cost1 : %lf", cost[ty*width + tx]);
      //ROS_INFO("cost2 : %lf", cost_e[ty*width + tx]);
      continue; //similar time similar place
    }

    int ix, iy;
    double icost;
    //ROS_INFO("path finding");
    //8 neighbor search
    for(int i = 0; i < 8; i++){
      ix = tx + array_x[i];
      iy = ty + array_y[i];
      //map contraint add!!!!


      if(ix > 0 && ix < (int)width && iy > 0 && iy < (int)height){//map range consider
        //map constraint consider, visited check
        if(map.data[iy*(int)width + ix] == 100 || map.data[iy*(int)width + ix] == -1 || visited[ix][iy] ) continue;



        //if(track_e[iy*width + ix]) continue;
        //cost calculate => distance from end point + cost from start point
        icost = ((double)ex - (double)ix) * ((double)ex - (double)ix) +  ((double)ey - (double)iy) * ((double)ey - (double)iy) + cost[ty*width + tx];

        q.push({icost,{ix,iy}});
        visited[ix][iy] = true;

        //tracking
        ways[ix][iy] = {tx,ty};

        //cost
        if(i < 4){//when move diagonal
          cost[iy*width + ix] = cost[ty*width + tx] + 1.4142135; //2 square root
        }
        else{//when move Up Down Left Right
          cost[iy*width + ix] = cost[ty*width + tx] + 1;
        }
        //ROS_INFO("ix : %d iy : %d", ix, iy);
      }
    }


  }


  //scell = sy*width + sx;
  //ecell = ey*width + ex;

  //path make;
  //ROS_INFO("%d , %d",ex, ey);
  //ROS_INFO("path tracking");

  int tmp1, tmp2, tmp3, tmp4;
  float fx, fy;

  tmp1 = (int)ex; tmp2 = (int)ey;
  tmp3 = (int)ex; tmp4 = (int)ey;

  //path save
  geometry_msgs::PoseStamped pose_tmp;
  while(true){
    //ROS_INFO("%d , %d",tmp1, tmp2 );
    fx = (float)tmp1 / ratio + map.info.origin.position.x;
    fy = (float)tmp2 / ratio + map.info.origin.position.y;
    //ROS_INFO("%f , %f",fx, fy );

    trackPadding(track,tmp1,tmp2,paddingSize,width,height);

    pose_tmp.pose.position.x = fx;
    pose_tmp.pose.position.y = fy;
    pose_tmp.pose.position.z = 0;
    path_con.push_back(pose_tmp);

    if(tmp1 == (int)sx && tmp2 == (int)sy) break;

    tmp1 = ways[tmp3][tmp4].first;
    tmp2 = ways[tmp3][tmp4].second;
    tmp3 = tmp1;
    tmp4 = tmp2;
  }

  ROS_INFO("cost : %lf", cost[ey*width + ex]);
  reverse(path_con.begin(), path_con.end()); //end to start => start to end

  return true;
  delete[] visited;
  //delete[] cost;
  delete[] ways;

}

int main(int argc, char **argv)
{
  //init
  ros::init(argc, argv, "path_pub");
  ros::NodeHandle nh;
  ros::Subscriber map_sub = nh.subscribe("map", 1000, map_call);
  ros::Subscriber sub = nh.subscribe("clicked_point", 1000, point_sub);
  ros::Publisher path1_pub = nh.advertise<nav_msgs::Path>("path1",1000);
  ros::Publisher path2_pub = nh.advertise<nav_msgs::Path>("path2",1000);
  ros::Publisher path3_pub = nh.advertise<nav_msgs::Path>("path3",1000);
  ros::Publisher path4_pub = nh.advertise<nav_msgs::Path>("path4",1000);
  ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("test_map",1000);
  ros::Publisher mark1_pub = nh.advertise<visualization_msgs::MarkerArray>("marker1",1000);
  ros::Publisher mark2_pub = nh.advertise<visualization_msgs::MarkerArray>("marker2",1000);
  ros::Rate loop_rate(1000);

   std::fstream f;
   f.open("/home/csw/map/result.txt");

  //path header set
  nav_msgs::Path path1;
  nav_msgs::Path path2;
  visualization_msgs::Marker marker1_s;
  visualization_msgs::Marker marker1_e;
  visualization_msgs::Marker marker2_s;
  visualization_msgs::Marker marker2_e;
  visualization_msgs::MarkerArray mark_a1;
  visualization_msgs::MarkerArray mark_a2;



  marker1_s.header.frame_id = "map";
  marker1_s.header.seq = 1;
  marker1_s.header.stamp.sec = 0;
  marker1_s.header.stamp.nsec = 0;
  marker1_s.type = 1; //Cube
  marker1_s.scale.x = 0.25;
  marker1_s.scale.y = 0.25;
  marker1_s.scale.z = 0.01;
  marker1_s.color.r = 1.0;
  marker1_s.color.g = 0.0;
  marker1_s.color.b = 0.0;
  marker1_s.color.a = 0.5;
  marker1_s.id = 0;

  marker1_e.header.frame_id = "map";
  marker1_e.header.seq = 1;
  marker1_e.header.stamp.sec = 0;
  marker1_e.header.stamp.nsec = 0;
  marker1_e.type = 1; //Cube
  marker1_e.scale.x = 0.25;
  marker1_e.scale.y = 0.25;
  marker1_e.scale.z = 0.01;
  marker1_e.color.r = 0.0;
  marker1_e.color.g = 0.0;
  marker1_e.color.b = 1.0;
  marker1_e.color.a = 0.5;
  marker1_e.id = 1;

  marker2_s.header.frame_id = "map";
  marker2_s.header.seq = 1;
  marker2_s.header.stamp.sec = 0;
  marker2_s.header.stamp.nsec = 0;
  marker2_s.type = 1; //Cube
  marker2_s.scale.x = 0.25;
  marker2_s.scale.y = 0.25;
  marker2_s.scale.z = 0.01;
  marker2_s.color.r = 1.0;
  marker2_s.color.g = 0.0;
  marker2_s.color.b = 0.0;
  marker2_s.color.a = 0.5;
  marker2_s.id = 3;

  marker2_e.header.frame_id = "map";
  marker2_e.header.seq = 1;
  marker2_e.header.stamp.sec = 0;
  marker2_e.header.stamp.nsec = 0;
  marker2_e.type = 1; //Cube
  marker2_e.scale.x = 0.25;
  marker2_e.scale.y = 0.25;
  marker2_e.scale.z = 0.01;
  marker2_e.color.r = 0.0;
  marker2_e.color.g = 0.0;
  marker2_e.color.b = 1.0;
  marker2_e.color.a = 0.5;
  marker2_e.id = 4;



  path1.header.frame_id = "map";
  path1.header.seq = 1;
  path1.header.stamp.sec = 0;
  path1.header.stamp.nsec = 0;

  path2.header.frame_id = "map";
  path2.header.seq = 1;
  path2.header.stamp.sec = 0;
  path2.header.stamp.nsec = 0;

  //map.data.resize(map.data.size());
  //unsigned int j = 0;
  bool path1_check = false; //why?




  while(!mapReceive){
    ROS_INFO("map wait");
    ros::spinOnce();
    loop_rate.sleep();
  }

  double *cost1, *cost2;
  bool *track1, *track2;

  unsigned int width = map.info.width;
  unsigned int height = map.info.height;

  unsigned int wh = width * height;

  ROS_INFO("width : %d ,height :  %d",width, height);

  cost1 = new double[wh];
  cost2 = new double[wh];
  track1 = new bool[wh];
  track2 = new bool[wh];

  for(unsigned int i = 0; i < width; i++){
    for(unsigned int j = 0; j < height; j++){
      cost1[i*height + j] = 987654321.0;
      cost2[i*height + j] = 987654321.0;
      track1[i*height + j] = false;
      track2[i*height + j] = false;
    }
  }
  std::vector<geometry_msgs::PoseStamped> path_con;
  //bool firstTime = true;
  while(ros::ok()){
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> wid1(5, width - 5);
    std::uniform_int_distribution<int> hei1(5, height - 5);


    int rX1 = wid1(gen);
    int rY1 = hei1(gen);
    int rX2 = wid1(gen);
    int rY2 = hei1(gen);
    int rX3 = wid1(gen);
    int rY3 = hei1(gen);
    int rX4 = wid1(gen);
    int rY4 = hei1(gen);

    ROS_INFO("x1:%d, y1:%d" ,rX1, rY1);
    ROS_INFO("x2:%d, y2:%d" ,rX2, rY2);
    ROS_INFO("x3:%d, y3:%d" ,rX3, rY3);
    ROS_INFO("x4:%d, y4:%d" ,rX4, rY4);
    float ratio = 1 / map.info.resolution;

    if(map.data[rY1*width + rX1] == 100 || map.data[rY1*width + rX1] == -1 || map.data[rY2*width + rX2] == 100 || map.data[rY2*width + rX2] == -1 ||
       map.data[rY3*width + rX3] == 100 || map.data[rY3*width + rX3] == -1 || map.data[rY4*width + rX4] == 100 || map.data[rY4*width + rX4] == -1){
      ROS_INFO("pass");
      continue;
    }
    else{
      p1.point.x = rX1 / ratio + map.info.origin.position.x; p1.point.y = rY1 / ratio + map.info.origin.position.y;
      p2.point.x = rX2/ ratio + map.info.origin.position.x; p2.point.y = rY2/ ratio + map.info.origin.position.y;
      p3.point.x = rX3/ ratio + map.info.origin.position.x; p3.point.y = rY3/ ratio + map.info.origin.position.y;
      p4.point.x = rX4/ ratio + map.info.origin.position.x; p4.point.y = rY4/ ratio + map.info.origin.position.y;
    }

    if( true || map.info.resolution == 0.05){ //path make;
      //distance
      double dist1, dist2;
      dist1 = sqrt((p1.point.x - p2.point.x) * (p1.point.x - p2.point.x) + (p1.point.y - p2.point.y) * (p1.point.y - p2.point.y));
      dist2 = sqrt((p3.point.x - p4.point.x) * (p3.point.x - p4.point.x) + (p3.point.y - p4.point.y) * (p3.point.y - p4.point.y));

      ROS_INFO("dist1 : %lf", dist1);
      ROS_INFO("dist2 : %lf", dist2);


      mark_a1.markers.resize(2);
      ROS_INFO("path1 first");
      //ROS_INFO("path1 make");
      path_con.clear();
      if(path_planning(path_con,map,p1, p2,cost1, cost2, track1, track2)){

        path1.poses.resize(path_con.size());
        for(unsigned int i=0; i < path_con.size(); i++){
              path1.poses[i] = path_con[i];
        }


        //ROS_INFO("path publish!");
        path1_check = true;

        marker1_s.pose.position.x = p1.point.x;
        marker1_s.pose.position.y = p1.point.y;
        marker1_s.pose.position.z = p1.point.z;

        marker1_s.pose.orientation.w = 1.0;
        marker1_s.pose.orientation.x = 0.0;
        marker1_s.pose.orientation.y = 0.0;
        marker1_s.pose.orientation.z = 0.0;

        marker1_e.pose.position.x = p2.point.x;
        marker1_e.pose.position.y = p2.point.y;
        marker1_e.pose.position.z = p2.point.z;

        marker1_e.pose.orientation.w = 1.0;
        marker1_e.pose.orientation.x = 0.0;
        marker1_e.pose.orientation.y = 0.0;
        marker1_e.pose.orientation.z = 0.0;

        mark_a1.markers[0] = marker1_s;
        mark_a1.markers[1] = marker1_e;

        mark1_pub.publish(mark_a1);
      }
      else{
        ROS_INFO("search fail");
        path1.poses.clear();
      }
      path1_pub.publish(path1);

      mark_a2.markers.resize(2);

      //ROS_INFO("path2 make");
      path_con.clear();

      if(path_planning(path_con,map,p3,p4,cost2, cost1,track2, track1)){

        path2.poses.resize(path_con.size());
        for(unsigned int i=0; i < path_con.size(); i++){
              path2.poses[i] = path_con[i];
        }

        //ROS_INFO("path publish!");
        count = 0;
        path1_check = false;

        marker2_s.pose.position.x = p3.point.x;
        marker2_s.pose.position.y = p3.point.y;
        marker2_s.pose.position.z = p4.point.z;

        marker2_s.pose.orientation.w = 1.0;
        marker2_s.pose.orientation.x = 0.0;
        marker2_s.pose.orientation.y = 0.0;
        marker2_s.pose.orientation.z = 0.0;

        marker2_e.pose.position.x = p4.point.x;
        marker2_e.pose.position.y = p4.point.y;
        marker2_e.pose.position.z = p4.point.z;

        marker2_e.pose.orientation.w = 1.0;
        marker2_e.pose.orientation.x = 0.0;
        marker2_e.pose.orientation.y = 0.0;
        marker2_e.pose.orientation.z = 0.0;

        mark_a2.markers[0] = marker2_s;
        mark_a2.markers[1] = marker2_e;

        mark2_pub.publish(mark_a2);

      }
      else{
        ROS_INFO("search fail");
        path2.poses.clear();
        //count = 2;
      }

      path2_pub.publish(path2);


      unsigned int ex1 = (unsigned int)((p2.point.x - map.info.origin.position.x) * ratio);
      unsigned int ey1 = (unsigned int)((p2.point.y - map.info.origin.position.y) * ratio);
      unsigned int ex2 = (unsigned int)((p4.point.x - map.info.origin.position.x) * ratio);
      unsigned int ey2 = (unsigned int)((p4.point.y - map.info.origin.position.y) * ratio);
      double cost_sum = cost1[ey1*width + ex1]+cost2[ey2*width + ex2] ;
      ROS_INFO("sum : %lf", cost_sum);

      //init cost, track
      for(unsigned int i = 0; i < width; i++){
        for(unsigned int j = 0; j < height; j++){
          cost1[i*height + j] = 987654321.0;
          cost2[i*height + j] = 987654321.0;
          track1[i*height + j] = false;
          track2[i*height + j] = false;
        }
      }
      ROS_INFO("path2 first");
      mark_a1.markers.resize(2);

      //ROS_INFO("path1 make");
      path_con.clear();
      if(path_planning(path_con,map,p3, p4,cost1, cost2, track1, track2)){

        path1.poses.resize(path_con.size());
        for(unsigned int i=0; i < path_con.size(); i++){
              path1.poses[i] = path_con[i];
        }


        //ROS_INFO("path publish!");
        path1_check = true;

        marker1_s.pose.position.x = p1.point.x;
        marker1_s.pose.position.y = p1.point.y;
        marker1_s.pose.position.z = p1.point.z;

        marker1_s.pose.orientation.w = 1.0;
        marker1_s.pose.orientation.x = 0.0;
        marker1_s.pose.orientation.y = 0.0;
        marker1_s.pose.orientation.z = 0.0;

        marker1_e.pose.position.x = p2.point.x;
        marker1_e.pose.position.y = p2.point.y;
        marker1_e.pose.position.z = p2.point.z;

        marker1_e.pose.orientation.w = 1.0;
        marker1_e.pose.orientation.x = 0.0;
        marker1_e.pose.orientation.y = 0.0;
        marker1_e.pose.orientation.z = 0.0;

        mark_a1.markers[0] = marker1_s;
        mark_a1.markers[1] = marker1_e;

        mark1_pub.publish(mark_a1);
      }
      else{
        ROS_INFO("search fail");
        path1.poses.clear();
      }
      path3_pub.publish(path1);

      mark_a2.markers.resize(2);

      //ROS_INFO("path2 make");
      path_con.clear();

      if(path_planning(path_con,map,p1,p2,cost2, cost1,track2, track1)){

        path2.poses.resize(path_con.size());
        for(unsigned int i=0; i < path_con.size(); i++){
              path2.poses[i] = path_con[i];
        }

        //ROS_INFO("path publish!");

        path1_check = false;

        marker2_s.pose.position.x = p3.point.x;
        marker2_s.pose.position.y = p3.point.y;
        marker2_s.pose.position.z = p3.point.z;

        marker2_s.pose.orientation.w = 1.0;
        marker2_s.pose.orientation.x = 0.0;
        marker2_s.pose.orientation.y = 0.0;
        marker2_s.pose.orientation.z = 0.0;

        marker2_e.pose.position.x = p4.point.x;
        marker2_e.pose.position.y = p4.point.y;
        marker2_e.pose.position.z = p4.point.z;

        marker2_e.pose.orientation.w = 1.0;
        marker2_e.pose.orientation.x = 0.0;
        marker2_e.pose.orientation.y = 0.0;
        marker2_e.pose.orientation.z = 0.0;

        mark_a2.markers[0] = marker2_s;
        mark_a2.markers[1] = marker2_e;

        mark2_pub.publish(mark_a2);

      }
      else{
        ROS_INFO("search fail");
        path2.poses.clear();
        //count = 2;
      }
      path4_pub.publish(path2);

      ex1 = (unsigned int)((p4.point.x - map.info.origin.position.x) * ratio);
      ey1 = (unsigned int)((p4.point.y - map.info.origin.position.y) * ratio);
      ex2 = (unsigned int)((p2.point.x - map.info.origin.position.x) * ratio);
      ey2 = (unsigned int)((p2.point.y - map.info.origin.position.y) * ratio);
      count = 0;
      double cost_sum2 = cost1[ey1*width + ex1] +cost2[ey2*width + ex2] ;
      ROS_INFO("sum : %lf", cost_sum);
      if(cost_sum < 50000 && cost_sum2 < 50000){
        f << dist1 << "," << dist2 << "," << cost_sum << "," << cost_sum2 << "\n";
      }
      else{
        ROS_INFO("Pass case!!");
      }
    }







    for(unsigned int i = 0; i < width; i++){
      for(unsigned int j = 0; j < height; j++){
        if(track1[i*height + j] || track2[i*height + j]){
          map2.data[i*height + j] = -1;
        }
        else{
          map2.data[i*height + j] = 0;
        }
      }
    }

    map_pub.publish(map2);

    ros::spinOnce();
    loop_rate.sleep();
  }


  f.close();

  delete [] cost1;
  delete [] cost2;
  delete [] track1;
  delete [] track2;

  return 0;
}
