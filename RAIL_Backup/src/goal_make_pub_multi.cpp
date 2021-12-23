#include "goal_function.cpp"
#include <random>

std_msgs::String msg_;

void reach_sub(std_msgs::String msg){
  msg_ = msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "goal_make_pub");
  ros::NodeHandle nh;

  std_msgs::String str;


  ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1000);
  ros::Subscriber reached = nh.subscribe("reach",10,reach_sub);

  ros::Rate loop_rate(1);

  geometry_msgs::PoseStamped *goal;

  std::ifstream f;
  std::string group = ros::this_node::getNamespace();

  if(group == "/"){
    ROS_WARN("There is No group!! This node only activate in groups");
  }
  else{
    f.open("/home/csw/catkin_ws/src/multi_robot_ros/src/goal_multi" + group);
  }

  std::string str_tmp;

  getline(f,str_tmp);
  int goal_num = std::stoi(str_tmp);
  goal = new geometry_msgs::PoseStamped[goal_num];





  double x,y;
  for(int i = 0; i < goal_num; i++){
    getline(f,str_tmp);
    x = std::stod(str_tmp);
    getline(f,str_tmp);
    y = std::stod(str_tmp);
    getline(f,str_tmp); //junk
    goal[i].header.frame_id = "map";
    goal[i].pose.position.x = x;
    goal[i].pose.position.y = y;
    goal[i].pose.position.z = 0;
    goal[i].pose.orientation.w = (goal::random_make(-10000,10000) / 10000.0);
    goal[i].pose.orientation.x = 0;
    goal[i].pose.orientation.y = 0;
    goal[i].pose.orientation.z = (goal::random_make(-10000,10000) / 10000.0);
    ROS_INFO("%lf", x);
  }

  int count = 1;

  goal_pub.publish(goal[0]);
  int check = 0;
  while (ros::ok())
  {
    if(msg_.data == "reach"){
      ROS_INFO("reach!");
      check = (check + 1) %2;
      if(check == 1){
        count = (count + 1) % goal_num;
      }
      msg_.data = "not";
    }

    goal_pub.publish(goal[count]);
    ROS_INFO("pub goal %d",count);
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
