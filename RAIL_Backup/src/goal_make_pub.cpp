#include "goal_function.cpp"

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

  ros::Rate loop_rate(0.3);

  geometry_msgs::PoseStamped goal[4];
  goal::goal_make(goal);
  int count = 1;

  goal_pub.publish(goal[0]);
  int check = 0;
  while (ros::ok())
  {

    if(msg_.data == "reach"){
      ROS_INFO("reach!");
      check = (check + 1) %2;
      if(check == 1){
        count = (count + 1) % 2;
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
