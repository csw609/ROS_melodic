#include "goal_make_pub/goal_function.h"

namespace goal {
  void goal_make(geometry_msgs::PoseStamped *goal){
    //----------------------
    goal[0].header.frame_id = "map";
    goal[0].pose.position.x = 0.205600261688;
    goal[0].pose.position.y = 0.134298324585;
    goal[0].pose.position.z = 0;
    goal[0].pose.orientation.w = 1.0;
    goal[0].pose.orientation.x = 0;
    goal[0].pose.orientation.y = 0;
    goal[0].pose.orientation.z = 0;

    goal[1].header.frame_id = "map";
    goal[1].pose.position.x = 0.461568832397;
    goal[1].pose.position.y = 1.46658849716;
    goal[1].pose.position.z = 0;
    goal[1].pose.orientation.w = 1.0;
    goal[1].pose.orientation.x = 0;
    goal[1].pose.orientation.y = 0;
    goal[1].pose.orientation.z = 0;

    goal[2].header.frame_id = "map";
    goal[2].pose.position.x = 0.713989555836;
    goal[2].pose.position.y = 0.426037818193;
    goal[2].pose.position.z = 0;
    goal[2].pose.orientation.w = 1.0;
    goal[2].pose.orientation.x = 0;
    goal[2].pose.orientation.y = 0;
    goal[2].pose.orientation.z = 0;

    goal[3].header.frame_id = "map";
    goal[3].pose.position.x = 0.910477876663;
    goal[3].pose.position.y = 1.41189575195;
    goal[3].pose.position.z = 0;
    goal[3].pose.orientation.w = 1.0;
    goal[3].pose.orientation.x = 0;
    goal[3].pose.orientation.y = 0;
    goal[3].pose.orientation.z = 0;

    //==================================
  }

  int random_make(int min, int max){
    // 시드값을 얻기 위한 random_device 생성.
    std::random_device rd;
    // random_device 를 통해 난수 생성 엔진을 초기화 한다.
    std::mt19937 gen(rd());
    //균등하게 나타나는 난수열을 생성하기 위해 균등 분포 정의.
    std::uniform_int_distribution<int> dis(min, max);


    return dis(gen);
  }

};
