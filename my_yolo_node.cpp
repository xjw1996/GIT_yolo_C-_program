#include <my_yolo/my_yolo.hpp>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_yolo");

  my_yolo::MyYolo my_yolo_;

  for(auto i = 1; i != 11; i++){
    my_yolo_.oneshot();
    ROS_INFO(" [my_yolo] counter = %d", i);
  }

  return 0;
}
