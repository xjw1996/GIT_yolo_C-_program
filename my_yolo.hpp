// メンバ変数には_を最後につけること

#ifndef MY_YOLO
#define MY_YOLO

// ros
#include <ros/ros.h>
// ros-msgs
#include <sensor_msgs/Image.h>
// boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>
// action
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/goal_id_generator.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>

namespace my_yolo{
  

  typedef actionlib::SimpleActionClient<darknet_ros_msgs::CheckForObjectsAction> CheckForObjects;


  class MyYolo{
  public:
    MyYolo(void);

    const darknet_ros_msgs::BoundingBoxes& oneshot(void);

    const std::vector<darknet_ros_msgs::BoundingBoxes>& anyshot(unsigned int count);

    unsigned int img_height(void);

    unsigned int img_width(void);

    virtual ~MyYolo(void);

  private:
    void erase(unsigned int size);
    void detect_processing(void);
    darknet_ros_msgs::CheckForObjectsResultConstPtr get_detect(void);
    void CallBack(const sensor_msgs::Image::ConstPtr& msg);

    ros::Subscriber sub_;

    CheckForObjects cfo_;

    ros::AsyncSpinner spinner;
    darknet_ros_msgs::CheckForObjectsGoal cfo_goal_;

    boost::mutex mutex_;
    bool cb_flag_;

    boost::mutex img_mutex_;
    bool img_flag_;
    sensor_msgs::Image image_data_;
    boost::condition_variable im_condition_;

    std::vector<darknet_ros_msgs::BoundingBoxes> result_boxes_;
    darknet_ros_msgs::BoundingBoxes result_box_;

    boost::mutex info_mutex_;
    unsigned int height, width;
  };

};

#endif
