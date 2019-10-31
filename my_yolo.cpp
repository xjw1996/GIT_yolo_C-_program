#include <my_yolo/my_yolo.hpp>

namespace my_yolo{
  MyYolo::MyYolo():
  cfo_("/darknet_ros/check_for_objects", true), spinner(1),
  cb_flag_(false), img_flag_(false)
  {
    while(!cfo_.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO(" [MyYolo] Waiting for the action server to come up");
    }
    ROS_INFO(" [MyYolo] Succeeded for the action server to come up");

    ros::NodeHandle private_nh("~");
    std::string sub_topic_name;
    private_nh.param("sub_topic_name", sub_topic_name, std::string("/image_raw"));
    sub_ = private_nh.subscribe(sub_topic_name, 1, &MyYolo::CallBack, this);
  }

  MyYolo::~MyYolo()
  {

    ros::waitForShutdown();
  }

  const darknet_ros_msgs::BoundingBoxes& MyYolo::oneshot(void)
  {
    erase(1);

    ROS_INFO(" [MyYolo][Func:oneshot] start");
    spinner.start();
    detect_processing();
    spinner.stop();
    cfo_.cancelAllGoals();
    ROS_INFO(" [MyYolo][Func:oneshot] stop");
    return result_boxes_[0];
  }


  const std::vector<darknet_ros_msgs::BoundingBoxes>& MyYolo::anyshot(unsigned int count)
  {
    ros::Rate loop(2);
    erase(count);
    ROS_INFO(" [MyYolo][Func:anyshot] start");
    spinner.start();
    for(unsigned int index = 0; index != count; index++){
      detect_processing();
      loop.sleep();
    }
    spinner.stop();
    cfo_.cancelAllGoals();
    ROS_INFO(" [MyYolo][Func:anyshot] stop");
    return result_boxes_;
  }

  void MyYolo::erase(unsigned int size)
  {
    if( size < result_boxes_.size()){
      result_boxes_.erase( result_boxes_.begin() + size, result_boxes_.end());
    }else{
      result_boxes_.resize(size);
    }
  }

  void MyYolo::detect_processing(void)
  {
    darknet_ros_msgs::CheckForObjectsResultConstPtr cfo_result_ptr = get_detect();
    ROS_INFO(" [MyYolo][Func:detect_processing] get data and set data");
    result_box_.header = cfo_result_ptr->bounding_boxes.header;
    result_box_.image_header = cfo_result_ptr->bounding_boxes.image_header;
    result_box_.bounding_boxes = cfo_result_ptr->bounding_boxes.bounding_boxes;
    result_boxes_.push_back(result_box_);
  }

  darknet_ros_msgs::CheckForObjectsResultConstPtr MyYolo::get_detect(void)
  {
    ros::NodeHandle nh;
    ros::Rate loop(20);

    boost::unique_lock<boost::mutex> cb_l(mutex_);
    cb_flag_ = true;
    cb_l.unlock();
    ROS_INFO(" [MyYolo][Func:get_detect] wait callback");

    boost::unique_lock<boost::mutex> lock(img_mutex_);
    while(!img_flag_)
      im_condition_.wait(lock);
    img_flag_ = false;
    cfo_goal_.image = image_data_;
    lock.unlock();

    {
      boost::mutex::scoped_lock s_l(info_mutex_);
      height = cfo_goal_.image.height;
      width = cfo_goal_.image.width;
    }
    ROS_INFO(" [MyYolo][Func:get_detect] encoding = %s", cfo_goal_.image.encoding.c_str());
    // loop.sleep();

    cfo_.sendGoal(cfo_goal_);
    bool result_flag_ = false;
    while( ros::ok() ){
      result_flag_ = cfo_.waitForResult(ros::Duration(1.0));
      if(result_flag_) break;
      ROS_INFO(" [MyYolo][Func:get_detect] wait action result");

    }
    ROS_INFO(" [MyYolo][Func:get_detect] get result");

    darknet_ros_msgs::CheckForObjectsResultConstPtr ptr = cfo_.getResult();
    cfo_.cancelAllGoals();

    return ptr;
  }

  void MyYolo::CallBack(const sensor_msgs::Image::ConstPtr& msg)
  {
    boost::unique_lock<boost::mutex> cb_l(mutex_);
    if( !cb_flag_ ){
      cb_l.unlock();
      return;
    }else{
      std::string encoding = msg->encoding;
      if( encoding == "rgb8" ){
        std::cout << "[MyYolo][Func:get_detect] ENCODING " << encoding << std::endl;
        cb_flag_ = false;
        cb_l.unlock();
        ROS_INFO(" [MyYolo][Func:get_detect] get data");
        boost::unique_lock< boost::mutex > lock(img_mutex_);
        img_flag_ = true;
        image_data_ = *msg;
        lock.unlock();
        im_condition_.notify_one();
      }else{
        cb_l.unlock();
        ROS_INFO(" [MyYolo][Func:get_detect] ENCODING is BAD '%s'", encoding.c_str());
        return;
      }
    }
  }

  unsigned int MyYolo::img_height(void)
  {
    boost::unique_lock<boost::mutex> lock(info_mutex_);
    return height;
  }

  unsigned int MyYolo::img_width(void)
  {
    boost::unique_lock<boost::mutex> lock(info_mutex_);
    return width;
  }
};
