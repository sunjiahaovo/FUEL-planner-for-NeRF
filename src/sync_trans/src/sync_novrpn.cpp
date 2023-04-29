#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/UInt16.h>

#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <typeinfo>
#include <fstream>
#include <string>
#include <vector>
#include <Python.h>

#include <thread>
#include <httplib.h>

using namespace std;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>  mySyncPolicy;

string basedir = "data/test_v2/";
int frame_n = 0;
bool save_image = false;
PyObject* pFunc;

struct pic_trans{
  httplib::Client* cli;
  httplib::MultipartFormDataItems items;
  pic_trans(string host, int port){
    cli = new httplib::Client(host, port);
  }
  ~pic_trans(){
    delete cli;
  }
};

pic_trans http_trans("192.168.50.189", 7300);
httplib::MultipartFormData rgb_item;
httplib::MultipartFormData depth_item;
httplib::MultipartFormData pose_item;


void send_picture(){
  ros::Time ts = ros::Time::now();

  if(auto res = http_trans.cli->Post("/uploadpicture/", http_trans.items)){
    if(res->status == 200){
        cout << res->body << endl;
        cout << "transfer successfully" << endl;
    }
    else{
        cout << "connect failed" << endl;
    }
  }
  double tran_time = (ros::Time::now() - ts).toSec();
  ROS_WARN("trans time: %lf", tran_time);
}

void show_rgb(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("rgb_view", cv_bridge::toCvShare(msg, "bgr8")->image);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void show_depth(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::Mat depth_image = cv_bridge::toCvShare(msg, "16UC1")->image;
    cv::imshow("depth_view", depth_image);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

Eigen::Matrix3d Quaternion2RotationMatrix(const double x, const double y, const double z,const double w){

  Eigen::Quaterniond q;
  q.x() = x;
  q.y() = y;
  q.z() = z;
  q.w() = w;
  Eigen::Matrix3d R = q.normalized().toRotationMatrix();
  return R;

}

class talker{
    public:
      talker();
      void registerNodeHandle(ros::NodeHandle& _nh);
      void registerPubSub();
      void cd_callback(const sensor_msgs::ImageConstPtr rgb, const sensor_msgs::ImageConstPtr depth);
    
    private:
      message_filters::Subscriber<sensor_msgs::Image>* rgb_sub;             // topic1 input
      message_filters::Subscriber<sensor_msgs::Image>* depth_sub;           // topic2 input
      message_filters::Synchronizer<mySyncPolicy>* sync; 
      ros::NodeHandle nh; 
};

talker::talker(){
};

void talker::registerNodeHandle(ros::NodeHandle& _nh){
   nh = _nh;
}
void talker::registerPubSub(){
  image_transport::ImageTransport it(nh);           
  rgb_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh, "/camera/color/image_raw", 10);
  depth_sub  = new message_filters::Subscriber<sensor_msgs::Image>(nh, "/camera/aligned_depth_to_color/image_raw", 10);
  sync = new  message_filters::Synchronizer<mySyncPolicy>(mySyncPolicy(10), *rgb_sub, *depth_sub);
  sync->registerCallback(boost::bind(&talker::cd_callback,this, _1, _2));
}

void talker::cd_callback(const sensor_msgs::ImageConstPtr rgb, const sensor_msgs::ImageConstPtr depth) 
{
  // show_rgb(rgb);
  // show_depth(depth);
  // cout << "rgb stamp = " << rgb -> header.stamp <<endl;
  // cout << "depth stamp = " << depth -> header.stamp <<endl;

  if (save_image){
    save_image = false;
    ros::Time t0 = ros::Time::now();
    cout << "save success" << endl;
    cv::imwrite(basedir + to_string(frame_n) + "_main.png",cv_bridge::toCvShare(rgb, "bgr8")->image);
    cv::imwrite(basedir + to_string(frame_n) + "_depth.png",cv_bridge::toCvShare(depth, "16UC1")->image);
    

    ros::Time t1 = ros::Time::now();
    double save_time = (ros::Time::now() - t0).toSec();
    ROS_WARN("save time: %lf", save_time);
  
    rgb_item.filename = basedir + to_string(frame_n) + "_main.png";
    rgb_item.name = "rgb";
    ifstream in1(rgb_item.filename, ios::binary);
    if(in1){
      ostringstream contents;
      contents << in1.rdbuf();
      in1.close();
      rgb_item.content = contents.str();
    }
    rgb_item.content_type = "image/png";
    http_trans.items.push_back(rgb_item);

    depth_item.filename = basedir + to_string(frame_n) + "_depth.png";
    depth_item.name = "depth";
    ifstream in2(depth_item.filename, ios::binary);
    if(in2){
      ostringstream contents;
      contents << in2.rdbuf();
      in2.close();
      depth_item.content = contents.str();
    }
    depth_item.content_type = "image/png";
    http_trans.items.push_back(depth_item);

    pose_item.filename = "/home/nvidia/nerf-demo/src/sync_trans/calib_data/calib_v1/0_pose.txt";
    pose_item.name = "pose";
    ifstream in3(pose_item.filename, ios::binary);
    if(in3){
      ostringstream contents;
      contents << in3.rdbuf();
      in3.close();
      pose_item.content = contents.str();
    }
    pose_item.content_type = "text/plain";
    http_trans.items.push_back(pose_item);

    ros::Time t2 = ros::Time::now();
    double read_time = (ros::Time::now() - t1).toSec();
    ROS_WARN("read time: %lf", read_time);
    

    // if(auto res = http_trans.cli->Post("/uploadpicture/", http_trans.items)){
    //     if(res->status == 200){
    //         cout << res->body << endl;
    //         cout << "transfer successfully" << endl;
    //     }
    //     else{
    //         cout << "connect failed" << endl;
    //     }
    // }
    
    // double trans_time = (ros::Time::now() - t2).toSec();
    // ROS_WARN("trans time: %lf", trans_time);


    // std::thread send_picture_thread(send_picture);
    // send_picture_thread.detach();


    ros::Time ts = ros::Time::now();
    PyObject_CallObject(pFunc, NULL);
    double trans_time = (ros::Time::now() - ts).toSec();
    ROS_WARN("trans time: %lf", trans_time);

  
    frame_n ++;
  
  }


  
}


void key_callback(std_msgs::UInt16ConstPtr msg){
  if(msg->data == 1){
    save_image = true;
  }
  else{
    save_image = false;
  }
}




int main(int argc, char** argv){

  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;

  // 1、初始化python接口  
	Py_Initialize();
	if(!Py_IsInitialized()){
		cout << "python init fail" << endl;
		return 0;
	}
  // 2、初始化python系统文件路径，保证可以访问到 .py文件
	PyRun_SimpleString("import sys");
  PyRun_SimpleString("import os");
	PyRun_SimpleString("sys.path.append('/home/nvidia/nerf-demo/')");
  PyRun_SimpleString("print(sys.path)");


  // 3、调用python文件名，不用写后缀
	PyObject* pModule = PyImport_ImportModule("tran_data");
	if( pModule == NULL ){
		cout <<"module not found" << endl;
		return 1;
	}

  cout <<"file found" << endl;
  // 4、调用函数
	pFunc = PyObject_GetAttrString(pModule, "upload_picture");
	if( !pFunc || !PyCallable_Check(pFunc)){
		cout <<"not found function add_num" << endl;
		return 0;
	}




  // cv::namedWindow("rgb_view");
  // cv::namedWindow("depth_view");
  // cv::startWindowThread();
  talker mytalker;
  mytalker.registerNodeHandle(nh);
  mytalker.registerPubSub();
  ros::Subscriber kb_sub = nh.subscribe("/key_command", 10, key_callback);
  ros::spin();
  // cv::destroyWindow("rgb_view");
  // cv::destroyWindow("depth_view");
}