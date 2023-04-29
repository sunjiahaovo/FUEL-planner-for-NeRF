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

#include <thread>
#include "nav_msgs/Odometry.h"

using namespace std;
typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, geometry_msgs::PoseStamped>  mySyncPolicy;
int N = 0;


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
      void cd_callback(const nav_msgs::OdometryConstPtr vins, const geometry_msgs::PoseStampedConstPtr optitrack);

    
    private:
      message_filters::Subscriber<nav_msgs::Odometry>* vins_sub;             // topic1 input
      message_filters::Subscriber<geometry_msgs::PoseStamped>* optitrack_sub;           // topic2 input

      message_filters::Synchronizer<mySyncPolicy>* sync; 
      ros::NodeHandle nh; 
      ofstream f_vins_out,f_optitrack_out;
};

talker::talker(){
};

void talker::registerNodeHandle(ros::NodeHandle& _nh){
   nh = _nh;
}
void talker::registerPubSub(){
  // image_transport::ImageTransport it(nh);  

  f_vins_out.open("vins_pose.txt",ios::app);  
  f_optitrack_out.open("optitrack_pose.txt",ios::app);        
  vins_sub  = new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/vins_estimator/odometry", 1000);
  optitrack_sub  = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh, "/vrpn_client_node/jiahao_sun/pose", 1000);

  sync = new  message_filters::Synchronizer<mySyncPolicy>(mySyncPolicy(1000), *vins_sub, *optitrack_sub);
  sync->registerCallback(boost::bind(&talker::cd_callback,this, _1, _2));
}

void talker::cd_callback(const nav_msgs::OdometryConstPtr vins, const geometry_msgs::PoseStampedConstPtr optitrack) 
{
   N++;
   if (N%20==0)
      cout << "come in " << N << endl;

  // double px = vins->pose.pose.position.x;
  // double py = vins->pose.pose.position.y;
  // double pz = vins->pose.pose.position.z;
  // double x = vins->pose.pose.orientation.x;
  // double y = vins->pose.pose.orientation.y;
  // double z = vins->pose.pose.orientation.z;
  // double w = vins->pose.pose.orientation.w;

  // // cout << "x,y,z,w = " << x << ", "<< y<< "," << z << "," << w << endl;
  // Eigen::Matrix3d R = Quaternion2RotationMatrix(x,y,z,w);
  // // cout << "R: \n" << R << endl;
  // Eigen::Matrix4d T ;
  // T.setZero();
  // T.block(0,0,3,3) = R;
  // T(0,3) = px;
  // T(1,3) = py;
  // T(2,3) = pz;
  // T(3,3) = 1;
  // cout << "vins_T: \n" << T << endl;


    
  // for (int i =0; i< 4;i++){
  //     for (int j = 0; j < 4; j++){
  //       if (j== 3)
  //           f_vins_out << T(i,j);
  //       else
  //           f_vins_out << T(i,j) << " ";
  //     }
  //     f_vins_out << endl; 
  // }


  // px = optitrack->pose.position.x;
  // py = optitrack->pose.position.y;
  // pz = optitrack->pose.position.z;
  // x = optitrack->pose.orientation.x;
  // y = optitrack->pose.orientation.y;
  // z = optitrack->pose.orientation.z;
  // w = optitrack->pose.orientation.w;

  // // cout << "x,y,z,w = " << x << ", "<< y<< "," << z << "," << w << endl;
  // R = Quaternion2RotationMatrix(x,y,z,w);
  // // cout << "R: \n" << R << endl;
  // T.setZero();
  // T.block(0,0,3,3) = R;
  // T(0,3) = px;
  // T(1,3) = py;
  // T(2,3) = pz;
  // T(3,3) = 1;
  // cout << "optitrack_T: \n" << T << endl;

  
    
  // for (int i =0; i< 4;i++){
  //     for (int j = 0; j < 4; j++){
  //       if (j== 3)
  //           f_optitrack_out << T(i,j);
  //       else
  //           f_optitrack_out << T(i,j) << " ";
  //     }
  //     f_optitrack_out << endl; 
  // }


  
}


int main(int argc, char** argv){

  ros::init(argc, argv, "vins_publisher");
  ros::NodeHandle nh;

  talker mytalker;
  mytalker.registerNodeHandle(nh);
  mytalker.registerPubSub();
  cout << "init" << endl;
  ros::spin();

}