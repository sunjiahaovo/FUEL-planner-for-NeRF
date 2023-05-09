#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <ros/ros.h>
#include <cmath>
#include <angles/angles.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <fstream>
#include <sstream>

#include<tf2_ros/static_transform_broadcaster.h>
#include<geometry_msgs/TransformStamped.h>
#include<tf2/LinearMath/Quaternion.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.h>

class Controller{
private:
    std::vector<std::vector<double> >  waypoints;
    int idx;
    bool isarrived;
public:
    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped local_pos;
    nav_msgs::Odometry local_odom;
    mavros_msgs::PositionTarget target_pos;
    ros::Publisher target_pub;
    ros::Publisher vision_pub;
    ros::Publisher odometry_pub;
    tf2_ros::StaticTransformBroadcaster broadcaster;
    Eigen::Vector3d pos_drone_t265;
    Eigen::Quaterniond q_t265;
    Eigen::Vector3d pos_drone_vins;
    Eigen::Quaterniond q_vins;    

public:
    Controller();
    void setWaypoints();    
    geometry_msgs::PoseStamped getPose();
    nav_msgs::Odometry getOdom();
    mavros_msgs::PositionTarget getWaypoint();
    void pos_callback(nav_msgs::OdometryConstPtr msg);
    void odom_callback(nav_msgs::OdometryConstPtr msg);
    void state_callback(mavros_msgs::StateConstPtr state);
};
inline void Quaternion2Euler(geometry_msgs::PoseStamped& pose_msg){
    tf::Quaternion quat;
    tf::quaternionMsgToTF(pose_msg.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    ROS_INFO_STREAM("RPY is " << roll*180/M_PI << " " << pitch*180/M_PI << " " << yaw*180/M_PI);
}
#endif