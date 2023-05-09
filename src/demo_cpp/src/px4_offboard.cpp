#include<demo_cpp/controller.h>
#include<boost/bind.hpp>
#include<mavros_msgs/CommandBool.h>
#include<mavros_msgs/SetMode.h>
#include<vector>

bool trigger2offboard = false;
bool trigger2arm = false;

void retPose_callback(geometry_msgs::PoseStampedConstPtr msg){
    // ROS_INFO_STREAM("current pos is " << msg->pose.position.x << " " << msg->pose.position.y << " " << msg->pose.position.z);
    return;
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "demo_cpp_node");
    ros::NodeHandle nh;
    Controller ctl;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, boost::bind(&Controller::state_callback, &ctl, _1));
    // ros::Subscriber pose_sub = nh.subscribe<nav_msgs::Odometry>("/cam_2/odom/sample", 10, boost::bind(&Controller::pos_callback, &ctl, _1)); 
    ros::Subscriber pose_sub = nh.subscribe<nav_msgs::Odometry>("/vins_fusion/odometry", 10, boost::bind(&Controller::odom_callback, &ctl, _1));  
    ctl.odometry_pub = nh.advertise<nav_msgs::Odometry>("/mavros/odometry/out", 10);                   
    ctl.vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
    ros::Subscriber retPose = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, retPose_callback);
    ctl.target_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    ros::Rate rate(200);
    
    while(ros::ok() && !ctl.current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    
    for(int i = 100; ros::ok() && i > 0; --i){
        // ctl.vision_pub.publish(ctl.getPose());
        ctl.odometry_pub.publish(ctl.getOdom());
        ctl.target_pub.publish(ctl.getWaypoint());
        ros::spinOnce();
        rate.sleep();
    }


    while(ros::ok()){
        ROS_INFO_STREAM("current mode is : " << ctl.current_state.mode);
        if(ctl.current_state.mode == "OFFBOARD" && !trigger2offboard){
            ROS_INFO("Offboard enabled");
            trigger2offboard = true;
        }
        
        if(ctl.current_state.armed && !trigger2arm){
            ROS_INFO("Vehicle armed");
            trigger2arm = true;
        }
            
        // ctl.vision_pub.publish(ctl.getPose());
        ctl.odometry_pub.publish(ctl.getOdom());
        ctl.target_pub.publish(ctl.getWaypoint());
        ros::spinOnce();
        rate.sleep();
    }
}


