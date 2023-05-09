#include<demo_cpp/controller.h>


Controller::Controller() : idx(0), isarrived(false){
    waypoints.clear();
    this->setWaypoints();
    target_pos.position.x = waypoints[0].at(0);
    target_pos.position.y = waypoints[0].at(1);
    target_pos.position.z = waypoints[0].at(2);
}

void Controller::setWaypoints(){
    double r = 1.32;
    double angle = 0;
    double bodyangle = angle;
    double angleStep = 30;
    double initx = -1.32;
    double inity = 0;
    double initz = 0.8;
    waypoints.push_back({initx, inity, initz, angles::from_degrees(bodyangle)});
    double posx;
    double posy;
    double posz;
    for(int i = 1; i < 12; i++){
        angle = angle - angleStep;
        bodyangle = angle;
        if(bodyangle < -180){
            bodyangle += 360;
        }
        posx = initx + r - r*cos(angles::from_degrees(bodyangle));
        posy = inity - r*sin(angles::from_degrees(bodyangle));
        posz = initz;
        waypoints.push_back({posx, posy, posz, angles::from_degrees(bodyangle)});
    }

    // std::ifstream fin("/home/perple/nerf_demo/data/views.txt", std::ios::in | std::ios::binary);
    // std::string temp;
    // while(getline(fin, temp, '\n')){
    //     std::vector<double> waypoint;
    //     std::istringstream sin(temp);
    //     for(int i=0; i<=3; i++){
    //         double data;
    //         sin >> data;
    //         waypoint.push_back(data);
    //     }
    //     waypoints.push_back(waypoint);
    // }
    // ROS_INFO_STREAM("the waypoints size is " << waypoints.size());
    // return;
}

mavros_msgs::PositionTarget Controller::getWaypoint(){
    std::vector<double> waypoint;
    Eigen::Vector3d dis = {this->target_pos.position.x - this->local_pos.pose.position.x, this->target_pos.position.y - this->local_pos.pose.position.y
    , this->target_pos.position.z - this->local_pos.pose.position.z};
    // ROS_INFO_STREAM("distance is " << dis.squaredNorm());
    static ros::Time start_time;
    if(dis.squaredNorm() < 0.05 && !isarrived){
        isarrived = true;
        start_time = ros::Time::now();
    }
    // ROS_INFO_STREAM(isarrived);
    if(isarrived && ros::Time::now() - start_time > ros::Duration(5)){
        isarrived = false;
        this->idx = (this->idx + 1) % waypoints.size();
    }
    // ROS_INFO_STREAM("target pos is " << target_pos.position.x << " " << target_pos.position.y << " " << target_pos.position.z);
    ROS_INFO_STREAM("waypoint idx is " << this->idx);
    waypoint = waypoints.at(idx);
    target_pos.coordinate_frame = 1;
    target_pos.header.stamp = ros::Time::now();
    target_pos.type_mask = 0xBF8;
    target_pos.position.x = waypoint.at(0);
    target_pos.position.y = waypoint.at(1);
    target_pos.position.z = waypoint.at(2);
    target_pos.yaw = waypoint.at(3);
    return target_pos;
}

geometry_msgs::PoseStamped Controller::getPose(){
    // tf2::Quaternion transform;
    // transform.setRPY(0, 0, 0);
    // tf2::Quaternion q;
    // tf2::fromMsg(this->local_pos.pose.orientation, q);
    // q.normalize();
    // tf2::Quaternion q_new ;
    // q_new = transform * q;
    // q_new.normalize();
    // this->local_pos.pose.orientation = tf2::toMsg(q_new);

    this->local_pos.pose.position.x = pos_drone_t265[0];
    this->local_pos.pose.position.y = pos_drone_t265[1];
    this->local_pos.pose.position.z = pos_drone_t265[2];

    this->local_pos.pose.orientation.x = q_t265.x();
    this->local_pos.pose.orientation.y = q_t265.y();
    this->local_pos.pose.orientation.z = q_t265.z();
    this->local_pos.pose.orientation.w = q_t265.w();
    
    this->local_pos.header.stamp = ros::Time::now();
    // this->local_pos.header.frame_id = "world";
    return local_pos;
}

nav_msgs::Odometry Controller::getOdom(){
    // this->local_pos.pose.position.x = pos_drone_vins[0];
    // this->local_pos.pose.position.y = pos_drone_vins[1];
    // this->local_pos.pose.position.z = pos_drone_vins[2];

    // this->local_pos.pose.orientation.x = q_vins.x();
    // this->local_pos.pose.orientation.y = q_vins.y();
    // this->local_pos.pose.orientation.z = q_vins.z();
    // this->local_pos.pose.orientation.w = q_vins.w();
    
    this->local_odom.header.stamp = ros::Time::now();
    this->local_odom.header.frame_id = "odom";
    this->local_odom.child_frame_id = "base_link";
    return local_odom;
}

void Controller::pos_callback(nav_msgs::OdometryConstPtr msg){
    // this->local_pos.pose.position.x = -msg->pose.pose.position.y;
    // this->local_pos.pose.position.y = msg->pose.pose.position.x;
    // this->local_pos.pose.position.z = msg->pose.pose.position.z;
    // this->local_pos.pose.orientation.w = msg->pose.pose.orientation.w;
    // this->local_pos.pose.orientation.x = msg->pose.pose.orientation.x;
    // this->local_pos.pose.orientation.y = msg->pose.pose.orientation.y;
    // this->local_pos.pose.orientation.z = msg->pose.pose.orientation.z;
    pos_drone_t265 = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    q_t265 = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, 
                                msg->pose.pose.orientation.z);
    // Quaternion2Euler(this->local_pos);
}

void Controller::odom_callback(nav_msgs::Odometry::ConstPtr msg){
    this->local_odom = *msg;
    // pos_drone_vins = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    // q_vins = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, 
    //                             msg->pose.pose.orientation.z);
    // Quaternion2Euler(this->local_pos);
}


void Controller::state_callback(mavros_msgs::StateConstPtr state){
    this->current_state = *state;
}

