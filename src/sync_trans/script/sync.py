#! /home/nuc/miniconda3/envs/sync/bin/python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import UInt16
from nav_msgs.msg import Odometry
# from cv_bridge import CvBridge
import cv2
from message_filters import ApproximateTimeSynchronizer
import message_filters
import os
import ros_numpy
import numpy as np
import requests
from scipy.spatial.transform import Rotation

basedir = None
max_num = 0

# waypoint_size = rospy.get_param('waypoint_size')

class RGBDSynchronizer:
    def __init__(self):
        # self.bridge = CvBridge()
        self.rgb_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
        self.depth_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)
        self.vins_sub = message_filters.Subscriber("/vins_fusion/odometry", Odometry)
        self.kb_sub = rospy.Subscriber("/key_command", UInt16, self.kb_callback)
        self.sync = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub, self.vins_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.callback)

        self.rgb_image = None
        self.depth_image = None
        self.vins_pose = None
        self.frame_n = 0
        # self.basedir = "data"
        self.saveImage = False


    def callback(self, rgb_msg, depth_msg, vins_msg):
        if(self.saveImage):
            self.rgb_image = ros_numpy.numpify(rgb_msg)
            self.rgb_image = np.ascontiguousarray(self.rgb_image)
            self.rgb_image = self.rgb_image[..., ::-1]
            self.depth_image = ros_numpy.numpify(depth_msg)
            self.depth_image = np.ascontiguousarray(self.depth_image)
            # print(np.max(self.depth_image))
            image_path = os.path.join(basedir,  f"{self.frame_n}_main.png")
            cv2.imwrite(image_path, self.rgb_image)
            depth_path = os.path.join(basedir,  f"{self.frame_n}_depth.png")
            cv2.imwrite(depth_path, self.depth_image)
            vins_path = os.path.join(basedir, f"{self.frame_n}_vins.txt")
            vins_msg = q2T(vins_msg)
            np.savetxt(vins_path, vins_msg)
            with open(image_path, 'rb') as rgb_file:
                rgb_data = rgb_file.read()
            with open(depth_path, 'rb') as depth_file:
                depth_data = depth_file.read()
            with open(vins_path, 'rb') as vins_file:
                vins_data = vins_file.read()
            files = {'rgb': (image_path, rgb_data, 'image/png'),
                'depth': (depth_path, depth_data, 'image/png'),
                'vins': (vins_path, vins_data, 'text/plain')}
            flying_finished = False
            print("save success!")
            self.frame_n += 1

            # if(self.frame_n == waypoint_size):
            #     flying_finished = True

            self.saveImage = False

            server_url = 'http://192.168.50.56:7300/get_flying_data/'
            # server_url = 'http://192.168.1.112:7300/get_flying_data/'
            response = requests.post(server_url, data={"flying_finished":flying_finished}, files=files)
            print(response.text)  # 输出服务器返回的响应内容



    def kb_callback(self, msg):
        if(msg.data == 1):
            self.saveImage = True
        else:
            self.saveImage = False


def q2T(Odometry):
    px = Odometry.pose.pose.position.x
    py = Odometry.pose.pose.position.y
    pz = Odometry.pose.pose.position.z
    x = Odometry.pose.pose.orientation.x
    y = Odometry.pose.pose.orientation.y
    z = Odometry.pose.pose.orientation.z
    w = Odometry.pose.pose.orientation.w
    R = Rotation.from_quat([x, y, z, w]).as_matrix()
    T = np.zeros((4, 4))
    T[:3,:3] = np.array(R)
    T[:, 3] = np.array([px, py, pz, 1])
    return T



def check_dir(obj_dir):
    global max_num
    if not os.path.exists(obj_dir):
        print("the parent dir not exists")
        return ""
     
    for dir_item in os.listdir(obj_dir):
        dir_item_path = os.path.join(obj_dir, dir_item)
        if os.path.isdir(dir_item_path):
            v_num = dir_item.split('_')[-1]
            num = int(v_num[1:])
            if num > max_num:
                max_num = num

    max_num += 1
    
    new_folder = os.path.join(obj_dir, f"obj_v{max_num}/")

    os.mkdir(new_folder)

    return new_folder
    




if __name__ == '__main__':
    basedir = check_dir("/home/nuc/netease_demo/data")

    # cv2.namedWindow("rgb_view")
    rospy.init_node('pytrans_node_for_pic')
    sync = RGBDSynchronizer()

    rospy.spin()