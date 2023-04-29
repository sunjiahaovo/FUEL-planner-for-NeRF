# import threading
# import requests
# import numpy as np
# import sys
# import json
# import time
# import os

# base_dir = "data/test_v2"

# ## 上传图片和位姿到服务器
# def upload_picture():
#     up_files = {'rgb': open(os.path.join(base_dir,"0_main.png"), 'rb'),
#            'depth': open(os.path.join(base_dir,"0_depth.png"), 'rb'),
#            'pose': open(os.path.join(base_dir,"0_pose.txt"), 'rb')}
    
#     response = requests.post("http://192.168.50.189:7300/uploadpicture/", files = up_files)
    
#     # print(response.text)
#     return response.text


# if __name__=='__main__':
#     while(1):
#         t0 = time.time()
#         upload_picture()
#         print("time = ", time.time()-t0)



def say():
    print("hello")