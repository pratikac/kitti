import numpy as np
import sys, os, pdb, glob, time, datetime, readline
import cv2

import lcm
from kitti import   image_t, pointcloud_t, imu_t, \
                    calib_t, \
                    tracked_object_t, tracked_object_list_t


win = cv2.namedWindow("debug")

idx = 0

def handler(c, d):
    global idx

    msg = image_t.decode(d)

    data = np.array(bytearray(msg.data))
    tmp = cv2.imdecode(data, cv2.CV_LOAD_IMAGE_COLOR)
    cv2.imshow("debug", tmp)
    cv2.waitKey(10)

    idx += 1

lc = lcm.LCM()
lc.subscribe("CAM_LEFT", handler)

try:
    while True:
        lc.handle()

except KeyboardInterrupt:
    cv2.destroyAllWindows()
    cv2.waitKey(1)
    pass
