import numpy as np
import sys, os, pdb, glob, time, datetime, readline
import cv2

import lcm
from kitti import   image_t, pointcloud_t, imu_t, \
                    calib_t, \
                    tracked_object_t, tracked_object_list_t


win = cv2.namedWindow("debug")

idx = 0
last_tracked_object_list = None

def on_cam_left(c, d):
    global idx

    msg = image_t.decode(d)

    data = np.array(bytearray(msg.data))
    tmp = cv2.imdecode(data, cv2.CV_LOAD_IMAGE_COLOR)

    if last_tracked_object_list is not None:
        for i in xrange(last_tracked_object_list.size):
            o = last_tracked_object_list.objects[i]
            cv2.rectangle(tmp, o.bbox_cam_left[0], o.bbox_cam_left[1], \
                (0,0,255), 1)

    cv2.imshow("debug", tmp)
    cv2.waitKey(10)

    idx += 1

def on_tracked_object(c, d):
    global last_tracked_object_list

    msg  = tracked_object_list_t.decode(d)
    last_tracked_object_list = msg

lc = lcm.LCM()
lc.subscribe("CAM_LEFT", on_cam_left)
lc.subscribe("TRACKED_OBJECTS", on_tracked_object)

try:
    while True:
        lc.handle()

except KeyboardInterrupt:
    cv2.destroyAllWindows()
    cv2.waitKey(1)