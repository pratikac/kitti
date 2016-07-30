import numpy as np
import pykitti
import sys, os, pdb, glob, time, datetime
import cv2

import lcm
from kitti import image_t, pointcloud_t, imu_t, calib_t

def convert_timestamp(t):
    return int(time.mktime(t.utctimetuple())*1e6 + t.microsecond)

def publish_calib(idx):
    msg = calib_t()
    msg.utime = convert_timestamp(dataset.timestamps[idx])

    msg.K_cam_left = dataset.calib.K_cam0.flatten().tolist()
    msg.K_cam_right = dataset.calib.K_cam1.flatten().tolist()

    msg.cam_left_imu = dataset.calib.T_cam0_imu.flatten().tolist()
    msg.cam_right_imu = dataset.calib.T_cam1_imu.flatten().tolist()

    msg.cam_left_velo = dataset.calib.T_cam0_velo.flatten().tolist()
    msg.cam_right_velo = dataset.calib.T_cam1_velo.flatten().tolist()

    msg.velo_imu = dataset.calib.T_velo_imu.flatten().tolist()

    lc.publish('CALIB', msg.encode())

def publish_imu(idx):
    msg = imu_t()
    msg.utime = convert_timestamp(dataset.timestamps[idx])

    o = dataset.oxts[idx]

    msg.vel = [o.packet.vf, o.packet.vl, o.packet.vu]
    msg.accel = [o.packet.ax, o.packet.ay, o.packet.az]
    msg.rotation_rate = [o.packet.wx, o.packet.wy, o.packet.wz]
    msg.pose = o.T_w_imu.flatten().tolist()
    lc.publish('IMU', msg.encode())

def publish_image(idx):
    msg = image_t()
    msg.utime = convert_timestamp(dataset.timestamps[idx])

    def helper(c, img):
        msg.width, msg.height = img.shape[1], img.shape[0]
        msg.data = cv2.imencode('.jpg', img)[1].tostring()
        msg.size = len(msg.data)
        lc.publish(c, msg.encode())

    helper("CAM_LEFT", dataset.rgb[idx].left)
    helper("CAM_RIGHT", dataset.rgb[idx].right)

def publish_velodyne(idx):
    msg = pointcloud_t()
    msg.utime = convert_timestamp(dataset.timestamps[idx])
    v = dataset.velo[idx]

    N = v.shape[0]
    msg.size = N
    msg.points = v[:N].tolist()
    lc.publish('VELODYNE', msg.encode())


if not len(sys.argv) == 3:
    print 'Usage: python kitti2lcm.py [date] [drive]'
    sys.exit(1)

base_dir = '/local/kitti'
date = sys.argv[1]
drive = sys.argv[2]

img0_dir = os.path.join(base_dir, date, date + '_drive_' + drive + '_sync/image_00/data')
N = len(os.listdir(img0_dir))
frame_range = range(0, N)

dataset = pykitti.raw(base_dir, date, drive, frame_range)

dataset.load_calib()
dataset.load_timestamps()
dataset.load_oxts()
dataset.load_rgb(format='cv2')
dataset.load_velo()

print 'Loaded: drive %s' % (date + '_' + drive)

lc = lcm.LCM()

for i in xrange(N):
    publish_calib(i)
    publish_imu(i)
    publish_image(i)
    publish_velodyne(i)

    print 'Published: [%05d]' % i
    time.sleep(1)