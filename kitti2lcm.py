"""
Notes:
    cam0, cam1 are grayscale cameras
    cam2, cam3 are color, we will only output color cameras
    i.e., cam_left = cam2, cam_right = cam3
"""

import numpy as np
import sys, os, pdb, glob, time, datetime, readline
import cv2

import pykitti
from tracklets import *

import lcm
from kitti import   image_t, pointcloud_t, imu_t, \
                    calib_t, \
                    tracked_object_t, tracked_object_list_t

object_type_to_idx = {'Car': 0,
                      'Van' : 1,
                      'Truck' : 2,
                      'Pedestrian' : 3,
                      'Person (sitting)' : 4,
                      'Cyclist' : 5,
                      'Tram' : 6,
                      'Misc' : 7
                    }

def project_velo_to_cam(X, K_cam_velo, K_cam):
    assert X.shape[0] == 3, 'X.shape[0]: %d' %X.shape[0]
    assert K_cam_velo.shape[0] == 4, K
    tmp = np.vstack((X, np.ones(X.shape[1])))
    x_in_cam = np.dot(K_cam_velo, tmp)
    x_in_img = np.dot(K_cam, x_in_cam[:3])
    x_in_img[0] /= x_in_img[2]
    x_in_img[1] /= x_in_img[2]
    return x_in_img[:2]

def bbox_from_corners(x):
    x1, x2 = int(np.min(x[0])), int(np.max(x[0]))
    y1, y2 = int(np.min(x[1])), int(np.max(x[1]))
    return [[x1, y1], [x2, y2]] 

def convert_timestamp(t):
    return int(time.mktime(t.utctimetuple())*1e6 + t.microsecond)

def publish_calib(idx):
    msg = calib_t()
    msg.utime = convert_timestamp(dataset.timestamps[idx])

    msg.K_cam_left = dataset.calib.K_cam2.tolist()
    msg.K_cam_right = dataset.calib.K_cam3.tolist()

    msg.cam_left_imu = dataset.calib.T_cam2_imu.tolist()
    msg.cam_right_imu = dataset.calib.T_cam3_imu.tolist()

    msg.cam_left_velo = dataset.calib.T_cam2_velo.tolist()
    msg.cam_right_velo = dataset.calib.T_cam3_velo.tolist()

    msg.velo_imu = dataset.calib.T_velo_imu.tolist()

    lc.publish('CALIB', msg.encode())

def publish_imu(idx):
    msg = imu_t()
    msg.utime = convert_timestamp(dataset.timestamps[idx])

    o = dataset.oxts[idx]

    msg.vel = [o.packet.vf, o.packet.vl, o.packet.vu]
    msg.accel = [o.packet.ax, o.packet.ay, o.packet.az]
    msg.rotation_rate = [o.packet.wx, o.packet.wy, o.packet.wz]
    msg.pose = o.T_w_imu.tolist()
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

def publish_tracked_objects(idx):
    msg = tracked_object_list_t()
    msg.utime = convert_timestamp(dataset.timestamps[idx])

    msg.size = len(frame_objects[idx])
    for i in xrange(msg.size):
        msg.objects.append(frame_objects[idx][i])

    lc.publish('TRACKED_OBJECTS', msg.encode())

def load_tracklets():
    global frame_objects

    tracklet_file = os.path.join(data_dir, 'tracklet_labels.xml')
    tracklets = parse_xml(tracklet_file)

    for i in range(len(tracklets)):
        t = tracklets[i]
        print 'tracklet %3d' % i 

        h, w, l = t.size
        box = np.array([
        [-l/2, -l/2,  l/2, l/2, -l/2, -l/2,  l/2, l/2], \
        [ w/2, -w/2, -w/2, w/2,  w/2, -w/2, -w/2, w/2], \
        [ 0.0,  0.0,  0.0, 0.0,    h,     h,   h,   h]])
        
        for nf in xrange(t.num_frames):
            trans, rot = t.trans[nf], t.rot[nf]

            # re-create 3D bounding box in velodyne coordinate system
            yaw = rot[2]   # other rotations are 0 in all xml files I checked            

            R = np.array([
                [np.cos(yaw), -np.sin(yaw), 0.0],
                [np.sin(yaw),  np.cos(yaw), 0.0],
                [        0.0,          0.0, 1.0]])

            corners_in_velo = np.dot(R, box) + np.tile(trans, (8,1)).T
            corners_in_cam_left = project_velo_to_cam(corners_in_velo,
                                    dataset.calib.T_cam2_velo, dataset.calib.K_cam2)
            corners_in_cam_right = project_velo_to_cam(corners_in_velo,
                                    dataset.calib.T_cam3_velo, dataset.calib.K_cam3)

            # build object
            o = tracked_object_t()
            frame_idx = t.first_frame + nf

            o.object_type = [0 for i in xrange(8)]
            o.object_type[object_type_to_idx[t.object_type]] = 1.0

            o.state, o.occluded, o.truncated = t.state[nf], t.occ[nf, 0], t.trunc[nf]
            
            o.size = t.size
            o.pos_in_velo = t.trans[nf]
            o.quat_in_velo = [np.cos(yaw/2.), 0, 0, np.sin(yaw/2.)]

            o.bbox_cam_left = bbox_from_corners(corners_in_cam_left)
            o.bbox_cam_right = bbox_from_corners(corners_in_cam_right)

            frame_objects[frame_idx].append(o)

if not len(sys.argv) == 4:
    print 'Usage: python kitti2lcm.py [base_dir] [date] [drive]'
    sys.exit(1)

base_dir = sys.argv[1]
date = sys.argv[2]
drive = sys.argv[3]
data_dir = os.path.join(base_dir, date, date + '_drive_' + drive + '_sync')

N = len(os.listdir(os.path.join(data_dir, 'image_00/data')))
frame_range = range(0, N)

frame_objects = [[] for i in xrange(N)]

dataset = pykitti.raw(base_dir, date, drive, frame_range)
dataset.load_calib()
dataset.load_timestamps()
# dataset.load_oxts()
dataset.load_rgb(format='cv2')
#dataset.load_velo()
load_tracklets()

print 'Loaded: drive %s' % (date + '_' + drive)

lc = lcm.LCM()

for j in xrange(10):
    for i in xrange(N):
        # publish_calib(i)
        # publish_imu(i)
        publish_image(i)
        # publish_velodyne(i)
        # publish_tracked_objects(i)

        print '[%05d]' % i
        time.sleep(0.2)