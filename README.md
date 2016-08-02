Introduction
============

This is a data loader for KITTI. It publishes the RGB image data, velodyne pointclouds and IMU
data at 10 Hz. We use LCM to publish these messages. Also, we publish the tracklet information
as a set of tracked objects, their position and classes (both in velodyne coordinate frame and as
bounding boxes in each camera) at 10 Hz.

Dependencies
============

You will need to install LCM to use this and export a variable ``JAVA_CLASSPATH`` that gives the
location of ``lcm.jar`` on your computer. Either use ``lcm-gen`` to compile the LCM messages in
kitti.lcm or use the included ``lcmtypes_kitti.jar`` file.

You will also need to download the KITTI dataset.

Usage
=====

1. Run
```
    python kitti2lcm.py [base_dir] [date] [drive name]
```
2. Run ``./spy.sh`` which should open ``lcm-spy`` program that can show each message as it is being published.
You can then run ``lcm-logger`` to log these messages as an LCM file and play them back later.


