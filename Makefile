all:
	lcm-gen -p kitti.lcm
	lcm-gen -j kitti.lcm
	javac -cp $(LCM_JAR_PATH) kitti/*.java
	jar cf lcmtypes_kitti.jar kitti/*.class

clean:
	rm -rf kitti
	rm -rf lcmtypes_kitti.jar