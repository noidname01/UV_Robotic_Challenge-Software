# UV_Robotic_Challenge-Software

## About Map
Following is the method of getting a 2d map of the room.
* Open 3 terminals, and run these commands.
`roscore`
`roslaunch realsense2_camera opensource_tracking.launch`
`rosrun pcl_ros pointcloud_to_pcd input:=/rtabmap/cloud_map`
* Hold the camera and turn it around slowly.
* Lots of .pcd files will be saved. We only need the newest one.
* Use pcd_python.py to make the 3d pointcloud file become 2d.
* Use pcd_to_pgm.py to make the 2d pcd file become a pgm file.
