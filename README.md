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

## Workflow
TODO:流程圖

## Vision

## Mapping

## Navigation
(TODO:left_navi流程圖)  
* To ensure the UV robot work properly, users have to place it at a corner of the room. The robot will firstly adjust its direction so that the wall is on the left of itself, and then start to move. The navigation stategy in step1 basicly is to walk along the wall(or the area the robot has sterilized) while simultaneously recording the area the robot can't enter(ex: obstacles), the path the robot has walk through, and the area the robot has sterilized on the map array. The robot will finally stop at a certain position in the room.
* In the step2 the robot will check if there's any area unsterilized on the map array, and then will navigate to the nearest part to sterilized it. The navigatation method in this step is to find the best path with A* algorithm. After entering the unsterilized area, the robot will move in the same manner with that in step1. 
* The robot will perform step1 and step2 alternately until almost every part of the room is sterilized.

## Miscellaneous Action

## Data Flow
