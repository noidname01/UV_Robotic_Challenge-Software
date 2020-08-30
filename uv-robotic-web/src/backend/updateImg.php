<?php

//POST資料時必須要先有這些header檔

header("Access-Control-Allow-Origin: *");
header("Access-Control-Allow-Headers: access");
header("Access-Control-Allow-Methods: POST");
header("Content-Type: application/json; charset=UTF-8");
header("Access-Control-Allow-Headers: Content-Type, Access-Control-Allow-Headers, Authorization, X-Requested-With");

$data = json_decode(file_get_contents("php://input"));
$combined_img_path = "/home/noidname/UV_Robotic_Challenge-Software/catkin_ws/uv_robot_ros/src/combine_map.jpg"
$camera_stream_img_path = "/home/noidname/UV_Robotic_Challenge-Software/uv-robotic-web/src/image/camera_stream.jpg"

echo json_encode(["success"=> 1, "combined_img_path"=> $combined_img_path, "camera_stream_img_path" => $camera_stream_img_path ]);

?>
