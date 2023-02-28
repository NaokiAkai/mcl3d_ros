#! /bin/bash

pcd_file=$1
publish_cycle=10.0
frame_id="map"
topic_name="/map_points"

rosrun pcl_ros pcd_to_pointcloud $pcd_file $publish_cycle _frame_id:=$frame_id /cloud_pcd:=$topic_name
