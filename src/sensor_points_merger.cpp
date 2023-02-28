/****************************************************************************
 * mcl3d_ros: 3D Monte Carlo localization for ROS use
 * Copyright (C) 2023 Naoki Akai
 *
 * Licensed under the Apache License, Version 2.0 (the “License”);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an “AS IS” BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @author Naoki Akai
 ****************************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <mcl3d_ros/Pose.h>

class SensorPointsMerger {
private:
    ros::NodeHandle nh_;
    std::string baseFrame_;
    std::vector<std::string> sensorFrames_, sensorTopicNames_;
    std::vector<ros::Subscriber> pointsSubs_;
    std::vector<mcl3d::Pose> displacements_;
    std::vector<bool> gotPoints_;

    std::string mergedPointsName_, mergedPointsFrame_;
    ros::Publisher pointsPub_;
    sensor_msgs::PointCloud mergedPoints_;

    tf::TransformListener tfListener_;

public:
    SensorPointsMerger(void):
        nh_("~"),
        baseFrame_("base_link"),
        sensorTopicNames_({"/cloud", "/cloud"}),
        sensorFrames_({"hokuyo3d_front", "hokuyo3d_rear"}),
        gotPoints_({false, false}),
        mergedPointsName_("/velodyne_points"),
        mergedPointsFrame_("velodyne")
    {
        nh_.param("base_frame", baseFrame_, baseFrame_);
        nh_.param("sensor_frames", sensorFrames_, sensorFrames_);
        nh_.param("sensor_topic_names", sensorTopicNames_, sensorTopicNames_);

        nh_.param("merged_points_name", mergedPointsName_, mergedPointsName_);
        nh_.param("merged_points_frame", mergedPointsFrame_, mergedPointsFrame_);

        // read transformations from TF tree
        // printf("read transformations from TF tree\n");
        for (int i = 0; i < (int)sensorFrames_.size(); ++i) {
            tf::StampedTransform trans;
            int tfFailedCnt = 0;
            ros::Rate loopRate(10.0);
            while (ros::ok()) {
                ros::spinOnce();
                try {
                    ros::Time now = ros::Time::now();
                    tfListener_.waitForTransform(baseFrame_, sensorFrames_[i], now, ros::Duration(1.0));
                    tfListener_.lookupTransform(baseFrame_, sensorFrames_[i], now, trans);
                    break;
                } catch (tf::TransformException ex) {
                    tfFailedCnt++;
                    if (tfFailedCnt >= 100) {
                        ROS_ERROR("Cannot get the relative pose from the base link to the laser from the tf tree."
                            " Did you set the static transform publisher between %s to %s?",
                            baseFrame_.c_str(), sensorFrames_[i].c_str());
                        exit(1);
                    }
                    loopRate.sleep();
                }
            }
            tf::Quaternion quat(trans.getRotation().x(), trans.getRotation().y(),
                trans.getRotation().z(), trans.getRotation().w());
            double roll, pitch, yaw;
            tf::Matrix3x3 rotMat(quat);
            rotMat.getRPY(roll, pitch, yaw);
            mcl3d::Pose poseTrans(trans.getOrigin().x(), trans.getOrigin().y(), trans.getOrigin().z(), roll, pitch, yaw);
            displacements_.push_back(poseTrans);
        }
        // printf("All the transformations have been obtained.\n");

        // set subscribers
        for (int i = 0; i < (int)sensorTopicNames_.size(); ++i) {
            ros::Subscriber sub = nh_.subscribe(sensorTopicNames_[i], 1, &SensorPointsMerger::pointsCB, this);
            pointsSubs_.push_back(sub);
        }

        // set publisher
        pointsPub_ = nh_.advertise<sensor_msgs::PointCloud2>(mergedPointsName_, 5);

        // make empty
        mergedPoints_.points.clear();
        printf("Initialization has been done.\n");
    }

    void pointsCB(const ros::MessageEvent<sensor_msgs::PointCloud2 const> &event) {
        // check number of the sensor data
        std::string topicName = event.getConnectionHeader().at("topic");
        std::string frame = event.getMessage()->header.frame_id;
        // printf("topicName = %s, frame = %s\n", topicName.c_str(), frame.c_str());
        mcl3d::Pose poseTrans;
        for (int i = 0; i < (int)sensorTopicNames_.size(); ++i) {
            if (topicName == sensorTopicNames_[i] && frame == sensorFrames_[i]) {
                // return return if already have
                if (gotPoints_[i])
                    return;
                // add obtained points
                poseTrans = displacements_[i];
                gotPoints_[i] = true;
                break;
            }
        }

        // transform and merge
        double cr = cos(poseTrans.getRoll());
        double sr = sin(poseTrans.getRoll());
        double cp = cos(poseTrans.getPitch());
        double sp = sin(poseTrans.getPitch());
        double cy = cos(poseTrans.getYaw());
        double sy = sin(poseTrans.getYaw());

        double m11 = cy * cp;
        double m12 = cy * sp * sr - sy * cr;
        double m13 = sy * sr + cy * sp * cr;
        double m21 = sy * cp;
        double m22 = cy * cr + sy * sp * sr;
        double m23 = sy * sp * cr - cy * sr;
        double m31 = -sp;
        double m32 = cp * sr;
        double m33 = cp * cr;

        pcl::PointCloud<pcl::PointXYZ> points;
        pcl::fromROSMsg(*event.getMessage(), points);

        for (int i = 0; i < (int)points.size(); ++i) {
            float x = points.points[i].x;
            float y = points.points[i].y;
            float z = points.points[i].z;
            geometry_msgs::Point32 p;
            p.x = x * m11 + y * m12 + z * m13 + poseTrans.getX();
            p.y = x * m21 + y * m22 + z * m23 + poseTrans.getY();
            p.z = x * m31 + y * m32 + z * m33 + poseTrans.getZ();
            mergedPoints_.points.push_back(p);
        }

        // check whether all the sensor data has been obtained
        for (int i = 0; i < (int)gotPoints_.size(); ++i) {
            if (!gotPoints_[i])
                return;
        }

        // publish the merged point cloud and make it empty for next publish
        sensor_msgs::PointCloud2 sensorPoints;
        sensor_msgs::convertPointCloudToPointCloud2(mergedPoints_, sensorPoints);
        sensorPoints.header.stamp = ros::Time::now();
        sensorPoints.header.frame_id = mergedPointsFrame_;
        pointsPub_.publish(sensorPoints);

        // printf("publish\n");
        mergedPoints_.points.clear();
        for (int i = 0; i < (int)gotPoints_.size(); ++i)
            gotPoints_[i] = false;
    }

    void spin(void) {
        ros::spin();
    }
}; // class SensorPointsMerger

int main(int argc, char **argv) {
    ros::init(argc, argv, "sensor_points_merger");
    SensorPointsMerger node;
    node.spin();
    return 0;
}