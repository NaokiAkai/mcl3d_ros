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
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <mcl3d_ros/DistanceField.h>

mcl3d::DistanceField distMap;
bool doneMapBuild = false;

std::string mapFileName, yamlFilePath;
float resolution, subMapResolution, mapMargin;

bool buildDistanceField(pcl::PointCloud<pcl::PointXYZ> mapPoints, std::string mapFileName,
    float resolution, float subMapResolution, float mapMargin, std::string yamlFilePath)
{
    mcl3d::Point minPoint(mapPoints.points[0].x, mapPoints.points[0].y, mapPoints.points[0].z);
    mcl3d::Point maxPoint(mapPoints.points[0].x, mapPoints.points[0].y, mapPoints.points[0].z);
    for (int i = 1; i < (int)mapPoints.size(); ++i) {
        float x = mapPoints[i].x;
        float y = mapPoints[i].y;
        float z = mapPoints[i].z;
        if (minPoint.getX() > x)
            minPoint.setX(x);
        if (minPoint.getY() > y)
            minPoint.setY(y);
        if (minPoint.getZ() > z)
            minPoint.setZ(z);
        if (maxPoint.getX() < x)
            maxPoint.setX(x);
        if (maxPoint.getY() < y)
            maxPoint.setY(y);
        if (maxPoint.getZ() < z)
            maxPoint.setZ(z);
    }
    printf("Min map point: %f %f %f\n", minPoint.getX(), minPoint.getY(), minPoint.getZ());
    printf("Max map point: %f %f %f\n", maxPoint.getX(), maxPoint.getY(), maxPoint.getZ());

    distMap = mcl3d::DistanceField(mapFileName, resolution, subMapResolution, mapMargin, minPoint, maxPoint, yamlFilePath);
    // FILE *fp = fopen("/tmp/map_points_by_pc_to_df.txt", "w");
    for (int i = 0; i < (int)mapPoints.size(); ++i) {
        float x = mapPoints.points[i].x;
        float y = mapPoints.points[i].y;
        float z = mapPoints.points[i].z;
        // fprintf(fp, "%f %f %f\n", x, y, z);
        distMap.addPoint(x, y, z);
    }
    // fclose(fp);

    // distMap.writeMapPoints("/tmp/df_map_points.txt", minPoint.getX(), maxPoint.getX(),
    //     minPoint.getY(), maxPoint.getY(), minPoint.getZ(), maxPoint.getZ());
    // exit(0);

    if (!distMap.saveDistanceMap()) {
        fprintf(stderr, "Error occurred during the distance field building.\n");
        exit(1);
    }

/*
    distMap.loadDistanceMap();
    FILE *fp = fopen("/tmp/dist_map_2d.txt", "w");
    for (float x = minPoint.getX(); x < maxPoint.getX(); x += distMap.getSubMapResolution()) {
        for (float y = minPoint.getY(); y < maxPoint.getY(); y += distMap.getSubMapResolution()) {
            float d = distMap.getDistance(x, y, 0.0f);
            if (d >= 0.0f)
                fprintf(fp, "%f %f %f\n", x, y, d);
        }
    }
    fclose(fp);
 */

    return true;
}

void mapPointsCB(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    pcl::PointCloud<pcl::PointXYZ> mapPoints;
    pcl::fromROSMsg(*msg, mapPoints);
    doneMapBuild = buildDistanceField(mapPoints, mapFileName, resolution, subMapResolution, mapMargin, yamlFilePath);
}

int main(int argc, char **argv) {
    if (argv[4] != NULL && argv[6] != NULL) {
        // build a distance field map from a PCD file
        // do not use any ROS functions
        std::string pcdFile = argv[1];
        mapFileName = argv[2];
        resolution = std::atof(argv[3]);
        subMapResolution = std::stof(argv[4]);
        mapMargin = std::atof(argv[5]);
        yamlFilePath = argv[6];

        printf("pcdFile: %s\n", pcdFile.c_str());
        printf("mapFileName: %s\n", mapFileName.c_str());
        printf("resolution: %f [m]\n", resolution);
        printf("subMapResolution: %f [m]\n", subMapResolution);
        printf("mapMargin: %f [m]\n", mapMargin);
        printf("yamlFilePath: %s\n", yamlFilePath.c_str());

        pcl::PointCloud<pcl::PointXYZ> mapPoints;
        pcl::io::loadPCDFile(pcdFile, mapPoints);
        doneMapBuild = buildDistanceField(mapPoints, mapFileName, resolution, subMapResolution, mapMargin, yamlFilePath);
    } else {
        // get point cloud from a ROS message and build a distance field map
        ros::init(argc, argv, "pc_to_df");
        ros::NodeHandle nh("~");
        std::string mapPointsName = "/map_points";

        mapFileName = "dist_map.bin";
        resolution = 5.0f;
        subMapResolution = 0.1f;
        mapMargin = 1.0f;
        yamlFilePath = "/tmp/dist_map.yaml";

        nh.param("map_points_name", mapPointsName, mapPointsName);
        nh.param("map_file_name", mapFileName, mapFileName);
        nh.param("resolution", resolution, resolution);
        nh.param("sub_map_resolution", subMapResolution, subMapResolution);
        nh.param("map_margin", mapMargin, mapMargin);
        nh.param("yaml_file_path", yamlFilePath, yamlFilePath);

        printf("mapPointsName: %s\n", mapPointsName.c_str());
        printf("mapFileName: %s\n", mapFileName.c_str());
        printf("resolution: %f [m]\n", resolution);
        printf("subMapResolution: %f [m]\n", subMapResolution);
        printf("mapMargin: %f [m]\n", mapMargin);
        printf("yamlFilePath: %s\n", yamlFilePath.c_str());

        ros::Subscriber mapPointsSub = nh.subscribe(mapPointsName, 1, mapPointsCB);

        ros::Rate loopRate(1.0);
        while (ros::ok()) {
            ros::spinOnce();
            if (doneMapBuild)
                break;
            loopRate.sleep();
        }
    }

    return 0;
}