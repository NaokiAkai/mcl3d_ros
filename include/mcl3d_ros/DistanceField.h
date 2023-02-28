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

#ifndef __DISTANCE_FIELD_H__
#define __DISTANCE_FIELD_H__

#include <stdio.h>
#include <omp.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <mcl3d_ros/Point.h>
#include <distance_transform/distance_transform.hpp>

namespace mcl3d {

#define MAP_VAL_EXIST_POINT (0)
#define MAP_VAL_INVALID (255)

class DistanceField {
public:
    std::string rootDirName_, mapFileName_;
    float resolution_, subMapResolution_;
    std::vector<float> origin_;
    int width_, depth_, height_, subMapWidth_, subMapDepth_, subMapHeight_;
    unsigned int totalMapPointsNum_;
    std::vector<std::vector<std::vector<unsigned int>>> subMapPointsNums_;
    std::vector<std::vector<std::vector<std::vector<float>>>> subMapOrigins_;
    std::vector<std::vector<std::vector<std::vector<std::vector<std::vector<unsigned char>>>>>> subMaps_;
    std::vector<std::vector<std::vector<std::vector<float>>>> distanceLists_;
    bool isAvailable_, debugFlag_;
    int dfComputationThreadsNum_;

    DistanceField(void) {}
    DistanceField(std::string yamlFilePath);
    DistanceField(std::string mapFileName, float resolution, float subMapResolution, float mapMargin,
        Point minPoint, Point maxPoint, std::string yamlFilePath);
    ~DistanceField() {};
    void initializeSubMaps(void);
    void saveSubDistanceMap(int uo, int vo, int wo, FILE *fp);
    bool saveDistanceMap(void);
    bool loadDistanceMap(void);
    void writeMapPoints(std::string fname, float minX, float maxX, float minY, float maxY, float minZ, float maxZ);
    std::vector<Point> getMapPoints(float minX, float maxX, float minY, float maxY, float minZ, float maxZ);

    // functions to get parameters
    inline int getWidth(void) { return width_; }
    inline int getDepth(void) { return depth_; }
    inline int getHeight(void) { return height_; }
    inline float getResolution(void) { return resolution_; }
    inline float getSubMapResolution(void) { return subMapResolution_; }

    // functions to set parameters
    inline void setDebugFlag(bool debugFlag) { debugFlag_ = debugFlag; }
    inline void setDFComputationThreadsNum(int dfComputationThreadsNum) { dfComputationThreadsNum_ = dfComputationThreadsNum; }

    inline bool xyz2uvw(std::vector<float> origin, float resolution, int width, int depth, int height, float x, float y, float z, int *u, int *v, int *w) {
        *u = (int)((x - origin[0]) / resolution);
        *v = (int)((y - origin[1]) / resolution);
        *w = (int)((z - origin[2]) / resolution);
        if (0 <= *u && *u < width && 0 <= *v && *v <depth && 0 <= *w && *w < height)
            return true;
        else
            return false;
    }

    inline void uvw2xyz(std::vector<float> origin, float resolution, int u, int v, int w, float *x, float *y, float *z) {
        *x = (float)u * resolution + origin[0];
        *y = (float)v * resolution + origin[1];
        *z = (float)w * resolution + origin[2];
    }

    void addPoint(float x, float y, float z);

    inline void addPoint(Point p) {
        addPoint(p.getX(), p.getY(), p.getZ());
    }

    inline float getDistance(float x, float y, float z) {
        if (!isAvailable_)
            return -1000.0f;
        int u, v, w;
        if (!xyz2uvw(origin_, resolution_, width_, depth_, height_, x, y, z, &u, &v, &w))
            return -1000.0f;
        if (subMapPointsNums_[u][v][w] == 0)
            return -1000.0f;
        int uu, vv, ww;
        if (xyz2uvw(subMapOrigins_[u][v][w], subMapResolution_, subMapWidth_, subMapDepth_, subMapHeight_, x, y, z, &uu, &vv, &ww))
            return distanceLists_[u][v][w][subMaps_[u][v][w][uu][vv][ww]];
        else
            return -1000.0f;
    }

    inline float getDistance(Point point) {
        return getDistance(point.getX(), point.getY(), point.getZ());
    }

    inline std::vector<float> getDistances(std::vector<Point> points) {
        std::vector<float> distances((int)points.size());
        for (int i = 0; i < (int)points.size(); i++)
            distances[i] = getDistance(points[i].getX(), points[i].getY(), points[i].getZ());
        return distances;
    }

    inline unsigned int getTotalMapPointsNum(void) {
        return totalMapPointsNum_;
    }
}; // class DistanceField

} // namespace mcl3d

#endif // __DISTANCE_FIELD_H__