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

#include <mcl3d_ros/DistanceField.h>

namespace mcl3d {

DistanceField::DistanceField(std::string yamlFilePath):
    isAvailable_(false),
    debugFlag_(false),
    dfComputationThreadsNum_(4)
{
    // get root directory name from the given yaml file path path
    rootDirName_ = yamlFilePath.substr(0, yamlFilePath.find_last_of("/") + 1);
    FILE *fp = fopen(yamlFilePath.c_str(), "r");
    if (fp == NULL) {
        fprintf(stderr, "The yaml file does not exist -> %s\n", yamlFilePath.c_str());
        return;
    }
    fclose(fp);

    // load main map parameters
    YAML::Node lconf = YAML::LoadFile(yamlFilePath);
    mapFileName_ = lconf["map_file_name"].as<std::string>();
    resolution_ = lconf["resolution"].as<float>();
    origin_ = lconf["origin"].as<std::vector<float> >();
    width_ = lconf["width"].as<int>();
    depth_ = lconf["depth"].as<int>();
    height_ = lconf["height"].as<int>();
    subMapResolution_ = lconf["sub_map_resolution"].as<float>();
    subMapWidth_ = lconf["sub_map_width"].as<int>();
    subMapDepth_ = lconf["sub_map_depth"].as<int>();
    subMapHeight_ = lconf["sub_map_height"].as<int>();

    // initialization
    initializeSubMaps();
    isAvailable_ = true;
}

DistanceField::DistanceField(std::string mapFileName, float resolution, float subMapResolution, float mapMargin,
    Point minPoint, Point maxPoint, std::string yamlFilePath):
    mapFileName_(mapFileName),
    resolution_(resolution),
    subMapResolution_(subMapResolution),
    isAvailable_(false),
    debugFlag_(false),
    dfComputationThreadsNum_(4)
{
    // get root directory name from the given yaml file path path
    rootDirName_ = yamlFilePath.substr(0, yamlFilePath.find_last_of("/") + 1);

    // calculate map parameters
    float minX = minPoint.getX() - mapMargin;
    float minY = minPoint.getY() - mapMargin;
    float minZ = minPoint.getZ() - mapMargin;
    float maxX = maxPoint.getX() + mapMargin;
    float maxY = maxPoint.getY() + mapMargin;
    float maxZ = maxPoint.getZ() + mapMargin;
    float dx = maxX - minX;
    float dy = maxY - minY;
    float dz = maxZ - minZ;
    float mapRangeX = 0.0f, mapRangeY = 0.0f, mapRangeZ = 0.0;
    width_ = depth_ = height_ = 0;
    while (mapRangeX < dx) {
        mapRangeX += resolution_;
        width_++;
    }
    while (mapRangeY < dy) {
        mapRangeY += resolution_;
        depth_++;
    }
    while (mapRangeZ < dz) {
        mapRangeZ += resolution_;
        height_++;
    }
    origin_.resize(3);
    origin_[0] = minX;
    origin_[1] = minY;
    origin_[2] = minZ;
    subMapWidth_ = (int)(resolution_ / subMapResolution_);
    subMapDepth_ = (int)(resolution_ / subMapResolution_);
    subMapHeight_ = (int)(resolution_ / subMapResolution_);

    // make an yaml file
    FILE *fp = fopen(yamlFilePath.c_str(), "w");
    fprintf(fp, "map_file_name: %s\n", mapFileName_.c_str());
    fprintf(fp, "resolution: %f\n", resolution_);
    fprintf(fp, "origin: [%f, %f, %f]\n", origin_[0], origin_[1], origin_[2]);
    fprintf(fp, "width: %d\n", width_);
    fprintf(fp, "depth: %d\n", depth_);
    fprintf(fp, "height: %d\n", height_);
    fprintf(fp, "sub_map_resolution: %f\n", subMapResolution_);
    fprintf(fp, "sub_map_width: %d\n", subMapWidth_);
    fprintf(fp, "sub_map_depth: %d\n", subMapDepth_);
    fprintf(fp, "sub_map_height: %d\n", subMapHeight_);
    fclose(fp);

    // initialization
    initializeSubMaps();
    isAvailable_ = true;
}

void DistanceField::initializeSubMaps(void) {
    totalMapPointsNum_ = 0;
    subMapPointsNums_.resize(width_);
    subMapOrigins_.resize(width_);
    subMaps_.resize(width_);
    distanceLists_.resize(width_);
    for (int i = 0; i < width_; i++) {
        subMapPointsNums_[i].resize(depth_);
        subMapOrigins_[i].resize(depth_);
        subMaps_[i].resize(depth_);
        distanceLists_[i].resize(depth_);
        for (int j = 0; j < depth_; j++) {
            subMapPointsNums_[i][j].resize(height_, 0);
            subMapOrigins_[i][j].resize(height_);
            subMaps_[i][j].resize(height_);
            distanceLists_[i][j].resize(height_);
        }
    }
}

void DistanceField::addPoint(float x, float y, float z) {
    int u, v, w;
    if (!xyz2uvw(origin_, resolution_, width_, depth_, height_, x, y, z, &u, &v, &w))
        return;

    if (subMapPointsNums_[u][v][w] == 0) {
        distanceLists_[u][v][w].resize(256);
        subMapOrigins_[u][v][w].resize(3);
        subMapOrigins_[u][v][w][0] = origin_[0] + (float)u * resolution_;
        subMapOrigins_[u][v][w][1] = origin_[1] + (float)v * resolution_;
        subMapOrigins_[u][v][w][2] = origin_[2] + (float)w * resolution_;
        int subMapSize = (int)(resolution_ / subMapResolution_);
        subMaps_[u][v][w].resize(subMapSize);
        for (int i = 0; i < subMapSize; i++) {
            subMaps_[u][v][w][i].resize(subMapSize);
            for (int j = 0; j < subMapSize; j++)
                subMaps_[u][v][w][i][j].resize(subMapSize, MAP_VAL_INVALID);
        }
    }

    int uu, vv, ww;
    if (xyz2uvw(subMapOrigins_[u][v][w], subMapResolution_, subMapWidth_, subMapDepth_, subMapHeight_, x, y, z, &uu, &vv, &ww)) {
        subMapPointsNums_[u][v][w]++;
        subMaps_[u][v][w][uu][vv][ww] = MAP_VAL_EXIST_POINT;
        totalMapPointsNum_++;
    }
}

void DistanceField::saveSubDistanceMap(int uo, int vo, int wo, FILE *fp) {
    // initialize distance map
    dope::Index3 mapSize({(long unsigned int)subMapWidth_, (long unsigned int)subMapDepth_, (long unsigned int)subMapHeight_});
    dope::Grid<float, 3> distMap(mapSize);
    float maxValue = std::numeric_limits<float>::max();
    for (int u = 0; u < subMapWidth_; u++) {
        for (int v = 0; v < subMapDepth_; v++) {
            for (int w = 0; w < subMapHeight_; w++) {
                if (subMaps_[uo][vo][wo][u][v][w] == MAP_VAL_EXIST_POINT)
                    distMap[u][v][w] = 0.0f;
                else
                    distMap[u][v][w] = maxValue;
            }
        }
    }

    // perform distance transformation
    bool returnSquaredNorm = false;
    dt::DistanceTransform::distanceTransformL2(distMap, distMap, returnSquaredNorm, dfComputationThreadsNum_);

    // create distance list in centimeter
    std::vector<int> distanceListCM(256);
    for (int i = 0; i < mapSize[0]; i++) {
        for (int j = 0; j < mapSize[1]; j++) {
            for (int k = 0; k < mapSize[2]; k++) {
                int dist = (int)(distMap[i][j][k] * subMapResolution_ * 100.0); // cm
                // printf("%d %d %d %d\n", i, j, k, dist);
                if (dist <= 0) {
                    continue;
                } else {
                    for (int idx = 1; idx < 256; idx++) {
                        if (dist == distanceListCM[idx]) {
                            break;
                        } else if (dist < distanceListCM[idx] || distanceListCM[idx] == 0) {
                            distanceListCM.insert(distanceListCM.begin() + idx, dist);
                            distanceListCM.resize(256);
                            break;
                        }
                    }
                }
            }
        }
    }

    // create distance list in meter and its inverse list
    std::vector<float> distanceList(256);
    for (int i = 0; i < (int)distanceListCM.size(); i++) {
        distanceList[i] = (float)distanceListCM[i] / 100.0f; // cm -> m
        // printf("%d, %f [m]\n", i, distanceList[i]);
    }
    distanceList[255] = -1.0;
    std::map<int, int> inverseDistanceList;
    for (int i = 0; i < 256; i++)
        inverseDistanceList.insert(std::make_pair(distanceListCM[i], i));

    // save the results
    int maxDist = distanceListCM[254];
    for (int i = 0; i < 256; i++)
        fwrite(&distanceList[i], sizeof(float), 1, fp);
    for (int i = 0; i < mapSize[0]; i++) {
        for (int j = 0; j < mapSize[1]; j++) {
            for (int k = 0; k < mapSize[2]; k++) {
                int dist = (int)(distMap[i][j][k] * subMapResolution_ * 100.0);
                if (dist <= maxDist) {
                    int datai[4] = {i, j, k, inverseDistanceList.at(dist)};
                    // printf("%d %d %d %d\n", i, j, k, dist);
                    fwrite(datai, sizeof(int), 4, fp);
                }
            }
        }
    }
}

bool DistanceField::saveDistanceMap(void) {
    std::string mapFilePath = rootDirName_ + mapFileName_;
    FILE *fp = fopen(mapFilePath.c_str(), "w");
    if (fp == NULL) {
        fprintf(stderr, "a map file could not be open -> %s\n", mapFilePath.c_str());
        return false;
    }

    if (debugFlag_)
        printf("save the 3D distance map.\n");
    for (int u = 0; u < width_; u++) {
        printf("%.2f [%%] process done\n", (float)(u + 1) / (float)width_ * 100.0f);
        for (int v = 0; v < depth_; v++) {
            for (int w = 0; w < height_; w++) {
                if (subMapPointsNums_[u][v][w] == 0)
                    continue;
                int datai[4] = {u, v, w, (int)subMapPointsNums_[u][v][w]};
                fwrite(datai, sizeof(int), 4, fp);
                float dataf[3] = {subMapOrigins_[u][v][w][0], subMapOrigins_[u][v][w][1], subMapOrigins_[u][v][w][2]};
                fwrite(dataf, sizeof(float), 3, fp);
                saveSubDistanceMap(u, v, w, fp);
                datai[0] = datai[1] = datai[2] = -1, datai[3] = 0; // this means end of the submap
                fwrite(datai, sizeof(int), 4, fp);
            }
        }
    }

    fclose(fp);
    return true;
}

bool DistanceField::loadDistanceMap(void) {
    std::string mapFilePath = rootDirName_ + mapFileName_;
    FILE *fp = fopen(mapFilePath.c_str(), "r");
    if (fp == NULL) {
        fprintf(stderr, "a map file could not be open -> %s\n", mapFilePath.c_str());
        return false;
    }

    if (debugFlag_)
        printf("start loading the distance map -> %s\n", mapFilePath.c_str());

    int datai[4];
    float dataf[3];
    size_t size;
    for (;;) {
        size = fread(datai, sizeof(int), 4, fp);
        if (size <= 0)
            break;
        int uo = datai[0];
        int vo = datai[1];
        int wo = datai[2];
        int pointsNum = datai[3];
        subMapPointsNums_[uo][vo][wo] = pointsNum;
        totalMapPointsNum_ += pointsNum;

        size = fread(dataf, sizeof(float), 3, fp);
        subMapOrigins_[uo][vo][wo].resize(3);
        subMapOrigins_[uo][vo][wo][0] = dataf[0];
        subMapOrigins_[uo][vo][wo][1] = dataf[1];
        subMapOrigins_[uo][vo][wo][2] = dataf[2];

        subMaps_[uo][vo][wo].resize(subMapWidth_);
        for (int i = 0; i < subMapWidth_; i++) {
            subMaps_[uo][vo][wo][i].resize(subMapDepth_);
            for (int j = 0; j < subMapDepth_; j++)
                subMaps_[uo][vo][wo][i][j].resize(subMapHeight_, MAP_VAL_INVALID);
        }

        float dist;
        distanceLists_[uo][vo][wo].resize(256);
        for (int i = 0; i < 256; i++) {
            size = fread(&dist, sizeof(float), 1, fp);
            distanceLists_[uo][vo][wo][i] = dist;
        }
        int num = 0;
        for (;;) {
            size = fread(datai, sizeof(int), 4, fp);
            int u = datai[0];
            int v = datai[1];
            int w = datai[2];
            unsigned int val = (unsigned int)datai[3];
            if (u < 0 || v < 0 || w < 0 || size != 4)
                break;
            subMaps_[uo][vo][wo][u][v][w] = val;
        }
    }
    fclose(fp);

    if (debugFlag_)
        printf("finish loading the distance map -> %s\n", mapFilePath.c_str());
    return true;
}

void DistanceField::writeMapPoints(std::string fname,
    float minX, float maxX, float minY, float maxY, float minZ, float maxZ)
{
    FILE *fp = fopen(fname.c_str(), "w");
    if (fp == NULL) {
        fprintf(stderr, "Could not open file in writeMapPoints() -> %s\n", fname.c_str());
        return;
    }

    for (int u = 0; u < width_; u++) {
        if (debugFlag_)
            printf("%.3f [%%] process done\n", (float)(u + 1) / (float)width_ * 100.0f);
        for (int v = 0; v < depth_; v++) {
            for (int w = 0; w < height_; w++) {
                if (subMapPointsNums_[u][v][w] == 0)
                    continue;
                for (int uu = 0; uu < subMapWidth_; uu++) {
                    for (int vv = 0; vv < subMapDepth_; vv++) {
                        for (int ww = 0; ww < subMapHeight_; ww++) {
                            if (subMaps_[u][v][w][uu][vv][ww] == MAP_VAL_EXIST_POINT) {
                                float x, y, z;
                                uvw2xyz(subMapOrigins_[u][v][w], subMapResolution_, uu, vv, ww, &x, &y, &z);
                                if (minX <= x && x <= maxX && minY <= y && y <= maxY && minZ <= z && z <= maxZ)
                                    fprintf(fp, "%f %f %f\n", x, y, z);
                            }
                        }
                    }
                }
            }
        }
    }
    fclose(fp);
}

std::vector<Point> DistanceField::getMapPoints(float minX, float maxX, float minY, float maxY, float minZ, float maxZ) {
    std::vector<Point> points;
    for (int u = 0; u < width_; u++) {
        for (int v = 0; v < depth_; v++) {
            for (int w = 0; w < height_; w++) {
                if (subMapPointsNums_[u][v][w] == 0)
                    continue;
                for (int uu = 0; uu < subMapWidth_; uu++) {
                    for (int vv = 0; vv < subMapDepth_; vv++) {
                        for (int ww = 0; ww < subMapHeight_; ww++) {
                            if (subMaps_[u][v][w][uu][vv][ww] == MAP_VAL_EXIST_POINT) {
                                float x, y, z;
                                uvw2xyz(subMapOrigins_[u][v][w], subMapResolution_, uu, vv, ww, &x, &y, &z);
                                if (minX <= x && x <= maxX && minY <= y && y <= maxY && minZ <= z && z <= maxZ) {
                                    Point p(x, y, z);
                                    points.push_back(p);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    return points;
}

} // namespace mcl3d