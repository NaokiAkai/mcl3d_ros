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

#ifndef __MCL_H__
#define __MCL_H__

#include <sys/time.h>
#include <ctime>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <mcl3d_ros/Point.h>
#include <mcl3d_ros/Pose.h>
#include <mcl3d_ros/Particle.h>
#include <mcl3d_ros/DistanceField.h>
#include <mcl3d_ros/IMU.h>

namespace mcl3d {

class MCL {
private:
    DistanceField distMap_;

    int localizationMode_;
    // 0: measurement model optimization (similar to ICP scan matching)
    // 1: normal particle filter (PF)
    // 2: fusion of the optimization- and PF-based localizations (proposed method)
    // 3: extended kalman filter (EKF)

    int measurementModelType_;
    // 0: use distance (only available for the measurement-model-optimization-based localization)
    // 1: normal distribution (only available in particle-filter-based localization)
    // 2: likelihood field model
    // 3: class conditional measurement model

    int particleNum_, optParticleNum_;
    std::vector<Particle> particles_, optParticles_;

    Pose mclPose_, prevMCLPose_, odomPose_, odomVel_, baseLink2Laser_, optPose_;
    double optPoseCovCoef_, gmmPosVar_, gmmAngVar_;
    std::vector<std::vector<double>> approximateHessian_, optPoseCov_;
    std::vector<std::vector<double>> poseCov_, ekfPoseJacob_;
    bool odomAvailable_;

    IMU imu_;
    bool imuAvailable_;

    std::vector<double> odomNoise_;
    bool useOmniModel_;
    bool useLinearInterpolation_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredPoints_;
    int sensorPointsNum_;
    double voxelLeafSize_;
    double zHit_, zRand_, zMax_;
    double rangeReso_, varHit_, rangeMax_, pRand_, pMax_, unknownLambda_;
    double hitNormConst_, ndMax_, lfmMax_, ccmmMax_;
    double totalLikelihood_, averageLikelihood_, maxLikelihood_;
    int maxLikelihoodParticleIdx_;

    int optMaxIterNum_;
    double optMaxError_, convergenceThreshold_;
    bool optHasConverged_;

    double randomParticleRate_, resampleThreshold_;
    Pose resampleNoise_;

    double effectiveSampleSize_;

public:
    MCL(void);
    bool loadDistanceMap(std::string mapYamlFile);
    std::vector<Point> getMapPoints(void);
    void initializeParticles(Pose initialPose, Pose initialNoise);
    void initializeOptParticls(int optParticleNum);
    void setMeasurementModelParameters(double zHit, double zRand, double zMax,
        double varHit, double unknownLambda, double rangeReso, double rangeMax);
    bool checkParameters(void);
    void printMCLResult(void);
    void updatePoses(void);
    void calculateLikelihoodsByMeasurementModel(pcl::PointCloud<pcl::PointXYZ>::Ptr sensorPoints);
    void optimizeMeasurementModel(pcl::PointCloud<pcl::PointXYZ>::Ptr sensorPoints);
    void resampleParticles1(void);
    void resampleParticles2(void);

    // public inline functions
    // getter
    inline Pose getMCLPose(void) { return mclPose_; }
    inline Pose getOdomPose(void) { return odomPose_; }
    inline Pose getOptPose(void) { return optPose_; }
    inline std::vector<Particle> getParticles(void) { return particles_; }
    inline std::vector<Particle> getOptParticles(void) { return optParticles_; }
    inline std::vector<std::vector<double>> getPoseCovariance(void) { return poseCov_; }
    inline std::vector<std::vector<double>> getOptPoseCovariance(void) { return optPoseCov_; }
    inline pcl::PointCloud<pcl::PointXYZ> getAlignedPointsOpt(void) { return *filteredPoints_; }

    // setter
    inline void setParticleNum(int particleNum) { particleNum_ = particleNum, particles_.resize(particleNum_); }
    inline void setOptParticlsNum(int optParticleNum) { optParticleNum_ = optParticleNum; optParticles_.resize(optParticleNum_); }
    inline void setInitialPose(Pose initialPose) { mclPose_ = prevMCLPose_ = initialPose; }
    inline void setLocalizationMode(int localizationMode) { localizationMode_ = localizationMode; }
    inline void setMeasurementModelType(int measurementModelType) { measurementModelType_ = measurementModelType; }
    inline void setBaseLink2Laser(Pose baseLink2Laser) { baseLink2Laser_ = baseLink2Laser; }
    inline void setOdomNoise(std::vector<double> odomNoise) { odomNoise_ = odomNoise; }
    inline void setUseLinearInterpolation(bool useLinearInterpolation) { useLinearInterpolation_ = useLinearInterpolation; }
    inline void setSensorPointsNum(int sensorPointsNum) { sensorPointsNum_ = sensorPointsNum; }
    inline void setVoxelLeafSize(double voxelLeafSize) { voxelLeafSize_ = voxelLeafSize; }
    inline void setRandomParticleRate(double randomParticleRate) { randomParticleRate_ = randomParticleRate; }
    inline void setResampleThreshold(double resampleThreshold) { resampleThreshold_ = resampleThreshold; }
    inline void setResampleNoise(Pose resampleNoise) { resampleNoise_ = resampleNoise; }
    inline void setOptMaxIterNum(int optMaxIterNum) { optMaxIterNum_ = optMaxIterNum; }
    inline void setOptMaxError(double optMaxError) { optMaxError_ = optMaxError; }
    inline void setConvergenceThreshold(double convergenceThreshold) { convergenceThreshold_ = convergenceThreshold; }
    inline void setIMU(IMU imu) { imu_ = imu, imuAvailable_ = true; }
    inline void setOdomPose(Pose odomPose) { odomPose_ = odomPose; }
    inline void setOdomVelocities(Pose odomVel) { odomVel_ = odomVel; odomAvailable_ = true; }
    inline void setOptPoseCovCoef(double optPoseCovCoef) { optPoseCovCoef_ = optPoseCovCoef; }
    inline void setGMMPosVar(double gmmPosVar) { gmmPosVar_ = gmmPosVar; }
    inline void setGMMAngVar(double gmmAngVar) { gmmAngVar_ = gmmAngVar; }

private:
    inline double nrand(double n) {
        return (n * sqrt(-2.0 * log((double)rand() / RAND_MAX)) * cos(2.0 * M_PI * rand() / RAND_MAX));
    }

    inline double getTime(void) {
        struct timeval timeNow{};
        gettimeofday(&timeNow, nullptr);
        return (double)((timeNow.tv_sec) + (timeNow.tv_usec / 1000000.0));
    }

    inline double modAngle(double a) {
        while (a < -M_PI)
            a += 2.0 * M_PI;
        while (a > M_PI)
            a -= 2.0 * M_PI;
        return a;
    }

    inline void getCosSin(double roll, double pitch, double yaw,
        double *cr, double *sr, double *cp, double *sp, double *cy, double *sy)
    {
        *cr = cos(roll);
        *sr = sin(roll);
        *cp = cos(pitch);
        *sp = sin(pitch);
        *cy = cos(yaw);
        *sy = sin(yaw);
    }

    inline std::vector<double> getRotMat(double cr, double sr, double cp, double sp, double cy, double sy) {
        std::vector<double> rm(9);
        rm[0] = cy * cp;
        rm[1] = cy * sp * sr - sy * cr;
        rm[2] = sy * sr + cy * sp * cr;
        rm[3] = sy * cp;
        rm[4] = cy * cr + sy * sp * sr;
        rm[5] = sy * sp * cr - cy * sr;
        rm[6] = -sp;
        rm[7] = cp * sr;
        rm[8] = cp * cr;
        return rm;
    }

    inline void transformPoint(double transX, double transY, double transZ, std::vector<double> rotMat,
        double x, double y, double z, double *xx, double *yy, double *zz)
    {
        *xx = x * rotMat[0] + y * rotMat[1] + z * rotMat[2] + transX;
        *yy = x * rotMat[3] + y * rotMat[4] + z * rotMat[5] + transY;
        *zz = x * rotMat[6] + y * rotMat[7] + z * rotMat[8] + transZ;
    }

    std::vector<std::vector<double>> getInverseMatrix(std::vector<std::vector<double>> mat);
    std::vector<std::vector<double>> addMatrix(std::vector<std::vector<double>> m1, std::vector<std::vector<double>> m2, int row, int col);
    std::vector<std::vector<double>> multiplyMatrix(std::vector<std::vector<double>> m1, int row1, int col1, std::vector<std::vector<double>> m2, int row2, int col2);
    std::vector<std::vector<double>> transposeMatrix(std::vector<std::vector<double>> m, int row, int col);
    void printMatrixd(std::vector<std::vector<double>> m, int row, int col, std::string name);
    std::vector<std::vector<double>> CholeskyDecomposition(std::vector<std::vector<double>> mat);
    double getDeterminant(std::vector<std::vector<double>> mat, int n);
    double getError(float x, float y, float z, float r);
}; // class MCL

} // namespace mcl3d

#endif // __MCL_H__