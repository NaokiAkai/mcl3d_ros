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

#include <mcl3d_ros/MCL.h>

namespace mcl3d {

MCL::MCL(void):
    filteredPoints_(new pcl::PointCloud<pcl::PointXYZ>)
{
    localizationMode_ = 2;
    measurementModelType_ = 3;

    odomPose_.setPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    odomVel_.setPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    poseCov_ =  {{1.0, 0.1, 0.1, 0.1, 0.1, 0.1},
                 {0.1, 1.0, 0.1, 0.1, 0.1, 0.1},
                 {0.1, 0.1, 1.0, 0.1, 0.1, 0.1},
                 {0.1, 0.1, 0.1, 1.0, 0.1, 0.1},
                 {0.1, 0.1, 0.1, 0.1, 1.0, 0.1},
                 {0.1, 0.1, 0.1, 0.1, 0.1, 1.0}};

    optPoseCov_ = poseCov_;

    ekfPoseJacob_ = {{1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                     {0.0, 1.0, 0.0, 0.0, 0.0, 0.0},
                     {0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
                     {0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
                     {0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
                     {0.0, 0.0, 0.0, 0.0, 0.0, 1.0}};

    odomNoise_ = {1.0, 0.1, 0.1, 0.1, 0.1, 0.1,
                  0.1, 1.0, 0.1, 0.1, 0.1, 0.1,
                  0.1, 0.1, 1.0, 0.1, 0.1, 0.1,
                  0.1, 0.1, 0.1, 1.0, 0.1, 0.1,
                  0.1, 0.1, 0.1, 0.1, 1.0, 0.1,
                  0.1, 0.1, 0.1, 0.1, 0.1, 1.0};

    baseLink2Laser_.setPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    sensorPointsNum_ = 200;
    voxelLeafSize_ = 1.0;
    zHit_ = 0.9;
    zRand_ = 0.05;
    zMax_ = 0.05;
    rangeReso_ = 0.1;
    varHit_ = 0.01;
    rangeMax_ = 120.0;
    pRand_ = 1.0 / (rangeMax_ / rangeReso_);
    pMax_ = 1.0;
    unknownLambda_ = 0.001;
    hitNormConst_ = 1.0 / sqrt(2.0 * M_PI * varHit_);
    ndMax_ = hitNormConst_;
    lfmMax_ = zHit_ * hitNormConst_ + zRand_ * pRand_;
    ccmmMax_ = (hitNormConst_ +  (unknownLambda_ / (1.0 - unknownLambda_ * rangeMax_))) * 0.5;

    randomParticleRate_ = 0.1;
    resampleNoise_.setPose(0.1, 0.1, 0.1, 0.05, 0.05, 0.05);

    optMaxIterNum_ = 30;
    optMaxError_ = 1.0;
    convergenceThreshold_ = 0.02;

    useOmniModel_ = false;
    odomAvailable_ = false;

    imuAvailable_ = false;
}

void MCL::setMeasurementModelParameters(double zHit, double zRand, double zMax,
    double varHit, double unknownLambda, double rangeReso, double rangeMax)
{
    zHit_ = zHit;
    zRand_ = zRand;
    zMax_ = zMax;
    varHit_ = varHit;
    unknownLambda_ = unknownLambda;
    rangeReso_ = rangeReso;
    rangeMax_ = rangeMax;

    pRand_ = 1.0 / (rangeMax_ / rangeReso_);
    pMax_ = 1.0;
    hitNormConst_ = 1.0 / sqrt(2.0 * M_PI * varHit_);
    ndMax_ = hitNormConst_;
    lfmMax_ = zHit_ * hitNormConst_ + zRand_ * pRand_;
    ccmmMax_ = (hitNormConst_ +  (unknownLambda_ / (1.0 - unknownLambda_ * rangeMax_))) * 0.5;
}

bool MCL::loadDistanceMap(std::string mapYamlFile) {
    distMap_ = DistanceField(mapYamlFile);
    return distMap_.loadDistanceMap();
}

std::vector<Point> MCL::getMapPoints(void) {
    return distMap_.getMapPoints(-999999.9, 999999.9, -999999.9, 999999.9, -999999.9, 999999.9);
}

void MCL::initializeParticles(Pose initialPose, Pose initialNoise) {
    // particles are not used if localization mode is optimization
    if (localizationMode_ == 0 || localizationMode_ == 3)
        return;

    double wo = 1.0 / (double)particleNum_;
    for (int i = 0; i < particleNum_; ++i) {
        double x = initialPose.getX() + nrand(initialNoise.getX());
        double y = initialPose.getY() + nrand(initialNoise.getY());
        double z = initialPose.getZ() + nrand(initialNoise.getZ());
        double roll = modAngle(initialPose.getRoll() + nrand(initialNoise.getRoll()));
        double pitch = modAngle(initialPose.getPitch() + nrand(initialNoise.getPitch()));
        double yaw = modAngle(initialPose.getYaw() + nrand(initialNoise.getYaw()));
        particles_[i].setParticle(x, y, z, roll, pitch, yaw, wo);
    }
}

void MCL::initializeOptParticls(int optParticleNum) {
    // optimized particles are only used if localization mode is particle-filter-based fusion
    if (localizationMode_ != 2)
        return;

    optParticleNum_ = optParticleNum_;
    optParticles_.resize(optParticleNum_);
    optPoseCovCoef_ = 1.0 / (1.0 * 1.0);
}

bool MCL::checkParameters(void) {
    if (localizationMode_ < 0 || 3 < localizationMode_) {
        fprintf(stderr, "Unexpected localization mode is selected. The mode must be 0, 1, or 2.\n");
        return false;
    }

    if (measurementModelType_ < 0 || 3 < measurementModelType_) {
        fprintf(stderr, "Unexpected measurement model type is selected. The type must be 0, 1, 2, or 3.\n");
        return false;
    }

    if (localizationMode_ != 0) {
        if (measurementModelType_ == 0 || measurementModelType_ == 1) {
            fprintf(stderr, "Measurement model type 0 and 1 is not supported for localization mode 0.\n");
            return 0;
        }
    }

    for (int i = 0; i < (int)odomNoise_.size(); ++i) {
        if (odomNoise_[i] < 0.0) {
            fprintf(stderr, "All the elements of odometry noises must be 0 or more.\n");
            return false;
        }
    }

    if (zHit_ + zRand_ + zMax_ != 1.0) {
        fprintf(stderr, "Sum of z_hit, z_rand, and z_max must be 1.\n");
        return false;
    }

    if (rangeReso_ <= 0.0) {
        fprintf(stderr, "range_reso must be larger than 0.\n");
        return false;
    }

    if (varHit_ <= 0.0) {
        fprintf(stderr, "z_var must be larger than 0.\n");
        return false;
    }

    if (rangeMax_ <= 0.0) {
        fprintf(stderr, "range_max must be larger than 0.\n");
        return false;
    }

    if (unknownLambda_ <= 0.0) {
        fprintf(stderr, "unknown_lambda must be larger than 0.\n");
        return false;
    }

    if (randomParticleRate_ < 0.0 || 1.0 < randomParticleRate_) {
        fprintf(stderr, "random_particle_rate must be included from 0 to 1.\n");
        return false;
    }

    return true;
}

void MCL::printMCLResult(void) {
    if (localizationMode_ != 1) {
        printf("optPose: x = %lf, y = %lf, z = %lf, roll = %lf, pitch = %lf, yaw = %lf\n", 
            optPose_.getX(), optPose_.getY(), optPose_.getZ(),
            optPose_.getRoll() * 180.0 / M_PI, optPose_.getPitch() * 180.0 / M_PI, optPose_.getYaw() * 180.0 / M_PI);
    }
    if (localizationMode_ != 0) {
        printf("mclPose: x = %lf, y = %lf, z = %lf, roll = %lf, pitch = %lf, yaw = %lf\n", 
            mclPose_.getX(), mclPose_.getY(), mclPose_.getZ(),
            mclPose_.getRoll() * 180.0 / M_PI, mclPose_.getPitch() * 180.0 / M_PI, mclPose_.getYaw() * 180.0 / M_PI);
    }
    if (localizationMode_ == 1 || localizationMode_ == 3)
        printMatrixd(poseCov_, 6, 6, "pose covariance");
    if (localizationMode_ == 1 || localizationMode_ == 2)
        printf("effectiveSampleSize = %lf, particleNum = %d\n", effectiveSampleSize_, particleNum_);
    if (localizationMode_ == 2)
        printf("optParticleNum = %d\n", optParticleNum_);
    printf("\n");
}

void MCL::updatePoses(void) {
    static bool isFirst = true;
    static double prevTime;
    if (isFirst) {
        prevTime = getTime();
        isFirst = false;
        return;
    }
    double currTime = getTime();
    double dt = currTime - prevTime;
    prevTime = currTime;

    double dx, dy, dyaw;
    double dz = 0.0, droll = 0.0, dpitch = 0.0; 
    if (odomAvailable_) {
        dyaw = odomVel_.getYaw() * dt;
        double t = mclPose_.getYaw() + dyaw / 2.0;
        double dd = odomVel_.getX() * dt;
        dx = dd * cos(t);
        dy = dd * sin(t);
        if (useOmniModel_) {
            double ddl = odomVel_.getY() * dt;
            dx += ddl * cos(t + M_PI / 2.0);
            dy += ddl * sin(t + M_PI / 2.0);
        }
    } else {
        if (useLinearInterpolation_) {
            dx = mclPose_.getX() - prevMCLPose_.getX();
            dy = mclPose_.getY() - prevMCLPose_.getY();
            dyaw = modAngle(mclPose_.getYaw() - prevMCLPose_.getYaw());
        } else {
            dx = dy = dyaw = 0.0;
        }
    }
    if (useLinearInterpolation_) {
        dz = mclPose_.getZ() - prevMCLPose_.getZ();
        droll = modAngle(mclPose_.getRoll() - prevMCLPose_.getRoll());
        dpitch = modAngle(mclPose_.getPitch() - prevMCLPose_.getPitch());
    }

//    if (imuAvailable_) {
//
//    }

    // previous pose is replaced before pose prediction for linear interpolation
    prevMCLPose_ = mclPose_;

    // particles' poses are updated if the particle filter is used
    double xNoise = 0.0, yNoise = 0.0, zNoise = 0.0, rollNoise = 0.0, pitchNoise = 0.0, yawNoise = 0.0;
    if (localizationMode_ != 0) {
        std::vector<double> moves2(6);
        moves2[0] = dx * dx;
        moves2[1] = dy * dy;
        moves2[2] = dz * dz;
        moves2[3] = droll * droll;
        moves2[4] = dpitch * dpitch;
        moves2[5] = dyaw * dyaw;

        for (int i = 0; i < 6; ++i) {
            xNoise += odomNoise_[i] * moves2[i];
            yNoise += odomNoise_[6 + i] * moves2[i];
            zNoise += odomNoise_[12 + i] * moves2[i];
            rollNoise += odomNoise_[18 + i] * moves2[i];
            pitchNoise += odomNoise_[24 + i] * moves2[i];
            yawNoise += odomNoise_[30 + i] * moves2[i];
        }

        // particle poses are not predicted if localization mode is extended kalman filter
        if (localizationMode_ != 3) {
            for (int i = 0; i < particleNum_; ++i) {
                double x = particles_[i].getX() + dx + nrand(xNoise);
                double y = particles_[i].getY() + dy + nrand(yNoise);
                double z = particles_[i].getZ() + dz + nrand(zNoise);
                double roll = modAngle(particles_[i].getRoll() + droll + nrand(rollNoise));
                double pitch = modAngle(particles_[i].getPitch() + dpitch + nrand(pitchNoise));
                double yaw = modAngle(particles_[i].getYaw() + dyaw + nrand(yawNoise));
                particles_[i].setPose(x, y, z, roll, pitch, yaw);
            }
        }
    }

    // pose covariance is updated if localization mode is extended kalman filter
    if (localizationMode_ == 3) {
        if (odomAvailable_) {
            double t = mclPose_.getYaw() + dyaw / 2.0;
            double dd = odomVel_.getX() * dt;
            ekfPoseJacob_[0][5] = -dd * sin(t);
            ekfPoseJacob_[1][5] = dd * cos(t);
            if (useOmniModel_) {
                double ddl = odomVel_.getY() * dt;
                ekfPoseJacob_[0][5] += -ddl * sin(t + M_PI / 2.0);
                ekfPoseJacob_[1][5] += ddl * cos(t + M_PI / 2.0);
            }
        }
        std::vector<std::vector<double>> ekfPoseJacobT = transposeMatrix(ekfPoseJacob_, 6, 6);
        std::vector<std::vector<double>> poseCovMatTmp = multiplyMatrix(poseCov_, 6, 6, ekfPoseJacobT, 6, 6);
        poseCov_ = multiplyMatrix(ekfPoseJacob_, 6, 6, poseCovMatTmp, 6, 6);
        if (useLinearInterpolation_) {
            poseCov_[0][0] += xNoise;
            poseCov_[1][1] += yNoise;
            poseCov_[2][2] += zNoise;
            poseCov_[3][3] += rollNoise;
            poseCov_[4][4] += pitchNoise;
            poseCov_[5][5] += yawNoise;
        }
        printMatrixd(poseCov_, 6, 6, "pred pose covariance");
    }

    // update the MCL pose for the optimization-based and/or extended-kalman-filter-based localization
    // this will be used as the initial estimate for them
    if (localizationMode_ != 1) {
        double x = mclPose_.getX() + dx;
        double y = mclPose_.getY() + dy;
        double z = mclPose_.getZ() + dz;
        double roll = modAngle(mclPose_.getRoll() + droll);
        double pitch = modAngle(mclPose_.getPitch() + dpitch);
        double yaw = modAngle(mclPose_.getYaw() + dyaw);
        mclPose_.setPose(x, y, z, roll, pitch, yaw);
    }
}

void MCL::calculateLikelihoodsByMeasurementModel(pcl::PointCloud<pcl::PointXYZ>::Ptr sensorPoints) {
    // this function is only used if localization mode is particle filter
    if (localizationMode_ != 1)
        return;

    // sensor poses of each particle is first calculated
    std::vector<std::vector<double>> sensorPositions(particleNum_);
    std::vector<std::vector<double>> sensorRotMats(particleNum_);
    double bl2lx = baseLink2Laser_.getX();
    double bl2ly = baseLink2Laser_.getY();
    double bl2lz = baseLink2Laser_.getZ();
    for (int i = 0; i < particleNum_; ++i) {
        double cr, sr, cp, sp, cy, sy;
        getCosSin(particles_[i].getRoll(), particles_[i].getPitch(), particles_[i].getYaw(), &cr, &sr, &cp, &sp, &cy, &sy);
        std::vector<double> rotMat = getRotMat(cr, sr, cp, sp, cy, sy);
        double sensorX, sensorY, sensorZ;
        transformPoint(particles_[i].getX(), particles_[i].getY(), particles_[i].getZ(), rotMat,
            bl2lx, bl2ly, bl2lz, &sensorX, &sensorY, &sensorZ);
        std::vector<double> position = {sensorX, sensorY, sensorZ};
        sensorPositions[i] = position;
        sensorRotMats[i] = rotMat;
        particles_[i].setW(0.0);
    }

    // calculate likelihoods
    int scanStep = sensorPoints->points.size() / sensorPointsNum_;
    if (scanStep == 0)
        scanStep = 1;
    for (int i = 0; i < (int)sensorPoints->points.size(); i += scanStep) {
        double x = sensorPoints->points[i].x;
        double y = sensorPoints->points[i].y;
        double z = sensorPoints->points[i].z;
        double max;
        for (int j = 0; j < particleNum_; ++j) {
            double xx, yy, zz;
            transformPoint(sensorPositions[j][0], sensorPositions[j][1], sensorPositions[j][2],
                sensorRotMats[j], x, y, z, &xx, &yy, &zz);
            float d = distMap_.getDistance((float)xx, (float)yy, (float)zz);
            double p;
            if (measurementModelType_ == 2) {
                // likelihood field model
                if (d < 0.0f)
                    p = zRand_ * pRand_ * zMax_ + pMax_;
                else
                    p = hitNormConst_ * exp(-(d * d) / (2.0 * varHit_)) * rangeReso_ + zRand_ * pRand_;
            } else if (measurementModelType_ == 3) {
                // class conditional measurement model
                float r = sqrt(x * x + y * y + z * z);
                double pUnknown = unknownLambda_ * exp(-unknownLambda_ * r) / (1.0 - unknownLambda_ * rangeMax_);
                double pKnown = 0.0;
                if (d >= 0.0f)
                    pKnown = hitNormConst_ * exp(-(d * d) / (2.0 * varHit_));
                p = (pKnown + pUnknown) * rangeReso_ * 0.5;
            }
            double w = particles_[j].getW();
            w += log(p);
            particles_[j].setW(w);
            if (j == 0) {
                max = w;
            } else {
                if (max < w)
                    max = w;
            }
        }

        // Too small values cannot be calculated.
        // The log sum values are shifted if the maximum value is less than threshold.
        if (max < -400.0) {
            for (int j = 0; j < particleNum_; ++j) {
                double w = particles_[j].getW() + 400.0;
                particles_[j].setW(w);
            }
        }
    }

    double sum = 0.0;
    double max;
    int maxIdx;
    for (int i = 0; i < particleNum_; ++i) {
        // log sum is converted to the probability.
        double w = exp(particles_[i].getW());
        particles_[i].setW(w);
        sum += w;
        if (i == 0) {
            max = w;
            maxIdx = i;
        } else if (max < w) {
            max = w;
            maxIdx = i;
        }
    }
    totalLikelihood_ = sum;
    averageLikelihood_ = sum / (double)particleNum_;
    maxLikelihood_ = max;
    maxLikelihoodParticleIdx_ = maxIdx;

    // normalize particle weights and estimate mcl pose and effective sample size
    effectiveSampleSize_ = 0.0;
    double xSum = 0.0, ySum = 0.0, zSum = 0.0, drollSum = 0.0, dpitchSum = 0.0, dyawSum = 0.0;
    for (int i = 0; i < particleNum_; ++i) {
        double w = particles_[i].getW() / sum;
        particles_[i].setW(w);
        effectiveSampleSize_ += w * w;
        xSum += particles_[i].getX() * w;
        ySum += particles_[i].getY() * w;
        zSum += particles_[i].getZ() * w;
        double droll = modAngle(mclPose_.getRoll() - particles_[i].getRoll());
        drollSum += droll * w;
        double dpitch = modAngle(mclPose_.getPitch() - particles_[i].getPitch());
        dpitchSum += dpitch * w;
        double dyaw = modAngle(mclPose_.getYaw() - particles_[i].getYaw());
        dyawSum += dyaw * w;
    }
    effectiveSampleSize_ = 1.0 / effectiveSampleSize_;
    double roll = modAngle(mclPose_.getRoll() - drollSum);
    double pitch = modAngle(mclPose_.getPitch() - dpitchSum);
    double yaw = modAngle(mclPose_.getYaw() - dyawSum);
    mclPose_.setPose(xSum, ySum, zSum, roll, pitch, yaw);

    // calculate covariance from particle distribution
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j)
            poseCov_[i][j] = 0.0;
    }
    for (int i = 0; i < particleNum_; ++i) {
        double w = particles_[i].getW();
        std::vector<double> diffs(6);
        diffs[0] = xSum - particles_[i].getX();
        diffs[1] = ySum - particles_[i].getY();
        diffs[2] = zSum - particles_[i].getZ();
        diffs[3] = modAngle(roll - particles_[i].getRoll());
        diffs[4] = modAngle(pitch - particles_[i].getPitch());
        diffs[5] = modAngle(yaw - particles_[i].getYaw());
        for (int j = 0; j < 6; ++j) {
            for (int k = 0; k < 6; ++k)
                poseCov_[j][k] += w * diffs[j] * diffs[k];
        }
    }
}

std::vector<std::vector<double>> MCL::addMatrix(std::vector<std::vector<double>> m1, std::vector<std::vector<double>> m2, int row, int col) {
    std::vector<std::vector<double>> ret(row);
    for (int i = 0; i < row; ++i)
        ret[i].resize(col);

    for(int i = 0; i < row; i++) {
        for(int j = 0; j < col; j++)
            ret[i][j] = m1[i][j] + m2[i][j];
    }
    return ret;
}

std::vector<std::vector<double>> MCL::multiplyMatrix(std::vector<std::vector<double>> m1, int row1, int col1, std::vector<std::vector<double>> m2, int row2, int col2) {
    std::vector<std::vector<double>> ret(row1);
    for (int i = 0; i < row1; ++i)
        ret[i].resize(col2);

    for(int i = 0; i < row1; i++) {
        for(int j = 0; j < col2; j++) {
            for(int k = 0; k < col1; k++)
                ret[i][j] += m1[i][k] * m2[k][j];
        }
    }
    return ret;
}

std::vector<std::vector<double>> MCL::transposeMatrix(std::vector<std::vector<double>> m, int row, int col) {
    std::vector<std::vector<double>> ret(col);
    for (int i = 0; i < col; ++i)
        ret[i].resize(row);

    for (int i = 0; i < row; ++i) {
        for (int j = 0; j < col; ++j)
            ret[j][i] = m[i][j];
    }
    return ret;
}

std::vector<std::vector<double>> MCL::getInverseMatrix(std::vector<std::vector<double>> mat) {
    std::vector<std::vector<double>> inv(6);
    for (int i = 0; i < 6; ++i) {
        inv[i].resize(6);
        inv[i][i] = 1.0;
    }

    for (int i = 0; i < 6; ++i) {
        double a = 1.0 / mat[i][i];
        for (int j = 0; j < 6; ++j) {
            mat[i][j] *= a;
            inv[i][j] *= a;
        }
        for (int j = 0; j < 6; ++j) {
            if (i != j) {
                a = mat[j][i];
                for (int k = 0; k < 6; ++k) {
                    mat[j][k] -= mat[i][k] * a;
                    inv[j][k] -= inv[i][k] * a;
                }
            }
        }
    }

    return inv;
}

std::vector<std::vector<double>> MCL::CholeskyDecomposition(std::vector<std::vector<double>> mat) {
    std::vector<std::vector<double>> L(6);
    for (int i = 0; i < 6; ++i)
        L[i].resize(6, 0.0);

    L[0][0] = sqrt(mat[0][0]);
    for (int i = 1; i < 6; ++i) {
        for (int j = 0; j < i; ++j) {
            double lld = mat[i][j];
            for (int k = 0; k < j; ++k)
                lld -= L[i][k] * L[i][k];
            L[i][j] = lld / L[j][j];
        }
        double ld = mat[i][i];
        for (int k = 0; k < i; ++k)
            ld -= L[i][k] * L[i][k];
        L[i][i] = sqrt(ld);
        // to avoid invalid calculation
        if (std::isnan(L[i][i]))
            L[i][i] = 10e-3;
    }
    return L;
}

double MCL::getDeterminant(std::vector<std::vector<double>> mat, int n) {
    double det = 0;
    if (n == 1) {
        det = mat[0][0];
    } else if (n == 2) {
        det = mat[0][0] * mat[1][1] - mat[1][0] * mat[0][1];
    } else {
        for (int k = 0; k < n; ++k) {
            std::vector<std::vector<double>> submat(n);
            for (int i = 0; i < n; ++i)
                submat[i].resize(n);
            int si = 0, sj = 0;
            for (int i = 1; i < n; ++i) {
                for (int j = 0; j < n; ++j) {
                    if (j == k)
                        continue;
                    submat[si][sj] = mat[i][j];
                    sj++;
                    if (sj == n - 1) {
                        si++;
                        sj = 0;
                    }
                }
            }
            det += pow(-1, k) * mat[0][k] * getDeterminant(submat, n - 1);
        }
    }
    return det;
}

void MCL::printMatrixd(std::vector<std::vector<double>> m, int row, int col, std::string name) {
    for (int i = 0; i < row; ++i) {
        for (int j = 0; j < col; ++j)
            printf("%lf ", m[i][j]);
        printf("\n");
    }
    printf("%s\n\n", name.c_str());
}

double MCL::getError(float x, float y, float z, float r) {
    double d = (double)distMap_.getDistance(x, y, z);
    if (d < 0.0)
        return -1.0;

    if (measurementModelType_ == 0)
        return d;

    if (measurementModelType_ == 1) {
        double e = 1.0 - exp(-(d * d) / (2.0 * varHit_));
        // double e = -log(exp(-(d * d) / (2.0 * varHit_)));
        return e;
    } else if (measurementModelType_ == 2) {
        double lfm = 0.95 * hitNormConst_ * exp(-(d * d) / (2.0 * varHit_)) + 0.05 * 10e-6;
        double e = 1.0 - lfm / lfmMax_;
        // double e = -log(lfm / lfmMax);
        return e;
    } else {
        double known = hitNormConst_ * exp(-(d * d) / (2.0 * varHit_));
        double unknown = (unknownLambda_ * exp(-unknownLambda_ * r) / (1.0 - unknownLambda_ * rangeMax_));
        double ccmm = (known + unknown) * 0.5;
        double e = 1.0 - ccmm / ccmmMax_;
        // double e = -log(ccmm / ccmmMax_);
        return e;
    }
}

void MCL::optimizeMeasurementModel(pcl::PointCloud<pcl::PointXYZ>::Ptr sensorPoints) {
    // optimization is not performed if localization mode is particle filter
    if (localizationMode_ == 1)
        return;

    pcl::ApproximateVoxelGrid<pcl::PointXYZ> vgf;
    vgf.setLeafSize(voxelLeafSize_, voxelLeafSize_, voxelLeafSize_);
    vgf.setInputCloud(sensorPoints);
    vgf.filter(*filteredPoints_);

    double dTheta = 1.0 * M_PI / 180.0;
    int pointsSize = (int)filteredPoints_->points.size();

    double cr, sr, cp, sp, cy, sy;
    getCosSin(mclPose_.getRoll(), mclPose_.getPitch(), mclPose_.getYaw(), &cr, &sr, &cp, &sp, &cy, &sy);
    std::vector<double> rotMat = getRotMat(cr, sr, cp, sp, cy, sy);
    double sensorX, sensorY, sensorZ;
    transformPoint(mclPose_.getX(), mclPose_.getY(), mclPose_.getZ(), rotMat,
        baseLink2Laser_.getX(), baseLink2Laser_.getY(), baseLink2Laser_.getZ(), &sensorX, &sensorY, &sensorZ);
    double sensorRoll = modAngle(baseLink2Laser_.getRoll() + mclPose_.getRoll());
    double sensorPitch = modAngle(baseLink2Laser_.getPitch() + mclPose_.getPitch());
    double sensorYaw = modAngle(baseLink2Laser_.getYaw() + mclPose_.getYaw());

    optHasConverged_ = false;
    float reso = distMap_.getSubMapResolution();
    std::vector<std::vector<double>> es(pointsSize), prevEs(pointsSize);
    for (int i = 0; i < optMaxIterNum_; ++i) {
        getCosSin(sensorRoll, sensorPitch, sensorYaw, &cr, &sr, &cp, &sp, &cy, &sy);
        rotMat = getRotMat(cr, sr, cp, sp, cy, sy);
        for (int j = 0; j < pointsSize; ++j) {
            double x = filteredPoints_->points[j].x;
            double y = filteredPoints_->points[j].y;
            double z = filteredPoints_->points[j].z;
            float r = sqrt(x * x + y * y + z * z);
            double xx, yy, zz;
            transformPoint(sensorX, sensorY, sensorZ, rotMat, x, y, z, &xx, &yy, &zz);
            double error = getError((float)xx, (float)yy, (float)zz, r);
            es[j].resize(1);
            if (0.0 <= error && error <= optMaxError_)
                es[j][0] = error;
            else
                es[j][0] = -1.0;
        }

        std::vector<std::vector<double>> J, esUse;
        std::vector<double> tmpJ(6);
        for (int j = 0; j < pointsSize; ++j) {
            if (es[j][0] < 0.0)
                continue;

            double x_ = filteredPoints_->points[j].x;
            double y_ = filteredPoints_->points[j].y;
            double z_ = filteredPoints_->points[j].z;
            float r_ = sqrt(x_ * x_ + y_ * y_ + z_ * z_);

            double xx_, yy_, zz_;
            double cr_, sr_, cp_, sp_, cy_, sy_;
            std::vector<double> rotMat_;
            double error_;
            double sum = 0.0;

            getCosSin(sensorRoll, sensorPitch, sensorYaw, &cr_, &sr_, &cp_, &sp_, &cy_, &sy_);
            rotMat_ = getRotMat(cr_, sr_, cp_, sp_, cy_, sy_);
            transformPoint(sensorX + reso, sensorY, sensorZ, rotMat_, x_, y_ ,z_, &xx_, &yy_, &zz_);
            error_ = getError((float)xx_, (float)yy_, (float)zz_, r_);
            if (0.0 <= error_) {
                tmpJ[0] = (error_ - es[j][0]) / reso;
                sum += fabs(tmpJ[0]);
            } else {
                tmpJ[0] = 0.0;
            }

            xx_ -= reso;
            yy_ += reso;
            error_ = getError((float)xx_, (float)yy_, (float)zz_, r_);
            if (0.0 <= error_) {
                tmpJ[1] = (error_ - es[j][0]) / reso;
                sum += fabs(tmpJ[1]);
            } else {
                tmpJ[1] = 0.0;
            }

            yy_ -= reso;
            zz_ += reso;
            error_ = getError((float)xx_, (float)yy_, (float)zz_, r_);
            if (0.0 <= error_) {
                tmpJ[2] = (error_ - es[j][0]) / reso;
                sum += fabs(tmpJ[2]);
            } else {
                tmpJ[2] = 0.0;
            }

            cr_ = cos(sensorRoll + dTheta);
            sr_ = sin(sensorRoll + dTheta);
            rotMat_ = getRotMat(cr_, sr_, cp_, sp_, cy_, sy_);
            transformPoint(sensorX, sensorY, sensorZ, rotMat_, x_, y_ ,z_, &xx_, &yy_, &zz_);
            error_ = getError((float)xx_, (float)yy_, (float)zz_, r_);
            if (0.0 <= error_) {
                tmpJ[3] = (error_ - es[j][0]) / dTheta;
                sum += fabs(tmpJ[3]);
            } else {
                tmpJ[3] = 0.0;
            }

            cr_ = cos(sensorRoll);
            sr_ = sin(sensorRoll);
            cp_ = cos(sensorPitch + dTheta);
            sp_ = sin(sensorPitch + dTheta);
            rotMat_ = getRotMat(cr_, sr_, cp_, sp_, cy_, sy_);
            transformPoint(sensorX, sensorY, sensorZ, rotMat_, x_, y_ ,z_, &xx_, &yy_, &zz_);
            error_ = getError((float)xx_, (float)yy_, (float)zz_, r_);
            if (0.0 <= error_) {
                tmpJ[4] = (error_ - es[j][0]) / dTheta;
                sum += fabs(tmpJ[4]);
            } else {
                tmpJ[4] = 0.0;
            }

            sp_ = sin(sensorPitch);
            cy_ = cos(sensorYaw + dTheta);
            sy_ = sin(sensorYaw + dTheta);
            rotMat_ = getRotMat(cr_, sr_, cp_, sp_, cy_, sy_);
            transformPoint(sensorX, sensorY, sensorZ, rotMat_, x_, y_ ,z_, &xx_, &yy_, &zz_);
            error_ = getError((float)xx_, (float)yy_, (float)zz_, r_);
            if (0.0 <= error_) {
                tmpJ[5] = (error_ - es[j][0]) / dTheta;
                sum += fabs(tmpJ[5]);
            } else {
                tmpJ[5] = 0.0;
            }

            if (sum > 10e-10) {
                J.push_back(tmpJ);
                esUse.push_back(es[j]);
            }
        }

        int effectivePointsSize = (int)J.size();
        std::vector<std::vector<double>> JT = transposeMatrix(J, effectivePointsSize, 6);
        std::vector<std::vector<double>> JTJ = multiplyMatrix(JT, 6, effectivePointsSize, J, effectivePointsSize, 6);
        std::vector<std::vector<double>> JTJInv = getInverseMatrix(JTJ);
        if (std::isnan(JTJInv[0][0])) {
            fprintf(stderr, "NaN values are detected in the inverse matrix calculation process.\n");
            break;
        }
        std::vector<std::vector<double>> Je = multiplyMatrix(JT, 6, effectivePointsSize, esUse, effectivePointsSize, 1);
        std::vector<std::vector<double>> g = multiplyMatrix(JTJInv, 6, 6, Je, 6, 1);
        double sum = g[0][0] * g[0][0] + g[1][0] * g[1][0] + g[2][0] * g[2][0];
                   + g[3][0] * g[3][0] + g[4][0] * g[4][0] + g[5][0] * g[5][0];
        if (std::isnan(sum))
            break;
        if (sum > 1.0)
            break;
        sensorX -= g[0][0];
        sensorY -= g[1][0];
        sensorZ -= g[2][0];
        sensorRoll = modAngle(sensorRoll - g[3][0]);
        sensorPitch = modAngle(sensorPitch - g[4][0]);
        sensorYaw = modAngle(sensorYaw - g[5][0]);
        if (i >= 1) {
            double sum = 0.0;
            for (int j = 0; j < pointsSize; ++j) {
                if (es[j][0] >= 0.0 && prevEs[j][0] >= 0.0)
                    sum += fabs(es[j][0] - prevEs[j][0]);
            }
            double ave = sum / (double)pointsSize;
            // printf("i = %d, sum = %lf, ave = %lf\n", i, sum, ave);
            if (ave < convergenceThreshold_) {
                optHasConverged_ = true;
                approximateHessian_ = JTJ;
                break;
            }
            if (i == optMaxIterNum_ - 1)
                fprintf(stderr, "Scan matching has not been converded. The average error is %lf\n.", ave);
        }
        prevEs = es;
    }

    if (!optHasConverged_) {
        fprintf(stderr, "Scan matching has not been converged.\n");
        return;
    }

    getCosSin(sensorRoll, sensorPitch, sensorYaw, &cr, &sr, &cp, &sp, &cy, &sy);
    rotMat = getRotMat(cr, sr, cp, sp, cy, sy);
    double optPoseTmpX, optPoseTmpY, optPoseTmpZ;
    transformPoint(0.0, 0.0, 0.0, rotMat, baseLink2Laser_.getX(), baseLink2Laser_.getY(), baseLink2Laser_.getZ(),
        &optPoseTmpX, &optPoseTmpY, &optPoseTmpZ);

    double optPoseX = sensorX - optPoseTmpX;
    double optPoseY = sensorY - optPoseTmpY;
    double optPoseZ = sensorZ - optPoseTmpZ;
    double optPoseRoll = modAngle(sensorRoll - baseLink2Laser_.getRoll());
    double optPosePitch = modAngle(sensorPitch - baseLink2Laser_.getPitch());
    double optPoseYaw = modAngle(sensorYaw - baseLink2Laser_.getYaw());
    optPose_.setPose(optPoseX, optPoseY, optPoseZ, optPoseRoll, optPosePitch, optPoseYaw);

    // determine covariane matrix against the optimization result with the approximated Hessian by Jacobian
    optPoseCov_ = getInverseMatrix(approximateHessian_);
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j)
            optPoseCov_[i][j] *= optPoseCovCoef_;
    }

    // poes estimate result is replaced as the optimization result if localization mode is optimization
    // particles will be drawn if localization mode is particle-filter-based fusion
    if (localizationMode_ == 0) {
        mclPose_ = optPose_;
        poseCov_ = optPoseCov_;
        return;
    }

    // fusion with extended kalman filter
    if (localizationMode_ == 3) {
        std::vector<std::vector<double>> kgTmp = addMatrix(poseCov_, optPoseCov_, 6, 6);
        std::vector<std::vector<double>> kgTmpInv = getInverseMatrix(kgTmp);
        std::vector<std::vector<double>> kg = multiplyMatrix(poseCov_, 6, 6, kgTmpInv, 6, 6);

        std::vector<std::vector<double>> inov(6), covTmp(6);
        for (int i = 0; i < 6; ++i) {
            inov[i].resize(1);
            covTmp[i].resize(6);
        }
        inov[0][0] = optPose_.getX() - mclPose_.getX();
        inov[1][0] = optPose_.getY() - mclPose_.getY();
        inov[2][0] = optPose_.getZ() - mclPose_.getZ();
        inov[3][0] = modAngle(optPose_.getRoll() - mclPose_.getRoll());
        inov[4][0] = modAngle(optPose_.getPitch() - mclPose_.getPitch());
        inov[5][0] = modAngle(optPose_.getYaw() - mclPose_.getYaw());

        std::vector<std::vector<double>> poseTmp = multiplyMatrix(kg, 6, 6, inov, 6, 1);
        double x = mclPose_.getX() + poseTmp[0][0];
        double y = mclPose_.getY() + poseTmp[1][0];
        double z = mclPose_.getZ() + poseTmp[2][0];
        double roll = modAngle(mclPose_.getRoll() + poseTmp[3][0]);
        double pitch = modAngle(mclPose_.getPitch() + poseTmp[4][0]);
        double yaw = modAngle(mclPose_.getYaw() + poseTmp[5][0]);
        mclPose_.setPose(x, y, z, roll, pitch, yaw);
        // printf("%lf %lf %lf %lf %lf %lf\n", poseTmp[0][0], poseTmp[1][0], poseTmp[2][0],
        //     poseTmp[3][0], poseTmp[4][0], poseTmp[5][0]);

        double poseCovTrace = 0.0;
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                if (i == j) {
                    covTmp[i][j] = 1.0 - kg[i][j];
                    poseCovTrace += poseCov_[i][j];
                } else {
                    covTmp[i][j] = -kg[i][j];
                }
            }
        }
        if (poseCovTrace > 0.1)
            poseCov_ = multiplyMatrix(covTmp, 6, 6, poseCov_, 6, 6);
        return;
    }

    // calculate likelihoods of standard particles by the approximated measurement model with the normal distribution
    // double normConstStd = 1.0 / (pow(sqrt(2.0 * M_PI), 6) * sqrt(pow(optVarPos, 3) * pow(optVarAng, 3)));
    double det = getDeterminant(optPoseCov_, 6);
    double normConstStd = 1.0 / sqrt((pow(2.0 * M_PI, 6) * det));
    double sum = 0.0;
    double minProb = 10e-30;
    std::vector<std::vector<double>> del(6);
    for (int i = 0; i < 6; ++i)
        del[i].resize(1);
    std::vector<std::vector<double>> optPoseCovInv = getInverseMatrix(optPoseCov_);
    for (int i = 0; i < particleNum_; ++i) {
        del[0][0] = particles_[i].getX() - optPose_.getX();
        del[1][0] = particles_[i].getY() - optPose_.getY();
        del[2][0] = particles_[i].getZ() - optPose_.getZ();
        del[3][0] = modAngle(particles_[i].getRoll() - optPose_.getRoll());
        del[4][0] = modAngle(particles_[i].getPitch() - optPose_.getPitch());
        del[5][0] = modAngle(particles_[i].getYaw() - optPose_.getYaw());
        std::vector<std::vector<double>> delT = transposeMatrix(del, 6, 1);
        std::vector<std::vector<double>> distTmp = multiplyMatrix(optPoseCovInv, 6, 6, del, 6, 1);
        std::vector<std::vector<double>> dist = multiplyMatrix(delT, 1, 6, distTmp, 6, 1);
        double w = normConstStd * exp(-dist[0][0]) + minProb;
        particles_[i].setW(w);
        sum += w;
    }

    // sampling from the measurement model
    std::vector<std::vector<double>> L = CholeskyDecomposition(optPoseCov_);
    double normConstGMM = 1.0 / (pow(sqrt(2.0 * M_PI), 6) * sqrt(pow(gmmPosVar_, 3) * pow(gmmAngVar_, 3)));
    std::vector<std::vector<double>> rand(6);
    for (int i = 0; i < 6; ++i)
        rand[i].resize(1);

    for (int i = 0; i < optParticleNum_; ++i) {
        for (int j = 0; j < 6; ++j)
            rand[j][0] = nrand(1.0);
        std::vector<std::vector<double>> sampledPose = multiplyMatrix(L, 6, 6, rand, 6, 1);
        double x = optPose_.getX() + sampledPose[0][0];
        double y = optPose_.getY() + sampledPose[1][0];
        double z = optPose_.getZ() + sampledPose[2][0];
        double roll = modAngle(optPose_.getRoll() + sampledPose[3][0]);
        double pitch = modAngle(optPose_.getPitch() + sampledPose[4][0]);
        double yaw = modAngle(optPose_.getYaw() + sampledPose[5][0]);
        optParticles_[i].setPose(x, y, z, roll, pitch, yaw);

        double gmmSum = 0.0;
        for (int j = 0; j < particleNum_; ++j) {
            double dx = x - particles_[j].getX();
            double dy = y - particles_[j].getY();
            double dz = z - particles_[j].getZ();
            double droll = modAngle(roll - particles_[j].getRoll());
            double dpitch = modAngle(pitch - particles_[j].getPitch());
            double dyaw = modAngle(yaw - particles_[j].getYaw());
            double d = (dx * dx + dy * dy + dz * dz) / gmmPosVar_ + (droll * droll + dpitch * dpitch + dyaw * dyaw) / gmmAngVar_;
            gmmSum += exp(-d * d * 0.5);
        }
        double w = normConstGMM * gmmSum / (double)particleNum_ + minProb;
        optParticles_[i].setW(w);
    }
}

void MCL::resampleParticles1(void) {
    // this function is used only when localization mode is particle filter
    if (localizationMode_ != 1)
        return;

    if (effectiveSampleSize_ > particleNum_ * resampleThreshold_)
        return;

    std::vector<double> wBuffer(particleNum_);
    wBuffer[0] = particles_[0].getW();
    for (int i = 1; i < particleNum_; ++i)
        wBuffer[i] = particles_[i].getW() + wBuffer[i - 1];

    int resampledParticleNum = particleNum_ * (1.0 - randomParticleRate_);
    int randomParticleNum = particleNum_ - resampledParticleNum;
    double wo = 1.0 / (double)particleNum_;
    std::vector<Particle> tmpParticles = particles_;
    for (int i = 0; i < resampledParticleNum; ++i) {
        double darts = (double)rand() / ((double)RAND_MAX + 1.0);
        for (int j = 0; j < particleNum_; ++j) {
            if (darts < wBuffer[j]) {
                double x = tmpParticles[j].getX();
                double y = tmpParticles[j].getY();
                double z = tmpParticles[j].getZ();
                double roll = tmpParticles[j].getRoll();
                double pitch = tmpParticles[j].getPitch();
                double yaw = tmpParticles[j].getYaw();
                particles_[i].setParticle(x, y, z, roll, pitch, yaw, wo);
                break;
            }
        }
    }   

    for (int i = resampledParticleNum; i < resampledParticleNum + randomParticleNum; ++i) {
        double x = mclPose_.getX() + nrand(resampleNoise_.getX());
        double y = mclPose_.getY() + nrand(resampleNoise_.getY());
        double z = mclPose_.getZ() + nrand(resampleNoise_.getZ());
        double roll = modAngle(mclPose_.getRoll() + nrand(resampleNoise_.getRoll()));
        double pitch = modAngle(mclPose_.getPitch() + nrand(resampleNoise_.getPitch()));
        double yaw = modAngle(mclPose_.getYaw() + nrand(resampleNoise_.getYaw()));
        particles_[i].setParticle(x, y, z, roll, pitch, yaw, wo);
    }
}

void MCL::resampleParticles2(void) {
    // this function is used only when localization mode is particle-filter-based fusion
    if (localizationMode_ != 2)
        return;

    // pose estimate and re-samping are not performed if scan matching has failed
    if (!optHasConverged_)
        return;

    // normalize weight
    double sum = 0.0;
    for (int i = 0; i < particleNum_; ++i)
        sum += particles_[i].getW();
    for (int i = 0; i < optParticleNum_; ++i)
        sum += optParticles_[i].getW();

    // calculate effective sample size and estimate pose
    effectiveSampleSize_ = 0.0;
    double xSum = 0.0, ySum = 0.0, zSum = 0.0, drollSum = 0.0, dpitchSum = 0.0, dyawSum = 0.0;    
    for (int i = 0; i < particleNum_; ++i) {
        double w = particles_[i].getW() / sum;
        particles_[i].setW(w);
        effectiveSampleSize_ += w * w;
        xSum += particles_[i].getX() * w;
        ySum += particles_[i].getY() * w;
        zSum += particles_[i].getZ() * w;
        double droll = modAngle(mclPose_.getRoll() - particles_[i].getRoll());
        drollSum += droll * w;
        double dpitch = modAngle(mclPose_.getPitch() - particles_[i].getPitch());
        dpitchSum += dpitch * w;
        double dyaw = modAngle(mclPose_.getYaw() - particles_[i].getYaw());
        dyawSum += dyaw * w;
    }
    for (int i = 0; i < optParticleNum_; ++i) {
        double w = optParticles_[i].getW() / sum;
        optParticles_[i].setW(w);
        effectiveSampleSize_ += w * w;
        xSum += optParticles_[i].getX() * w;
        ySum += optParticles_[i].getY() * w;
        zSum += optParticles_[i].getZ() * w;
        double droll = modAngle(mclPose_.getRoll() - optParticles_[i].getRoll());
        drollSum += droll * w;
        double dpitch = modAngle(mclPose_.getPitch() - optParticles_[i].getPitch());
        dpitchSum += dpitch * w;
        double dyaw = modAngle(mclPose_.getYaw() - optParticles_[i].getYaw());
        dyawSum += dyaw * w;
    }
    effectiveSampleSize_ = 1.0 / effectiveSampleSize_;
    double roll = modAngle(mclPose_.getRoll() - drollSum);
    double pitch = modAngle(mclPose_.getPitch() - dpitchSum);
    double yaw = modAngle(mclPose_.getYaw() - dyawSum);
    mclPose_.setPose(xSum, ySum, zSum, roll, pitch, yaw);

    // calculate covariance from particle distribution
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j)
            poseCov_[i][j] = 0.0;
    }
    for (int i = 0; i < particleNum_; ++i) {
        double w = particles_[i].getW();
        std::vector<double> diffs(6);
        diffs[0] = xSum - particles_[i].getX();
        diffs[1] = ySum - particles_[i].getY();
        diffs[2] = zSum - particles_[i].getZ();
        diffs[3] = modAngle(roll - particles_[i].getRoll());
        diffs[4] = modAngle(pitch - particles_[i].getPitch());
        diffs[5] = modAngle(yaw - particles_[i].getYaw());
        for (int j = 0; j < 6; ++j) {
            for (int k = 0; k < 6; ++k)
                poseCov_[j][k] += w * diffs[j] * diffs[k];
        }
    }
    for (int i = 0; i < optParticleNum_; ++i) {
        double w = optParticles_[i].getW();
        std::vector<double> diffs(6);
        diffs[0] = xSum - optParticles_[i].getX();
        diffs[1] = ySum - optParticles_[i].getY();
        diffs[2] = zSum - optParticles_[i].getZ();
        diffs[3] = modAngle(roll - optParticles_[i].getRoll());
        diffs[4] = modAngle(pitch - optParticles_[i].getPitch());
        diffs[5] = modAngle(yaw - optParticles_[i].getYaw());
        for (int j = 0; j < 6; ++j) {
            for (int k = 0; k < 6; ++k)
                poseCov_[j][k] += w * diffs[j] * diffs[k];
        }
    }

    int totalParticleNum = particleNum_ + optParticleNum_;
    if (effectiveSampleSize_ > totalParticleNum * resampleThreshold_)
        return;

    std::vector<double> wBuffer(totalParticleNum);
    wBuffer[0] = particles_[0].getW();
    for (int i = 1; i < particleNum_; ++i)
        wBuffer[i] = particles_[i].getW() + wBuffer[i - 1];
    for (int i = particleNum_; i < totalParticleNum; ++i)
        wBuffer[i] = optParticles_[i - particleNum_].getW() + wBuffer[i - 1];

    int resampledParticleNum = particleNum_ * (1.0 - randomParticleRate_);
    int randomParticleNum = particleNum_ - resampledParticleNum;
    double wo = 1.0 / (double)particleNum_;
    std::vector<Particle> tmpParticles = particles_;
    std::vector<Particle> tmpOptParticles = optParticles_;
    int stdParNum = 0, optParNum = 0, rndParNum = 0;
    for (int i = 0; i < resampledParticleNum; ++i) {
        double darts = (double)rand() / ((double)RAND_MAX + 1.0);
        bool resampled = false;
        for (int j = 0; j < particleNum_; ++j) {
            if (darts < wBuffer[j]) {
                double x = tmpParticles[j].getX();
                double y = tmpParticles[j].getY();
                double z = tmpParticles[j].getZ();
                double roll = tmpParticles[j].getRoll();
                double pitch = tmpParticles[j].getPitch();
                double yaw = tmpParticles[j].getYaw();
                particles_[i].setParticle(x, y, z, roll, pitch, yaw, wo);
                resampled = true;
                stdParNum++;
                break;
            }
        }

        if (!resampled) {
            for (int j = 0; j < optParticleNum_; ++j) {
                if (darts < wBuffer[particleNum_ + j]) {
                    double x = tmpOptParticles[j].getX();
                    double y = tmpOptParticles[j].getY();
                    double z = tmpOptParticles[j].getZ();
                    double roll = tmpOptParticles[j].getRoll();
                    double pitch = tmpOptParticles[j].getPitch();
                    double yaw = tmpOptParticles[j].getYaw();
                    particles_[i].setParticle(x, y, z, roll, pitch, yaw, wo);
                    resampled = true;
                    optParNum++;
                    break;
                }
            }
        }
    }

    for (int i = resampledParticleNum; i < resampledParticleNum + randomParticleNum; ++i) {
        double x = mclPose_.getX() + nrand(resampleNoise_.getX());
        double y = mclPose_.getY() + nrand(resampleNoise_.getY());
        double z = mclPose_.getZ() + nrand(resampleNoise_.getZ());
        double roll = modAngle(mclPose_.getRoll() + nrand(resampleNoise_.getRoll()));
        double pitch = modAngle(mclPose_.getPitch() + nrand(resampleNoise_.getPitch()));
        double yaw = modAngle(mclPose_.getYaw() + nrand(resampleNoise_.getYaw()));
        particles_[i].setParticle(x, y, z, roll, pitch, yaw, wo);
        rndParNum++;
    }

    // printf("from resample2: resample rates: std = %.2lf, opt = %.2lf, rnd = %.2lf\n",
    //     (double)stdParNum / (double)particleNum_ * 100.0, (double)optParNum / (double)particleNum_ * 100.0, (double)rndParNum / (double)particleNum_ * 100.0);
}

} // namespace mcl3d