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

#ifndef __IMU_H__
#define __IMU_H__

#include <iostream>
#include <cmath>

namespace mcl3d {

class IMU {
private:
    double ax_, ay_, az_, gx_, gy_, gz_;

    double q0_, q1_, q2_, q3_;
    double roll_, pitch_, yaw_;

    double sampleFreq_;
    double twoKp_, twoKi_;
    double integralFBx_, integralFBy_, integralFBz_;

public:
    IMU(void): roll_(0.0), pitch_(0.0), yaw_(0.0), ax_(0.0), ay_(0.0), az_(0.0), gx_(0.0), gy_(0.0), gz_(0.0) {}
    IMU(double roll, double pitch, double yaw, double ax, double ay, double az, double gx, double gy, double gz):
        roll_(roll), pitch_(pitch), yaw_(yaw), ax_(ax), ay_(ay), az_(az), gx_(gx), gy_(gy), gz_(gz) {}

    void init(void);
    void printIMUResult(void);
    void updateOrientation(void);

    // inline functions
    inline double getRoll(void) { return roll_; }
    inline double getPitch(void) { return pitch_; }
    inline double getYaw(void) { return yaw_; }
    inline double getAx(void) { return ax_; }
    inline double getAy(void) { return ay_; }
    inline double getAz(void) { return az_; }
    inline double getGx(void) { return gx_; }
    inline double getGy(void) { return gy_; }
    inline double getGz(void) { return gz_; }
    inline void getRPY(double *roll, double *pitch, double *yaw) { *roll = roll_, *pitch = pitch_, *yaw = yaw_; }
    inline void getAcceleration(double *ax, double *ay, double *az) { *ax = ax_, *ay = ay_, *az = az_; }
    inline void getAngularVelocities(double *gx, double *gy, double *gz) { *gx = gx_, *gy = gy_, *gz = gz_; }

    inline void setRoll(double roll) { roll_ = roll; }
    inline void setPitch(double pitch) { pitch_ = pitch; }
    inline void setYaw(double yaw) { yaw_ = yaw; }
    inline void setAx(double ax) { ax_ = ax; }
    inline void setAy(double ay) { ay_ = ay; }
    inline void setAz(double az) { az_ = az; }
    inline void setGx(double gx) { gx_ = gx; }
    inline void setGy(double gy) { gy_ = gy; }
    inline void setGz(double gz) { gz_ = gz; }
    inline void setRPY(double roll, double pitch, double yaw) { roll_ = roll, pitch_ = pitch, yaw_ = yaw; }
    inline void setAcceleration(double ax, double ay, double az) { ax_ = ax, ay_ = ay, az_ = az; }
    inline void setAngularVelocities(double gx, double gy, double gz) { gx_ = gx, gy_ = gy, gz_ = gz; }
    inline void setSampleFreq(double sampleFreq) { sampleFreq_ = sampleFreq; }
    inline void setAHRSFilterGains(double kp, double ki) { twoKp_ = kp, twoKi_ = ki; }

    inline double invSqrt(double x) { return 1.0 / sqrt(x); }
}; // class IMU

} // namespace mcl3d

#endif // __IMU_H__