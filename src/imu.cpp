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

#include <mcl3d_ros/IMU.h>

namespace mcl3d {

void IMU::init(void) {
    q0_ = 1.0;
    q1_ = 0.0;
    q2_ = 0.0;
    q3_ = 0.0;

    roll_ = 0.0;
    pitch_ = 0.0;
    yaw_ = 0.0;

    twoKp_ = 0.0;
    twoKi_ = 0.0;

    integralFBx_ = 0.0;
    integralFBy_ = 0.0;
    integralFBz_ = 0.0;

    sampleFreq_ = 100.0;
}

void IMU::printIMUResult(void) {
    printf("sampleFreq_ = %lf\n", sampleFreq_);
    printf("ax = %lf, ay = %lf, az = %lf\n", ax_, ay_, az_);
    printf("gx = %lf, gy = %lf, gz = %lf\n", gx_, gy_, gz_);
    printf("roll = %lf [deg], pitch = %lf [deg], yaw = %lf [deg]\n",
        roll_ * 180.0 / M_PI, pitch_ * 180.0 / M_PI, yaw_ * 180.0 / M_PI);
    printf("q0 = %lf, q1 = %lf, q2 = %lf, q3 = %lf\n", q0_, q1_, q2_, q3_);
    printf("\n");
}

void IMU::updateOrientation(void) {
    if (ax_ != 0.0 || ay_ != 0.0 || az_ != 0.0) {
        double recipNorm = invSqrt(ax_ * ax_ + ay_ * ay_ + az_ * az_);
        ax_ *= recipNorm;
        ay_ *= recipNorm;
        az_ *= recipNorm;

        double halfvx = q1_ * q3_ - q0_ * q2_;
        double halfvy = q0_ * q1_ + q2_ * q3_;
        double halfvz = q0_ * q0_ - 0.5 + q3_ * q3_;

        double halfex = (ay_ * halfvz - az_ * halfvy);
        double halfey = (az_ * halfvx - ax_ * halfvz);
        double halfez = (ax_ * halfvy - ay_ * halfvx);

        if (twoKi_ > 0.0) {
            integralFBx_ += twoKi_ * halfex * (1.0 / sampleFreq_);
            integralFBy_ += twoKi_ * halfey * (1.0 / sampleFreq_);
            integralFBz_ += twoKi_ * halfez * (1.0 / sampleFreq_);
            gx_ += integralFBx_;
            gy_ += integralFBy_;
            gz_ += integralFBz_;
        } else {
            integralFBx_ = 0.0;
            integralFBy_ = 0.0;
            integralFBz_ = 0.0;
        }

        gx_ += twoKp_ * halfex;
        gy_ += twoKp_ * halfey;
        gz_ += twoKp_ * halfez;
    }

    gx_ *= (0.5 * (1.0 / sampleFreq_));
    gy_ *= (0.5 * (1.0 / sampleFreq_));
    gz_ *= (0.5 * (1.0 / sampleFreq_));
    double qa = q0_;
    double qb = q1_;
    double qc = q2_;
    q0_ += (-qb * gx_ - qc * gy_ - q3_ * gz_);
    q1_ += (qa * gx_ + qc * gz_ - q3_ * gy_);
    q2_ += (qa * gy_ - qb * gz_ + q3_ * gx_);
    q3_ += (qa * gz_ + qb * gy_ - qc * gx_);

    double recipNorm = invSqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
    q0_ *= recipNorm;
    q1_ *= recipNorm;
    q2_ *= recipNorm;
    q3_ *= recipNorm;

    double q0q0 = q0_ * q0_;
    double q1q1 = q1_ * q1_;
    double q2q2 = q2_ * q2_;
    double q3q3 = q3_ * q3_;
    double q0q1 = q0_ * q1_;
    double q0q2 = q0_ * q2_;
    double q0q3 = q0_ * q3_;
    double q1q2 = q1_ * q2_;
    double q1q3 = q1_ * q3_;
    double q2q3 = q2_ * q3_;
    roll_ = atan2((2.0 * (q2q3 + q0q1)), (q0q0 - q1q1 - q2q2 + q3q3));
    pitch_ = -asin((2.0 * (q1q3 - q0q2)));
    yaw_ = atan2((2.0 * (q1q2 + q0q3)), (q0q0 + q1q1 - q2q2 - q3q3));
}

} // namespace mcl3d