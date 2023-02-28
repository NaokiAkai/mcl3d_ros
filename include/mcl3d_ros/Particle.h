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

#ifndef __PARTICLE_H__
#define __PARTICLE_H__

#include <mcl3d_ros/Pose.h>

namespace mcl3d {

class Particle {
    Pose pose_;
    double w_;

public:
    Particle(void): pose_(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), w_(0.0) {}
    Particle(double x, double y, double z, double roll, double pitch, double yaw, double w):
        pose_(x, y, z, roll, pitch, yaw), w_(w) {}

    inline double getX(void) { return pose_.getX(); }
    inline double getY(void) { return pose_.getY(); }
    inline double getZ(void) { return pose_.getZ(); }
    inline double getRoll(void) { return pose_.getRoll(); }
    inline double getPitch(void) { return pose_.getPitch(); }
    inline double getYaw(void) { return pose_.getYaw(); }
    inline Pose getPose(void) { return pose_; }
    inline double getW(void) { return w_; }
    inline Particle getParticle(void) {
        return Particle(pose_.getX(), pose_.getY(), pose_.getZ(), pose_.getRoll(), pose_.getPitch(), pose_.getYaw(), w_);
    }

    inline void setX(double x) { pose_.setX(x); }
    inline void setY(double y) { pose_.setY(y); }
    inline void setZ(double z) { pose_.setZ(z); }
    inline void setRoll(double roll) { pose_.setRoll(roll); }
    inline void setPitch(double pitch) { pose_.setX(pitch); }
    inline void setYaw(double yaw) { pose_.setYaw(yaw); }
    inline void setPose(double x, double y, double z, double roll, double pitch, double yaw) {
        pose_.setPose(x, y, z, roll, pitch, yaw);
    }
    inline void setW(double w) { w_ = w; }
    inline void setParticle(double x, double y, double z, double roll, double pitch, double yaw, double w) {
        pose_.setPose(x, y, z, roll, pitch, yaw);
        w_ = w;
    }
}; // class Particle

} // namespace mcl3d

#endif // __PARTICLE_H__