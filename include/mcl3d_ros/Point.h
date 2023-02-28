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

#ifndef __POINT_H__
#define __POINT_H__

namespace mcl3d {

class Point {
private:
    double x_, y_, z_;

public:
    Point(void): x_(0.0), y_(0.0), z_(0.0) {}
    Point(double x, double y, double z): x_(x), y_(y), z_(z) {}

    inline double getX(void) { return x_; }
    inline double getY(void) { return y_; }
    inline double getZ(void) { return z_; }
    inline Point getPoint(void) { return Point(x_, y_, z_); }

    inline void setX(double x) { x_ = x; }
    inline void setY(double y) { y_ = y; }
    inline void setZ(double z) { z_ = z; }
    inline void setPoint(double x, double y, double z) { x_ = x, y_ = y, z_ = z; }
}; // class Point

} // namespace mcl3d

#endif // __POINT_H__