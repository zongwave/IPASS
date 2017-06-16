// Digital Video Stabilization using Gyroscope & Accelerometer (6-DOF)
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// Copyright (C) 2017 Zong Wei <zongwave@hotmail.com>
//

#ifndef CALCULATE_PROJECTIVE_2D_H
#define CALCULATE_PROJECTIVE_2D_H

#include <vector>

#include "quaternion.h"

typedef Vector2<double> Vec2;
typedef Vector3<double> Vec3;
typedef Vector4<double> Vec4;
typedef Matrix3<double> Mat3;
typedef Matrix4<double> Mat4;
typedef Quaternion<double> Quatern;

struct CalibrationParams {
    double fx;  //Focal length, x axis, in pixels
    double fy;  //Focal length, y axis, in pixels
    double cx;  //Principal point x coordinate on the image, in pixels
    double cy;  //Principal point y coordinate on the image, in pixels
    double skew; //in case if the image coordinate axes u and v are not orthogonal to each other
    Vec4 gyro_drift;
    double gyro_delay;
    double readout_time;
};

void calc_projective2d(Vec4* gyro_quat,
                       Vec3* acc_trans,
                       size_t num_gyro_samp,
                       double* time_stamp,
                       double* frame_time,
                       CalibrationParams calib,
                       std::vector<Mat3>& projective);

Mat3 calc_projective2d(Vec4* gyro_quat,
                       Vec3* acc_trans,
                       double* time_stamp,
                       double frame_time,
                       CalibrationParams calib);

#endif
