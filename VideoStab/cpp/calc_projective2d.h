/*
 * cal_projective2d.h - Calculate 2D image projective matrix
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Author: Zong Wei <wei.zong@intel.com>
 */

#ifndef CALCULATE_PROJECTIVE_2D_H
#define CALCULATE_PROJECTIVE_2D_H

#include <vector>

#include "quaternion.h"

#define PRINT_MATRIX 0

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

void calc_projective (const std::vector<double>& frame_ts,
                      const std::vector<Vec4>& gyro_quat,
                      const std::vector<Vec3>& acc_trans,
                      const std::vector<double>& gyro_ts,
                      CalibrationParams calib,
                      std::vector<Mat3>& projective);

void calc_projective (const std::vector<double>& frame_ts,
                      const std::vector<Vec4>& gyro_quat,
                      const std::vector<Vec3>& acc_trans,
                      const std::vector<double>& gyro_ts,
                      CalibrationParams calib,
                      std::vector<Mat4>& projective);

Mat3 calc_projective (double* frame_ts,
                      Vec4* gyro_quat,
                      Vec3* acc_trans,
                      double* gyro_ts,
                      CalibrationParams calib);

#endif
