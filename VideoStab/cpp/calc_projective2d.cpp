/*
 * cal_projective2d.cpp - Calculate 2D image projective matrix
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

#include <vector>
#include "mex.h"

#include "calc_projective2d.h"

#define LOG_MESSAGE mexPrintf
#define ASSERT assert

enum CoordinateAxisType{
    AXIS_X = 0,
    AXIS_MINUS_X,
    AXIS_Y,
    AXIS_MINUS_Y,
    AXIS_Z,
    AXIS_MINUS_Z,
};

//interp_gyro_quatern(ts, quat, gyro_ts, index);
Quatern interp_gyro_quatern (double ts,
                             const std::vector<Vec4>& quat,
                             const std::vector<double>& gyro_ts,
                             int& index)
{
    int count = gyro_ts.size();
    int i = index;
    ASSERT(0 <= i && i < count);

    while (i >= 0 && gyro_ts[i] > ts) { i--; }
    if (i < 0) return Quatern(quat[0]);

    while (i+1 < count && gyro_ts[i+1] < ts) { i++; }
    if (i >= count) return Quatern(quat[count - 1]);

    index = i;

    double weight_start = (gyro_ts[i+1] - ts) / (gyro_ts[i+1] - gyro_ts[i]);
    double weight_end = 1.0f - weight_start;
    ASSERT(weight_start >= 0 && weight_start <= 1.0);
    ASSERT(weight_end >= 0 && weight_end <= 1.0);

    return Quatern(quat[i] * weight_start + quat[i+1] * weight_end);
    //return Quatern(quat[i]).slerp(weight_start, Quatern(quat[i + 1]));
}

// rotate coordinate system keeps the handedness of original coordinate system unchanged
//
// axis_to_x: defines the axis of the new cooridinate system that
//    coincide with the X axis of the original coordinate system.
// axis_to_y: defines the axis of the new cooridinate system that
//    coincide with the Y axis of the original coordinate system.
//
Mat3 rotate_coordinate_system (CoordinateAxisType axis_to_x, CoordinateAxisType axis_to_y)
{
    Mat3 t_mat;
    if (axis_to_x == AXIS_X && axis_to_y == AXIS_MINUS_Z) {
        t_mat = Mat3(Vec3(1, 0, 0),
                     Vec3(0, 0, 1),
                     Vec3(0, -1, 0));
    } else if (axis_to_x == AXIS_X && axis_to_y == AXIS_MINUS_Y) {
        t_mat = Mat3(Vec3(1, 0, 0),
                     Vec3(0, -1, 0),
                     Vec3(0, 0, -1));
    } else if (axis_to_x == AXIS_X && axis_to_y == AXIS_Z) {
        t_mat = Mat3(Vec3(1, 0, 0),
                     Vec3(0, 0, -1),
                     Vec3(0, 1, 0));
    } else if (axis_to_x == AXIS_MINUS_Z && axis_to_y == AXIS_Y) {
        t_mat = Mat3(Vec3(0, 0, 1),
                    Vec3(0, 1, 0),
                     Vec3(-1, 0, 0));
    } else if (axis_to_x == AXIS_MINUS_X && axis_to_y == AXIS_Y) {
        t_mat = Mat3(Vec3(-1, 0, 0),
                     Vec3(0, 1, 0),
                     Vec3(0, 0, -1));
    } else if (axis_to_x == AXIS_Z && axis_to_y == AXIS_Y) {
        t_mat = Mat3(Vec3(0, 0, -1),
                     Vec3(0, 1, 0),
                     Vec3(1, 0, 0));
    } else if (axis_to_x == AXIS_MINUS_Y && axis_to_y == AXIS_X) {
        t_mat = Mat3(Vec3(0, 1, 0),
                    Vec3(-1, 0, 0),
                     Vec3(0, 0, 1));
    } else if (axis_to_x == AXIS_MINUS_X && axis_to_y == AXIS_MINUS_Y) {
        t_mat = Mat3(Vec3(-1, 0, 0),
                     Vec3(0, -1, 0),
                     Vec3(0, 0, 1));
    } else if (axis_to_x == AXIS_Y && axis_to_y == AXIS_MINUS_X) {
        t_mat = Mat3(Vec3(0, -1, 0),
                     Vec3(1, 0, 0),
                     Vec3(0, 0, 1));
    } else  {
        t_mat = Mat3();
    }
    return t_mat;
}


// mirror coordinate system will change the handedness of original coordinate system
//
// axis_mirror: defines the axis that coordinate system mirror on
//
Mat3 mirror_coordinate_system (CoordinateAxisType axis_mirror)
{
    Mat3 t_mat;

    switch (axis_mirror) {
    case AXIS_X:
    case AXIS_MINUS_X:
        t_mat = Mat3(Vec3(-1, 0, 0),
                     Vec3(0, 1, 0),
                     Vec3(0, 0, 1));
        break;
    case AXIS_Y:
    case AXIS_MINUS_Y:
        t_mat = Mat3(Vec3(1, 0, 0),
                    Vec3(0, -1, 0),
                    Vec3(0, 0, 1));
        break;
    case AXIS_Z:
    case AXIS_MINUS_Z:
        t_mat = Mat3(Vec3(1, 0, 0),
                    Vec3(0, 1, 0),
                    Vec3(0, 0, -1));
        break;
    default:
        t_mat = Mat3();
        break;
    }

    return t_mat;
}

// transform coordinate system will change the handedness of original coordinate system
//
// axis_to_x: defines the axis of the new cooridinate system that
//    coincide with the X axis of the original coordinate system.
// axis_to_y: defines the axis of the new cooridinate system that
//    coincide with the Y axis of the original coordinate system.
// axis_mirror: defines the axis that coordinate system mirror on
Mat3 transform_coordinate_system (CoordinateAxisType axis_to_x, CoordinateAxisType axis_to_y, CoordinateAxisType axis_mirror)
{
    return mirror_coordinate_system(axis_mirror) * rotate_coordinate_system(axis_to_x, axis_to_y);
}


Mat3 calc_intrinsic (double focal_x, double focal_y, double offset_x, double offset_y, double skew)
{
    Mat3 intrinsic(Vec3(focal_x, 0, 0),
           Vec3(skew, focal_y, 0),
           Vec3(offset_x, offset_y, 1));

#if PRINT_MATRIX
    LOG_MESSAGE("%%Intrinsic Matrix(3x3) \n");
    LOG_MESSAGE("intrinsic = [ %lf, %lf, %lf ; %lf, %lf, %lf ; %lf, %lf, %lf ] \n",
                     intrinsic(1, 1), intrinsic(1, 2), intrinsic(1, 3),
                     intrinsic(2, 1), intrinsic(2, 2), intrinsic(2, 3),
                     intrinsic(3, 1), intrinsic(3, 2), intrinsic(3, 3));
#endif

    return intrinsic;
}

Mat3 calc_extrinsic (Quatern rotation)
{
    Mat3 extrinsic = rotation.rotation_matrix();
#if PRINT_MATRIX
        LOG_MESSAGE("%%extrinsic Matrix(3x3) \n");
        LOG_MESSAGE("R33 = [ %lf, %lf, %lf; %lf, %lf, %lf; %lf, %lf, %lf ] \n",
                           extrinsic(1, 1), extrinsic(1, 2), extrinsic(1, 3),
                           extrinsic(2, 1), extrinsic(2, 2), extrinsic(2, 3),
                           extrinsic(3, 1), extrinsic(3, 2), extrinsic(3, 3));
#endif

   return extrinsic;
}

Mat4 calc_extrinsic (Quatern rotation, Vec3 translation)
{
    Mat3 rot = rotation.rotation_matrix();
    Vec3 trans = translation;
    Mat4 extrinsic(Vec4(rot.v0, 0),
                   Vec4(rot.v1, 0),
                   Vec4(rot.v2, 0),
                   Vec4((rot * trans), 1));
#if PRINT_MATRIX
        LOG_MESSAGE("%%extrinsic Matrix(4x4) \n");
        LOG_MESSAGE("R44 = [ %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ] \n",
                           extrinsic(1, 1), extrinsic(1, 2), extrinsic(1, 3), extrinsic(1, 4),
                           extrinsic(2, 1), extrinsic(2, 2), extrinsic(2, 3), extrinsic(2, 4),
                           extrinsic(3, 1), extrinsic(3, 2), extrinsic(3, 3), extrinsic(3, 4),
                           extrinsic(4, 1), extrinsic(4, 2), extrinsic(4, 3), extrinsic(4, 4));
#endif

   return extrinsic;
}


void calc_projective (const std::vector<double>& frame_ts,
                      const std::vector<Vec4>& gyro_quat,
                      const std::vector<Vec3>& acc_trans,
                      const std::vector<double>& gyro_ts,
                      CalibrationParams calib,
                      std::vector<Mat3>& projective)
{
    int index0 = 0;
    int index1 = 0;

    size_t frame_count = frame_ts.size();

    for (int fid = 0; fid < frame_count; fid++) {
        const double ts0 = frame_ts[fid] + calib.gyro_delay;
        Quatern quat0 = interp_gyro_quatern(ts0, gyro_quat, gyro_ts, index0) + Quatern(calib.gyro_drift);

        const double ts1 = frame_ts[fid + 1] + calib.gyro_delay;
        Quatern quat1 = interp_gyro_quatern(ts1, gyro_quat, gyro_ts, index1) + Quatern(calib.gyro_drift);

        Mat3 extr0 = calc_extrinsic(quat0);
        Mat3 extr1 = calc_extrinsic(quat1);

        Mat3 intrinsic = calc_intrinsic(calib.fx, calib.fy, calib.cx, calib.cy, calib.skew);

        Mat3 extrinsic0 = rotate_coordinate_system(AXIS_X, AXIS_MINUS_Z) * extr0  * mirror_coordinate_system(AXIS_Y);
        Mat3 extrinsic1 = rotate_coordinate_system(AXIS_X, AXIS_MINUS_Z) * extr1  * mirror_coordinate_system(AXIS_Y);

        projective[fid] = intrinsic * extrinsic0 * extrinsic1.transpose() * intrinsic.inverse();
    }
}

void calc_projective (const std::vector<double>& frame_ts,
                      const std::vector<Vec4>& gyro_quat,
                      const std::vector<Vec3>& acc_trans,
                      const std::vector<double>& gyro_ts,
                      CalibrationParams calib,
                      std::vector<Mat4>& projective)
{
    int index0 = 0;
    int index1 = 0;

    size_t frame_count = frame_ts.size();

    for (int fid = 0; fid < frame_count; fid++) {
        const double ts0 = frame_ts[fid] + calib.gyro_delay;
        Quatern quat0 = interp_gyro_quatern(ts0, gyro_quat, gyro_ts, index0) + Quatern(calib.gyro_drift);

        const double ts1 = frame_ts[fid + 1] + calib.gyro_delay;
        Quatern quat1 = interp_gyro_quatern(ts1, gyro_quat, gyro_ts, index1) + Quatern(calib.gyro_drift);

        Vec3 trans0 = acc_trans[fid];
        Vec3 trans1 = acc_trans[fid + 1];

        Mat4 extr0 = calc_extrinsic(quat0, trans0);
        Mat4 extr1 = calc_extrinsic(quat1, trans1);

        Mat3 intr = calc_intrinsic(calib.fx, calib.fy, calib.cx, calib.cy, calib.skew);

        Mat4 intrinsic = Mat4(Vec4(intr.v0, 0),
                              Vec4(intr.v1, 0),
                              Vec4(intr.v2, 0),
                              Vec4(0, 0, 0, 1));

        projective[fid] = intrinsic * extr0 * extr1.transpose() * intrinsic.inverse();
    }
}

Mat3 calc_projective (double* frame_ts,
                      Vec4* gyro_quat,
                      Vec3* acc_trans,
                      double* gyro_ts,
                      CalibrationParams calib)
{
    double ts0 = frame_ts[0] + calib.gyro_delay;
    Quatern rot0 = Quatern(gyro_quat[0] + calib.gyro_drift);
    double ts1 = frame_ts[1] + calib.gyro_delay;
    Quatern rot1 = Quatern(gyro_quat[1] + calib.gyro_drift);

    Vec3 trans0 = acc_trans[0];
    Vec3 trans1 = acc_trans[1];

    Mat3 extr0 = calc_extrinsic(rot0);
    Mat3 extr1 = calc_extrinsic(rot1);
    Mat3 intrin = calc_intrinsic(calib.fx, calib.fy, calib.cx, calib.cy, calib.skew);

    return intrin * extr0 * extr1.transpose() * intrin.inverse();
}

