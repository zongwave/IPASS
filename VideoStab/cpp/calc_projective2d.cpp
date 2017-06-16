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

#include <vector>
#include "mex.h"

#include "calc_projective2d.h"

#define LOG_MESSAGE mexPrintf
#define ASSERT assert

Mat3 negive_second(Vec3(1, 0, 0),
                   Vec3(0, 1, 0),
                   Vec3(0, 0, 1));

//interp(gyro_time, gyro_quat, ts, num_gyro_samp, start);
Quatern interp(double* tx, Vec4* x, double ty, size_t num, int& start)
{
    int i = start;
    ASSERT(0 <= i && i < num-1, "i is outside array range");

    while (i >= 0 && tx[i] > ty) { --i; }
    if (i < 0) return Quatern(x[0]);

    while (i+1 < num && tx[i+1] < ty) { ++i; }
    if (i+1 >= num) return Quatern(x[num-1]);

    start = i;

    double dt = tx[i+1] - tx[i];
    double dy = ty - tx[i];
    double w = dy / dt;

    ASSERT(dt >= 0, "time is not monotonically increasing");
    ASSERT(0 <= dy && dy <= dt, "");

    return Quatern(x[i] * (1-w) + x[i+1] * w);
    //return Quatern(x[i]).slerp(w, Quatern(x[i + 1]));
}

Mat3 calc_intrinsic(double focal_x, double focal_y, double offset_x, double offset_y, double skew)
{
    Mat3 k(Vec3(focal_x, 0, 0),
           Vec3(skew, focal_y, 0),
           Vec3(offset_x, offset_y, 1));
    Mat3 invK = k.inverse();
#if PRINT_MATRIX
    LOG_MESSAGE("%%Intrinsic Matrix(3x3) \n");
    LOG_MESSAGE("k = [ %lf, %lf, %lf ; %lf, %lf, %lf ; %lf, %lf, %lf ] \n",
                     k(1, 1), k(1, 2), k(1, 3),
                     k(2, 1), k(2, 2), k(2, 3),
                     k(3, 1), k(3, 2), k(3, 3));
    LOG_MESSAGE("invK = [ %lf, %lf, %lf ; %lf, %lf, %lf ; %lf, %lf, %lf ] \n",
                        invK(1, 1), invK(1, 2), invK(1, 3),
                        invK(2, 1), invK(2, 2), invK(2, 3),
                        invK(3, 1), invK(3, 2), invK(3, 3));
#endif

    return k * negive_second;
}

Mat3 calc_extrinsic(Quatern rotation)
{
    // specify the camera's pose directly rather than
    // specifying how world points should transform to camera coordinates.
    Mat3 extrinsic = rotation.rotation_matrix();

#if PRINT_MATRIX
    LOG_MESSAGE("%%extrinsic Matrix(3x3) \n");
    LOG_MESSAGE("R33 = [ %lf, %lf, %lf ; %lf, %lf, %lf ; %lf, %lf, %lf ] \n",
                       extrinsic(1, 1), extrinsic(1, 2), extrinsic(1, 3),
                       extrinsic(2, 1), extrinsic(2, 2), extrinsic(2, 3),
                       extrinsic(3, 1), extrinsic(3, 2), extrinsic(3, 3));
#endif

   return negive_second * extrinsic;
}

Mat4 calc_extrinsic(Quatern rotation, Vec3 translation)
{
    // specify the camera's pose directly rather than
    // specifying how world points should transform to camera coordinates.
    Mat3 rot = rotation.rotation_matrix().transpose();
    Vec3 trans = -translation;
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

Mat3 calc_homography(Mat3 rot_mat0, Mat3 rot_mat1, Mat3 k)
{
    return k * rot_mat1 * rot_mat0.transpose() * k.inverse();
}

void calc_projective2d(Vec4* gyro_quat,
                       Vec3* acc_trans,
                       size_t num_gyro_samp,
                       double* time_stamp,
                       double* frame_time,
                       CalibrationParams calib,
                       std::vector<Mat3>& projective)
{
    int start0 = 0;
    int start1 = 0;

    for (int fid = 0; fid < num_gyro_samp; fid++) {
        const double ts0 = frame_time[fid] + calib.gyro_delay;
        Quatern rot0 = interp(time_stamp, gyro_quat, ts0, num_gyro_samp, start0) + Quatern(calib.gyro_drift);

        const double ts1 = frame_time[fid + 1] + calib.gyro_delay;
        Quatern rot1 = interp(time_stamp, gyro_quat, ts1, num_gyro_samp, start1) + Quatern(calib.gyro_drift);

        Vec3 trans0 = acc_trans[fid];
        Vec3 trans1 = acc_trans[fid + 1];

        //Mat4 extr0 = calc_extrinsic(rot0, trans0);
        //Mat4 extr1 = calc_extrinsic(rot1, trans1);
        Mat3 extr0 = calc_extrinsic(rot0);
        Mat3 extr1 = calc_extrinsic(rot1);
        Mat3 intrin = calc_intrinsic(calib.fx, calib.fy, calib.cx, calib.cy, calib.skew);

        projective[fid] = calc_homography(extr0, extr1, intrin);
    }
}

Mat3 calc_projective2d(Vec4* gyro_quat,
                       Vec3* acc_trans,
                       double* time_stamp,
                       double frame_time,
                       CalibrationParams calib)
{
    double ts0 = time_stamp[0] + calib.gyro_delay;
    Quatern rot0 = gyro_quat[0];// + Quatern(calib.gyro_drift);

    double ts1 = time_stamp[1] + calib.gyro_delay;
    Quatern rot1 = gyro_quat[1];// + Quatern(calib.gyro_drift);

    Vec3 trans0 = acc_trans[0];
    Vec3 trans1 = acc_trans[1];

    //Mat4 extr0 = calc_extrinsic(rot0, trans0);
    //Mat4 extr1 = calc_extrinsic(rot1, trans1);
    Mat3 extr0 = calc_extrinsic(rot0);
    Mat3 extr1 = calc_extrinsic(rot1);
    Mat3 intrin = calc_intrinsic(calib.fx, calib.fy, calib.cx, calib.cy, calib.skew);

    return calc_homography(extr0, extr1, intrin);
}

