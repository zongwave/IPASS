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

#include "../cpp/calc_projective2d.h"
#include "../cpp/stabilize_motion.h"
#include "mex.h"

#define PRINT_MATRIX 1

#define FRAME_TIME     prhs[0]
#define FRAME_SIZE     prhs[1]
#define ACC_TRANS      prhs[2]
#define GYRO_QUAT      prhs[3]
#define TIME_STAMP     prhs[4]
#define CALIB_PARAM    prhs[5]
#define HOMOGRAPHY     prhs[6]

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    if ( nrhs != 7 ||
        !mxIsDouble(FRAME_TIME) ||
        !mxIsDouble(FRAME_SIZE) ||
        !mxIsDouble(ACC_TRANS) ||
        !mxIsDouble(GYRO_QUAT) ||
        !mxIsDouble(TIME_STAMP) ||
        !mxIsDouble(CALIB_PARAM) ||
        !mxIsDouble(HOMOGRAPHY))
    {
        mexPrintf("input number %d \n", nrhs);
        mexPrintf("input %d, %d, %d, %d, %d, %d, %d \n",
            mxIsDouble(FRAME_TIME),
            mxIsDouble(FRAME_SIZE),
            mxIsDouble(ACC_TRANS),
            mxIsDouble(GYRO_QUAT),
            mxIsDouble(TIME_STAMP),
            mxIsDouble(CALIB_PARAM),
            mxIsDouble(HOMOGRAPHY));
        mexErrMsgTxt("Incorrect input/output argument format. Usage: find_homography(frame_idx, frame_time, vid_dim, gyro_quat, gyro_time, cam_param)");
    }

    double* frame_time = mxGetPr(FRAME_TIME);
    const double frame_width = mxGetPr(FRAME_SIZE)[0];
    const double frame_height = mxGetPr(FRAME_SIZE)[1];
    Vec3* acc_trans = (Vec3*)mxGetData(ACC_TRANS);
    Vec4* gyro_quat = (Vec4*)mxGetData(GYRO_QUAT);
    double* time_stamp = mxGetPr(TIME_STAMP);
    size_t num_gyro_samp = mxGetNumberOfElements(TIME_STAMP);

    double* params = mxGetPr(CALIB_PARAM);
    Mat3* homography = (Mat3*)mxGetPr(HOMOGRAPHY);
    size_t num_homo = mxGetNumberOfElements(HOMOGRAPHY) / 9;

    CalibrationParams calib;
    calib.fx = params[0];
    calib.fy = params[1];
    calib.cx = params[2];
    calib.cy = params[3];
    calib.skew = params[4];
    calib.gyro_drift.x = (double)params[5];
    calib.gyro_drift.y = (double)params[6];
    calib.gyro_drift.z = (double)params[7];
    calib.gyro_drift.w = (double)params[8];
    calib.gyro_delay = params[9];
    calib.readout_time = params[10];

    mexPrintf("calibration params: \n");
    mexPrintf("  camera focal length (%lf %lf) \n", calib.fx, calib.fy);
    mexPrintf("  camera skew parameter (%lf) \n", calib.skew);
    mexPrintf("  camera principal point coordinate x(%lf), y(%lf) \n", calib.cx, calib.cy);
    mexPrintf("  gyro drift x(%lf), y(%lf), z(%lf), w(%lf) \n", calib.gyro_drift.x, calib.gyro_drift.y, calib.gyro_drift.z, calib.gyro_drift.w);
    mexPrintf("  gyro delay (%lf) \n", calib.gyro_delay);
    mexPrintf("  gyro readout time (%lf)\n", calib.readout_time);

    std::vector<Mat3> projective;

    if (num_gyro_samp > 1) {
        projective.resize(num_gyro_samp);

        calc_projective2d(gyro_quat,
                   acc_trans,
                   num_gyro_samp,
                   frame_time,
                   time_stamp,
                   calib,
                   projective);
    } else {
        projective.resize(num_homo);

        for (int idx = 0; idx < num_homo; idx++) {
            projective[idx] = homography[idx];
        }
    }

    MotionFilter motion_filter(15, 10);
    for (int idx = 0; idx < num_homo; idx++) {
        homography[idx] = motion_filter.stabilize(idx, projective, (int)num_homo, 0);
        homography[idx] = homography[idx].inverse();
    }

    projective.clear();
}

