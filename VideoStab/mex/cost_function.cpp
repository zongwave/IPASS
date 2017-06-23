/*
 * cost_function.cpp - Cost function for sensor parameters calibration
 *
 *  Copyright (c) 2017 Intel Corporation
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

#include "mex.h"

#include "../cpp/quaternion.h"
#include "../cpp/calc_projective2d.h"

#define INIT_CALIB     prhs[0]
#define FRAME_ID       prhs[1]
#define PT0            prhs[2]
#define PT1            prhs[3]
#define FRAME_TS       prhs[4]
#define FRAME_SIZE     prhs[5]
#define ACC_TRANS      prhs[6]
#define GYRO_QUAT      prhs[7]
#define GYRO_TS        prhs[8]
#define PROJECTIVE2D   prhs[9]

#define ERR           plhs[0]

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    if ( nlhs != 1 ||nrhs != 10 ||
        !mxIsDouble(INIT_CALIB) ||
        !mxIsInt32(FRAME_ID) ||
        !mxIsDouble(PT0) ||
        !mxIsDouble(PT1) ||
        !mxIsDouble(FRAME_TS) ||
        !mxIsDouble(FRAME_SIZE) ||
        !mxIsDouble(ACC_TRANS) ||
        !mxIsDouble(GYRO_QUAT) ||
        !mxIsDouble(GYRO_TS) ||
        !mxIsDouble(PROJECTIVE2D))
    {
        mexPrintf("input number %d, output mumber %d \n", nrhs, nlhs);
        mexPrintf("input %d, %d, %d, %d, %d, %d, %d, %d, %d, %d \n",
            mxIsDouble(INIT_CALIB),
            mxIsInt32(FRAME_ID),
            mxIsDouble(PT0),
            mxIsDouble(PT1),
            mxIsDouble(FRAME_TS),
            mxIsDouble(FRAME_SIZE),
            mxIsDouble(ACC_TRANS),
            mxIsDouble(GYRO_QUAT),
            mxIsDouble(GYRO_TS),
            mxIsDouble(PROJECTIVE2D));
        mexErrMsgTxt("Incorrect input/output argument format. Usage: camera_calibration(frame_idx, p0, p1, frame_time, vid_dim, gyro_quat, gyro_time, cam_param)");
    }

    double* params = mxGetPr(INIT_CALIB);
    int* frame_idx = (int*)mxGetData(FRAME_ID);

    Vec2* p0 = (Vec2*)mxGetData(PT0);
    Vec2* p1 = (Vec2*)mxGetData(PT1);
    size_t num_pts = mxGetNumberOfElements(PT0) / 2;

    double* frame_timestamp = mxGetPr(FRAME_TS);
    size_t frame_count = mxGetNumberOfElements(FRAME_TS);

    const double frame_width = mxGetPr(FRAME_SIZE)[0];
    const double frame_height = mxGetPr(FRAME_SIZE)[1];

    Vec3* acc_trans = (Vec3*)mxGetData(ACC_TRANS);
    Vec4* gyro_quat = (Vec4*)mxGetData(GYRO_QUAT);
    double* gyro_timestamp = mxGetPr(GYRO_TS);
    size_t num_gyro_sampl = mxGetNumberOfElements(GYRO_TS);

    Mat3* projective = (Mat3*)mxGetPr(PROJECTIVE2D);

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

#if PRINT_MATRIX
    mexPrintf("calibration params: \n");
    mexPrintf("  camera focal length (%lf %lf) \n", calib.fx, calib.fy);
    mexPrintf("  camera skew parameter (%lf) \n", calib.skew);
    mexPrintf("  camera principal point coordinate x(%lf), y(%lf) \n", calib.cx, calib.cy);
    mexPrintf("  gyro drift x(%lf), y(%lf), z(%lf), w(%lf) \n", calib.gyro_drift.x, calib.gyro_drift.y, calib.gyro_drift.z, calib.gyro_drift.w);
    mexPrintf("  gyro delay (%lf) \n", calib.gyro_delay);
    mexPrintf("  gyro readout time (%lf)\n", calib.readout_time);
#endif

    ERR = mxCreateDoubleMatrix(1, 1, mxREAL);
    double &err = *mxGetPr(ERR);

    std::vector<double> frame_ts;
    frame_ts.resize(frame_count);

    for (int i = 0; i < frame_count; i++) {
        frame_ts[i] = frame_timestamp[i];
    }

    std::vector<Vec4> quaternion;
    std::vector<Vec3> translation;
    std::vector<double> gyro_ts;
    quaternion.resize(num_gyro_sampl);
    translation.resize(num_gyro_sampl);
    gyro_ts.resize(num_gyro_sampl);

    for (int i = 0; i < num_gyro_sampl; i++) {
        quaternion[i] = gyro_quat[i];
        translation[i] = acc_trans[i];
        gyro_ts[i] = gyro_timestamp[i];
    }

    std::vector<Mat3> proj_mat;
    proj_mat.resize(num_gyro_sampl);

    calc_projective(frame_ts,
                    quaternion,
                    translation,
                    gyro_ts,
                    calib,
                    proj_mat);

    for (int i = 0; i < num_gyro_sampl; i++) {
        projective[i] = proj_mat[i];
    }
    proj_mat.clear();

    err = 0;
    for (int pid = 0; pid < num_pts; ++pid) {
        int fid = frame_idx[pid];

        if (fid >= num_gyro_sampl) fid = num_gyro_sampl - 1;

        Vec3 p0_t = projective[fid-1] * Vec3(p0[pid].x, p0[pid].y, 1.0);
        Vec2 delta = p1[pid] - ( Vec2(p0_t.x, p0_t.y) / (p0_t.z) );
        err += delta.magnitude();
#if 0
        {
            mexPrintf("extrinsic Matrix[%d] for Point[%d]: \n", fid, pid);
            mexPrintf("point0: x(%lf), y(%lf) \n", p0[pid].x, p0[pid].y);
            mexPrintf("point1: x(%lf), y(%lf) \n", p1[pid].x, p1[pid].y);
            mexPrintf("point1': x'h(%lf), y'h(%lf), z'h(%lf) \n", p0_t.x, p0_t.y, p0_t.z);
            mexPrintf("point1': x'(%lf), y'(%lf) \n", p0_t.x/p0_t.z, p0_t.y/p0_t.z);
        }
#endif
    }

    err /= num_pts;
}
