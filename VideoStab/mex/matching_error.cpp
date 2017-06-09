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

#include "mex.h"

#include <time.h>

#include "../cpp/quaternion.h"
#include "../cpp/find_homography.h"

#define FRAME_ID       prhs[0]
#define PT0            prhs[1]
#define PT1            prhs[2]
#define FRAME_TIME     prhs[3]
#define FRAME_SIZE     prhs[4]
#define ACC_TRANS      prhs[5]
#define GYRO_QUAT      prhs[6]
#define TIME_STAMP     prhs[7]
#define CALIB_PARAM    prhs[8]
#define HOMOGRAPHY_MAT prhs[9]

#define ERR    plhs[0]

// matching_error(frame_idx, p0, p1, frame_time, frame_size, acc_trans, gyro_quat, gyro_time, calib_param)
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    if ( nlhs != 1 ||nrhs != 10 ||
        !mxIsInt32(FRAME_ID) ||
        !mxIsDouble(PT0) ||
        !mxIsDouble(PT1) ||
        !mxIsDouble(FRAME_TIME) ||
        !mxIsDouble(FRAME_SIZE) ||
        !mxIsDouble(ACC_TRANS) ||
        !mxIsDouble(GYRO_QUAT) ||
        !mxIsDouble(TIME_STAMP) ||
        !mxIsDouble(CALIB_PARAM) ||
        !mxIsDouble(HOMOGRAPHY_MAT))
    {
        mexPrintf("input number %d \n", nrhs);
		mexPrintf("input %d, %d, %d, %d, %d, %d, %d, %d, %d, %d \n",
			mxIsInt32(FRAME_ID),
			mxIsDouble(PT0),
			mxIsDouble(PT1),
			mxIsDouble(FRAME_TIME),
			mxIsDouble(FRAME_SIZE),
			mxIsDouble(ACC_TRANS),
			mxIsDouble(GYRO_QUAT),
			mxIsDouble(TIME_STAMP),
			mxIsDouble(CALIB_PARAM),
			mxIsDouble(HOMOGRAPHY_MAT));
        mexErrMsgTxt("Incorrect input/output argument format. Usage: camera_calibration(frame_idx, p0, p1, frame_time, vid_dim, gyro_quat, gyro_time, cam_param)");
    }

    int* frame_idx = (int*)mxGetData(FRAME_ID);
    Vec2* p0 = (Vec2*)mxGetData(PT0);
    Vec2* p1 = (Vec2*)mxGetData(PT1);
    double* frame_time = mxGetPr(FRAME_TIME);
    const double frame_width = mxGetPr(FRAME_SIZE)[0];
    const double frame_height = mxGetPr(FRAME_SIZE)[1];
	Vec3* acc_trans = (Vec3*)mxGetData(ACC_TRANS);
    Vec4* gyro_quat = (Vec4*)mxGetData(GYRO_QUAT);
    double* time_stamp = mxGetPr(TIME_STAMP);
	double* params = mxGetPr(CALIB_PARAM);
    Mat3* homograpy = (Mat3*)mxGetPr(HOMOGRAPHY_MAT);

    size_t num_points = mxGetNumberOfElements(PT0);

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

    ERR = mxCreateDoubleMatrix(1, 1, mxREAL);
    double &err = *mxGetPr(ERR);

    size_t num_pts = mxGetNumberOfElements(PT0) / 2;
    size_t num_gyro_samp = mxGetNumberOfElements(TIME_STAMP);

	findHomography(gyro_quat,
					acc_trans,
					num_gyro_samp,
					frame_time,
					time_stamp,
					calib,
					homograpy);

    err = 0;
    for (int pid = 0; pid < num_pts; ++pid) {
        int fid = frame_idx[pid];

        if (fid >= num_gyro_samp) fid = num_gyro_samp - 1;

        Vec3 P1 = homograpy[fid-1] * Vec3(p0[pid].x, p0[pid].y, 1.0);
        Vec2 dxy = p1[pid] - ( Vec2(P1.x, P1.y) / (P1.z) );
		err += dxy.magnitude();
#if PRINT_MATRIX
		{
			//mexPrintf("line_length0 (%lf), ts0 (%lf) \n", line_length0, ts0);
			//mexPrintf("line_length1 (%lf), ts1 (%lf) \n", line_length1, ts1);
			mexPrintf("extrinsic Matrix[%d] for Point[%d]: \n", fid, pid);
			mexPrintf("point0: x(%lf), y(%lf) \n", p0[pid].x, p0[pid].y);
			mexPrintf("point1: x(%lf), y(%lf) \n", p1[pid].x, p1[pid].y);
			mexPrintf("point1': x'h(%lf), y'h(%lf), z'h(%lf) \n", P1.x, P1.y, P1.z);
			mexPrintf("point1': x'(%lf), y'(%lf) \n", P1.x/P1.z, P1.y/P1.z);
			mexPrintf("diff (%lf)x(%lf)=(%lf) \n", dxy.x, dxy.y, dxy.magnitude());
		}
#endif
    }

    err /= num_pts;
}
