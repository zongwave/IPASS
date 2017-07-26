/*
 * video_stab_interface.cpp - video stabilization interface function
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

#include "../cpp/calc_projective2d.h"
#include "../cpp/stabilize_motion.h"
#include "mex.h"


#define FRAME_TS       prhs[0]
#define FRAME_SIZE     prhs[1]
#define ACC_TRANS      prhs[2]
#define GYRO_QUAT      prhs[3]
#define GYRO_TS        prhs[4]
#define CALIB_PARAM    prhs[5]
#define HOMOGRAPHY     prhs[6]

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    if ( nrhs != 7 ||
        !mxIsDouble(FRAME_TS) ||
        !mxIsDouble(FRAME_SIZE) ||
        !mxIsDouble(ACC_TRANS) ||
        !mxIsDouble(GYRO_QUAT) ||
        !mxIsDouble(GYRO_TS) ||
        !mxIsDouble(CALIB_PARAM) ||
        !mxIsDouble(HOMOGRAPHY))
    {
        mexPrintf("input number %d \n", nrhs);
        mexPrintf("input %d, %d, %d, %d, %d, %d, %d \n",
            mxIsDouble(FRAME_TS),
            mxIsDouble(FRAME_SIZE),
            mxIsDouble(ACC_TRANS),
            mxIsDouble(GYRO_QUAT),
            mxIsDouble(GYRO_TS),
            mxIsDouble(CALIB_PARAM),
            mxIsDouble(HOMOGRAPHY));
        mexErrMsgTxt("Incorrect input/output argument format. Usage: find_homography(frame_idx, frame_time, vid_dim, gyro_quat, gyro_time, cam_param)");
    }

    double* frame_timestamp = mxGetPr(FRAME_TS);
    size_t frame_count = mxGetNumberOfElements(FRAME_TS);
    const double frame_width = mxGetPr(FRAME_SIZE)[0];
    const double frame_height = mxGetPr(FRAME_SIZE)[1];
    Vec3* acc_trans = (Vec3*)mxGetData(ACC_TRANS);
    Vec4* gyro_quat = (Vec4*)mxGetData(GYRO_QUAT);
    double* gyro_timestamp = mxGetPr(GYRO_TS);
    size_t num_gyro_sampl = mxGetNumberOfElements(GYRO_TS);

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

    std::vector<Mat3> projective;
    if (num_gyro_sampl > 1) {
        projective.resize(num_gyro_sampl);

        calc_projective(frame_ts,
                        quaternion,
                        translation,
                        gyro_ts,
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

