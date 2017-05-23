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

#include "quaternion.h"

typedef Vector2<double> Vec2;
typedef Vector3<double> Vec3;
typedef Vector4<double> Vec4;
typedef Matrix3<double> Mat3;
typedef Matrix4<double> Mat4;
typedef Quaternion<double> Quatern;

#define PRINTMATRIX 0

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

//interp(gyro_time, gyro_quat, ts, num_gyro_samp, start);
Quatern interp(double* tx, Vec4* x, double ty, size_t num, int& start)
{
    int i = start;
    mxAssert(0 <= i && i < num-1, "i is outside array range");

    while (i >= 0 && tx[i] > ty) { --i; }
    if (i < 0) return Quatern(x[0]);

    while (i+1 < num && tx[i+1] < ty) { ++i; }
    if (i+1 >= num) return Quatern(x[num-1]);

    start = i;

    double dt = tx[i+1] - tx[i];
    double dy = ty - tx[i];
    double w = dy / dt;

    mxAssert(dt >= 0, "time is not monotonically increasing");
    mxAssert(0 <= dy && dy <= dt, "");

    return Quatern(x[i] * (1-w) + x[i+1] * w);
    //return Quatern(x[i]).slerp(w, Quatern(x[i + 1]));
}

Mat3 getIntrinsicMatrix(double focal_x, double focal_y, double offset_x, double offset_y, double skew)
{
    Mat3 K(Vec3(focal_x, 0, 0),
           Vec3(skew, focal_y, 0),
           Vec3(offset_x, offset_y, 1));
    Mat3 invK = K.inverse();
#if PRINTMATRIX
        mexPrintf("%%Intrinsic Matrix(3x3) \n");
        mexPrintf("K = [ %lf, %lf, %lf ; %lf, %lf, %lf ; %lf, %lf, %lf ] \n",
                         K(1, 1), K(1, 2), K(1, 3),
                         K(2, 1), K(2, 2), K(2, 3),
                         K(3, 1), K(3, 2), K(3, 3));
        mexPrintf("invK = [ %lf, %lf, %lf ; %lf, %lf, %lf ; %lf, %lf, %lf ] \n",
                            invK(1, 1), invK(1, 2), invK(1, 3),
                            invK(2, 1), invK(2, 2), invK(2, 3),
                            invK(3, 1), invK(3, 2), invK(3, 3));
#endif

    return K;
}

Mat3 getExtrinsicMatrix(Quatern rotation)
{
    // specify the camera's pose directly rather than
    // specifying how world points should transform to camera coordinates.
    Mat3 extrinsic = rotation.rotationMatrix().transpose();
    //Mat3 extrinsic = rotation.rotationMatrix();
#if PRINTMATRIX
        mexPrintf("%%extrinsic Matrix(3x3) \n");
        mexPrintf("R33 = [ %lf, %lf, %lf ; %lf, %lf, %lf ; %lf, %lf, %lf ] \n",
                           extrinsic(1, 1), extrinsic(1, 2), extrinsic(1, 3),
                           extrinsic(2, 1), extrinsic(2, 2), extrinsic(2, 3),
                           extrinsic(3, 1), extrinsic(3, 2), extrinsic(3, 3));
#endif

   return extrinsic.transpose();
}

Mat4 getExtrinsicMatrix(Quatern rotation, Vec3 translation)
{
    // specify the camera's pose directly rather than
    // specifying how world points should transform to camera coordinates.
    Mat3 rot = rotation.rotationMatrix().transpose();
    Vec3 trans = -translation;
    Mat4 extrinsic(Vec4(rot.v0, 0),
                     Vec4(rot.v1, 0),
                     Vec4(rot.v2, 0),
                     Vec4((rot * trans), 1));
#if PRINTMATRIX
        //mexPrintf("%%extrinsic Matrix(4x4) \n");
        //mexPrintf("R44 = [ %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ] \n",
        //                   extrinsic(1, 1), extrinsic(1, 2), extrinsic(1, 3), extrinsic(1, 4),
        //                   extrinsic(2, 1), extrinsic(2, 2), extrinsic(2, 3), extrinsic(2, 4),
        //                   extrinsic(3, 1), extrinsic(3, 2), extrinsic(3, 3), extrinsic(3, 4),
        //                   extrinsic(4, 1), extrinsic(4, 2), extrinsic(4, 3), extrinsic(4, 4));
#endif

   return extrinsic;
}

void testMatrixMultiplication()
{
        srand((unsigned)time(0));
        Mat3 A3(Vec3(rand()/(double)(RAND_MAX/10),
                    -rand()/(double)(RAND_MAX/10),
                    rand()/(double)(RAND_MAX/10)),
               Vec3(rand()/(double)(RAND_MAX/10),
                    rand()/(double)(RAND_MAX/10),
                    rand()/(double)(RAND_MAX/10)),
               Vec3(rand()/(double)(RAND_MAX/10),
                    rand()/(double)(RAND_MAX/10),
                    rand()/(double)(RAND_MAX/10)));

        Mat3 B3(Vec3(-rand()/(double)(RAND_MAX/10),
                    rand()/(double)(RAND_MAX/10),
                    -rand()/(double)(RAND_MAX/10)),
               Vec3(rand()/(double)(RAND_MAX/10),
                    -rand()/(double)(RAND_MAX/10),
                    rand()/(double)(RAND_MAX/10)),
               Vec3(rand()/(double)(RAND_MAX/10),
                    rand()/(double)(RAND_MAX/10),
                    -rand()/(double)(RAND_MAX/10)));

        Mat3 C3 = A3 * B3;
        Mat3 D3 = B3 * A3;

        Mat3 AA3 = A3.transpose();
        Mat3 BB3 = B3.transpose();
        Mat3 CC3 = C3.transpose();
        Mat3 DD3 = D3.transpose();

        Mat3 a3 = A3.inverse();
        Mat3 b3 = B3.inverse();
        Mat3 c3 = C3.inverse();
        Mat3 d3 = D3.inverse();

        mexPrintf("A3 = [ %lf, %lf, %lf ; %lf, %lf, %lf ; %lf, %lf, %lf ] \n",
                          A3.v0.x, A3.v1.x, A3.v2.x,
                          A3.v0.y, A3.v1.y, A3.v2.y,
                          A3.v0.z, A3.v1.z, A3.v2.z);
        mexPrintf("A3_T = [ %lf, %lf, %lf ; %lf, %lf, %lf ; %lf, %lf, %lf ] \n",
                            AA3.v0.x, AA3.v1.x, AA3.v2.x,
                            AA3.v0.y, AA3.v1.y, AA3.v2.y,
                            AA3.v0.z, AA3.v1.z, AA3.v2.z);
        mexPrintf("invA3 = [ %lf, %lf, %lf ; %lf, %lf, %lf ; %lf, %lf, %lf ] \n",
                             a3.v0.x, a3.v1.x, a3.v2.x,
                             a3.v0.y, a3.v1.y, a3.v2.y,
                             a3.v0.z, a3.v1.z, a3.v2.z);
        mexPrintf("B3 = [ %lf, %lf, %lf ; %lf, %lf, %lf ; %lf, %lf, %lf ] \n",
                          B3.v0.x, B3.v1.x, B3.v2.x,
                          B3.v0.y, B3.v1.y, B3.v2.y,
                          B3.v0.z, B3.v1.z, B3.v2.z);
        mexPrintf("invB3 = [ %lf, %lf, %lf ; %lf, %lf, %lf ; %lf, %lf, %lf ] \n",
                              b3.v0.x, b3.v1.x, b3.v2.x,
                              b3.v0.y, b3.v1.y, b3.v2.y,
                              b3.v0.z, b3.v1.z, b3.v2.z);
        mexPrintf("B3_T = [ %lf, %lf, %lf ; %lf, %lf, %lf ; %lf, %lf, %lf ] \n",
                            BB3.v0.x, BB3.v1.x, BB3.v2.x,
                            BB3.v0.y, BB3.v1.y, BB3.v2.y,
                            BB3.v0.z, BB3.v1.z, BB3.v2.z);
        mexPrintf("C3 = [ %lf, %lf, %lf ; %lf, %lf, %lf ; %lf, %lf, %lf ] \n",
                          C3.v0.x, C3.v1.x, C3.v2.x,
                          C3.v0.y, C3.v1.y, C3.v2.y,
                          C3.v0.z, C3.v1.z, C3.v2.z);
        mexPrintf("invC3 = [ %lf, %lf, %lf ; %lf, %lf, %lf ; %lf, %lf, %lf ] \n",
                             c3.v0.x, c3.v1.x, c3.v2.x,
                             c3.v0.y, c3.v1.y, c3.v2.y,
                             c3.v0.z, c3.v1.z, c3.v2.z);
        mexPrintf("C3_T = [ %lf, %lf, %lf ; %lf, %lf, %lf ; %lf, %lf, %lf ] \n",
                            CC3.v0.x, CC3.v1.x, CC3.v2.x,
                            CC3.v0.y, CC3.v1.y, CC3.v2.y,
                            CC3.v0.z, CC3.v1.z, CC3.v2.z);
        mexPrintf("D3 = [ %lf, %lf, %lf ; %lf, %lf, %lf ; %lf, %lf, %lf ] \n",
                          D3.v0.x, D3.v1.x, D3.v2.x,
                          D3.v0.y, D3.v1.y, D3.v2.y,
                          D3.v0.z, D3.v1.z, D3.v2.z);
        mexPrintf("invD3 = [ %lf, %lf, %lf ; %lf, %lf, %lf ; %lf, %lf, %lf ] \n",
                             d3.v0.x, d3.v1.x, d3.v2.x,
                             d3.v0.y, d3.v1.y, d3.v2.y,
                             d3.v0.z, d3.v1.z, d3.v2.z);
        mexPrintf("D3_T = [ %lf, %lf, %lf ; %lf, %lf, %lf ; %lf, %lf, %lf ] \n",
                            DD3.v0.x, DD3.v1.x, DD3.v2.x,
                            DD3.v0.y, DD3.v1.y, DD3.v2.y,
                            DD3.v0.z, DD3.v1.z, DD3.v2.z);

        Mat4 A(Vec4(rand()/(double)(RAND_MAX/10), -rand()/(double)(RAND_MAX/10),
                    rand()/(double)(RAND_MAX/10), -rand()/(double)(RAND_MAX/10)),
               Vec4(rand()/(double)(RAND_MAX/10), rand()/(double)(RAND_MAX/10),
                    rand()/(double)(RAND_MAX/10), rand()/(double)(RAND_MAX/10)),
               Vec4(-rand()/(double)(RAND_MAX/10), rand()/(double)(RAND_MAX/10),
                    rand()/(double)(RAND_MAX/10), rand()/(double)(RAND_MAX/10)),
               Vec4(-rand()/(double)(RAND_MAX/10), rand()/(double)(RAND_MAX/10),
                    rand()/(double)(RAND_MAX/10), -rand()/(double)(RAND_MAX/10)));
        Mat4 B(Vec4(rand()/(double)(RAND_MAX/10), rand()/(double)(RAND_MAX/10),
                    -rand()/(double)(RAND_MAX/10), rand()/(double)(RAND_MAX/10)),
               Vec4(rand()/(double)(RAND_MAX/10), rand()/(double)(RAND_MAX/10),
                    rand()/(double)(RAND_MAX/10), rand()/(double)(RAND_MAX/10)),
               Vec4(rand()/(double)(RAND_MAX/10), -rand()/(double)(RAND_MAX/10),
                    -rand()/(double)(RAND_MAX/10), rand()/(double)(RAND_MAX/10)),
               Vec4(rand()/(double)(RAND_MAX/10), rand()/(double)(RAND_MAX/10),
                    -rand()/(double)(RAND_MAX/10), rand()/(double)(RAND_MAX/10)));

        Mat4 C = A * B;
        Mat4 D = B * A;

        Mat4 AA = A.transpose();
        Mat4 BB = B.transpose();
        Mat4 CC = C.transpose();
        Mat4 DD = D.transpose();

        Mat4 a = A.inverse();
        Mat4 b = B.inverse();
        Mat4 c = C.inverse();
        Mat4 d = D.inverse();

        mexPrintf("A = [ %lf, %lf, %lf , %lf ; %lf, %lf, %lf , %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ] \n",
                          A.v0.x, A.v1.x, A.v2.x, A.v3.x,
                          A.v0.y, A.v1.y, A.v2.y, A.v3.y,
                          A.v0.z, A.v1.z, A.v2.z, A.v3.z,
                          A.v0.w, A.v1.w, A.v2.w, A.v3.w);
        mexPrintf("invA = [ %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ] \n",
                            a.v0.x, a.v1.x, a.v2.x, a.v3.x,
                            a.v0.y, a.v1.y, a.v2.y, a.v3.y,
                            a.v0.z, a.v1.z, a.v2.z, a.v3.z,
                            a.v0.w, a.v1.w, a.v2.w, a.v3.w);
        mexPrintf("A_T = [ %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ] \n",
                           AA.v0.x, AA.v1.x, AA.v2.x, AA.v3.x,
                           AA.v0.y, AA.v1.y, AA.v2.y, AA.v3.y,
                           AA.v0.z, AA.v1.z, AA.v2.z, AA.v3.z,
                           AA.v0.w, AA.v1.w, AA.v2.w, AA.v3.w);
        mexPrintf("B = [ %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ] \n",
                         B.v0.x, B.v1.x, B.v2.x, B.v3.x,
                         B.v0.y, B.v1.y, B.v2.y, B.v3.y,
                         B.v0.z, B.v1.z, B.v2.z, B.v3.z,
                         B.v0.w, B.v1.w, B.v2.w, B.v3.w);
        mexPrintf("invB = [ %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ] \n",
                            b.v0.x, b.v1.x, b.v2.x, b.v3.x,
                            b.v0.y, b.v1.y, b.v2.y, b.v3.y,
                            b.v0.z, b.v1.z, b.v2.z, b.v3.z,
                            b.v0.w, b.v1.w, b.v2.w, b.v3.w);
        mexPrintf("B_T = [ %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ] \n",
                           BB.v0.x, BB.v1.x, BB.v2.x, BB.v3.x,
                           BB.v0.y, BB.v1.y, BB.v2.y, BB.v3.y,
                           BB.v0.z, BB.v1.z, BB.v2.z, BB.v3.z,
                           BB.v0.w, BB.v1.w, BB.v2.w, BB.v3.w);
        mexPrintf("C = [ %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ] \n",
                         C.v0.x, C.v1.x, C.v2.x, C.v3.x,
                         C.v0.y, C.v1.y, C.v2.y, C.v3.y,
                         C.v0.z, C.v1.z, C.v2.z, C.v3.z,
                         C.v0.w, C.v1.w, C.v2.w, C.v3.w);
        mexPrintf("invC = [ %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ] \n",
                            c.v0.x, c.v1.x, c.v2.x, c.v3.x,
                            c.v0.y, c.v1.y, c.v2.y, c.v3.y,
                            c.v0.z, c.v1.z, c.v2.z, c.v3.z,
                            c.v0.w, c.v1.w, c.v2.w, c.v3.w);
        mexPrintf("C_T = [ %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ] \n",
                           CC.v0.x, CC.v1.x, CC.v2.x, CC.v3.x,
                           CC.v0.y, CC.v1.y, CC.v2.y, CC.v3.y,
                           CC.v0.z, CC.v1.z, CC.v2.z, CC.v3.z,
                           CC.v0.w, CC.v1.w, CC.v2.w, CC.v3.w);
        mexPrintf("D = [ %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ] \n",
                         D.v0.x, D.v1.x, D.v2.x, D.v3.x,
                         D.v0.y, D.v1.y, D.v2.y, D.v3.y,
                         D.v0.z, D.v1.z, D.v2.z, D.v3.z,
                         D.v0.w, D.v1.w, D.v2.w, D.v3.w);
        mexPrintf("invD = [ %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ] \n",
                            d.v0.x, d.v1.x, d.v2.x, d.v3.x,
                            d.v0.y, d.v1.y, d.v2.y, d.v3.y,
                            d.v0.z, d.v1.z, d.v2.z, d.v3.z,
                            d.v0.w, d.v1.w, d.v2.w, d.v3.w);
        mexPrintf("D_T = [ %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ] \n",
                           DD.v0.x, DD.v1.x, DD.v2.x, DD.v3.x,
                           DD.v0.y, DD.v1.y, DD.v2.y, DD.v3.y,
                           DD.v0.z, DD.v1.z, DD.v2.z, DD.v3.z,
                           DD.v0.w, DD.v1.w, DD.v2.w, DD.v3.w);
}

Mat3 getHomographyMatrix(Mat3 rotMat0, Mat3 rotMat1, Mat3 K)
{
    return K * rotMat1 * rotMat0.transpose() * K.inverse();
}

#define FRAME_TIME     prhs[0]
#define FRAME_SIZE     prhs[1]
#define ACC_TRANS      prhs[2]
#define GYRO_QUAT      prhs[3]
#define TIME_STAMP     prhs[4]
#define CALIB_PARAM    prhs[5]
#define HOMOGRAPHY_MAT prhs[6]

// find_homograpy(frame_idx, p0, p1, frame_time, frame_size, acc_trans, gyro_quat, gyro_time, calib_param)
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    if ( nrhs != 7 ||
        !mxIsDouble(FRAME_TIME) ||
        !mxIsDouble(FRAME_SIZE) ||
        !mxIsDouble(ACC_TRANS) ||
        !mxIsDouble(GYRO_QUAT) ||
        !mxIsDouble(TIME_STAMP) ||
        !mxIsDouble(CALIB_PARAM) ||
        !mxIsDouble(HOMOGRAPHY_MAT))
    {
        mexPrintf("input number %d \n", nrhs);
        mexPrintf("input %d, %d, %d, %d, %d, %d, %d \n",
            mxIsDouble(FRAME_TIME),
            mxIsDouble(FRAME_SIZE),
            mxIsDouble(ACC_TRANS),
            mxIsDouble(GYRO_QUAT),
            mxIsDouble(TIME_STAMP),
            mxIsDouble(CALIB_PARAM),
            mxIsDouble(HOMOGRAPHY_MAT));
        mexErrMsgTxt("Incorrect input/output argument format. Usage: find_homography(frame_idx, frame_time, vid_dim, gyro_quat, gyro_time, cam_param)");
    }

    double* frame_time = mxGetPr(FRAME_TIME);
    const double frame_width = mxGetPr(FRAME_SIZE)[0];
    const double frame_height = mxGetPr(FRAME_SIZE)[1];
    Vec3* acc_trans = (Vec3*)mxGetData(ACC_TRANS);
    Vec4* gyro_quat = (Vec4*)mxGetData(GYRO_QUAT);
    double* time_stamp = mxGetPr(TIME_STAMP);
    double* params = mxGetPr(CALIB_PARAM);
    Mat3* homograpy = (Mat3*)mxGetPr(HOMOGRAPHY_MAT);

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

    size_t num_gyro_samp = mxGetNumberOfElements(TIME_STAMP);

    int start0 = 0;
    int start1 = 0;

    for (int fid = 0; fid < num_gyro_samp; fid++) {

        const double ts0 = frame_time[fid] + calib.gyro_delay;
        Quatern rot0 = interp(time_stamp, gyro_quat, ts0, num_gyro_samp, start0) + Quatern(calib.gyro_drift);

        const double ts1 = frame_time[fid + 1] + calib.gyro_delay;
        Quatern rot1 = interp(time_stamp, gyro_quat, ts1, num_gyro_samp, start1) + Quatern(calib.gyro_drift);

        Vec3 trans0 = acc_trans[fid];
        Vec3 trans1 = acc_trans[fid + 1];

        //Mat4 extr0 = getExtrinsicMatrix(rot0, trans0);
        //Mat4 extr1 = getExtrinsicMatrix(rot1, trans1);
        Mat3 extr0 = getExtrinsicMatrix(rot0);
        Mat3 extr1 = getExtrinsicMatrix(rot1);

        Mat3 intrin = getIntrinsicMatrix(calib.fx, calib.fy, calib.cx, calib.cy, calib.skew);

        homograpy[fid] = getHomographyMatrix(extr0, extr1, intrin);
    }

}
