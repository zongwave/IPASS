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

#include <math.h> 
#include <time.h>

#include "../../cpp/quaternion.h"

typedef Vector2<double> Vec2;
typedef Vector3<double> Vec3;
typedef Vector4<double> Vec4;
typedef Matrix3<double> Mat3;
typedef Matrix4<double> Mat4;
typedef Quaternion<double> Quatern;

#define CONVERSION   prhs[0]
#define QUATERNION   prhs[1]
#define EULER_ANGLE  prhs[2]
#define ROTA_ANGLE   prhs[3]
#define ROTA_MATRIX  prhs[4]

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    if ( nrhs != 5 ||
         !mxIsInt32(CONVERSION)  ||
         !mxIsDouble(QUATERNION) ||
         !mxIsDouble(EULER_ANGLE)||
         !mxIsDouble(ROTA_ANGLE) ||
         !mxIsDouble(ROTA_MATRIX) )
    {
        mexPrintf("input %d, %d, %d, %d, %d \n",
            !mxIsInt32(CONVERSION),
            !mxIsDouble(QUATERNION),
            !mxIsDouble(EULER_ANGLE),
            !mxIsDouble(ROTA_ANGLE),
            !mxIsDouble(ROTA_MATRIX));
        mexErrMsgTxt("Incorrect input/output argument format");
    }

    int conversion = mxGetScalar(CONVERSION);
    Vec4* quat = (Vec4*)mxGetData(QUATERNION);
    Vec3* euler = (Vec3*)mxGetData(EULER_ANGLE);
    Vec4* rotAixs = (Vec4*)mxGetData(ROTA_ANGLE);
    Mat3* matrix = (Mat3*)mxGetData(ROTA_MATRIX);
    int num_quat = mxGetNumberOfElements(QUATERNION) / 4;

    for (int i = 0; i < num_quat; ++i)
    {
        Quatern quater(quat[i]);
        Quatern qt;

        switch (conversion) {
        case QUAT2MATRIX:
            matrix[i] = quater.rotation_matrix();
            qt = Quatern::create_quaternion(matrix[i]);
            mexPrintf("quatern(%d) = [ %lf, %lf, %lf, %lf ] \n", i, qt.v.x, qt.v.y, qt.v.z, qt.w);
            break;

        case QUAT2EULER:
            euler[i] = quater.euler_angles();
            break;

        case QUAT2ROTA:
            rotAixs[i] = quater.rotation_axis();
            break;

        case MATRIX2QUAT:
            qt = Quatern::create_quaternion(matrix[i]);
            quat[i].x = qt.v.x;
            quat[i].y = qt.v.y;
            quat[i].z = qt.v.z;
            quat[i].w = qt.w;
            break;

        case EULER2QUAT:
            qt = Quatern::create_quaternion(euler[i]);
            quat[i].x = qt.v.x;
            quat[i].y = qt.v.y;
            quat[i].z = qt.v.z;
            quat[i].w = qt.w;
            break;

        case ROTA2QUAT:
            qt = Quatern::create_quaternion(Vec3(rotAixs[i].x, rotAixs[i].y, rotAixs[i].z), rotAixs[i].w);
            quat[i].x = qt.v.x;
            quat[i].y = qt.v.y;
            quat[i].z = qt.v.z;
            quat[i].w = qt.w;
            break;

        default:
            break;
        }
    }
}


void test_matrix_multiplication()
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

