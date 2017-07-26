/*
 * convert_rotation.cpp - Quaternion/Matrix rotation test function
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
                      A3(1, 1), A3(1, 2), A3(1, 3),
                      A3(2, 1), A3(2, 2), A3(2, 3),
                      A3(3, 1), A3(3, 2), A3(3, 3));
    mexPrintf("A3_T = [ %lf, %lf, %lf ; %lf, %lf, %lf ; %lf, %lf, %lf ] \n",
                        AA3(1, 1), AA3(1, 2), AA3(1, 3),
                        AA3(2, 1), AA3(2, 2), AA3(2, 3),
                        AA3(3, 1), AA3(3, 2), AA3(3, 3));
    mexPrintf("invA3 = [ %lf, %lf, %lf ; %lf, %lf, %lf ; %lf, %lf, %lf ] \n",
                         a3(1, 1), a3(1, 2), a3(1, 3),
                         a3(2, 1), a3(2, 2), a3(2, 3),
                         a3(3, 1), a3(3, 2), a3(3, 3));
    mexPrintf("B3 = [ %lf, %lf, %lf ; %lf, %lf, %lf ; %lf, %lf, %lf ] \n",
                      B3(1, 1), B3(1, 2), B3(1, 3),
                      B3(2, 1), B3(2, 2), B3(2, 3),
                      B3(3, 1), B3(3, 2), B3(3, 3));
    mexPrintf("invB3 = [ %lf, %lf, %lf ; %lf, %lf, %lf ; %lf, %lf, %lf ] \n",
                          b3(1, 1), b3(1, 2), b3(1, 3),
                          b3(2, 1), b3(2, 2), b3(2, 3),
                          b3(3, 1), b3(3, 2), b3(3, 3));
    mexPrintf("B3_T = [ %lf, %lf, %lf ; %lf, %lf, %lf ; %lf, %lf, %lf ] \n",
                        BB3(1, 1), BB3(1, 2), BB3(1, 3),
                        BB3(2, 1), BB3(2, 2), BB3(2, 3),
                        BB3(3, 1), BB3(3, 2), BB3(3, 3));
    mexPrintf("C3 = [ %lf, %lf, %lf ; %lf, %lf, %lf ; %lf, %lf, %lf ] \n",
                      C3(1, 1), C3(1, 2), C3(1, 3),
                      C3(2, 1), C3(2, 2), C3(2, 3),
                      C3(3, 1), C3(3, 2), C3(3, 3));
    mexPrintf("invC3 = [ %lf, %lf, %lf ; %lf, %lf, %lf ; %lf, %lf, %lf ] \n",
                         c3(1, 1), c3(1, 2), c3(1, 3),
                         c3(2, 1), c3(2, 2), c3(2, 3),
                         c3(3, 1), c3(3, 2), c3(3, 3));
    mexPrintf("C3_T = [ %lf, %lf, %lf ; %lf, %lf, %lf ; %lf, %lf, %lf ] \n",
                        CC3(1, 1), CC3(1, 2), CC3(1, 3),
                        CC3(2, 1), CC3(2, 2), CC3(2, 3),
                        CC3(3, 1), CC3(3, 2), CC3(3, 3));
    mexPrintf("D3 = [ %lf, %lf, %lf ; %lf, %lf, %lf ; %lf, %lf, %lf ] \n",
                      D3(1, 1), D3(1, 2), D3(1, 3),
                      D3(2, 1), D3(2, 2), D3(2, 3),
                      D3(3, 1), D3(3, 2), D3(3, 3));
    mexPrintf("invD3 = [ %lf, %lf, %lf ; %lf, %lf, %lf ; %lf, %lf, %lf ] \n",
                         d3(1, 1), d3(1, 2), d3(1, 3),
                         d3(2, 1), d3(2, 2), d3(2, 3),
                         d3(3, 1), d3(3, 2), d3(3, 3));
    mexPrintf("D3_T = [ %lf, %lf, %lf ; %lf, %lf, %lf ; %lf, %lf, %lf ] \n",
                        DD3(1, 1), DD3(1, 2), DD3(1, 3),
                        DD3(2, 1), DD3(2, 2), DD3(2, 3),
                        DD3(3, 1), DD3(3, 2), DD3(3, 3));

    Mat4 A4(Vec4(rand()/(double)(RAND_MAX/10), -rand()/(double)(RAND_MAX/10),
                rand()/(double)(RAND_MAX/10), -rand()/(double)(RAND_MAX/10)),
           Vec4(rand()/(double)(RAND_MAX/10), rand()/(double)(RAND_MAX/10),
                rand()/(double)(RAND_MAX/10), rand()/(double)(RAND_MAX/10)),
           Vec4(-rand()/(double)(RAND_MAX/10), rand()/(double)(RAND_MAX/10),
                rand()/(double)(RAND_MAX/10), rand()/(double)(RAND_MAX/10)),
           Vec4(-rand()/(double)(RAND_MAX/10), rand()/(double)(RAND_MAX/10),
                rand()/(double)(RAND_MAX/10), -rand()/(double)(RAND_MAX/10)));
    Mat4 B4(Vec4(rand()/(double)(RAND_MAX/10), rand()/(double)(RAND_MAX/10),
                -rand()/(double)(RAND_MAX/10), rand()/(double)(RAND_MAX/10)),
           Vec4(rand()/(double)(RAND_MAX/10), rand()/(double)(RAND_MAX/10),
                rand()/(double)(RAND_MAX/10), rand()/(double)(RAND_MAX/10)),
           Vec4(rand()/(double)(RAND_MAX/10), -rand()/(double)(RAND_MAX/10),
                -rand()/(double)(RAND_MAX/10), rand()/(double)(RAND_MAX/10)),
           Vec4(rand()/(double)(RAND_MAX/10), rand()/(double)(RAND_MAX/10),
                -rand()/(double)(RAND_MAX/10), rand()/(double)(RAND_MAX/10)));

    Mat4 C4 = A4 * B4;
    Mat4 D4 = B4 * A4;

    Mat4 AA4 = A4.transpose();
    Mat4 BB4 = B4.transpose();
    Mat4 CC4 = C4.transpose();
    Mat4 DD4 = D4.transpose();

    Mat4 a4 = A4.inverse();
    Mat4 b4 = B4.inverse();
    Mat4 c4 = C4.inverse();
    Mat4 d4 = D4.inverse();

    mexPrintf("A4 = [ %lf, %lf, %lf , %lf ; %lf, %lf, %lf , %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ] \n",
                      A4(1, 1), A4(1, 2), A4(1, 3), A4(1, 4),
                      A4(2, 1), A4(2, 2), A4(2, 3), A4(2, 4),
                      A4(3, 1), A4(3, 2), A4(3, 3), A4(3, 4),
                      A4(4, 1), A4(4, 2), A4(4, 3), A4(4, 4));
    mexPrintf("invA4 = [ %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ] \n",
                        a4(1, 1), a4(1, 2), a4(1, 3), a4(1, 4),
                        a4(2, 1), a4(2, 2), a4(2, 3), a4(2, 4),
                        a4(3, 1), a4(3, 2), a4(3, 3), a4(3, 4),
                        a4(4, 1), a4(4, 2), a4(4, 3), a4(4, 4));
    mexPrintf("A4_T = [ %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ] \n",
                       AA4(1, 1), AA4(1, 2), AA4(1, 3), AA4(1, 4),
                       AA4(2, 1), AA4(2, 2), AA4(2, 3), AA4(2, 4),
                       AA4(3, 1), AA4(3, 2), AA4(3, 3), AA4(3, 4),
                       AA4(4, 1), AA4(4, 2), AA4(4, 3), AA4(4, 4));
    mexPrintf("B4 = [ %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ] \n",
                     B4(1, 1), B4(1, 2), B4(1, 3), B4(1, 4),
                     B4(2, 1), B4(2, 2), B4(2, 3), B4(2, 4),
                     B4(3, 1), B4(3, 2), B4(3, 3), B4(3, 4),
                     B4(4, 1), B4(4, 2), B4(4, 3), B4(4, 4));
    mexPrintf("invB4 = [ %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ] \n",
                        b4(1, 1), b4(1, 2), b4(1, 3), b4(1, 4),
                        b4(2, 1), b4(2, 2), b4(2, 3), b4(2, 4),
                        b4(3, 1), b4(3, 2), b4(3, 3), b4(3, 4),
                        b4(4, 1), b4(4, 2), b4(4, 3), b4(4, 4));
    mexPrintf("B4_T = [ %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ] \n",
                       BB4(1, 1), BB4(1, 2), BB4(1, 3), BB4(1, 4),
                       BB4(2, 1), BB4(2, 2), BB4(2, 3), BB4(2, 4),
                       BB4(3, 1), BB4(3, 2), BB4(3, 3), BB4(3, 4),
                       BB4(4, 1), BB4(4, 2), BB4(4, 3), BB4(4, 4));
    mexPrintf("C4 = [ %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ] \n",
                     C4(1, 1), C4(1, 2), C4(1, 3), C4(1, 4),
                     C4(2, 1), C4(2, 2), C4(2, 3), C4(2, 4),
                     C4(3, 1), C4(3, 2), C4(3, 3), C4(3, 4),
                     C4(4, 1), C4(4, 2), C4(4, 3), C4(4, 4));
    mexPrintf("invC4 = [ %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ] \n",
                        c4(1, 1), c4(1, 2), c4(1, 3), c4(1, 4),
                        c4(2, 1), c4(2, 2), c4(2, 3), c4(2, 4),
                        c4(3, 1), c4(3, 2), c4(3, 3), c4(3, 4),
                        c4(4, 1), c4(4, 2), c4(4, 3), c4(4, 4));
    mexPrintf("C4_T = [ %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ] \n",
                       CC4(1, 1), CC4(1, 2), CC4(1, 3), CC4(1, 4),
                       CC4(2, 1), CC4(2, 2), CC4(2, 3), CC4(2, 4),
                       CC4(3, 1), CC4(3, 2), CC4(3, 3), CC4(3, 4),
                       CC4(4, 1), CC4(4, 2), CC4(4, 3), CC4(4, 4));
    mexPrintf("D4 = [ %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ] \n",
                     D4(1, 1), D4(1, 2), D4(1, 3), D4(1, 4),
                     D4(2, 1), D4(2, 2), D4(2, 3), D4(2, 4),
                     D4(3, 1), D4(3, 2), D4(3, 3), D4(3, 4),
                     D4(4, 1), D4(4, 2), D4(4, 3), D4(4, 4));
    mexPrintf("invD4 = [ %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ] \n",
                        d4(1, 1), d4(1, 2), d4(1, 3), d4(1, 4),
                        d4(2, 1), d4(2, 2), d4(2, 3), d4(2, 4),
                        d4(3, 1), d4(3, 2), d4(3, 3), d4(3, 4),
                        d4(4, 1), d4(4, 2), d4(4, 3), d4(4, 4));
    mexPrintf("D4_T = [ %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ; %lf, %lf, %lf, %lf ] \n",
                       DD4(1, 1), DD4(1, 2), DD4(1, 3), DD4(1, 4),
                       DD4(2, 1), DD4(2, 2), DD4(2, 3), DD4(2, 4),
                       DD4(3, 1), DD4(3, 2), DD4(3, 3), DD4(3, 4),
                       DD4(4, 1), DD4(4, 2), DD4(4, 3), DD4(4, 4));
}

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

	//test_matrix_multiplication();
}

