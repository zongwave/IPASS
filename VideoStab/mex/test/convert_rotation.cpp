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

#include "cpp/quaternion.h"

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
            matrix[i] = quater.rotationMatrix();
            break;

        case QUAT2EULER:
            euler[i] = quater.eulerAngles();
            break;

        case QUAT2ROTA:
            rotAixs[i] = quater.rotationAxis();
            break;

        case MATRIX2QUAT:
            qt = Quatern::createQuaternionFromRotaionMatrix(matrix[i]);
            quat[i].x = qt.v.x;
            quat[i].y = qt.v.y;
            quat[i].z = qt.v.z;
            quat[i].w = qt.w;
            break;

        case EULER2QUAT:
            qt = Quatern::createQuaternionFromEulerAngles(euler[i]);
            quat[i].x = qt.v.x;
            quat[i].y = qt.v.y;
            quat[i].z = qt.v.z;
            quat[i].w = qt.w;
            break;

        case ROTA2QUAT:
            qt = Quatern::createQuaternionFromRotationAxis(Vec3(rotAixs[i].x, rotAixs[i].y, rotAixs[i].z), rotAixs[i].w);
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
