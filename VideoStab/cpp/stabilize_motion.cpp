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

#include "stabilize_motion.h"
#include "mex.h"

#define LOG_MESSAGE mexPrintf

MotionFilter::MotionFilter (int radius, float stdev)
         : _radius (radius),
           _stdev (stdev)
{
    setFilters(radius, stdev);
}

void MotionFilter::setFilters(int radius, float stdev)
{
    _radius = radius;
    _stdev = stdev > 0.f ? stdev : std::sqrt(static_cast<float>(radius));

    int scale = 2 * _radius + 1;
    float dis = 0.0f;
    float sum = 0.0f;

    _weight.resize(2 * _radius + 1);

    for (int i = 0; i < scale; i++) {
        dis = ((float)i - radius) * ((float)i - radius);
        _weight[i] = exp(-dis / (_stdev * _stdev));
        sum += _weight[i];
    }

    for (int i = 0; i <= scale; i++) {
        _weight[i] /= sum;
    }

}


Mat3 MotionFilter::getMotion(int from, int to, const std::vector<Mat3>& motions)
{
    Mat3 M;
    M.eye();
    if (to > from)
    {
        for (int i = from; i < to; ++i)
            M = motions[i] * M;
    }
    else if (from > to)
    {
        for (int i = to; i < from; ++i)
            M = motions[i] * M;
        M = M.inverse();
    }
    return M;
}


Mat3 MotionFilter::stabilize (int idx, std::vector<Mat3>& motions, int max, int min)
{
    Mat3 res;
    res.zeros();

    double sum = 0.0f;
    int iMin = (idx - _radius) > min ? (idx - _radius) : min;
    int iMax = (idx + _radius) < max ? (idx + _radius) : max;

    for (int i = iMin; i <= iMax; ++i)
    {
        res = res + getMotion(idx, i, motions) * _weight[_radius + i - idx];
        sum += _weight[_radius + i - idx];
    }
    if (sum > 0.0f) {
        return res * (1/sum);
    }
    else
        return Mat3();
}

