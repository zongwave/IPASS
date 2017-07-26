/*
 * stabilize_motion.cpp - Stabilize motion with Gaussian low pass filter
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

