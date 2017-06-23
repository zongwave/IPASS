/*
 * stabilize_motion.h - Stabilize motion with Gaussian low pass filter
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

#ifndef STABILIZE_MOTION_H
#define STABILIZE_MOTION_H

#include <vector>

#include "calc_projective2d.h"

class MotionFilter
{
public:
    MotionFilter (int radius, float stdev);

    void setFilters (int radius, float stdev = -1.0f);

    int radius () const {
        return _radius;
    };
    float stdev () const {
        return _stdev;
    };

    Mat3 stabilize (int idx, std::vector<Mat3>& motions, int max, int min);

protected:
    Mat3 getMotion (int from, int to, const std::vector<Mat3>& motions);

private:
    int _radius;
    float _stdev;
    std::vector<float> _weight;
};

#endif
