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
