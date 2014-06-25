/**
 * @file   vector_dtw.hpp
 * @author Toni Oliver <toni@shadowrobot.com>
 *
 * Copyright (c) 2013, Shadow Robot Company, All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library.
 *
 * @brief An implementation of Dynamic Time Warping algorithm.
 * Based on http://en.wikipedia.org/wiki/Dynamic_time_warping
 *
 */

#pragma once

#include <cmath>
#include <cassert>
#include <limits>
#include <vector>
#include <Eigen/Dense>

//-------------------------------------------------------------------------------

namespace shadowrobot {

class VectorDTW
{
public:
  /**
   * n is the length of the vectors
   *
   * window is the maximum warping distance.
   * Typically: window = n/10.
   * If you set window = n, the distance calculation will be slower.
   */
  VectorDTW(unsigned int n, unsigned int window)
    : DTW_( Eigen::MatrixXd::Constant(n+1, n+1, std::numeric_limits<double>::max()) ),
      N_(n),
      w_(window)
  {
  }

  /**
   * This uses Eigen::Vector3d norm().
   */
  inline double DTW_distance(const std::vector<Eigen::Vector3d> &s,
                             const std::vector<Eigen::Vector3d> &t)
  {
    assert(static_cast<int>(s.size()) == N_);
    assert(static_cast<int>(t.size()) == N_);

    DTW_(0, 0) = 0.0;

    double cost;
    for (int i = 0; i < N_; ++i)
    {
      for (int j = std::max(0, i - w_);
           j < std::min(N_, i + w_ + 1);
           ++j)
      {
        cost = (s[i] - t[j]).norm();
        DTW_(i+1, j+1) = cost + std::min(DTW_(i, j+1), std::min(DTW_(i+1, j), DTW_(i, j)));
      }
    }

    return DTW_(N_, N_);
  }

private:
  Eigen::MatrixXd DTW_;
  int N_;
  int w_;
};

} // end of namespace shadowrobot

//-------------------------------------------------------------------------------
