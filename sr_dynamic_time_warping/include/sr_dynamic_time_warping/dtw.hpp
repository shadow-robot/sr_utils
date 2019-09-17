/**
 * @file   dtw.hpp
 * @author Yi Li <yi@shadowrobot.com>
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
 * @brief An implementation of dynamic time warping (DTW).
 *
 *
 */

#pragma once

#include <ros/ros.h>
#include "sr_dynamic_time_warping/vector_dtw.hpp"

namespace shadowrobot {

class DTW
{
public:
  /**
   * A vector-based DTW constructor.
   *
   * Note that the chains can have different length.
   * In fact, even overlapping nodes are allowed.
   *
   * @param chain1 the first chain in 3D
   * @param chain2 the second chain in 3D
   * @param desired_number_of_nodes may be reset by the contructor
   */
  DTW(const std::vector<Eigen::Vector3d>& chain1,
      const std::vector<Eigen::Vector3d>& chain2,
      const unsigned int desired_number_of_nodes = 0);
  virtual ~DTW();

  /**
   * Get the first chain stored in the class.
   * @return the first chain
   */
  const std::vector<Eigen::Vector3d>& get_chain1(void) const;

  /**
   * Get the second chain stored in the class.
   * @return the second chain
   */
  const std::vector<Eigen::Vector3d>& get_chain2(void) const;

  /**
   * Get the number of desired nodes.
   * @return the number of desired nodes
   */
  const unsigned int get_desired_number_of_nodes(void) const;

  /**
   * Compute the DTW distance between the two chains in the class.
   * @return the DTW distance
   */
  double
  dtw_distance(void);

  /**
   * Compute the DTW distance between the two chains.
   * Note that the chains must have the same number of nodes.
   * @param chain1 the first chain
   * @param chain1 the second chain
   * @return the DTW distance
   */
  static double
  dtw_distance(const std::vector<Eigen::Vector3d>& chain1,
               const std::vector<Eigen::Vector3d>& chain2);

  /**
   * Insert additional nodes if necessary so that the returned chain
   * has the desired number of nodes.
   * @param chain_in the given chain
   * @param desired_number_of_nodes the desired number of nodes
   * @return a chain with the desired number of nodes
   */
  static std::vector<Eigen::Vector3d>
  insert_additional_nodes(const std::vector<Eigen::Vector3d>& chain_in,
                          const unsigned int desired_number_of_nodes);

  /**
   * Make sure that neighboring nodes inside the chain do not overlap.
   * @param chain_in the given chain
   * @return a chain with overlapping nodes removed
   */
  static std::vector<Eigen::Vector3d>
  remove_overlapping_nodes(const std::vector<Eigen::Vector3d>& chain_in);

  /**
   * Merge two chains at the connection point
   * @param chain_1 the first chain
   * @param connect_at_tail_1 the connection point is at the tail of chain_1
   * @param chain_2 the second chain
   * @param connect_at_tail_2 the connection point is at the tail of chain_2
   * @return one single chain after merging chain_1 and chain_2
   */
  static std::vector<Eigen::Vector3d>
  merge(const std::vector<Eigen::Vector3d>& chain_1,
        const bool connect_at_tail_1,
        const std::vector<Eigen::Vector3d>& chain_2,
        const bool connect_at_tail_2);

private:
  std::vector<Eigen::Vector3d> chain1_;
  std::vector<Eigen::Vector3d> chain2_;

  unsigned desired_number_of_nodes_;
};

} // namespace shadowrobot

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
