/**
 * @file   dtw.cpp
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

#include "sr_dynamic_time_warping/dtw.hpp"

namespace shadowrobot
{

//-------------------------------------------------------------------------------

DTW::DTW(const std::vector<Eigen::Vector3d>& chain1,
         const std::vector<Eigen::Vector3d>& chain2,
         const unsigned int desired_number_of_nodes)
{
  std::vector<Eigen::Vector3d> chain1m =
    DTW::remove_overlapping_nodes(chain1);
  std::vector<Eigen::Vector3d> chain2m =
    DTW::remove_overlapping_nodes(chain2);

  unsigned int max_chain_len = std::max<unsigned int>(chain1m.size(), chain2m.size());
  if (desired_number_of_nodes < max_chain_len)
    desired_number_of_nodes_ = max_chain_len;
  else
    desired_number_of_nodes_ = desired_number_of_nodes;

  chain1_ = DTW::insert_additional_nodes(chain1m, desired_number_of_nodes_);
  chain2_ = DTW::insert_additional_nodes(chain2m, desired_number_of_nodes_);
}

//-------------------------------------------------------------------------------

DTW::~DTW()
{
}

//-------------------------------------------------------------------------------

const std::vector<Eigen::Vector3d>&
DTW::get_chain1(void) const
{
  return chain1_;
}

//-------------------------------------------------------------------------------

const std::vector<Eigen::Vector3d>&
DTW::get_chain2(void) const
{
  return chain2_;
}

//-------------------------------------------------------------------------------

const unsigned int
DTW::get_desired_number_of_nodes(void) const
{
  return desired_number_of_nodes_;
}

//-------------------------------------------------------------------------------

double
DTW::dtw_distance(void)
{
  const std::vector<Eigen::Vector3d>& chain1 = this->get_chain1();
  const std::vector<Eigen::Vector3d>& chain2 = this->get_chain2();
  return DTW::dtw_distance(chain1, chain2);
}

//-------------------------------------------------------------------------------

double
DTW::dtw_distance(const std::vector<Eigen::Vector3d>& chain1,
                  const std::vector<Eigen::Vector3d>& chain2)
{
  assert(chain1.size() == chain2.size());

  // n: the length of the time series
  unsigned int n = chain1.size();

  // constraint: the maximum warping distance. Typically: constraint = n/10.
  unsigned int constraint = static_cast<unsigned int>(n / 10);
  constraint = (constraint >= 3) ? constraint : 3;

  // Compute DTW distance
  VectorDTW dtw(n, constraint);
  double dtw_dis = dtw.DTW_distance(chain1, chain2);
  return dtw_dis;
}

//-------------------------------------------------------------------------------

std::vector<Eigen::Vector3d>
DTW::insert_additional_nodes(const std::vector<Eigen::Vector3d>& chain_in,
                             const unsigned int desired_number_of_nodes)
{
  std::vector<Eigen::Vector3d> chain_out(chain_in);
  assert(chain_out.size() >= 2 && chain_out.size() <= desired_number_of_nodes);

  while (chain_out.size() < desired_number_of_nodes)
  {
    double max_len = 0.0;
    std::vector<Eigen::Vector3d>::iterator max_len_iter = chain_out.end();
    for (std::vector<Eigen::Vector3d>::iterator iter = chain_out.begin();
         iter != chain_out.end();
         iter++)
    {
      std::vector<Eigen::Vector3d>::iterator next_iter = iter;
      if (++next_iter == chain_out.end())
        break;

      const Eigen::Vector3d &pos_fr = *iter;
      const Eigen::Vector3d &pos_to = *next_iter;
      double len = (pos_to - pos_fr).norm();
      if (len > max_len)
      {
        max_len = len;
        max_len_iter = iter;
      }
    }

    if (max_len_iter != chain_out.end())
    {
      std::vector<Eigen::Vector3d>::iterator next_iter = max_len_iter;
      next_iter++;
      const Eigen::Vector3d &pos_fr = *max_len_iter;
      const Eigen::Vector3d &pos_to = *next_iter;
      const Eigen::Vector3d pos_mid = pos_fr + 0.5 * (pos_to - pos_fr);
      chain_out.insert(next_iter, pos_mid);
    }
  }

  return chain_out;
}

//-------------------------------------------------------------------------------

std::vector<Eigen::Vector3d>
DTW::remove_overlapping_nodes(const std::vector<Eigen::Vector3d>& chain_in)
{
  std::vector<Eigen::Vector3d> chain_out;

  if (chain_in.empty())
    return chain_out;

  chain_out.push_back(chain_in.front());

  if (chain_in.size() == 1)
    return chain_out;

  // Discard nodes that occupy the same position.
  for (unsigned int i = 1; i < chain_in.size(); i++)
  {
    const Eigen::Vector3d& n_out = chain_out.back();
    const Eigen::Vector3d& n_in  = chain_in[i];
    if ( (n_out - n_in).norm() > 0.0 )
      chain_out.push_back(n_in);
  }

  return chain_out;
}

//-------------------------------------------------------------------------------

std::vector<Eigen::Vector3d>
DTW::merge(const std::vector<Eigen::Vector3d>& chain_1,
           const bool connect_at_tail_1,
           const std::vector<Eigen::Vector3d>& chain_2,
           const bool connect_at_tail_2)
{
  std::vector<Eigen::Vector3d> chain_out;

  if (connect_at_tail_1)
    chain_out.insert(chain_out.end(), chain_1.begin(), chain_1.end());
  else
    chain_out.insert(chain_out.end(), chain_1.rbegin(), chain_1.rend());

  if (connect_at_tail_2)
    chain_out.insert(chain_out.end(), chain_2.rbegin(), chain_2.rend());
  else
    chain_out.insert(chain_out.end(), chain_2.begin(), chain_2.end());

  return chain_out;
}

//-------------------------------------------------------------------------------

} // namespace shadowrobot

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
