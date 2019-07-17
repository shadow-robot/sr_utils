#include <ros/ros.h>
#include <ros/package.h>
#include <gtest/gtest.h>
#include <cmath>

#include "sr_dynamic_time_warping/dtw.hpp"
#include "sr_dynamic_time_warping/vector_dtw.hpp"

using shadowrobot::DTW;
using shadowrobot::VectorDTW;

//-------------------------------------------------------------------------------

TEST(TestDTW, testChainMerge)
{
  std::vector<Eigen::Vector3d> chain_1;
  chain_1.push_back( Eigen::Vector3d(0.0, -1.0, 0.0) );
  chain_1.push_back( Eigen::Vector3d(1.0, -1.0, 0.0) );
  chain_1.push_back( Eigen::Vector3d(2.0, -1.0, 0.0) );

  std::vector<Eigen::Vector3d> chain_2;
  chain_2.push_back( Eigen::Vector3d(0.0,  1.0, 0.0) );
  chain_2.push_back( Eigen::Vector3d(1.0,  1.0, 0.0) );
  chain_2.push_back( Eigen::Vector3d(2.0,  1.0, 0.0) );

  bool connect_at_tail_1 = true;
  bool connect_at_tail_2 = true;

  std::vector<Eigen::Vector3d> chain_out =
    DTW::merge(chain_1, !connect_at_tail_1,
               chain_2, !connect_at_tail_2);

  ASSERT_EQ(chain_out.size(), chain_1.size() + chain_2.size());
}

//-------------------------------------------------------------------------------

TEST(TestDTW, testInsertAdditionalNodes)
{
  std::vector<Eigen::Vector3d> chain_in;
  chain_in.push_back( Eigen::Vector3d(0.0, 0.0, 0.0) );
  chain_in.push_back( Eigen::Vector3d(1.0, 0.0, 0.0) );
  chain_in.push_back( Eigen::Vector3d(1.0, 0.0, 0.0) );
  chain_in.push_back( Eigen::Vector3d(1.0, 0.0, 0.0) );
  chain_in.push_back( Eigen::Vector3d(5.0, 0.0, 0.0) );

  unsigned int desired_number_of_nodes = 6;
  std::vector<Eigen::Vector3d> chain_out =
    DTW::insert_additional_nodes(chain_in, desired_number_of_nodes);

  ASSERT_EQ(chain_out.size(), desired_number_of_nodes);
}

//-------------------------------------------------------------------------------

TEST(TestDTW, testDTW)
{
  std::vector<Eigen::Vector3d> chain1;
  chain1.push_back( Eigen::Vector3d(0.0, 0.0, 0.0) );
  chain1.push_back( Eigen::Vector3d(1.0, 0.0, 0.0) );
  chain1.push_back( Eigen::Vector3d(2.0, 0.0, 0.0) );

  std::vector<Eigen::Vector3d> chain2;
  chain2.push_back( Eigen::Vector3d(0.0, 1.0, 0.0) );
  chain2.push_back( Eigen::Vector3d(1.0, 1.0, 0.0) );
  chain2.push_back( Eigen::Vector3d(2.0, 1.0, 0.0) );
  chain2.push_back( Eigen::Vector3d(3.0, 1.0, 0.0) );

  unsigned int desired_number_of_nodes
    = std::max<unsigned int>(chain1.size(), chain2.size());
  DTW dtw(chain1, chain2);

  ASSERT_EQ( dtw.get_desired_number_of_nodes(), desired_number_of_nodes );
}

//-------------------------------------------------------------------------------

TEST(TestDTW, testDTWdistance)
{
  std::vector<Eigen::Vector3d> chain1;
  chain1.push_back( Eigen::Vector3d(0.0, 0.0, 0.0) );
  chain1.push_back( Eigen::Vector3d(1.0, 0.0, 0.0) );
  chain1.push_back( Eigen::Vector3d(2.0, 0.0, 0.0) );
  chain1.push_back( Eigen::Vector3d(2.0, 0.0, 2.0) );
  chain1.push_back( Eigen::Vector3d(4.0, 1.0, 2.0) );

  std::vector<Eigen::Vector3d> chain2;
  chain2.push_back( Eigen::Vector3d(0.0, 1.0, 0.0) );
  chain2.push_back( Eigen::Vector3d(1.0, 1.0, 0.0) );
  chain2.push_back( Eigen::Vector3d(2.0, 1.0, 0.0) );
  chain2.push_back( Eigen::Vector3d(3.0, 1.0, 0.0) );
  chain2.push_back( Eigen::Vector3d(3.0, 1.0, 0.0) );

  ASSERT_EQ(chain1.size(), chain2.size());

  size_t n = chain1.size();
  // constraint: the maximum warping distance. Typically: constraint = n/10.
  unsigned int window = static_cast<unsigned int>(n / 10);
  window = (window >= 3) ? window : 3;

  // Compute DTW distance
  VectorDTW dtw(n, window);
  double dtw_dis = dtw.DTW_distance(chain1, chain2);

  const float abs_error = 0.0001;

  ASSERT_NEAR(dtw_dis, 7.68556, abs_error);
}

//-------------------------------------------------------------------------------

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

//-------------------------------------------------------------------------------
