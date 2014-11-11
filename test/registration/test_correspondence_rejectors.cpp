/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#include <gtest/gtest.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_poly.h>

#include <boost/random.hpp>

pcl::PointCloud<pcl::PointXYZ> cloud;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (CorrespondenceRejectors, CorrespondenceRejectionMedianDistance)
{
  pcl::CorrespondencesPtr corresps (new pcl::Correspondences ());
  for (int i = 0; i <= 10; ++i)
  {
    pcl::Correspondence c;
    c.distance = static_cast<double> (i * i);
    corresps->push_back (c);
  }

  pcl::registration::CorrespondenceRejectorMedianDistance rejector;
  rejector.setInputCorrespondences (corresps);
  rejector.setMedianFactor (2.0);

  pcl::Correspondences corresps_filtered;
  rejector.getCorrespondences (corresps_filtered);

  EXPECT_EQ (corresps_filtered.size (), 8);
  for (int i = 0; i < 8; ++i)
    EXPECT_NEAR (corresps_filtered[i].distance, static_cast<double> (i * i), 1e-5);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (CorrespondenceRejectors, CorrespondenceRejectionPoly)
{
  // Size of point cloud
  const int size = static_cast<int> (cloud.size ());
  
  // Ground truth correspondences
  pcl::Correspondences corr (size);
  for (int i = 0; i < size; ++i)
    corr[i].index_query = corr[i].index_match = i;
  
  // Scramble the first floor(size*3/4) correspondences by adding floor(size/8) to the match index
  const int last = 3*size/4;
  const int inc = size/8;
  for (int i = 0; i < last; ++i)
    corr[i].index_match += inc;  
  
  // Transform the target
  pcl::PointCloud<pcl::PointXYZ> target;
  Eigen::Vector3d t(0.1f, 0.2f, 0.3f);
  Eigen::Quaterniond q (double (std::cos (0.5*M_PI_4)), 0.0f, 0.0f, double (std::sin (0.5*M_PI_4)));
  pcl::transformPointCloud (cloud, target, t, q);
  
  // Noisify the target with a known seed and N(0, 0.005) using deterministic sampling
  boost::mt19937 rng;
  rng.seed (1e6);
  boost::normal_distribution<> nd (0, 0.005);
  boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor (rng, nd);
  for (pcl::PointCloud<pcl::PointXYZ>::iterator it = target.begin (); it != target.end (); ++it)
  {
    it->x += static_cast<double> (var_nor ());
    it->y += static_cast<double> (var_nor ());
    it->z += static_cast<double> (var_nor ());
  }
  
  // Ensure deterministic sampling inside the rejector
  std::srand (1e6);
  
  // Create a rejection object
  pcl::registration::CorrespondenceRejectorPoly<pcl::PointXYZ, pcl::PointXYZ> reject;
  reject.setIterations (10000);
  reject.setCardinality (3);
  reject.setSimilarityThreshold (0.75f);
  reject.setInputSource (cloud.makeShared ());
  reject.setInputTarget (target.makeShared ());
  
  // Run rejection
  pcl::Correspondences result;
  reject.getRemainingCorrespondences (corr, result);
  
  // Ground truth fraction of inliers and estimated fraction of inliers
  const double ground_truth_frac = double (size-last) / double (size);
  const double accepted_frac = double (result.size()) / double (size);

  /*
   * Test criterion 1: verify that the method accepts at least 25 % of the input correspondences,
   * but not too many
   */
  EXPECT_GE(accepted_frac, ground_truth_frac);
  EXPECT_LE(accepted_frac, 1.5f*ground_truth_frac);
  
  /*
   * Test criterion 2: expect high precision/recall. The true positives are the unscrambled correspondences
   * where the query/match index are equal.
   */
  unsigned int true_positives = 0;
  for (unsigned int i = 0; i < result.size(); ++i)
    if (result[i].index_query == result[i].index_match)
      ++true_positives;
  const unsigned int false_positives = static_cast<unsigned int> (result.size()) - true_positives;

  const double precision = double(true_positives) / double(true_positives+false_positives);
  const double recall = double(true_positives) / double(size-last);
  EXPECT_NEAR(precision, 1.0, 0.4);
  EXPECT_NEAR(recall, 1.0, 0.2);
}


/* ---[ */
int
  main (int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "No test files given. Please download `bunny.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  // Input
  if (pcl::io::loadPCDFile (argv[1], cloud) < 0)
  {
    std::cerr << "Failed to read test file. Please download `bunny.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }
  
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
