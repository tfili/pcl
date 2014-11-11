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
#include <pcl/point_cloud.h>
#include <pcl/pcl_tests.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/board.h>

using namespace pcl;
using namespace pcl::test;
using namespace pcl::io;
using namespace std;

typedef search::KdTree<PointXYZ>::Ptr KdTreePtr;

PointCloud<PointXYZ> cloud;
vector<int> indices;
KdTreePtr tree;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, BOARDLocalReferenceFrameEstimation)
{
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  PointCloud<ReferenceFrame> bunny_LRF;

  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));

  // Compute normals
  NormalEstimation<PointXYZ, Normal> ne;

  ne.setRadiusSearch (0.01);
  ne.setViewPoint (1, 1, 10);
  ne.setInputCloud (cloud.makeShared ());
  ne.setSearchMethod (tree);
  ne.setIndices (indicesptr);

  ne.compute (*normals);

  // Compute BOARD LRF
  BOARDLocalReferenceFrameEstimation<PointXYZ, Normal, ReferenceFrame> lrf_estimator;

  double meshRes = 0.001;

  lrf_estimator.setFindHoles (true);
  lrf_estimator.setRadiusSearch (15 * meshRes);
  lrf_estimator.setTangentRadius (15 * meshRes);

  lrf_estimator.setInputCloud (cloud.makeShared ());
  lrf_estimator.setInputNormals (normals);
  lrf_estimator.setSearchMethod (tree);
  lrf_estimator.setIndices (indicesptr);

  lrf_estimator.compute (bunny_LRF);

  // TESTS
  EXPECT_EQ (indices.size (), bunny_LRF.size ());

  EXPECT_FALSE (bunny_LRF.is_dense);
  //EXPECT_EQ (numeric_limits<double>::max (), bunny_LRF.at (24).confidence);
  EXPECT_TRUE (pcl_isnan (bunny_LRF.at (24).x_axis[0]));

  // Expected Results
  //double point_15_conf = -9.06301;
  Eigen::Vector3d point_15_x (-0.784923f, 0.208529, 0.583448);
  Eigen::Vector3d point_15_y (0.334206, -0.650436, 0.682085);
  Eigen::Vector3d point_15_z (0.52173f, 0.730376, 0.440851);

  //double point_45_conf = -9.55398;
  Eigen::Vector3d point_45_x (0.909111, 0.30943f, 0.278874f);
  Eigen::Vector3d point_45_y (-0.362239, 0.917811, 0.162501);
  Eigen::Vector3d point_45_z (-0.205671, -0.248751, 0.946479);

  //double point_163_conf = -9.04891;
  Eigen::Vector3d point_163_x (-0.443962f, -0.890073f, -0.103285);
  Eigen::Vector3d point_163_y (0.746929, -0.30394f, -0.591369);
  Eigen::Vector3d point_163_z (0.494969, -0.339693f, 0.799759);

  //double point_253_conf = -9.09443;
  Eigen::Vector3d point_253_x (-0.616855, 0.757286, -0.214495);
  Eigen::Vector3d point_253_y (-0.661937, -0.646584f, -0.379168);
  Eigen::Vector3d point_253_z (-0.425827, -0.0919098, 0.900124f);

  //Test Results
  //EXPECT_NEAR (point_15_conf,bunny_LRF.at (15).confidence, 1E-3);
  //EXPECT_NEAR_VECTORS (point_15_x, bunny_LRF.at (15).x_axis.getNormalVector3dMap (), 1E-3);
  //EXPECT_NEAR_VECTORS (point_15_y, bunny_LRF.at (15).y_axis.getNormalVector3dMap (), 1E-3);
  //EXPECT_NEAR_VECTORS (point_15_z, bunny_LRF.at (15).z_axis.getNormalVector3dMap (), 1E-3);
  for (int d = 0; d < 3; ++d)
  {
    EXPECT_NEAR (point_15_x[d], bunny_LRF.at (15).x_axis[d], 1E-3);
    EXPECT_NEAR (point_15_y[d], bunny_LRF.at (15).y_axis[d], 1E-3);
    EXPECT_NEAR (point_15_z[d], bunny_LRF.at (15).z_axis[d], 1E-3);

    EXPECT_NEAR (point_45_x[d], bunny_LRF.at (45).x_axis[d], 1E-3);
    EXPECT_NEAR (point_45_y[d], bunny_LRF.at (45).y_axis[d], 1E-3);
    EXPECT_NEAR (point_45_z[d], bunny_LRF.at (45).z_axis[d], 1E-3);

    EXPECT_NEAR (point_163_x[d], bunny_LRF.at (163).x_axis[d], 1E-3);
    EXPECT_NEAR (point_163_y[d], bunny_LRF.at (163).y_axis[d], 1E-3);
    EXPECT_NEAR (point_163_z[d], bunny_LRF.at (163).z_axis[d], 1E-3);

    EXPECT_NEAR (point_253_x[d], bunny_LRF.at (253).x_axis[d], 1E-3);
    EXPECT_NEAR (point_253_y[d], bunny_LRF.at (253).y_axis[d], 1E-3);
    EXPECT_NEAR (point_253_z[d], bunny_LRF.at (253).z_axis[d], 1E-3);
  }
}

/* ---[ */
int
main (int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "No test file given. Please download `bun0.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  if (loadPCDFile<PointXYZ> (argv[1], cloud) < 0)
  {
    std::cerr << "Failed to read test file. Please download `bun0.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  indices.resize (cloud.points.size ());
  for (size_t i = 0; i < indices.size (); ++i)
    indices[i] = static_cast<int> (i);

  tree.reset (new search::KdTree<PointXYZ> (false));
  tree->setInputCloud (cloud.makeShared ());

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
