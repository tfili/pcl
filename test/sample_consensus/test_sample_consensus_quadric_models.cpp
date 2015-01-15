/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  Copyright (c) 2014-, Open Perception, Inc.
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
 */

#include <gtest/gtest.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model_cone.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl/sample_consensus/sac_model_normal_sphere.h>

using namespace pcl;

typedef SampleConsensusModelSphere<PointXYZ>::Ptr SampleConsensusModelSpherePtr;
typedef SampleConsensusModelCone<PointXYZ, Normal>::Ptr SampleConsensusModelConePtr;
typedef SampleConsensusModelCircle2D<PointXYZ>::Ptr SampleConsensusModelCircle2DPtr;
typedef SampleConsensusModelCircle3D<PointXYZ>::Ptr SampleConsensusModelCircle3DPtr;
typedef SampleConsensusModelCylinder<PointXYZ, Normal>::Ptr SampleConsensusModelCylinderPtr;
typedef SampleConsensusModelNormalSphere<PointXYZ, Normal>::Ptr SampleConsensusModelNormalSpherePtr;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (SampleConsensusModelSphere, RANSAC)
{
  srand (0);

  // Use a custom point cloud for these tests until we need something better
  PointCloud<PointXYZ> cloud;
  cloud.points.resize (10);
  cloud.points[0].getVector3dMap () << 1.7068, 1.0684f, 2.2147;
  cloud.points[1].getVector3dMap () << 2.4708, 2.3081, 1.1736;
  cloud.points[2].getVector3dMap () << 2.7609, 1.9095, 1.3574f;
  cloud.points[3].getVector3dMap () << 2.8016, 1.6704f, 1.5009;
  cloud.points[4].getVector3dMap () << 1.8517, 2.0276, 1.0112f;
  cloud.points[5].getVector3dMap () << 1.8726, 1.3539, 2.7523f;
  cloud.points[6].getVector3dMap () << 2.5179, 2.3218, 1.2074f;
  cloud.points[7].getVector3dMap () << 2.4026, 2.5114f, 2.7588;
  cloud.points[8].getVector3dMap () << 2.6999, 2.5606, 1.5571;
  cloud.points[9].getVector3dMap () << 0.0000, 0.0000, 0.0000;

  // Create a shared sphere model pointer directly
  SampleConsensusModelSpherePtr model (new SampleConsensusModelSphere<PointXYZ> (cloud.makeShared ()));

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac (model, 0.03);

  // Algorithm tests
  bool result = sac.computeModel ();
  ASSERT_TRUE (result);

  std::vector<int> sample;
  sac.getModel (sample);
  EXPECT_EQ (4, sample.size ());

  std::vector<int> inliers;
  sac.getInliers (inliers);
  EXPECT_EQ (9, inliers.size ());

  Eigen::VectorXd coeff;
  sac.getModelCoefficients (coeff);
  EXPECT_EQ (4, coeff.size ());
  EXPECT_NEAR (2, coeff[0] / coeff[3], 1e-2);
  EXPECT_NEAR (2, coeff[1] / coeff[3], 1e-2);
  EXPECT_NEAR (2, coeff[2] / coeff[3], 1e-2);

  Eigen::VectorXd coeff_refined;
  model->optimizeModelCoefficients (inliers, coeff, coeff_refined);
  EXPECT_EQ (4, coeff_refined.size ());
  EXPECT_NEAR (2, coeff_refined[0] / coeff_refined[3], 1e-2);
  EXPECT_NEAR (2, coeff_refined[1] / coeff_refined[3], 1e-2);
  EXPECT_NEAR (2, coeff_refined[2] / coeff_refined[3], 1e-2);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (SampleConsensusModelNormalSphere, RANSAC)
{
  srand (0);

  // Use a custom point cloud for these tests until we need something better
  PointCloud<PointXYZ> cloud;
  PointCloud<Normal> normals;
  cloud.points.resize (27); normals.points.resize (27);
  cloud.points[ 0].getVector3dMap () << -0.014695,  0.009549, 0.954775;
  cloud.points[ 1].getVector3dMap () <<  0.014695,  0.009549, 0.954775;
  cloud.points[ 2].getVector3dMap () << -0.014695,  0.040451, 0.954775;
  cloud.points[ 3].getVector3dMap () <<  0.014695,  0.040451, 0.954775;
  cloud.points[ 4].getVector3dMap () << -0.009082f, -0.015451, 0.972049;
  cloud.points[ 5].getVector3dMap () <<  0.009082f, -0.015451, 0.972049;
  cloud.points[ 6].getVector3dMap () << -0.038471,  0.009549, 0.972049;
  cloud.points[ 7].getVector3dMap () <<  0.038471,  0.009549, 0.972049;
  cloud.points[ 8].getVector3dMap () << -0.038471,  0.040451, 0.972049;
  cloud.points[ 9].getVector3dMap () <<  0.038471,  0.040451, 0.972049;
  cloud.points[10].getVector3dMap () << -0.009082f,  0.065451, 0.972049;
  cloud.points[11].getVector3dMap () <<  0.009082f,  0.065451, 0.972049;
  cloud.points[12].getVector3dMap () << -0.023776, -0.015451, 0.982725;
  cloud.points[13].getVector3dMap () <<  0.023776, -0.015451, 0.982725;
  cloud.points[14].getVector3dMap () << -0.023776,  0.065451, 0.982725;
  cloud.points[15].getVector3dMap () <<  0.023776,  0.065451, 0.982725;
  cloud.points[16].getVector3dMap () << -0.000000, -0.025000, 1.000000;
  cloud.points[17].getVector3dMap () <<  0.000000, -0.025000, 1.000000;
  cloud.points[18].getVector3dMap () << -0.029389, -0.015451, 1.000000;
  cloud.points[19].getVector3dMap () <<  0.029389, -0.015451, 1.000000;
  cloud.points[20].getVector3dMap () << -0.047553f,  0.009549, 1.000000;
  cloud.points[21].getVector3dMap () <<  0.047553f,  0.009549, 1.000000;
  cloud.points[22].getVector3dMap () << -0.047553f,  0.040451, 1.000000;
  cloud.points[23].getVector3dMap () <<  0.047553f,  0.040451, 1.000000;
  cloud.points[24].getVector3dMap () << -0.029389,  0.065451, 1.000000;
  cloud.points[25].getVector3dMap () <<  0.029389,  0.065451, 1.000000;
  cloud.points[26].getVector3dMap () <<  0.000000,  0.075000, 1.000000;

  normals.points[ 0].getNormalVector3dMap () << -0.293893f, -0.309017, -0.904509;
  normals.points[ 1].getNormalVector3dMap () <<  0.293893f, -0.309017, -0.904508;
  normals.points[ 2].getNormalVector3dMap () << -0.293893f,  0.309017, -0.904509;
  normals.points[ 3].getNormalVector3dMap () <<  0.293893f,  0.309017, -0.904508;
  normals.points[ 4].getNormalVector3dMap () << -0.181636, -0.809017, -0.559017;
  normals.points[ 5].getNormalVector3dMap () <<  0.181636, -0.809017, -0.559017;
  normals.points[ 6].getNormalVector3dMap () << -0.769421, -0.309017, -0.559017;
  normals.points[ 7].getNormalVector3dMap () <<  0.769421, -0.309017, -0.559017;
  normals.points[ 8].getNormalVector3dMap () << -0.769421,  0.309017, -0.559017;
  normals.points[ 9].getNormalVector3dMap () <<  0.769421,  0.309017, -0.559017;
  normals.points[10].getNormalVector3dMap () << -0.181636,  0.809017, -0.559017;
  normals.points[11].getNormalVector3dMap () <<  0.181636,  0.809017, -0.559017;
  normals.points[12].getNormalVector3dMap () << -0.475528, -0.809017, -0.345491;
  normals.points[13].getNormalVector3dMap () <<  0.475528, -0.809017, -0.345491;
  normals.points[14].getNormalVector3dMap () << -0.475528,  0.809017, -0.345491;
  normals.points[15].getNormalVector3dMap () <<  0.475528,  0.809017, -0.345491;
  normals.points[16].getNormalVector3dMap () << -0.000000, -1.000000,  0.000000;
  normals.points[17].getNormalVector3dMap () <<  0.000000, -1.000000,  0.000000;
  normals.points[18].getNormalVector3dMap () << -0.587785, -0.809017,  0.000000;
  normals.points[19].getNormalVector3dMap () <<  0.587785, -0.809017,  0.000000;
  normals.points[20].getNormalVector3dMap () << -0.951057, -0.309017,  0.000000;
  normals.points[21].getNormalVector3dMap () <<  0.951057, -0.309017,  0.000000;
  normals.points[22].getNormalVector3dMap () << -0.951057,  0.309017,  0.000000;
  normals.points[23].getNormalVector3dMap () <<  0.951057,  0.309017,  0.000000;
  normals.points[24].getNormalVector3dMap () << -0.587785,  0.809017,  0.000000;
  normals.points[25].getNormalVector3dMap () <<  0.587785,  0.809017,  0.000000;
  normals.points[26].getNormalVector3dMap () <<  0.000000,  1.000000,  0.000000;

  // Create a shared sphere model pointer directly
  SampleConsensusModelNormalSpherePtr model (new SampleConsensusModelNormalSphere<PointXYZ, Normal> (cloud.makeShared ()));
  model->setInputNormals (normals.makeShared ());

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac (model, 0.03);

  // Algorithm tests
  bool result = sac.computeModel ();
  ASSERT_TRUE (result);

  std::vector<int> sample;
  sac.getModel (sample);
  EXPECT_EQ (4, sample.size ());

  std::vector<int> inliers;
  sac.getInliers (inliers);
  EXPECT_EQ (27, inliers.size ());

  Eigen::VectorXd coeff;
  sac.getModelCoefficients (coeff);
  EXPECT_EQ (4, coeff.size ());
  EXPECT_NEAR (0.000, coeff[0], 1e-2);
  EXPECT_NEAR (0.025, coeff[1], 1e-2);
  EXPECT_NEAR (1.000, coeff[2], 1e-2);
  EXPECT_NEAR (0.050, coeff[3], 1e-2);

  Eigen::VectorXd coeff_refined;
  model->optimizeModelCoefficients (inliers, coeff, coeff_refined);
  EXPECT_EQ (4, coeff_refined.size ());
  EXPECT_NEAR (0.000, coeff_refined[0], 1e-2);
  EXPECT_NEAR (0.025, coeff_refined[1], 1e-2);
  EXPECT_NEAR (1.000, coeff_refined[2], 1e-2);
  EXPECT_NEAR (0.050, coeff_refined[3], 1e-2);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (SampleConsensusModelCone, RANSAC)
{
  srand (0);

  // Use a custom point cloud for these tests until we need something better
  PointCloud<PointXYZ> cloud;
  PointCloud<Normal> normals;
  cloud.points.resize (31); normals.points.resize (31);

  cloud.points[ 0].getVector3dMap () << -0.011247, 0.200000, 0.965384f;
  cloud.points[ 1].getVector3dMap () <<  0.000000, 0.200000, 0.963603f;
  cloud.points[ 2].getVector3dMap () <<  0.011247, 0.200000, 0.965384f;
  cloud.points[ 3].getVector3dMap () << -0.016045, 0.175000, 0.977916;
  cloud.points[ 4].getVector3dMap () << -0.008435, 0.175000, 0.974038;
  cloud.points[ 5].getVector3dMap () <<  0.004218, 0.175000, 0.973370;
  cloud.points[ 6].getVector3dMap () <<  0.016045, 0.175000, 0.977916;
  cloud.points[ 7].getVector3dMap () << -0.025420, 0.200000, 0.974580;
  cloud.points[ 8].getVector3dMap () <<  0.025420, 0.200000, 0.974580;
  cloud.points[ 9].getVector3dMap () << -0.012710, 0.150000, 0.987290;
  cloud.points[10].getVector3dMap () << -0.005624f, 0.150000, 0.982692f;
  cloud.points[11].getVector3dMap () <<  0.002812f, 0.150000, 0.982247;
  cloud.points[12].getVector3dMap () <<  0.012710, 0.150000, 0.987290;
  cloud.points[13].getVector3dMap () << -0.022084f, 0.175000, 0.983955;
  cloud.points[14].getVector3dMap () <<  0.022084f, 0.175000, 0.983955;
  cloud.points[15].getVector3dMap () << -0.034616, 0.200000, 0.988753f;
  cloud.points[16].getVector3dMap () <<  0.034616, 0.200000, 0.988753f;
  cloud.points[17].getVector3dMap () << -0.006044f, 0.125000, 0.993956;
  cloud.points[18].getVector3dMap () <<  0.004835, 0.125000, 0.993345;
  cloud.points[19].getVector3dMap () << -0.017308, 0.150000, 0.994376;
  cloud.points[20].getVector3dMap () <<  0.017308, 0.150000, 0.994376;
  cloud.points[21].getVector3dMap () << -0.025962f, 0.175000, 0.991565;
  cloud.points[22].getVector3dMap () <<  0.025962f, 0.175000, 0.991565;
  cloud.points[23].getVector3dMap () << -0.009099, 0.125000, 1.000000;
  cloud.points[24].getVector3dMap () <<  0.009099, 0.125000, 1.000000;
  cloud.points[25].getVector3dMap () << -0.018199, 0.150000, 1.000000;
  cloud.points[26].getVector3dMap () <<  0.018199, 0.150000, 1.000000;
  cloud.points[27].getVector3dMap () << -0.027298, 0.175000, 1.000000;
  cloud.points[28].getVector3dMap () <<  0.027298, 0.175000, 1.000000;
  cloud.points[29].getVector3dMap () << -0.036397, 0.200000, 1.000000;
  cloud.points[30].getVector3dMap () <<  0.036397, 0.200000, 1.000000;

  normals.points[ 0].getNormalVector3dMap () << -0.290381, -0.342020, -0.893701;
  normals.points[ 1].getNormalVector3dMap () <<  0.000000, -0.342020, -0.939693f;
  normals.points[ 2].getNormalVector3dMap () <<  0.290381, -0.342020, -0.893701;
  normals.points[ 3].getNormalVector3dMap () << -0.552338, -0.342020, -0.760227;
  normals.points[ 4].getNormalVector3dMap () << -0.290381, -0.342020, -0.893701;
  normals.points[ 5].getNormalVector3dMap () <<  0.145191, -0.342020, -0.916697;
  normals.points[ 6].getNormalVector3dMap () <<  0.552337, -0.342020, -0.760227;
  normals.points[ 7].getNormalVector3dMap () << -0.656282f, -0.342020, -0.656283f;
  normals.points[ 8].getNormalVector3dMap () <<  0.656282f, -0.342020, -0.656283f;
  normals.points[ 9].getNormalVector3dMap () << -0.656283f, -0.342020, -0.656282f;
  normals.points[10].getNormalVector3dMap () << -0.290381, -0.342020, -0.893701;
  normals.points[11].getNormalVector3dMap () <<  0.145191, -0.342020, -0.916697;
  normals.points[12].getNormalVector3dMap () <<  0.656282f, -0.342020, -0.656282f;
  normals.points[13].getNormalVector3dMap () << -0.760228, -0.342020, -0.552337;
  normals.points[14].getNormalVector3dMap () <<  0.760228, -0.342020, -0.552337;
  normals.points[15].getNormalVector3dMap () << -0.893701, -0.342020, -0.290380;
  normals.points[16].getNormalVector3dMap () <<  0.893701, -0.342020, -0.290380;
  normals.points[17].getNormalVector3dMap () << -0.624162f, -0.342020, -0.624162f;
  normals.points[18].getNormalVector3dMap () <<  0.499329, -0.342020, -0.687268;
  normals.points[19].getNormalVector3dMap () << -0.893701, -0.342020, -0.290380;
  normals.points[20].getNormalVector3dMap () <<  0.893701, -0.342020, -0.290380;
  normals.points[21].getNormalVector3dMap () << -0.893701, -0.342020, -0.290381;
  normals.points[22].getNormalVector3dMap () <<  0.893701, -0.342020, -0.290381;
  normals.points[23].getNormalVector3dMap () << -0.939693f, -0.342020,  0.000000;
  normals.points[24].getNormalVector3dMap () <<  0.939693f, -0.342020,  0.000000;
  normals.points[25].getNormalVector3dMap () << -0.939693f, -0.342020,  0.000000;
  normals.points[26].getNormalVector3dMap () <<  0.939693f, -0.342020,  0.000000;
  normals.points[27].getNormalVector3dMap () << -0.939693f, -0.342020,  0.000000;
  normals.points[28].getNormalVector3dMap () <<  0.939693f, -0.342020,  0.000000;
  normals.points[29].getNormalVector3dMap () << -0.939693f, -0.342020,  0.000000;
  normals.points[30].getNormalVector3dMap () <<  0.939693f, -0.342020,  0.000000;

  // Create a shared cylinder model pointer directly
  SampleConsensusModelConePtr model (new SampleConsensusModelCone<PointXYZ, Normal> (cloud.makeShared ()));
  model->setInputNormals (normals.makeShared ());

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac (model, 0.03);

  // Algorithm tests
  bool result = sac.computeModel ();
  ASSERT_TRUE (result);

  std::vector<int> sample;
  sac.getModel (sample);
  EXPECT_EQ (3, sample.size ());

  std::vector<int> inliers;
  sac.getInliers (inliers);
  EXPECT_EQ (31, inliers.size ());

  Eigen::VectorXd coeff;
  sac.getModelCoefficients (coeff);
  EXPECT_EQ (7, coeff.size ());
  EXPECT_NEAR (0.000000, coeff[0], 1e-2);
  EXPECT_NEAR (0.100000, coeff[1], 1e-2);
  EXPECT_NEAR (0.349066, coeff[6], 1e-2);

  Eigen::VectorXd coeff_refined;
  model->optimizeModelCoefficients (inliers, coeff, coeff_refined);
  EXPECT_EQ (7, coeff_refined.size ());
  EXPECT_NEAR (0.349066, coeff_refined[6], 1e-2);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (SampleConsensusModelCylinder, RANSAC)
{
  srand (0);

  // Use a custom point cloud for these tests until we need something better
  PointCloud<PointXYZ> cloud;
  PointCloud<Normal> normals;
  cloud.points.resize (20); normals.points.resize (20);

  cloud.points[ 0].getVector3dMap () << -0.499902f, 2.199701, 0.000008;
  cloud.points[ 1].getVector3dMap () << -0.875397, 2.030177, 0.050104f;
  cloud.points[ 2].getVector3dMap () << -0.995875, 1.635973f, 0.099846;
  cloud.points[ 3].getVector3dMap () << -0.779523f, 1.285527, 0.149961;
  cloud.points[ 4].getVector3dMap () << -0.373285, 1.216488, 0.199959;
  cloud.points[ 5].getVector3dMap () << -0.052893f, 1.475973f, 0.250101;
  cloud.points[ 6].getVector3dMap () << -0.036558, 1.887591, 0.299839;
  cloud.points[ 7].getVector3dMap () << -0.335048, 2.171994f, 0.350001;
  cloud.points[ 8].getVector3dMap () << -0.745456, 2.135528, 0.400072f;
  cloud.points[ 9].getVector3dMap () << -0.989282f, 1.803311, 0.449983f;
  cloud.points[10].getVector3dMap () << -0.900651, 1.400701, 0.500126;
  cloud.points[11].getVector3dMap () << -0.539658, 1.201468, 0.550079;
  cloud.points[12].getVector3dMap () << -0.151875, 1.340951, 0.599983f;
  cloud.points[13].getVector3dMap () << -0.000724f, 1.724373f, 0.649882f;
  cloud.points[14].getVector3dMap () << -0.188573f, 2.090983f, 0.699854f;
  cloud.points[15].getVector3dMap () << -0.587925, 2.192257, 0.749956;
  cloud.points[16].getVector3dMap () << -0.927724f, 1.958846, 0.800008;
  cloud.points[17].getVector3dMap () << -0.976888, 1.549655, 0.849970;
  cloud.points[18].getVector3dMap () << -0.702003f, 1.242707, 0.899954f;
  cloud.points[19].getVector3dMap () << -0.289916, 1.246296, 0.950075;

  normals.points[ 0].getNormalVector3dMap () <<  0.000098,  1.000098,  0.000008;
  normals.points[ 1].getNormalVector3dMap () << -0.750891,  0.660413f,  0.000104f;
  normals.points[ 2].getNormalVector3dMap () << -0.991765, -0.127949, -0.000154f;
  normals.points[ 3].getNormalVector3dMap () << -0.558918, -0.829439, -0.000039;
  normals.points[ 4].getNormalVector3dMap () <<  0.253627, -0.967447, -0.000041;
  normals.points[ 5].getNormalVector3dMap () <<  0.894105, -0.447965,  0.000101;
  normals.points[ 6].getNormalVector3dMap () <<  0.926852f,  0.375543f, -0.000161;
  normals.points[ 7].getNormalVector3dMap () <<  0.329948,  0.943941,  0.000001;
  normals.points[ 8].getNormalVector3dMap () << -0.490966,  0.871203f,  0.000072f;
  normals.points[ 9].getNormalVector3dMap () << -0.978507,  0.206425, -0.000017;
  normals.points[10].getNormalVector3dMap () << -0.801227, -0.598534f,  0.000126;
  normals.points[11].getNormalVector3dMap () << -0.079447, -0.996697,  0.000079;
  normals.points[12].getNormalVector3dMap () <<  0.696154f, -0.717889, -0.000017;
  normals.points[13].getNormalVector3dMap () <<  0.998685,  0.048502f, -0.000118;
  normals.points[14].getNormalVector3dMap () <<  0.622933f,  0.782133f, -0.000146;
  normals.points[15].getNormalVector3dMap () << -0.175948,  0.984480, -0.000044f;
  normals.points[16].getNormalVector3dMap () << -0.855476,  0.517824f,  0.000008;
  normals.points[17].getNormalVector3dMap () << -0.953769, -0.300571, -0.000030;
  normals.points[18].getNormalVector3dMap () << -0.404035, -0.914700, -0.000046;
  normals.points[19].getNormalVector3dMap () <<  0.420154f, -0.907445,  0.000075;

  // Create a shared cylinder model pointer directly
  SampleConsensusModelCylinderPtr model (new SampleConsensusModelCylinder<PointXYZ, Normal> (cloud.makeShared ()));
  model->setInputNormals (normals.makeShared ());

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac (model, 0.03);

  // Algorithm tests
  bool result = sac.computeModel ();
  ASSERT_TRUE (result);

  std::vector<int> sample;
  sac.getModel (sample);
  EXPECT_EQ (2, sample.size ());

  std::vector<int> inliers;
  sac.getInliers (inliers);
  EXPECT_EQ (20, inliers.size ());

  Eigen::VectorXd coeff;
  sac.getModelCoefficients (coeff);
  EXPECT_EQ (7, coeff.size ());
  EXPECT_NEAR (-0.5, coeff[0], 1e-3);
  EXPECT_NEAR ( 1.7, coeff[1], 1e-3);
  EXPECT_NEAR ( 0.5, coeff[6], 1e-3);

  Eigen::VectorXd coeff_refined;
  model->optimizeModelCoefficients (inliers, coeff, coeff_refined);
  EXPECT_EQ (7, coeff_refined.size ());
  EXPECT_NEAR (0.5, coeff_refined[6], 1e-3);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (SampleConsensusModelCircle2D, RANSAC)
{
  srand (0);

  // Use a custom point cloud for these tests until we need something better
  PointCloud<PointXYZ> cloud;
  cloud.points.resize (18);

  cloud.points[ 0].getVector3dMap () << 3.587751, -4.190982f, 0.0;
  cloud.points[ 1].getVector3dMap () << 3.808883f, -4.412265, 0.0;
  cloud.points[ 2].getVector3dMap () << 3.587525, -5.809143f, 0.0;
  cloud.points[ 3].getVector3dMap () << 2.999913f, -5.999980, 0.0;
  cloud.points[ 4].getVector3dMap () << 2.412224f, -5.809090, 0.0;
  cloud.points[ 5].getVector3dMap () << 2.191080, -5.587682f, 0.0;
  cloud.points[ 6].getVector3dMap () << 2.048941, -5.309003f, 0.0;
  cloud.points[ 7].getVector3dMap () << 2.000397, -4.999944f, 0.0;
  cloud.points[ 8].getVector3dMap () << 2.999953f, -6.000056, 0.0;
  cloud.points[ 9].getVector3dMap () << 2.691127, -5.951136, 0.0;
  cloud.points[10].getVector3dMap () << 2.190892f, -5.587838, 0.0;
  cloud.points[11].getVector3dMap () << 2.048874f, -5.309052f, 0.0;
  cloud.points[12].getVector3dMap () << 1.999990, -5.000147, 0.0;
  cloud.points[13].getVector3dMap () << 2.049026, -4.690918, 0.0;
  cloud.points[14].getVector3dMap () << 2.190956, -4.412162f, 0.0;
  cloud.points[15].getVector3dMap () << 2.412231, -4.190918, 0.0;
  cloud.points[16].getVector3dMap () << 2.691027, -4.049060, 0.0;
  cloud.points[17].getVector3dMap () << 2.000000, -3.000000, 0.0;

  // Create a shared 2d circle model pointer directly
  SampleConsensusModelCircle2DPtr model (new SampleConsensusModelCircle2D<PointXYZ> (cloud.makeShared ()));

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac (model, 0.03);

  // Algorithm tests
  bool result = sac.computeModel ();
  ASSERT_TRUE (result);

  std::vector<int> sample;
  sac.getModel (sample);
  EXPECT_EQ (3, sample.size ());

  std::vector<int> inliers;
  sac.getInliers (inliers);
  EXPECT_EQ (17, inliers.size ());

  Eigen::VectorXd coeff;
  sac.getModelCoefficients (coeff);
  EXPECT_EQ (3, coeff.size ());
  EXPECT_NEAR ( 3, coeff[0], 1e-3);
  EXPECT_NEAR (-5, coeff[1], 1e-3);
  EXPECT_NEAR ( 1, coeff[2], 1e-3);

  Eigen::VectorXd coeff_refined;
  model->optimizeModelCoefficients (inliers, coeff, coeff_refined);
  EXPECT_EQ (3, coeff_refined.size ());
  EXPECT_NEAR ( 3, coeff_refined[0], 1e-3);
  EXPECT_NEAR (-5, coeff_refined[1], 1e-3);
  EXPECT_NEAR ( 1, coeff_refined[2], 1e-3);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (SampleConsensusModelCircle3D, RANSAC)
{
  srand (0);

  // Use a custom point cloud for these tests until we need something better
  PointCloud<PointXYZ> cloud;
  cloud.points.resize (20);

  cloud.points[ 0].getVector3dMap () << 1.00000000, 5.0000000, -2.9000001;
  cloud.points[ 1].getVector3dMap () << 1.03420200, 5.0000000, -2.9060307;
  cloud.points[ 2].getVector3dMap () << 1.06427870, 5.0000000, -2.9233956;
  cloud.points[ 3].getVector3dMap () << 1.08660260, 5.0000000, -2.9500000;
  cloud.points[ 4].getVector3dMap () << 1.09848080, 5.0000000, -2.9826353f;
  cloud.points[ 5].getVector3dMap () << 1.09848080, 5.0000000, -3.0173647;
  cloud.points[ 6].getVector3dMap () << 1.08660260, 5.0000000, -3.0500000;
  cloud.points[ 7].getVector3dMap () << 1.06427870, 5.0000000, -3.0766044f;
  cloud.points[ 8].getVector3dMap () << 1.03420200, 5.0000000, -3.0939693f;
  cloud.points[ 9].getVector3dMap () << 1.00000000, 5.0000000, -3.0999999;
  cloud.points[10].getVector3dMap () << 0.96579796, 5.0000000, -3.0939693f;
  cloud.points[11].getVector3dMap () << 0.93572122f, 5.0000000, -3.0766044f;
  cloud.points[12].getVector3dMap () << 0.91339743f, 5.0000000, -3.0500000;
  cloud.points[13].getVector3dMap () << 0.90151924f, 5.0000000, -3.0173647;
  cloud.points[14].getVector3dMap () << 0.90151924f, 5.0000000, -2.9826353f;
  cloud.points[15].getVector3dMap () << 0.91339743f, 5.0000000, -2.9500000;
  cloud.points[16].getVector3dMap () << 0.93572122f, 5.0000000, -2.9233956;
  cloud.points[17].getVector3dMap () << 0.96579796, 5.0000000, -2.9060307;
  cloud.points[18].getVector3dMap () << 0.85000002f, 4.8499999, -3.1500001;
  cloud.points[19].getVector3dMap () << 1.15000000, 5.1500001, -2.8499999;

  // Create a shared 3d circle model pointer directly
  SampleConsensusModelCircle3DPtr model (new SampleConsensusModelCircle3D<PointXYZ> (cloud.makeShared ()));

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac (model, 0.03);

  // Algorithm tests
  bool result = sac.computeModel ();
  ASSERT_TRUE (result);

  std::vector<int> sample;
  sac.getModel (sample);
  EXPECT_EQ (3, sample.size ());

  std::vector<int> inliers;
  sac.getInliers (inliers);
  EXPECT_EQ (18, inliers.size ());

  Eigen::VectorXd coeff;
  sac.getModelCoefficients (coeff);
  EXPECT_EQ (7, coeff.size ());
  EXPECT_NEAR ( 1.0, coeff[0], 1e-3);
  EXPECT_NEAR ( 5.0, coeff[1], 1e-3);
  EXPECT_NEAR (-3.0, coeff[2], 1e-3);
  EXPECT_NEAR ( 0.1, coeff[3], 1e-3);
  EXPECT_NEAR ( 0.0, coeff[4], 1e-3);
  EXPECT_NEAR (-1.0, coeff[5], 1e-3);
  EXPECT_NEAR ( 0.0, coeff[6], 1e-3);

  Eigen::VectorXd coeff_refined;
  model->optimizeModelCoefficients (inliers, coeff, coeff_refined);
  EXPECT_EQ (7, coeff_refined.size ());
  EXPECT_NEAR ( 1.0, coeff_refined[0], 1e-3);
  EXPECT_NEAR ( 5.0, coeff_refined[1], 1e-3);
  EXPECT_NEAR (-3.0, coeff_refined[2], 1e-3);
  EXPECT_NEAR ( 0.1, coeff_refined[3], 1e-3);
  EXPECT_NEAR ( 0.0, coeff_refined[4], 1e-3);
  EXPECT_NEAR (-1.0, coeff_refined[5], 1e-3);
  EXPECT_NEAR ( 0.0, coeff_refined[6], 1e-3);
}

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}

