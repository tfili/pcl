/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 *
 */
#include <gtest/gtest.h>

#include <iostream>  // For debug

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/feature.h>
#include <pcl/common/eigen.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include <pcl/pcl_tests.h>

using namespace pcl;
using namespace pcl::io;
using namespace std;

const double PI = 3.14159265;
const double rho = sqrt (2.0) / 2.0;  // cos(PI/4) == sin(PI/4)

PointCloud<PointXYZ> cloud;
pcl::PCLPointCloud2 cloud_blob;

void 
init ()
{
  PointXYZ p0 (0, 0, 0);
  PointXYZ p1 (1, 0, 0);
  PointXYZ p2 (0, 1, 0);
  PointXYZ p3 (0, 0, 1);
  cloud.points.push_back (p0);
  cloud.points.push_back (p1);
  cloud.points.push_back (p2);
  cloud.points.push_back (p3);
}

//////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, DeMean)
{
  PointCloud<PointXYZ> cloud, cloud_demean;
  fromPCLPointCloud2 (cloud_blob, cloud);

  Eigen::Vector4d centroid;
  compute3DCentroid (cloud, centroid);
  EXPECT_NEAR (centroid[0], -0.0290809, 1e-4);
  EXPECT_NEAR (centroid[1],  0.102653,  1e-4);
  EXPECT_NEAR (centroid[2],  0.027302,  1e-4);
  EXPECT_NEAR (centroid[3],  1,         1e-4);

  // Check standard demean
  demeanPointCloud (cloud, centroid, cloud_demean);
  EXPECT_EQ (cloud_demean.is_dense, cloud.is_dense);
  EXPECT_EQ (cloud_demean.width, cloud.width);
  EXPECT_EQ (cloud_demean.height, cloud.height);
  EXPECT_EQ (cloud_demean.points.size (), cloud.points.size ());

  EXPECT_NEAR (cloud_demean.points[0].x, 0.034503, 1e-4);
  EXPECT_NEAR (cloud_demean.points[0].y, 0.010837, 1e-4);
  EXPECT_NEAR (cloud_demean.points[0].z, 0.013447, 1e-4);

  EXPECT_NEAR (cloud_demean.points[cloud_demean.points.size () - 1].x, -0.048849, 1e-4);
  EXPECT_NEAR (cloud_demean.points[cloud_demean.points.size () - 1].y,  0.072507, 1e-4);
  EXPECT_NEAR (cloud_demean.points[cloud_demean.points.size () - 1].z, -0.071702, 1e-4);

  vector<int> indices (cloud.points.size ());
  for (int i = 0; i < static_cast<int> (indices.size ()); ++i) { indices[i] = i; }

  // Check standard demean w/ indices
  demeanPointCloud (cloud, indices, centroid, cloud_demean);
  EXPECT_EQ (cloud_demean.is_dense, cloud.is_dense);
  EXPECT_EQ (cloud_demean.width, cloud.width);
  EXPECT_EQ (cloud_demean.height, cloud.height);
  EXPECT_EQ (cloud_demean.points.size (), cloud.points.size ());

  EXPECT_NEAR (cloud_demean.points[0].x, 0.034503, 1e-4);
  EXPECT_NEAR (cloud_demean.points[0].y, 0.010837, 1e-4);
  EXPECT_NEAR (cloud_demean.points[0].z, 0.013447, 1e-4);

  EXPECT_NEAR (cloud_demean.points[cloud_demean.points.size () - 1].x, -0.048849, 1e-4);
  EXPECT_NEAR (cloud_demean.points[cloud_demean.points.size () - 1].y,  0.072507, 1e-4);
  EXPECT_NEAR (cloud_demean.points[cloud_demean.points.size () - 1].z, -0.071702, 1e-4);

  // Check eigen demean
  Eigen::MatrixXd mat_demean;
  demeanPointCloud (cloud, centroid, mat_demean);
  EXPECT_EQ (mat_demean.cols (), int (cloud.points.size ()));
  EXPECT_EQ (mat_demean.rows (), 4);

  EXPECT_NEAR (mat_demean (0, 0), 0.034503, 1e-4);
  EXPECT_NEAR (mat_demean (1, 0), 0.010837, 1e-4);
  EXPECT_NEAR (mat_demean (2, 0), 0.013447, 1e-4);

  EXPECT_NEAR (mat_demean (0, cloud_demean.points.size () - 1), -0.048849, 1e-4);
  EXPECT_NEAR (mat_demean (1, cloud_demean.points.size () - 1),  0.072507, 1e-4);
  EXPECT_NEAR (mat_demean (2, cloud_demean.points.size () - 1), -0.071702, 1e-4);

  // Check eigen demean + indices
  demeanPointCloud (cloud, indices, centroid, mat_demean);
  EXPECT_EQ (mat_demean.cols (), int (cloud.points.size ()));
  EXPECT_EQ (mat_demean.rows (), 4);

  EXPECT_NEAR (mat_demean (0, 0), 0.034503, 1e-4);
  EXPECT_NEAR (mat_demean (1, 0), 0.010837, 1e-4);
  EXPECT_NEAR (mat_demean (2, 0), 0.013447, 1e-4);

  EXPECT_NEAR (mat_demean (0, cloud_demean.points.size () - 1), -0.048849, 1e-4);
  EXPECT_NEAR (mat_demean (1, cloud_demean.points.size () - 1),  0.072507, 1e-4);
  EXPECT_NEAR (mat_demean (2, cloud_demean.points.size () - 1), -0.071702, 1e-4);
}

//////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, Transform)
{
  Eigen::Vector3d offset (100, 0, 0);
  double angle = PI/4;
  Eigen::Quaterniond rotation (cos (angle / 2), 0, 0, sin (angle / 2));

  PointCloud<PointXYZ> cloud_out;
  const vector<PointXYZ, Eigen::aligned_allocator<PointXYZ> > &points (cloud_out.points);
  transformPointCloud (cloud, cloud_out, offset, rotation);

  EXPECT_EQ (cloud.points.size (), cloud_out.points.size ());
  EXPECT_EQ (100, points[0].x);
  EXPECT_EQ (0, points[0].y);
  EXPECT_EQ (0, points[0].z);
  EXPECT_NEAR (100+rho, points[1].x,  1e-4);
  EXPECT_NEAR (rho, points[1].y,  1e-4);
  EXPECT_EQ (0, points[1].z);
  EXPECT_NEAR (100-rho, points[2].x,  1e-4);
  EXPECT_NEAR (rho, points[2].y,  1e-4);
  EXPECT_EQ (0, points[2].z);
  EXPECT_EQ (100, points[3].x);
  EXPECT_EQ (0, points[3].y);
  EXPECT_EQ (1, points[3].z);

  PointCloud<PointXYZ> cloud_out2;
  const vector<PointXYZ, Eigen::aligned_allocator<PointXYZ> > &points2 (cloud_out2.points);
  Eigen::Translation3d translation (offset);
  Eigen::Affine3d transform;
  transform = translation * rotation;
  transformPointCloud (cloud, cloud_out2, transform);

  EXPECT_EQ (cloud.points.size (), cloud_out2.points.size ());
  EXPECT_EQ (100, points2[0].x);
  EXPECT_EQ (0, points2[0].y);
  EXPECT_EQ (0, points2[0].z);
  EXPECT_NEAR (100+rho, points2[1].x,  1e-4);
  EXPECT_NEAR (rho, points2[1].y,  1e-4);
  EXPECT_EQ (0, points2[1].z);
  EXPECT_NEAR (100-rho, points2[2].x,  1e-4);
  EXPECT_NEAR (rho, points2[2].y,  1e-4);
  EXPECT_EQ (0, points2[2].z);
  EXPECT_EQ (100, points2[3].x);
  EXPECT_EQ (0, points2[3].y);
  EXPECT_EQ (1, points2[3].z);
}

//////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, TransformCopyFields)
{
  Eigen::Affine3d transform;
  transform = Eigen::Translation3d (100, 0, 0);

  PointXYZRGBNormal empty_point;
  std::vector<int> indices (1);

  PointCloud<PointXYZRGBNormal> cloud (2, 1);
  cloud.points[0].rgba = 0xFF0000;
  cloud.points[1].rgba = 0x00FF00;

  // Preserve data in all fields
  {
    PointCloud<PointXYZRGBNormal> cloud_out;
    transformPointCloud (cloud, cloud_out, transform, true);
    ASSERT_EQ (cloud.size (), cloud_out.size ());
    EXPECT_RGBA_EQ (cloud.points[0], cloud_out.points[0]);
    EXPECT_RGBA_EQ (cloud.points[1], cloud_out.points[1]);
  }
  // Preserve data in all fields (with indices)
  {
    PointCloud<PointXYZRGBNormal> cloud_out;
    transformPointCloud (cloud, indices, cloud_out, transform, true);
    ASSERT_EQ (indices.size (), cloud_out.size ());
    EXPECT_RGBA_EQ (cloud.points[0], cloud_out.points[0]);
  }
  // Do not preserve data in all fields
  {
    PointCloud<PointXYZRGBNormal> cloud_out;
    transformPointCloud (cloud, cloud_out, transform, false);
    ASSERT_EQ (cloud.size (), cloud_out.size ());
    EXPECT_RGBA_EQ (empty_point, cloud_out.points[0]);
    EXPECT_RGBA_EQ (empty_point, cloud_out.points[1]);
  }
  // Do not preserve data in all fields (with indices)
  {
    PointCloud<PointXYZRGBNormal> cloud_out;
    transformPointCloud (cloud, indices, cloud_out, transform, false);
    ASSERT_EQ (indices.size (), cloud_out.size ());
    EXPECT_RGBA_EQ (empty_point, cloud_out.points[0]);
  }
  // Preserve data in all fields (with normals version)
  {
    PointCloud<PointXYZRGBNormal> cloud_out;
    transformPointCloudWithNormals (cloud, cloud_out, transform, true);
    ASSERT_EQ (cloud.size (), cloud_out.size ());
    EXPECT_RGBA_EQ (cloud.points[0], cloud_out.points[0]);
    EXPECT_RGBA_EQ (cloud.points[1], cloud_out.points[1]);
  }
  // Preserve data in all fields (with normals and indices version)
  {
    PointCloud<PointXYZRGBNormal> cloud_out;
    transformPointCloudWithNormals (cloud, indices, cloud_out, transform, true);
    ASSERT_EQ (indices.size (), cloud_out.size ());
    EXPECT_RGBA_EQ (cloud.points[0], cloud_out.points[0]);
  }
  // Do not preserve data in all fields (with normals version)
  {
    PointCloud<PointXYZRGBNormal> cloud_out;
    transformPointCloudWithNormals (cloud, cloud_out, transform, false);
    ASSERT_EQ (cloud.size (), cloud_out.size ());
    EXPECT_RGBA_EQ (empty_point, cloud_out.points[0]);
    EXPECT_RGBA_EQ (empty_point, cloud_out.points[1]);
  }
  // Do not preserve data in all fields (with normals and indices version)
  {
    PointCloud<PointXYZRGBNormal> cloud_out;
    transformPointCloudWithNormals (cloud, indices, cloud_out, transform, false);
    ASSERT_EQ (indices.size (), cloud_out.size ());
    EXPECT_RGBA_EQ (empty_point, cloud_out.points[0]);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, Matrix4Affine3Transform)
{
  double rot_x = 2.8827;
  double rot_y = -0.31190;
  double rot_z = -0.93058;
  Eigen::Affine3d affine;
  pcl::getTransformation (0, 0, 0, rot_x, rot_y, rot_z, affine);

  EXPECT_NEAR (affine (0, 0),  0.56854731, 1e-4); EXPECT_NEAR (affine (0, 1), -0.82217032f, 1e-4); EXPECT_NEAR (affine (0, 2), -0.028107658, 1e-4);
  EXPECT_NEAR (affine (1, 0), -0.76327348, 1e-4); EXPECT_NEAR (affine (1, 1), -0.51445758, 1e-4); EXPECT_NEAR (affine (1, 2), -0.39082864f, 1e-4);
  EXPECT_NEAR (affine (2, 0),  0.30686751, 1e-4); EXPECT_NEAR (affine (2, 1),  0.24365838, 1e-4); EXPECT_NEAR (affine (2, 2), -0.920034f, 1e-4);

  // Approximative!!! Uses SVD internally! See http://eigen.tuxfamily.org/dox/Transform_8h_source.html
  Eigen::Matrix3d rotation = affine.rotation ();

  EXPECT_NEAR (rotation (0, 0),  0.56854731, 1e-4); EXPECT_NEAR (rotation (0, 1), -0.82217032f, 1e-4); EXPECT_NEAR (rotation (0, 2), -0.028107658, 1e-4);
  EXPECT_NEAR (rotation (1, 0), -0.76327348, 1e-4); EXPECT_NEAR (rotation (1, 1), -0.51445758, 1e-4); EXPECT_NEAR (rotation (1, 2), -0.39082864f, 1e-4);
  EXPECT_NEAR (rotation (2, 0),  0.30686751, 1e-4); EXPECT_NEAR (rotation (2, 1),  0.24365838, 1e-4); EXPECT_NEAR (rotation (2, 2), -0.920034f, 1e-4);

  double trans_x, trans_y, trans_z;
  pcl::getTransformation (0.1, 0.2f, 0.3f, rot_x, rot_y, rot_z, affine);
  pcl::getTranslationAndEulerAngles (affine, trans_x, trans_y, trans_z, rot_x, rot_y, rot_z);
  EXPECT_FLOAT_EQ (trans_x, 0.1);
  EXPECT_FLOAT_EQ (trans_y, 0.2f);
  EXPECT_FLOAT_EQ (trans_z, 0.3f);
  EXPECT_FLOAT_EQ (rot_x, 2.8827);
  EXPECT_FLOAT_EQ (rot_y, -0.31190);
  EXPECT_FLOAT_EQ (rot_z, -0.93058);

  Eigen::Matrix4d transformation (Eigen::Matrix4d::Identity ());
  transformation.block<3, 3> (0, 0) = affine.rotation ();
  transformation.block<3, 1> (0, 3) = affine.translation ();

  PointXYZ p (1., 2., 3.);
  Eigen::Vector3d v3 = p.getVector3dMap ();
  Eigen::Vector4d v4 = p.getVector4dMap ();

  Eigen::Vector3d v3t (affine * v3);
  Eigen::Vector4d v4t (transformation * v4);

  PointXYZ pt = pcl::transformPoint (p, affine);

  EXPECT_NEAR (pt.x, v3t.x (), 1e-4); EXPECT_NEAR (pt.x, v4t.x (), 1e-4);
  EXPECT_NEAR (pt.y, v3t.y (), 1e-4); EXPECT_NEAR (pt.y, v4t.y (), 1e-4);
  EXPECT_NEAR (pt.z, v3t.z (), 1e-4); EXPECT_NEAR (pt.z, v4t.z (), 1e-4);

  PointNormal pn;
  pn.getVector3dMap () = p.getVector3dMap ();
  pn.getNormalVector3dMap () = Eigen::Vector3d (0.60, 0.48, 0.64);
  Eigen::Vector3d n3 = pn.getNormalVector3dMap ();
  Eigen::Vector4d n4 = pn.getNormalVector4dMap ();

  Eigen::Vector3d n3t (affine.rotation() * n3);
  Eigen::Vector4d n4t (transformation * n4);

  PointNormal pnt = pcl::transformPointWithNormal (pn, affine);

  EXPECT_NEAR (pnt.x, v3t.x (), 1e-4); EXPECT_NEAR (pnt.x, v4t.x (), 1e-4);
  EXPECT_NEAR (pnt.y, v3t.y (), 1e-4); EXPECT_NEAR (pnt.y, v4t.y (), 1e-4);
  EXPECT_NEAR (pnt.z, v3t.z (), 1e-4); EXPECT_NEAR (pnt.z, v4t.z (), 1e-4);
  EXPECT_NEAR (pnt.normal_x, n3t.x (), 1e-4); EXPECT_NEAR (pnt.normal_x, n4t.x (), 1e-4);
  EXPECT_NEAR (pnt.normal_y, n3t.y (), 1e-4); EXPECT_NEAR (pnt.normal_y, n4t.y (), 1e-4);
  EXPECT_NEAR (pnt.normal_z, n3t.z (), 1e-4); EXPECT_NEAR (pnt.normal_z, n4t.z (), 1e-4);

  PointCloud<PointXYZ> c, ct;
  c.push_back (p);
  pcl::transformPointCloud (c, ct, affine);
  EXPECT_FLOAT_EQ (pt.x, ct[0].x); 
  EXPECT_FLOAT_EQ (pt.y, ct[0].y); 
  EXPECT_FLOAT_EQ (pt.z, ct[0].z); 

  pcl::transformPointCloud (c, ct, transformation);
  EXPECT_FLOAT_EQ (pt.x, ct[0].x); 
  EXPECT_FLOAT_EQ (pt.y, ct[0].y); 
  EXPECT_FLOAT_EQ (pt.z, ct[0].z); 

  affine = transformation;

  std::vector<int> indices (1); indices[0] = 0;

  pcl::transformPointCloud (c, indices, ct, affine);
  EXPECT_FLOAT_EQ (pt.x, ct[0].x); 
  EXPECT_FLOAT_EQ (pt.y, ct[0].y); 
  EXPECT_FLOAT_EQ (pt.z, ct[0].z); 

  pcl::transformPointCloud (c, indices, ct, transformation);
  EXPECT_FLOAT_EQ (pt.x, ct[0].x); 
  EXPECT_FLOAT_EQ (pt.y, ct[0].y); 
  EXPECT_FLOAT_EQ (pt.z, ct[0].z); 
}

//////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, commonTransform)
{
  Eigen::Vector3d xaxis (1,0,0), yaxis (0,1,0), zaxis (0,0,1);
  Eigen::Affine3d trans = pcl::getTransFromUnitVectorsZY (zaxis, yaxis);
  Eigen::Vector3d xaxistrans=trans*xaxis, yaxistrans=trans*yaxis, zaxistrans=trans*zaxis;
  //std::cout << xaxistrans<<"\n"<<yaxistrans<<"\n"<<zaxistrans<<"\n";
  EXPECT_NEAR ((xaxistrans-xaxis).norm(), 0.0,  1e-6);
  EXPECT_NEAR ((yaxistrans-yaxis).norm(), 0.0,  1e-6);
  EXPECT_NEAR ((zaxistrans-zaxis).norm(), 0.0,  1e-6);
  
  trans = pcl::getTransFromUnitVectorsXY (xaxis, yaxis);
  xaxistrans=trans*xaxis, yaxistrans=trans*yaxis, zaxistrans=trans*zaxis;
  //std::cout << xaxistrans<<"\n"<<yaxistrans<<"\n"<<zaxistrans<<"\n";
  EXPECT_NEAR ((xaxistrans-xaxis).norm(), 0.0,  1e-6);
  EXPECT_NEAR ((yaxistrans-yaxis).norm(), 0.0,  1e-6);
  EXPECT_NEAR ((zaxistrans-zaxis).norm(), 0.0,  1e-6);
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
  if (loadPCDFile (argv[1], cloud_blob) < 0)
  {
    std::cerr << "Failed to read test file. Please download `bun0.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  testing::InitGoogleTest (&argc, argv);
  init();
  return (RUN_ALL_TESTS ());
}
/* ]--- */
