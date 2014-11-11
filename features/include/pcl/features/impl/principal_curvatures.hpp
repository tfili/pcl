/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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

#ifndef PCL_FEATURES_IMPL_PRINCIPAL_CURVATURES_H_
#define PCL_FEATURES_IMPL_PRINCIPAL_CURVATURES_H_

#include <pcl/features/principal_curvatures.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::PrincipalCurvaturesEstimation<PointInT, PointNT, PointOutT>::computePointPrincipalCurvatures (
      const pcl::PointCloud<PointNT> &normals, int p_idx, const std::vector<int> &indices,
      double &pcx, double &pcy, double &pcz, double &pc1, double &pc2)
{
  EIGEN_ALIGN16 Eigen::Matrix3d I = Eigen::Matrix3d::Identity ();
  Eigen::Vector3d n_idx (normals.points[p_idx].normal[0], normals.points[p_idx].normal[1], normals.points[p_idx].normal[2]);
  EIGEN_ALIGN16 Eigen::Matrix3d M = I - n_idx * n_idx.transpose ();    // projection matrix (into tangent plane)

  // Project normals into the tangent plane
  Eigen::Vector3d normal;
  projected_normals_.resize (indices.size ());
  xyz_centroid_.setZero ();
  for (size_t idx = 0; idx < indices.size(); ++idx)
  {
    normal[0] = normals.points[indices[idx]].normal[0];
    normal[1] = normals.points[indices[idx]].normal[1];
    normal[2] = normals.points[indices[idx]].normal[2];

    projected_normals_[idx] = M * normal;
    xyz_centroid_ += projected_normals_[idx];
  }

  // Estimate the XYZ centroid
  xyz_centroid_ /= static_cast<double> (indices.size ());

  // Initialize to 0
  covariance_matrix_.setZero ();

  double demean_xy, demean_xz, demean_yz;
  // For each point in the cloud
  for (size_t idx = 0; idx < indices.size (); ++idx)
  {
    demean_ = projected_normals_[idx] - xyz_centroid_;

    demean_xy = demean_[0] * demean_[1];
    demean_xz = demean_[0] * demean_[2];
    demean_yz = demean_[1] * demean_[2];

    covariance_matrix_(0, 0) += demean_[0] * demean_[0];
    covariance_matrix_(0, 1) += static_cast<double> (demean_xy);
    covariance_matrix_(0, 2) += static_cast<double> (demean_xz);

    covariance_matrix_(1, 0) += static_cast<double> (demean_xy);
    covariance_matrix_(1, 1) += demean_[1] * demean_[1];
    covariance_matrix_(1, 2) += static_cast<double> (demean_yz);

    covariance_matrix_(2, 0) += static_cast<double> (demean_xz);
    covariance_matrix_(2, 1) += static_cast<double> (demean_yz);
    covariance_matrix_(2, 2) += demean_[2] * demean_[2];
  }

  // Extract the eigenvalues and eigenvectors
  pcl::eigen33 (covariance_matrix_, eigenvalues_);
  pcl::computeCorrespondingEigenVector (covariance_matrix_, eigenvalues_ [2], eigenvector_);

  pcx = eigenvector_ [0];
  pcy = eigenvector_ [1];
  pcz = eigenvector_ [2];
  double indices_size = 1.0f / static_cast<double> (indices.size ());
  pc1 = eigenvalues_ [2] * indices_size;
  pc2 = eigenvalues_ [1] * indices_size;
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::PrincipalCurvaturesEstimation<PointInT, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
{
  // Allocate enough space to hold the results
  // \note This resize is irrelevant for a radiusSearch ().
  std::vector<int> nn_indices (k_);
  std::vector<double> nn_dists (k_);

  output.is_dense = true;
  // Save a few cycles by not checking every point for NaN/Inf values if the cloud is set to dense
  if (input_->is_dense)
  {
    // Iterating over the entire index vector
    for (size_t idx = 0; idx < indices_->size (); ++idx)
    {
      if (this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists) == 0)
      {
        output.points[idx].principal_curvature[0] = output.points[idx].principal_curvature[1] = output.points[idx].principal_curvature[2] =
          output.points[idx].pc1 = output.points[idx].pc2 = std::numeric_limits<double>::quiet_NaN ();
        output.is_dense = false;
        continue;
      }

      // Estimate the principal curvatures at each patch
      computePointPrincipalCurvatures (*normals_, (*indices_)[idx], nn_indices,
                                       output.points[idx].principal_curvature[0], output.points[idx].principal_curvature[1], output.points[idx].principal_curvature[2],
                                       output.points[idx].pc1, output.points[idx].pc2);
    }
  }
  else
  {
    // Iterating over the entire index vector
    for (size_t idx = 0; idx < indices_->size (); ++idx)
    {
      if (!isFinite ((*input_)[(*indices_)[idx]]) ||
          this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists) == 0)
      {
        output.points[idx].principal_curvature[0] = output.points[idx].principal_curvature[1] = output.points[idx].principal_curvature[2] =
          output.points[idx].pc1 = output.points[idx].pc2 = std::numeric_limits<double>::quiet_NaN ();
        output.is_dense = false;
        continue;
      }

      // Estimate the principal curvatures at each patch
      computePointPrincipalCurvatures (*normals_, (*indices_)[idx], nn_indices,
                                       output.points[idx].principal_curvature[0], output.points[idx].principal_curvature[1], output.points[idx].principal_curvature[2],
                                       output.points[idx].pc1, output.points[idx].pc2);
    }
  }
}

#define PCL_INSTANTIATE_PrincipalCurvaturesEstimation(T,NT,OutT) template class PCL_EXPORTS pcl::PrincipalCurvaturesEstimation<T,NT,OutT>;

#endif    // PCL_FEATURES_IMPL_PRINCIPAL_CURVATURES_H_
