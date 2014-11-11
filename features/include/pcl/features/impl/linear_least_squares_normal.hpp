/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 */

#ifndef PCL_FEATURES_LINEAR_LEAST_SQUARES_NORMAL_HPP_
#define PCL_FEATURES_LINEAR_LEAST_SQUARES_NORMAL_HPP_
#define EIGEN_II_METHOD 1

#include <pcl/features/linear_least_squares_normal.h>
#include <pcl/common/time.h>
//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT>
pcl::LinearLeastSquaresNormalEstimation<PointInT, PointOutT>::~LinearLeastSquaresNormalEstimation ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::LinearLeastSquaresNormalEstimation<PointInT, PointOutT>::computePointNormal (
    const int pos_x, const int pos_y, PointOutT &normal)
{
  const double bad_point = std::numeric_limits<double>::quiet_NaN ();

  const int width = input_->width;
  const int height = input_->height;

  const int x = pos_x;
  const int y = pos_y;

  const int index = y * width + x;

  const double px = input_->points[index].x;
  const double py = input_->points[index].y;
  const double pz = input_->points[index].z;

  if (pcl_isnan (px)) 
  {
    normal.normal_x = bad_point;
    normal.normal_y = bad_point;
    normal.normal_z = bad_point;
    normal.curvature = bad_point;

    return;
  }

  double smoothingSize = normal_smoothing_size_;
  if (use_depth_dependent_smoothing_) smoothingSize = smoothingSize*(pz+0.5f);

  const int smoothingSizeInt = static_cast<int> (smoothingSize);

  double matA0 = 0.0f;
  double matA1 = 0.0f;
  double matA3 = 0.0f;

  double vecb0 = 0.0f;
  double vecb1 = 0.0f;

  for (int v = y - smoothingSizeInt; v <= y + smoothingSizeInt; v += smoothingSizeInt)
  {
    for (int u = x - smoothingSizeInt; u <= x + smoothingSizeInt; u += smoothingSizeInt)
    {
      if (u < 0 || u >= width || v < 0 || v >= height) continue;

      const int index2 = v * width + u;

      const double qx = input_->points[index2].x;
      const double qy = input_->points[index2].y;
      const double qz = input_->points[index2].z;

      if (pcl_isnan (qx)) continue;

      const double delta = qz - pz;
      const double i = qx - px;
      const double j = qy - py;

      double depthChangeThreshold = pz*pz * 0.05f * max_depth_change_factor_;
      if (use_depth_dependent_smoothing_) depthChangeThreshold *= pz;

      const double f = fabs (delta) > depthChangeThreshold ? 0 : 1;

      matA0 += f * i * i;
      matA1 += f * i * j;
      matA3 += f * j * j;
      vecb0 += f * i * delta;
      vecb1 += f * j * delta;
    }
  }

  const double det = matA0 * matA3 - matA1 * matA1;
  const double ddx = matA3 * vecb0 - matA1 * vecb1;
  const double ddy = -matA1 * vecb0 + matA0 * vecb1;

  const double nx = ddx;
  const double ny = ddy;
  const double nz = -det * pz;

  const double length = nx * nx + ny * ny + nz * nz;

  if (length <= 0.01f)
  {
    normal.normal_x = bad_point;
    normal.normal_y = bad_point;
    normal.normal_z = bad_point;
    normal.curvature = bad_point;
  }
  else
  {
    const double normInv = 1.0f / sqrtf (length);

    normal.normal_x = -nx * normInv;
    normal.normal_y = -ny * normInv;
    normal.normal_z = -nz * normInv;
    normal.curvature = bad_point;
  }

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::LinearLeastSquaresNormalEstimation<PointInT, PointOutT>::computeFeature (PointCloudOut &output)
{
  const double bad_point = std::numeric_limits<double>::quiet_NaN ();

  const int width = input_->width;
  const int height = input_->height;

  // we compute the normals as follows:
  // ----------------------------------
  // 
  // for the depth-gradient you can make the following first-order Taylor approximation:
  //   D(x + dx) - D(x) = dx^T \Delta D + h.o.t.
  //     
  // build linear system by stacking up equation for 8 neighbor points:
  //   Y = X \Delta D
  // 
  // => \Delta D = (X^T X)^{-1} X^T Y
  // => \Delta D = (A)^{-1} b

  //const double smoothingSize = 30.0f;
  for (int y = 0; y < height; ++y)
  {
    for (int x = 0; x < width; ++x)
    {
      const int index = y * width + x;

      const double px = input_->points[index].x;
      const double py = input_->points[index].y;
      const double pz = input_->points[index].z;

      if (pcl_isnan(px)) continue;

      //double depthDependentSmoothingSize = smoothingSize + pz / 10.0f;

      double smoothingSize = normal_smoothing_size_;
      //if (use_depth_dependent_smoothing_) smoothingSize *= pz;
      //if (use_depth_dependent_smoothing_) smoothingSize += pz*5;
      //if (smoothingSize < 1.0f) smoothingSize += 1.0f;

      const int smoothingSizeInt = static_cast<int>(smoothingSize);

      double matA0 = 0.0f;
      double matA1 = 0.0f;
      double matA3 = 0.0f;

      double vecb0 = 0.0f;
      double vecb1 = 0.0f;

      for (int v = y - smoothingSizeInt; v <= y + smoothingSizeInt; v += smoothingSizeInt)
      {
        for (int u = x - smoothingSizeInt; u <= x + smoothingSizeInt; u += smoothingSizeInt)
        {
          if (u < 0 || u >= width || v < 0 || v >= height) continue;

          const int index2 = v * width + u;

          const double qx = input_->points[index2].x;
          const double qy = input_->points[index2].y;
          const double qz = input_->points[index2].z;

          if (pcl_isnan(qx)) continue;

          const double delta = qz - pz;
          const double i = qx - px;
          const double j = qy - py;

          const double depthDependendDepthChange = (max_depth_change_factor_ * (fabsf (pz) + 1.0f) * 2.0f);
          const double f = fabs(delta) > depthDependendDepthChange ? 0 : 1;

          //double f = fabs(delta) > (pz * 0.05f - 0.3f) ? 0 : 1;
          //const double f = fabs(delta) > (pz*pz * 0.05f * max_depth_change_factor_) ? 0 : 1;
          //double f = Math.Abs(delta) > (depth * Math.Log(depth + 1.0) * 0.02f - 0.2f) ? 0 : 1;

          matA0 += f * i * i;
          matA1 += f * i * j;
          matA3 += f * j * j;
          vecb0 += f * i * delta;
          vecb1 += f * j * delta;
        }
      }

      const double det = matA0 * matA3 - matA1 * matA1;
      const double ddx = matA3 * vecb0 - matA1 * vecb1;
      const double ddy = -matA1 * vecb0 + matA0 * vecb1;

      const double nx = ddx;
      const double ny = ddy;
      const double nz = -det * pz;

      const double length = nx * nx + ny * ny + nz * nz;

      if (length <= 0.0f)
      {
        output.points[index].normal_x = bad_point;
        output.points[index].normal_y = bad_point;
        output.points[index].normal_z = bad_point;
        output.points[index].curvature = bad_point;
      }
      else
      {
        const double normInv = 1.0f / sqrtf (length);

        output.points[index].normal_x = nx * normInv;
        output.points[index].normal_y = ny * normInv;
        output.points[index].normal_z = nz * normInv;
        output.points[index].curvature = bad_point;
      }
    }
  }
}

#define PCL_INSTANTIATE_LinearLeastSquaresNormalEstimation(T,NT) template class PCL_EXPORTS pcl::LinearLeastSquaresNormalEstimation<T,NT>;
//#define LinearLeastSquaresNormalEstimation(T,NT) template class PCL_EXPORTS pcl::LinearLeastSquaresNormalEstimation<T,NT>;

#endif

