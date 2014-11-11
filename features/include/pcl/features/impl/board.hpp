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
 *
 */

#ifndef PCL_FEATURES_IMPL_BOARD_H_
#define PCL_FEATURES_IMPL_BOARD_H_

#include <pcl/features/board.h>
#include <utility>
#include <pcl/common/transforms.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT> void
pcl::BOARDLocalReferenceFrameEstimation<PointInT, PointNT, PointOutT>::directedOrthogonalAxis (
                                                                                               Eigen::Vector3d const &axis,
                                                                                               Eigen::Vector3d const &axis_origin,
                                                                                               Eigen::Vector3d const &point,
                                                                                               Eigen::Vector3d &directed_ortho_axis)
{
  Eigen::Vector3d projection;
  projectPointOnPlane (point, axis_origin, axis, projection);
  directed_ortho_axis = projection - axis_origin;

  directed_ortho_axis.normalize ();

  // check if the computed x axis is orthogonal to the normal
  //assert(areEquals((double)(directed_ortho_axis.dot(axis)), 0.0, 1E-3f));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT> void
pcl::BOARDLocalReferenceFrameEstimation<PointInT, PointNT, PointOutT>::projectPointOnPlane (
                                                                                            Eigen::Vector3d const &point,
                                                                                            Eigen::Vector3d const &origin_point,
                                                                                            Eigen::Vector3d const &plane_normal,
                                                                                            Eigen::Vector3d &projected_point)
{
  double t;
  Eigen::Vector3d xo;

  xo = point - origin_point;
  t = plane_normal.dot (xo);

  projected_point = point - (t * plane_normal);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT> double
pcl::BOARDLocalReferenceFrameEstimation<PointInT, PointNT, PointOutT>::getAngleBetweenUnitVectors (
                                                                                                   Eigen::Vector3d const &v1,
                                                                                                   Eigen::Vector3d const &v2,
                                                                                                   Eigen::Vector3d const &axis)
{
  Eigen::Vector3d angle_orientation;
  angle_orientation = v1.cross (v2);
  double angle_radians = acos (std::max (-1.0, std::min (1.0, v1.dot (v2))));

  angle_radians = angle_orientation.dot (axis) < 0. ? (2 * static_cast<double> (M_PI) - angle_radians) : angle_radians;

  return (angle_radians);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT> void
pcl::BOARDLocalReferenceFrameEstimation<PointInT, PointNT, PointOutT>::randomOrthogonalAxis (
                                                                                             Eigen::Vector3d const &axis,
                                                                                             Eigen::Vector3d &rand_ortho_axis)
{
  if (!areEquals (axis.z (), 0.0))
  {
    rand_ortho_axis.x () = (static_cast<double> (rand ()) / static_cast<double> (RAND_MAX)) * 2.0 - 1.0;
    rand_ortho_axis.y () = (static_cast<double> (rand ()) / static_cast<double> (RAND_MAX)) * 2.0 - 1.0;
    rand_ortho_axis.z () = -(axis.x () * rand_ortho_axis.x () + axis.y () * rand_ortho_axis.y ()) / axis.z ();
  }
  else if (!areEquals (axis.y (), 0.0))
  {
    rand_ortho_axis.x () = (static_cast<double> (rand ()) / static_cast<double> (RAND_MAX)) * 2.0 - 1.0;
    rand_ortho_axis.z () = (static_cast<double> (rand ()) / static_cast<double> (RAND_MAX)) * 2.0 - 1.0;
    rand_ortho_axis.y () = -(axis.x () * rand_ortho_axis.x () + axis.z () * rand_ortho_axis.z ()) / axis.y ();
  }
  else if (!areEquals (axis.x (), 0.0))
  {
    rand_ortho_axis.y () = (static_cast<double> (rand ()) / static_cast<double> (RAND_MAX)) * 2.0 - 1.0;
    rand_ortho_axis.z () = (static_cast<double> (rand ()) / static_cast<double> (RAND_MAX)) * 2.0 - 1.0;
    rand_ortho_axis.x () = -(axis.y () * rand_ortho_axis.y () + axis.z () * rand_ortho_axis.z ()) / axis.x ();
  }

  rand_ortho_axis.normalize ();

  // check if the computed x axis is orthogonal to the normal
  //assert(areEquals(rand_ortho_axis.dot(axis), 0.0, 1E-6));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT> void
pcl::BOARDLocalReferenceFrameEstimation<PointInT, PointNT, PointOutT>::planeFitting (
                                                                                     Eigen::Matrix<double,
                                                                                         Eigen::Dynamic, 3> const &points,
                                                                                     Eigen::Vector3d &center,
                                                                                     Eigen::Vector3d &norm)
{
  // -----------------------------------------------------
  // Plane Fitting using Singular Value Decomposition (SVD)
  // -----------------------------------------------------

  int n_points = static_cast<int> (points.rows ());
  if (n_points == 0)
  {
    return;
  }

  //find the center by averaging the points positions
  center.setZero ();

  for (int i = 0; i < n_points; ++i)
  {
    center += points.row (i);
  }

  center /= static_cast<double> (n_points);

  //copy points - average (center)
  Eigen::Matrix<double, Eigen::Dynamic, 3> A (n_points, 3); //PointData
  for (int i = 0; i < n_points; ++i)
  {
    A (i, 0) = points (i, 0) - center.x ();
    A (i, 1) = points (i, 1) - center.y ();
    A (i, 2) = points (i, 2) - center.z ();
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd (A, Eigen::ComputeFullV);
  norm = svd.matrixV ().col (2);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT> void
pcl::BOARDLocalReferenceFrameEstimation<PointInT, PointNT, PointOutT>::normalDisambiguation (
                                                                                             pcl::PointCloud<PointNT> const &normal_cloud,
                                                                                             std::vector<int> const &normal_indices,
                                                                                             Eigen::Vector3d &normal)
{
  Eigen::Vector3d normal_mean;
  normal_mean.setZero ();

  for (size_t i = 0; i < normal_indices.size (); ++i)
  {
    const PointNT& curPt = normal_cloud[normal_indices[i]];

    normal_mean += curPt.getNormalVector3dMap ();
  }

  normal_mean.normalize ();

  if (normal.dot (normal_mean) < 0)
  {
    normal = -normal;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT> double
pcl::BOARDLocalReferenceFrameEstimation<PointInT, PointNT, PointOutT>::computePointLRF (const int &index,
                                                                                        Eigen::Matrix3d &lrf)
{
  //find Z axis

  //extract support points for Rz radius
  std::vector<int> neighbours_indices;
  std::vector<double> neighbours_distances;
  int n_neighbours = this->searchForNeighbors (index, search_parameter_, neighbours_indices, neighbours_distances);

  //check if there are enough neighbor points, otherwise compute a random X axis and use normal as Z axis
  if (n_neighbours < 6)
  {
    //PCL_WARN(
    //    "[pcl::%s::computePointLRF] Warning! Neighborhood has less than 6 vertices. Aborting description of point with index %d\n",
    //    getClassName().c_str(), index);

    //setting lrf to NaN
    lrf.setConstant (std::numeric_limits<double>::quiet_NaN ());

    return (std::numeric_limits<double>::max ());
  }

  //copy neighbours coordinates into eigen matrix
  Eigen::Matrix<double, Eigen::Dynamic, 3> neigh_points_mat (n_neighbours, 3);
  for (int i = 0; i < n_neighbours; ++i)
  {
    neigh_points_mat.row (i) = (*surface_)[neighbours_indices[i]].getVector3dMap ();
  }

  Eigen::Vector3d x_axis, y_axis;
  //plane fitting to find direction of Z axis
  Eigen::Vector3d fitted_normal; //z_axis
  Eigen::Vector3d centroid;
  planeFitting (neigh_points_mat, centroid, fitted_normal);

  //disambiguate Z axis with normal mean
  normalDisambiguation (*normals_, neighbours_indices, fitted_normal);

  //setting LRF Z axis
  lrf.row (2).matrix () = fitted_normal;

  /////////////////////////////////////////////////////////////////////////////////////////
  //find X axis

  //extract support points for Rx radius
  if (tangent_radius_ != 0.0 && search_parameter_ != tangent_radius_)
  {
    n_neighbours = this->searchForNeighbors (index, tangent_radius_, neighbours_indices, neighbours_distances);
  }

  //find point with the "most different" normal (with respect to fittedNormal)

  double min_normal_cos = std::numeric_limits<double>::max ();
  int min_normal_index = -1;

  bool margin_point_found = false;
  Eigen::Vector3d best_margin_point;
  bool best_point_found_on_margins = false;

  double radius2 = tangent_radius_ * tangent_radius_;

  double margin_distance2 = margin_thresh_ * margin_thresh_ * radius2;

  double max_boundary_angle = 0;

  if (find_holes_)
  {
    randomOrthogonalAxis (fitted_normal, x_axis);

    lrf.row (0).matrix () = x_axis;

    for (int i = 0; i < check_margin_array_size_; i++)
    {
      check_margin_array_[i] = false;
      margin_array_min_angle_[i] = std::numeric_limits<double>::max ();
      margin_array_max_angle_[i] = -std::numeric_limits<double>::max ();
      margin_array_min_angle_normal_[i] = -1.0;
      margin_array_max_angle_normal_[i] = -1.0;
    }
    max_boundary_angle = (2 * static_cast<double> (M_PI)) / static_cast<double> (check_margin_array_size_);
  }

  for (int curr_neigh = 0; curr_neigh < n_neighbours; ++curr_neigh)
  {
    const int& curr_neigh_idx = neighbours_indices[curr_neigh];
    const double& neigh_distance_sqr = neighbours_distances[curr_neigh];
    if (neigh_distance_sqr <= margin_distance2)
    {
      continue;
    }

    //point normalIndex is inside the ring between marginThresh and Radius
    margin_point_found = true;

    Eigen::Vector3d normal_mean = normals_->at (curr_neigh_idx).getNormalVector3dMap ();

    double normal_cos = fitted_normal.dot (normal_mean);
    if (normal_cos < min_normal_cos)
    {
      min_normal_index = curr_neigh_idx;
      min_normal_cos = normal_cos;
      best_point_found_on_margins = false;
    }

    if (find_holes_)
    {
      //find angle with respect to random axis previously calculated
      Eigen::Vector3d indicating_normal_vect;
      directedOrthogonalAxis (fitted_normal, input_->at (index).getVector3dMap (),
                              surface_->at (curr_neigh_idx).getVector3dMap (), indicating_normal_vect);
      double angle = getAngleBetweenUnitVectors (x_axis, indicating_normal_vect, fitted_normal);

      int check_margin_array_idx = std::min (static_cast<int> (floor (angle / max_boundary_angle)), check_margin_array_size_ - 1);
      check_margin_array_[check_margin_array_idx] = true;

      if (angle < margin_array_min_angle_[check_margin_array_idx])
      {
        margin_array_min_angle_[check_margin_array_idx] = angle;
        margin_array_min_angle_normal_[check_margin_array_idx] = normal_cos;
      }
      if (angle > margin_array_max_angle_[check_margin_array_idx])
      {
        margin_array_max_angle_[check_margin_array_idx] = angle;
        margin_array_max_angle_normal_[check_margin_array_idx] = normal_cos;
      }
    }

  } //for each neighbor

  if (!margin_point_found)
  {
    //find among points with neighDistance <= marginThresh*radius
    for (int curr_neigh = 0; curr_neigh < n_neighbours; curr_neigh++)
    {
      const int& curr_neigh_idx = neighbours_indices[curr_neigh];
      const double& neigh_distance_sqr = neighbours_distances[curr_neigh];

      if (neigh_distance_sqr > margin_distance2)
        continue;

      Eigen::Vector3d normal_mean = normals_->at (curr_neigh_idx).getNormalVector3dMap ();

      double normal_cos = fitted_normal.dot (normal_mean);

      if (normal_cos < min_normal_cos)
      {
        min_normal_index = curr_neigh_idx;
        min_normal_cos = normal_cos;
      }
    }//for each neighbor

    // Check if we are not in a degenerate case (all the neighboring normals are NaNs)
    if (min_normal_index == -1)
    {
      lrf.setConstant (std::numeric_limits<double>::quiet_NaN ());
      return (std::numeric_limits<double>::max ());
    }
    //find orthogonal axis directed to minNormalIndex point projection on plane with fittedNormal as axis
    directedOrthogonalAxis (fitted_normal, input_->at (index).getVector3dMap (),
                            surface_->at (min_normal_index).getVector3dMap (), x_axis);
    y_axis = fitted_normal.cross (x_axis);

    lrf.row (0).matrix () = x_axis;
    lrf.row (1).matrix () = y_axis;
    //z axis already set


    return (min_normal_cos);
  }

  if (!find_holes_)
  {
    if (best_point_found_on_margins)
    {
      //if most inclined normal is on support margin
      directedOrthogonalAxis (fitted_normal, input_->at (index).getVector3dMap (), best_margin_point, x_axis);
      y_axis = fitted_normal.cross (x_axis);

      lrf.row (0).matrix () = x_axis;
      lrf.row (1).matrix () = y_axis;
      //z axis already set

      return (min_normal_cos);
    }

    // Check if we are not in a degenerate case (all the neighboring normals are NaNs)
    if (min_normal_index == -1)
    {
      lrf.setConstant (std::numeric_limits<double>::quiet_NaN ());
      return (std::numeric_limits<double>::max ());
    }

    directedOrthogonalAxis (fitted_normal, input_->at (index).getVector3dMap (),
                            surface_->at (min_normal_index).getVector3dMap (), x_axis);
    y_axis = fitted_normal.cross (x_axis);

    lrf.row (0).matrix () = x_axis;
    lrf.row (1).matrix () = y_axis;
    //z axis already set

    return (min_normal_cos);
  }// if(!find_holes_)

  //check if there is at least a hole
  bool is_hole_present = false;
  for (int i = 0; i < check_margin_array_size_; i++)
  {
    if (!check_margin_array_[i])
    {
      is_hole_present = true;
      break;
    }
  }

  if (!is_hole_present)
  {
    if (best_point_found_on_margins)
    {
      //if most inclined normal is on support margin
      directedOrthogonalAxis (fitted_normal, input_->at (index).getVector3dMap (), best_margin_point, x_axis);
      y_axis = fitted_normal.cross (x_axis);

      lrf.row (0).matrix () = x_axis;
      lrf.row (1).matrix () = y_axis;
      //z axis already set

      return (min_normal_cos);
    }

    // Check if we are not in a degenerate case (all the neighboring normals are NaNs)
    if (min_normal_index == -1)
    {
      lrf.setConstant (std::numeric_limits<double>::quiet_NaN ());
      return (std::numeric_limits<double>::max ());
    }

    //find orthogonal axis directed to minNormalIndex point projection on plane with fittedNormal as axis
    directedOrthogonalAxis (fitted_normal, input_->at (index).getVector3dMap (),
                            surface_->at (min_normal_index).getVector3dMap (), x_axis);
    y_axis = fitted_normal.cross (x_axis);

    lrf.row (0).matrix () = x_axis;
    lrf.row (1).matrix () = y_axis;
    //z axis already set

    return (min_normal_cos);
  }//if (!is_hole_present)

  //case hole found
  //find missing region
  double angle = 0.0;
  int hole_end;
  int hole_first;

  //find first no border pie
  int first_no_border = -1;
  if (check_margin_array_[check_margin_array_size_ - 1])
  {
    first_no_border = 0;
  }
  else
  {
    for (int i = 0; i < check_margin_array_size_; i++)
    {
      if (check_margin_array_[i])
      {
        first_no_border = i;
        break;
      }
    }
  }

  //double steep_prob = 0.0;
  double max_hole_prob = -std::numeric_limits<double>::max ();

  //find holes
  for (int ch = first_no_border; ch < check_margin_array_size_; ch++)
  {
    if (!check_margin_array_[ch])
    {
      //border beginning found
      hole_first = ch;
      hole_end = hole_first + 1;
      while (!check_margin_array_[hole_end % check_margin_array_size_])
      {
        ++hole_end;
      }
      //border end found, find angle

      if ((hole_end - hole_first) > 0)
      {
        //check if hole can be a shapeness hole
        int previous_hole = (((hole_first - 1) < 0) ? (hole_first - 1) + check_margin_array_size_ : (hole_first - 1))
            % check_margin_array_size_;
        int following_hole = (hole_end) % check_margin_array_size_;
        double normal_begin = margin_array_max_angle_normal_[previous_hole];
        double normal_end = margin_array_min_angle_normal_[following_hole];
        normal_begin -= min_normal_cos;
        normal_end -= min_normal_cos;
        normal_begin = normal_begin / (1.0 - min_normal_cos);
        normal_end = normal_end / (1.0 - min_normal_cos);
        normal_begin = 1.0 - normal_begin;
        normal_end = 1.0 - normal_end;

        //evaluate P(Hole);
        double hole_width = 0.0;
        if (following_hole < previous_hole)
        {
          hole_width = margin_array_min_angle_[following_hole] + 2 * static_cast<double> (M_PI)
              - margin_array_max_angle_[previous_hole];
        }
        else
        {
          hole_width = margin_array_min_angle_[following_hole] - margin_array_max_angle_[previous_hole];
        }
        double hole_prob = hole_width / (2 * static_cast<double> (M_PI));

        //evaluate P(zmin|Hole)
        double steep_prob = (normal_end + normal_begin) / 2.0;

        //check hole prob and after that, check steepThresh

        if (hole_prob > hole_size_prob_thresh_)
        {
          if (steep_prob > steep_thresh_)
          {
            if (hole_prob > max_hole_prob)
            {
              max_hole_prob = hole_prob;

              double angle_weight = ((normal_end - normal_begin) + 1.0) / 2.0;
              if (following_hole < previous_hole)
              {
                angle = margin_array_max_angle_[previous_hole] + (margin_array_min_angle_[following_hole] + 2
                    * static_cast<double> (M_PI) - margin_array_max_angle_[previous_hole]) * angle_weight;
              }
              else
              {
                angle = margin_array_max_angle_[previous_hole] + (margin_array_min_angle_[following_hole]
                    - margin_array_max_angle_[previous_hole]) * angle_weight;
              }
            }
          }
        }
      } //(hole_end-hole_first) > 0

      if (hole_end >= check_margin_array_size_)
      {
        break;
      }
      else
      {
        ch = hole_end - 1;
      }
    }
  }

  if (max_hole_prob > -std::numeric_limits<double>::max ())
  {
    //hole found
    Eigen::AngleAxisd rotation = Eigen::AngleAxisd (angle, fitted_normal);
    x_axis = rotation * x_axis;

    min_normal_cos -= 10.0;
  }
  else
  {
    if (best_point_found_on_margins)
    {
      //if most inclined normal is on support margin
      directedOrthogonalAxis (fitted_normal, input_->at (index).getVector3dMap (), best_margin_point, x_axis);
    }
    else
    {
      // Check if we are not in a degenerate case (all the neighboring normals are NaNs)
      if (min_normal_index == -1)
      {
        lrf.setConstant (std::numeric_limits<double>::quiet_NaN ());
        return (std::numeric_limits<double>::max ());
      }

      //find orthogonal axis directed to minNormalIndex point projection on plane with fittedNormal as axis
      directedOrthogonalAxis (fitted_normal, input_->at (index).getVector3dMap (),
                              surface_->at (min_normal_index).getVector3dMap (), x_axis);
    }
  }

  y_axis = fitted_normal.cross (x_axis);

  lrf.row (0).matrix () = x_axis;
  lrf.row (1).matrix () = y_axis;
  //z axis already set

  return (min_normal_cos);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT> void
pcl::BOARDLocalReferenceFrameEstimation<PointInT, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
{
  //check whether used with search radius or search k-neighbors
  if (this->getKSearch () != 0)
  {
    PCL_ERROR(
        "[pcl::%s::computeFeature] Error! Search method set to k-neighborhood. Call setKSearch(0) and setRadiusSearch( radius ) to use this class.\n",
        getClassName().c_str());
    return;
  }

  this->resetData ();
  for (size_t point_idx = 0; point_idx < indices_->size (); ++point_idx)
  {
    Eigen::Matrix3d currentLrf;
    PointOutT &rf = output[point_idx];

    //rf.confidence = computePointLRF (*indices_[point_idx], currentLrf);
    //if (rf.confidence == std::numeric_limits<double>::max ())
    if (computePointLRF ((*indices_)[point_idx], currentLrf) == std::numeric_limits<double>::max ())
    {
      output.is_dense = false;
    }

    for (int d = 0; d < 3; ++d)
    {
      rf.x_axis[d] = currentLrf (0, d);
      rf.y_axis[d] = currentLrf (1, d);
      rf.z_axis[d] = currentLrf (2, d);
    }
  }
}

#define PCL_INSTANTIATE_BOARDLocalReferenceFrameEstimation(T,NT,OutT) template class PCL_EXPORTS pcl::BOARDLocalReferenceFrameEstimation<T,NT,OutT>;

#endif // PCL_FEATURES_IMPL_BOARD_H_
