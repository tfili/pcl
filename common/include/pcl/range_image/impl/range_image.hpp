/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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

#ifndef PCL_RANGE_IMAGE_IMPL_HPP_
#define PCL_RANGE_IMAGE_IMPL_HPP_

#include <pcl/pcl_macros.h>
#include <pcl/common/distances.h>

namespace pcl
{

/////////////////////////////////////////////////////////////////////////
inline double
RangeImage::asinLookUp (double value)
{
  return (asin_lookup_table[
      static_cast<int> (
        static_cast<double> (pcl_lrintf ( (static_cast<double> (lookup_table_size-1) / 2.0) * value)) + 
        static_cast<double> (lookup_table_size-1) / 2.0)]);
}

/////////////////////////////////////////////////////////////////////////
inline double
RangeImage::atan2LookUp (double y, double x)
{
  if (x==0 && y==0)
    return 0;
  double ret;
  if (fabsf (x) < fabsf (y)) 
  {
    ret = atan_lookup_table[
      static_cast<int> (
          static_cast<double> (pcl_lrintf ( (static_cast<double> (lookup_table_size-1) / 2.0) * (x / y))) + 
          static_cast<double> (lookup_table_size-1) / 2.0)];
    ret = static_cast<double> (x*y > 0 ? M_PI/2-ret : -M_PI/2-ret);
  }
  else
    ret = atan_lookup_table[
      static_cast<int> (
          static_cast<double> (pcl_lrintf ( (static_cast<double> (lookup_table_size-1) / 2.0) * (y / x))) + 
          static_cast<double> (lookup_table_size-1)/2.0)];
  if (x < 0)
    ret = static_cast<double> (y < 0 ? ret-M_PI : ret+M_PI);
  
  return (ret);
}

/////////////////////////////////////////////////////////////////////////
inline double
RangeImage::cosLookUp (double value)
{
  int cell_idx = static_cast<int> (pcl_lrintf ( (static_cast<double> (lookup_table_size-1)) * fabsf (value) / (2.0 * static_cast<double> (M_PI))));
  return (cos_lookup_table[cell_idx]);
}

/////////////////////////////////////////////////////////////////////////
template <typename PointCloudType> void 
RangeImage::createFromPointCloud (const PointCloudType& point_cloud, double angular_resolution,
                                  double max_angle_width, double max_angle_height,
                                  const Eigen::Affine3d& sensor_pose, RangeImage::CoordinateFrame coordinate_frame,
                                  double noise_level, double min_range, int border_size)
{
  createFromPointCloud (point_cloud, angular_resolution, angular_resolution, max_angle_width, max_angle_height,
                        sensor_pose, coordinate_frame, noise_level, min_range, border_size);
}

/////////////////////////////////////////////////////////////////////////
template <typename PointCloudType> void 
RangeImage::createFromPointCloud (const PointCloudType& point_cloud,
                                  double angular_resolution_x, double angular_resolution_y,
                                  double max_angle_width, double max_angle_height,
                                  const Eigen::Affine3d& sensor_pose, RangeImage::CoordinateFrame coordinate_frame,
                                  double noise_level, double min_range, int border_size)
{
  setAngularResolution (angular_resolution_x, angular_resolution_y);
  
  width  = static_cast<uint32_t> (pcl_lrint (floor (max_angle_width*angular_resolution_x_reciprocal_)));
  height = static_cast<uint32_t> (pcl_lrint (floor (max_angle_height*angular_resolution_y_reciprocal_)));
  
  int full_width  = static_cast<int> (pcl_lrint (floor (pcl::deg2rad (360.0)*angular_resolution_x_reciprocal_))),
      full_height = static_cast<int> (pcl_lrint (floor (pcl::deg2rad (180.0)*angular_resolution_y_reciprocal_)));
  image_offset_x_ = (full_width -static_cast<int> (width) )/2;
  image_offset_y_ = (full_height-static_cast<int> (height))/2;
  is_dense = false;
  
  getCoordinateFrameTransformation (coordinate_frame, to_world_system_);
  to_world_system_ = sensor_pose * to_world_system_;
  
  to_range_image_system_ = to_world_system_.inverse (Eigen::Isometry);
  //std::cout << "to_world_system_ is\n"<<to_world_system_<<"\nand to_range_image_system_ is\n"<<to_range_image_system_<<"\n\n";
  
  unsigned int size = width*height;
  points.clear ();
  points.resize (size, unobserved_point);
  
  int top=height, right=-1, bottom=-1, left=width;
  doZBuffer (point_cloud, noise_level, min_range, top, right, bottom, left);
  
  cropImage (border_size, top, right, bottom, left);
  
  recalculate3DPointPositions ();
}

/////////////////////////////////////////////////////////////////////////
template <typename PointCloudType> void
RangeImage::createFromPointCloudWithKnownSize (const PointCloudType& point_cloud, double angular_resolution,
                                              const Eigen::Vector3d& point_cloud_center, double point_cloud_radius,
                                              const Eigen::Affine3d& sensor_pose, RangeImage::CoordinateFrame coordinate_frame,
                                              double noise_level, double min_range, int border_size)
{
  createFromPointCloudWithKnownSize (point_cloud, angular_resolution, angular_resolution, point_cloud_center, point_cloud_radius,
                                     sensor_pose, coordinate_frame, noise_level, min_range, border_size);
}

/////////////////////////////////////////////////////////////////////////
template <typename PointCloudType> void
RangeImage::createFromPointCloudWithKnownSize (const PointCloudType& point_cloud,
                                               double angular_resolution_x, double angular_resolution_y,
                                               const Eigen::Vector3d& point_cloud_center, double point_cloud_radius,
                                               const Eigen::Affine3d& sensor_pose, RangeImage::CoordinateFrame coordinate_frame,
                                               double noise_level, double min_range, int border_size)
{
  //MEASURE_FUNCTION_TIME;
  
  //std::cout << "Starting to create range image from "<<point_cloud.points.size ()<<" points.\n";
  
  // If the sensor pose is inside of the sphere we have to calculate the image the normal way
  if ((point_cloud_center-sensor_pose.translation()).norm() <= point_cloud_radius) {
    createFromPointCloud (point_cloud, angular_resolution_x, angular_resolution_y,
                          pcl::deg2rad (360.0), pcl::deg2rad (180.0),
                          sensor_pose, coordinate_frame, noise_level, min_range, border_size);
    return;
  }
  
  setAngularResolution (angular_resolution_x, angular_resolution_y);
  
  getCoordinateFrameTransformation (coordinate_frame, to_world_system_);
  to_world_system_ = sensor_pose * to_world_system_;
  to_range_image_system_ = to_world_system_.inverse (Eigen::Isometry);
  
  double max_angle_size = getMaxAngleSize (sensor_pose, point_cloud_center, point_cloud_radius);
  int pixel_radius_x = pcl_lrint (ceil (0.5*max_angle_size*angular_resolution_x_reciprocal_)),
      pixel_radius_y = pcl_lrint (ceil (0.5*max_angle_size*angular_resolution_y_reciprocal_));
  width  = 2*pixel_radius_x;
  height = 2*pixel_radius_y;
  is_dense = false;
  
  image_offset_x_ = image_offset_y_ = 0;  // temporary values for getImagePoint
  int center_pixel_x, center_pixel_y;
  getImagePoint (point_cloud_center, center_pixel_x, center_pixel_y);
  image_offset_x_ = (std::max) (0, center_pixel_x-pixel_radius_x);
  image_offset_y_ = (std::max) (0, center_pixel_y-pixel_radius_y);

  points.clear ();
  points.resize (width*height, unobserved_point);
  
  int top=height, right=-1, bottom=-1, left=width;
  doZBuffer (point_cloud, noise_level, min_range, top, right, bottom, left);
  
  cropImage (border_size, top, right, bottom, left);
  
  recalculate3DPointPositions ();
}

/////////////////////////////////////////////////////////////////////////
template <typename PointCloudTypeWithViewpoints> void 
RangeImage::createFromPointCloudWithViewpoints (const PointCloudTypeWithViewpoints& point_cloud,
                                                double angular_resolution,
                                                double max_angle_width, double max_angle_height,
                                                RangeImage::CoordinateFrame coordinate_frame,
                                                double noise_level, double min_range, int border_size)
{
  createFromPointCloudWithViewpoints (point_cloud, angular_resolution, angular_resolution,
                                      max_angle_width, max_angle_height, coordinate_frame,
                                      noise_level, min_range, border_size);
}

/////////////////////////////////////////////////////////////////////////
template <typename PointCloudTypeWithViewpoints> void 
RangeImage::createFromPointCloudWithViewpoints (const PointCloudTypeWithViewpoints& point_cloud,
                                                double angular_resolution_x, double angular_resolution_y,
                                                double max_angle_width, double max_angle_height,
                                                RangeImage::CoordinateFrame coordinate_frame,
                                                double noise_level, double min_range, int border_size)
{
  Eigen::Vector3d average_viewpoint = getAverageViewPoint (point_cloud);
  Eigen::Affine3d sensor_pose = static_cast<Eigen::Affine3d> (Eigen::Translation3d (average_viewpoint));
  createFromPointCloud (point_cloud, angular_resolution_x, angular_resolution_y, max_angle_width, max_angle_height,
                        sensor_pose, coordinate_frame, noise_level, min_range, border_size);
}

/////////////////////////////////////////////////////////////////////////
template <typename PointCloudType> void 
RangeImage::doZBuffer (const PointCloudType& point_cloud, double noise_level, double min_range, int& top, int& right, int& bottom, int& left)
{
  typedef typename PointCloudType::PointType PointType2;
  const typename pcl::PointCloud<PointType2>::VectorType &points2 = point_cloud.points;
  
  unsigned int size = width*height;
  int* counters = new int[size];
  ERASE_ARRAY (counters, size);
  
  top=height; right=-1; bottom=-1; left=width;
  
  double x_real, y_real, range_of_current_point;
  int x, y;
  for (typename pcl::PointCloud<PointType2>::VectorType::const_iterator it=points2.begin (); it!=points2.end (); ++it)
  {
    if (!isFinite (*it))  // Check for NAN etc
      continue;
    Vector3dMapConst current_point = it->getVector3dMap ();
    
    this->getImagePoint (current_point, x_real, y_real, range_of_current_point);
    this->real2DToInt2D (x_real, y_real, x, y);
    
    if (range_of_current_point < min_range|| !isInImage (x, y))
      continue;
    //std::cout << " ("<<current_point[0]<<", "<<current_point[1]<<", "<<current_point[2]<<") falls into pixel "<<x<<","<<y<<".\n";
    
    // Do some minor interpolation by checking the three closest neighbors to the point, that are not filled yet.
    int floor_x = pcl_lrint (floor (x_real)), floor_y = pcl_lrint (floor (y_real)),
        ceil_x  = pcl_lrint (ceil (x_real)),  ceil_y  = pcl_lrint (ceil (y_real));
    
    int neighbor_x[4], neighbor_y[4];
    neighbor_x[0]=floor_x; neighbor_y[0]=floor_y;
    neighbor_x[1]=floor_x; neighbor_y[1]=ceil_y;
    neighbor_x[2]=ceil_x;  neighbor_y[2]=floor_y;
    neighbor_x[3]=ceil_x;  neighbor_y[3]=ceil_y;
    //std::cout << x_real<<","<<y_real<<": ";
    
    for (int i=0; i<4; ++i)
    {
      int n_x=neighbor_x[i], n_y=neighbor_y[i];
      //std::cout << n_x<<","<<n_y<<" ";
      if (n_x==x && n_y==y)
        continue;
      if (isInImage (n_x, n_y))
      {
        int neighbor_array_pos = n_y*width + n_x;
        if (counters[neighbor_array_pos]==0)
        {
          double& neighbor_range = points[neighbor_array_pos].range;
          neighbor_range = (pcl_isin (neighbor_range) ? range_of_current_point : (std::min) (neighbor_range, range_of_current_point));
          top= (std::min) (top, n_y); right= (std::max) (right, n_x); bottom= (std::max) (bottom, n_y); left= (std::min) (left, n_x);
        }
      }
    }
    //std::cout <<std::endl;
    
    // The point itself
    int arrayPos = y*width + x;
    double& range_at_image_point = points[arrayPos].range;
    int& counter = counters[arrayPos];
    bool addCurrentPoint=false, replace_with_current_point=false;
    
    if (counter==0)
    {
      replace_with_current_point = true;
    }
    else
    {
      if (range_of_current_point < range_at_image_point-noise_level)
      {
        replace_with_current_point = true;
      }
      else if (fabs (range_of_current_point-range_at_image_point)<=noise_level)
      {
        addCurrentPoint = true;
      }
    }
    
    if (replace_with_current_point)
    {
      counter = 1;
      range_at_image_point = range_of_current_point;
      top= (std::min) (top, y); right= (std::max) (right, x); bottom= (std::max) (bottom, y); left= (std::min) (left, x);
      //std::cout << "Adding point "<<x<<","<<y<<"\n";
    }
    else if (addCurrentPoint)
    {
      ++counter;
      range_at_image_point += (range_of_current_point-range_at_image_point)/counter;
    }
  }
  
  delete[] counters;
}

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::getImagePoint (double x, double y, double z, double& image_x, double& image_y, double& range) const 
{
  Eigen::Vector3d point (x, y, z);
  getImagePoint (point, image_x, image_y, range);
}

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::getImagePoint (double x, double y, double z, double& image_x, double& image_y) const 
{
  double range;
  getImagePoint (x, y, z, image_x, image_y, range);
}

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::getImagePoint (double x, double y, double z, int& image_x, int& image_y) const 
{
  double image_x_float, image_y_float;
  getImagePoint (x, y, z, image_x_float, image_y_float);
  real2DToInt2D (image_x_float, image_y_float, image_x, image_y);
}

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::getImagePoint (const Eigen::Vector3d& point, double& image_x, double& image_y, double& range) const 
{
  Eigen::Vector3d transformedPoint = to_range_image_system_ * point;
  range = transformedPoint.norm ();
  double angle_x = atan2LookUp (transformedPoint[0], transformedPoint[2]),
        angle_y = asinLookUp (transformedPoint[1]/range);
  getImagePointFromAngles (angle_x, angle_y, image_x, image_y);
  //std::cout << " ("<<point[0]<<","<<point[1]<<","<<point[2]<<")"
            //<< " => ("<<transformedPoint[0]<<","<<transformedPoint[1]<<","<<transformedPoint[2]<<")"
            //<< " => "<<angle_x<<","<<angle_y<<" => "<<image_x<<","<<image_y<<"\n";
}

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::getImagePoint (const Eigen::Vector3d& point, int& image_x, int& image_y, double& range) const {
  double image_x_float, image_y_float;
  getImagePoint (point, image_x_float, image_y_float, range);
  real2DToInt2D (image_x_float, image_y_float, image_x, image_y);
}

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::getImagePoint (const Eigen::Vector3d& point, double& image_x, double& image_y) const
{
  double range;
  getImagePoint (point, image_x, image_y, range);
}

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::getImagePoint (const Eigen::Vector3d& point, int& image_x, int& image_y) const
{
  double image_x_float, image_y_float;
  getImagePoint (point, image_x_float, image_y_float);
  real2DToInt2D (image_x_float, image_y_float, image_x, image_y);
}

/////////////////////////////////////////////////////////////////////////
double 
RangeImage::checkPoint (const Eigen::Vector3d& point, PointWithRange& point_in_image) const
{
  int image_x, image_y;
  double range;
  getImagePoint (point, image_x, image_y, range);
  if (!isInImage (image_x, image_y))
    point_in_image = unobserved_point;
  else
    point_in_image = getPoint (image_x, image_y);
  return range;
}

/////////////////////////////////////////////////////////////////////////
double 
RangeImage::getRangeDifference (const Eigen::Vector3d& point) const
{
  int image_x, image_y;
  double range;
  getImagePoint (point, image_x, image_y, range);
  if (!isInImage (image_x, image_y))
    return -std::numeric_limits<double>::infinity ();
  double image_point_range = getPoint (image_x, image_y).range;
  if (pcl_isin (image_point_range))
  {
    if (image_point_range > 0.0)
      return std::numeric_limits<double>::infinity ();
    else
      return -std::numeric_limits<double>::infinity ();
  }
  return image_point_range - range;
}

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::getImagePointFromAngles (double angle_x, double angle_y, double& image_x, double& image_y) const
{
  image_x = (angle_x*cosLookUp (angle_y) + static_cast<double> (M_PI))*angular_resolution_x_reciprocal_ - static_cast<double> (image_offset_x_);
  image_y = (angle_y + 0.5*static_cast<double> (M_PI))*angular_resolution_y_reciprocal_ - static_cast<double> (image_offset_y_);
}

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::real2DToInt2D (double x, double y, int& xInt, int& yInt) const
{
  xInt = static_cast<int> (pcl_lrintf (x)); 
  yInt = static_cast<int> (pcl_lrintf (y));
}

/////////////////////////////////////////////////////////////////////////
bool 
RangeImage::isInImage (int x, int y) const
{
  return (x >= 0 && x < static_cast<int> (width) && y >= 0 && y < static_cast<int> (height));
}

/////////////////////////////////////////////////////////////////////////
bool 
RangeImage::isValid (int x, int y) const
{
  return isInImage (x,y) && pcl_isfinite (getPoint (x,y).range);
}

/////////////////////////////////////////////////////////////////////////
bool 
RangeImage::isValid (int index) const
{
  return pcl_isfinite (getPoint (index).range);
}

/////////////////////////////////////////////////////////////////////////
bool 
RangeImage::isObserved (int x, int y) const
{
  if (!isInImage (x,y) || (pcl_isin (getPoint (x,y).range)&&getPoint (x,y).range<0.0))
    return false;
  return true;
}

/////////////////////////////////////////////////////////////////////////
bool 
RangeImage::isMaxRange (int x, int y) const
{
  double range = getPoint (x,y).range;
  return pcl_isin (range) && range>0.0;
}

/////////////////////////////////////////////////////////////////////////
const PointWithRange& 
RangeImage::getPoint (int image_x, int image_y) const
{
  if (!isInImage (image_x, image_y))
    return unobserved_point;
  return points[image_y*width + image_x];
}

/////////////////////////////////////////////////////////////////////////
const PointWithRange& 
RangeImage::getPointNoCheck (int image_x, int image_y) const
{
  return points[image_y*width + image_x];
}

/////////////////////////////////////////////////////////////////////////
PointWithRange& 
RangeImage::getPointNoCheck (int image_x, int image_y)
{
  return points[image_y*width + image_x];
}

/////////////////////////////////////////////////////////////////////////
PointWithRange& 
RangeImage::getPoint (int image_x, int image_y)
{
  return points[image_y*width + image_x];
}


/////////////////////////////////////////////////////////////////////////
const PointWithRange& 
RangeImage::getPoint (int index) const
{
  return points[index];
}

/////////////////////////////////////////////////////////////////////////
const PointWithRange&
RangeImage::getPoint (double image_x, double image_y) const
{
  int x, y;
  real2DToInt2D (image_x, image_y, x, y);
  return getPoint (x, y);
}

/////////////////////////////////////////////////////////////////////////
PointWithRange& 
RangeImage::getPoint (double image_x, double image_y)
{
  int x, y;
  real2DToInt2D (image_x, image_y, x, y);
  return getPoint (x, y);
}

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::getPoint (int image_x, int image_y, Eigen::Vector3d& point) const
{
  //std::cout << getPoint (image_x, image_y)<< " - " << getPoint (image_x, image_y).getVector3dMap ()<<"\n";
  point = getPoint (image_x, image_y).getVector3dMap ();
}

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::getPoint (int index, Eigen::Vector3d& point) const
{
  point = getPoint (index).getVector3dMap ();
}

/////////////////////////////////////////////////////////////////////////
const Eigen::Map<const Eigen::Vector3d> 
RangeImage::getEigenVector3d (int x, int y) const
{
  return getPoint (x, y).getVector3dMap ();
}

/////////////////////////////////////////////////////////////////////////
const Eigen::Map<const Eigen::Vector3d> 
RangeImage::getEigenVector3d (int index) const
{
  return getPoint (index).getVector3dMap ();
}

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::calculate3DPoint (double image_x, double image_y, double range, Eigen::Vector3d& point) const 
{
  double angle_x, angle_y;
  //std::cout << image_x<<","<<image_y<<","<<range;
  getAnglesFromImagePoint (image_x, image_y, angle_x, angle_y);
  
  double cosY = cos (angle_y);
  point = Eigen::Vector3d (range * sin (angle_x) * cosY, range * sin (angle_y), range * cos (angle_x)*cosY);
  point = to_world_system_ * point;
}

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::calculate3DPoint (double image_x, double image_y, Eigen::Vector3d& point) const
{
  const PointWithRange& point_in_image = getPoint (image_x, image_y);
  calculate3DPoint (image_x, image_y, point_in_image.range, point);
}

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::calculate3DPoint (double image_x, double image_y, double range, PointWithRange& point) const {
  point.range = range;
  Eigen::Vector3d tmp_point;
  calculate3DPoint (image_x, image_y, range, tmp_point);
  point.x=tmp_point[0];  point.y=tmp_point[1];  point.z=tmp_point[2];
}

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::calculate3DPoint (double image_x, double image_y, PointWithRange& point) const
{
  const PointWithRange& point_in_image = getPoint (image_x, image_y);
  calculate3DPoint (image_x, image_y, point_in_image.range, point);
}

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::getAnglesFromImagePoint (double image_x, double image_y, double& angle_x, double& angle_y) const 
{
  angle_y = (image_y+static_cast<double> (image_offset_y_))*angular_resolution_y_ - 0.5*static_cast<double> (M_PI);
  double cos_angle_y = cos (angle_y);
  angle_x = (cos_angle_y==0.0 ? 0.0 : ( (image_x+ static_cast<double> (image_offset_x_))*angular_resolution_x_ - static_cast<double> (M_PI))/cos_angle_y);
}

/////////////////////////////////////////////////////////////////////////
double 
RangeImage::getImpactAngle (int x1, int y1, int x2, int y2) const
{
  if (!isInImage (x1, y1) || !isInImage (x2,y2))
    return -std::numeric_limits<double>::infinity ();
  return getImpactAngle (getPoint (x1,y1),getPoint (x2,y2));
}

/////////////////////////////////////////////////////////////////////////
double 
RangeImage::getImpactAngle (const PointWithRange& point1, const PointWithRange& point2) const {
  if ( (pcl_isin (point1.range)&&point1.range<0) || (pcl_isin (point2.range)&&point2.range<0))
    return -std::numeric_limits<double>::infinity ();
  
  double r1 = (std::min) (point1.range, point2.range),
        r2 = (std::max) (point1.range, point2.range);
  double impact_angle = static_cast<double> (0.5 * M_PI);
  
  if (pcl_isin (r2)) 
  {
    if (r2 > 0.0 && !pcl_isin (r1))
      impact_angle = 0.0;
  }
  else if (!pcl_isin (r1)) 
  {
    double r1Sqr = r1*r1,
          r2Sqr = r2*r2,
          dSqr  = squaredEuclideanDistance (point1, point2),
          d     = sqrt (dSqr);
    double cos_impact_angle = (r2Sqr + dSqr - r1Sqr)/ (2.0*r2*d);
    cos_impact_angle = (std::max) (0.0, (std::min) (1.0, cos_impact_angle));
    impact_angle = acos (cos_impact_angle);  // Using the cosine rule
  }
  
  if (point1.range > point2.range)
    impact_angle = -impact_angle;
  
  return impact_angle;
}

/////////////////////////////////////////////////////////////////////////
double 
RangeImage::getAcutenessValue (const PointWithRange& point1, const PointWithRange& point2) const
{
  double impact_angle = getImpactAngle (point1, point2);
  if (pcl_isin (impact_angle))
    return -std::numeric_limits<double>::infinity ();
  double ret = 1.0 - double (fabs (impact_angle)/ (0.5*M_PI));
  if (impact_angle < 0.0)
    ret = -ret;
  //if (fabs (ret)>1)
    //std::cout << PVARAC (impact_angle)<<PVARN (ret);
  return ret;
}

/////////////////////////////////////////////////////////////////////////
double 
RangeImage::getAcutenessValue (int x1, int y1, int x2, int y2) const
{
  if (!isInImage (x1, y1) || !isInImage (x2,y2))
    return -std::numeric_limits<double>::infinity ();
  return getAcutenessValue (getPoint (x1,y1), getPoint (x2,y2));
}

/////////////////////////////////////////////////////////////////////////
const Eigen::Vector3d 
RangeImage::getSensorPos () const
{
  return Eigen::Vector3d (to_world_system_ (0,3), to_world_system_ (1,3), to_world_system_ (2,3));
}

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::getSurfaceAngleChange (int x, int y, int radius, double& angle_change_x, double& angle_change_y) const
{
  angle_change_x = angle_change_y = -std::numeric_limits<double>::infinity ();
  if (!isValid (x,y))
    return;
  Eigen::Vector3d point;
  getPoint (x, y, point);
  Eigen::Affine3d transformation = getTransformationToViewerCoordinateFrame (point);
  
  if (isObserved (x-radius, y) && isObserved (x+radius, y))
  {
    Eigen::Vector3d transformed_left;
    if (isMaxRange (x-radius, y))
      transformed_left = Eigen::Vector3d (0.0, 0.0, -1.0);
    else
    {
      Eigen::Vector3d left;
      getPoint (x-radius, y, left);
      transformed_left = - (transformation * left);
      //std::cout << PVARN (transformed_left[1]);
      transformed_left[1] = 0.0;
      transformed_left.normalize ();
    }
    
    Eigen::Vector3d transformed_right;
    if (isMaxRange (x+radius, y))
      transformed_right = Eigen::Vector3d (0.0, 0.0, 1.0);
    else
    {
      Eigen::Vector3d right;
      getPoint (x+radius, y, right);
      transformed_right = transformation * right;
      //std::cout << PVARN (transformed_right[1]);
      transformed_right[1] = 0.0;
      transformed_right.normalize ();
    }
    angle_change_x = transformed_left.dot (transformed_right);
    angle_change_x = (std::max) (0.0, (std::min) (1.0, angle_change_x));
    angle_change_x = acos (angle_change_x);
  }
  
  if (isObserved (x, y-radius) && isObserved (x, y+radius))
  {
    Eigen::Vector3d transformed_top;
    if (isMaxRange (x, y-radius))
      transformed_top = Eigen::Vector3d (0.0, 0.0, -1.0);
    else
    {
      Eigen::Vector3d top;
      getPoint (x, y-radius, top);
      transformed_top = - (transformation * top);
      //std::cout << PVARN (transformed_top[0]);
      transformed_top[0] = 0.0;
      transformed_top.normalize ();
    }
    
    Eigen::Vector3d transformed_bottom;
    if (isMaxRange (x, y+radius))
      transformed_bottom = Eigen::Vector3d (0.0, 0.0, 1.0);
    else
    {
      Eigen::Vector3d bottom;
      getPoint (x, y+radius, bottom);
      transformed_bottom = transformation * bottom;
      //std::cout << PVARN (transformed_bottom[0]);
      transformed_bottom[0] = 0.0;
      transformed_bottom.normalize ();
    }
    angle_change_y = transformed_top.dot (transformed_bottom);
    angle_change_y = (std::max) (0.0, (std::min) (1.0, angle_change_y));
    angle_change_y = acos (angle_change_y);
  }
}


//inline double RangeImage::getSurfaceChange (const PointWithRange& point, const PointWithRange& neighbor1, const PointWithRange& neighbor2) const
//{
  //if (!pcl_isfinite (point.range) || (!pcl_isfinite (neighbor1.range)&&neighbor1.range<0) || (!pcl_isfinite (neighbor2.range)&&neighbor2.range<0))
    //return -std::numeric_limits<double>::infinity ();
  //if (pcl_isin (neighbor1.range))
  //{
    //if (pcl_isin (neighbor2.range))
      //return 0.0;
    //else
      //return acos ( (Eigen::Vector3d (point.x, point.y, point.z)-getSensorPos ()).normalized ().dot ( (Eigen::Vector3d (neighbor2.x, neighbor2.y, neighbor2.z)-Eigen::Vector3d (point.x, point.y, point.z)).normalized ()));
  //}
  //if (pcl_isin (neighbor2.range))
    //return acos ( (Eigen::Vector3d (point.x, point.y, point.z)-getSensorPos ()).normalized ().dot ( (Eigen::Vector3d (neighbor1.x, neighbor1.y, neighbor1.z)-Eigen::Vector3d (point.x, point.y, point.z)).normalized ()));
  
  //double d1_squared = squaredEuclideanDistance (point, neighbor1),
        //d1 = sqrt (d1_squared),
        //d2_squared = squaredEuclideanDistance (point, neighbor2),
        //d2 = sqrt (d2_squared),
        //d3_squared = squaredEuclideanDistance (neighbor1, neighbor2);
  //double cos_surface_change = (d1_squared + d2_squared - d3_squared)/ (2.0*d1*d2),
        //surface_change = acos (cos_surface_change);
  //if (pcl_isnan (surface_change))
    //surface_change = static_cast<double> (M_PI);
  ////std::cout << PVARN (point)<<PVARN (neighbor1)<<PVARN (neighbor2)<<PVARN (cos_surface_change)<<PVARN (surface_change)<<PVARN (d1)<<PVARN (d2)<<PVARN (d1_squared)<<PVARN (d2_squared)<<PVARN (d3_squared);

  //return surface_change;
//}

/////////////////////////////////////////////////////////////////////////
double 
RangeImage::getMaxAngleSize (const Eigen::Affine3d& viewer_pose, const Eigen::Vector3d& center, double radius)
{
  return 2.0 * asin (radius/ (viewer_pose.translation ()-center).norm ());
}

/////////////////////////////////////////////////////////////////////////
Eigen::Vector3d 
RangeImage::getEigenVector3d (const PointWithRange& point)
{
  return Eigen::Vector3d (point.x, point.y, point.z);
}

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::get1dPointAverage (int x, int y, int delta_x, int delta_y, int no_of_points, PointWithRange& average_point) const
{
  //std::cout << __PRETTY_FUNCTION__<<" called.\n";
  //MEASURE_FUNCTION_TIME;
  double weight_sum = 1.0;
  average_point = getPoint (x,y);
  if (pcl_isin (average_point.range))
  {
    if (average_point.range>0.0)  // The first point is max range -> return a max range point
      return;
    weight_sum = 0.0;
    average_point.x = average_point.y = average_point.z = average_point.range = 0.0;
  }
  
  int x2=x, y2=y;
  Vector4dMap average_point_eigen = average_point.getVector4dMap ();
  //std::cout << PVARN (no_of_points);
  for (int step=1; step<no_of_points; ++step)
  {
    //std::cout << PVARC (step);
    x2+=delta_x;  y2+=delta_y;
    if (!isValid (x2, y2))
      continue;
    const PointWithRange& p = getPointNoCheck (x2, y2);
    average_point_eigen+=p.getVector4dMap (); average_point.range+=p.range;
    weight_sum += 1.0;
  }
  if (weight_sum<= 0.0)
  {
    average_point = unobserved_point;
    return;
  }
  double normalization_factor = 1.0/weight_sum;
  average_point_eigen *= normalization_factor;
  average_point.range *= normalization_factor;
  //std::cout << PVARN (average_point);
}

/////////////////////////////////////////////////////////////////////////
double 
RangeImage::getEuclideanDistanceSquared (int x1, int y1, int x2, int y2) const
{
  if (!isObserved (x1,y1)||!isObserved (x2,y2))
    return -std::numeric_limits<double>::infinity ();
  const PointWithRange& point1 = getPoint (x1,y1),
                      & point2 = getPoint (x2,y2);
  if (pcl_isin (point1.range) && pcl_isin (point2.range))
    return 0.0;
  if (pcl_isin (point1.range) || pcl_isin (point2.range))
    return std::numeric_limits<double>::infinity ();
  return squaredEuclideanDistance (point1, point2);
}

/////////////////////////////////////////////////////////////////////////
double 
RangeImage::getAverageEuclideanDistance (int x, int y, int offset_x, int offset_y, int max_steps) const
{
  double average_pixel_distance = 0.0;
  double weight=0.0;
  for (int i=0; i<max_steps; ++i)
  {
    int x1=x+i*offset_x,     y1=y+i*offset_y;
    int x2=x+ (i+1)*offset_x, y2=y+ (i+1)*offset_y;
    double pixel_distance = getEuclideanDistanceSquared (x1,y1,x2,y2);
    if (!pcl_isfinite (pixel_distance))
    {
      //std::cout << x<<","<<y<<"->"<<x2<<","<<y2<<": "<<pixel_distance<<"\n";
      if (i==0)
        return pixel_distance;
      else
        break;
    }
    //std::cout << x<<","<<y<<"->"<<x2<<","<<y2<<": "<<sqrt (pixel_distance)<<"m\n";
    weight += 1.0;
    average_pixel_distance += sqrt (pixel_distance);
  }
  average_pixel_distance /= weight;
  //std::cout << x<<","<<y<<","<<offset_x<<","<<offset_y<<" => "<<average_pixel_distance<<"\n";
  return average_pixel_distance;
}

/////////////////////////////////////////////////////////////////////////
double 
RangeImage::getImpactAngleBasedOnLocalNormal (int x, int y, int radius) const
{
  if (!isValid (x,y))
    return -std::numeric_limits<double>::infinity ();
  const PointWithRange& point = getPoint (x, y);
  int no_of_nearest_neighbors = static_cast<int> (pow (static_cast<double> ( (radius + 1.0)), 2.0));
  Eigen::Vector3d normal;
  if (!getNormalForClosestNeighbors (x, y, radius, point, no_of_nearest_neighbors, normal, 1))
    return -std::numeric_limits<double>::infinity ();
  return deg2rad (90.0) - acos (normal.dot ( (getSensorPos ()-getEigenVector3d (point)).normalized ()));
}


/////////////////////////////////////////////////////////////////////////
bool 
RangeImage::getNormal (int x, int y, int radius, Eigen::Vector3d& normal, int step_size) const
{
  VectorAverage3f vector_average;
  for (int y2=y-radius; y2<=y+radius; y2+=step_size)
  {
    for (int x2=x-radius; x2<=x+radius; x2+=step_size)
    {
      if (!isInImage (x2, y2))
        continue;
      const PointWithRange& point = getPoint (x2, y2);
      if (!pcl_isfinite (point.range))
        continue;
      vector_average.add (Eigen::Vector3d (point.x, point.y, point.z));
    }
  }
  if (vector_average.getNoOfSamples () < 3)
    return false;
  Eigen::Vector3d eigen_values, eigen_vector2, eigen_vector3;
  vector_average.doPCA (eigen_values, normal, eigen_vector2, eigen_vector3);
  if (normal.dot ( (getSensorPos ()-vector_average.getMean ()).normalized ()) < 0.0)
    normal *= -1.0;
  return true;
}

/////////////////////////////////////////////////////////////////////////
double 
RangeImage::getNormalBasedAcutenessValue (int x, int y, int radius) const
{
  double impact_angle = getImpactAngleBasedOnLocalNormal (x, y, radius);
  if (pcl_isin (impact_angle))
    return -std::numeric_limits<double>::infinity ();
  double ret = 1.0 - static_cast<double> ( (impact_angle / (0.5 * M_PI)));
  //std::cout << PVARAC (impact_angle)<<PVARN (ret);
  return ret;
}

/////////////////////////////////////////////////////////////////////////
bool 
RangeImage::getNormalForClosestNeighbors (int x, int y, int radius, const PointWithRange& point,
                                              int no_of_nearest_neighbors, Eigen::Vector3d& normal, int step_size) const
{
  return getNormalForClosestNeighbors (x, y, radius, Eigen::Vector3d (point.x, point.y, point.z), no_of_nearest_neighbors, normal, NULL, step_size);
}

/////////////////////////////////////////////////////////////////////////
bool 
RangeImage::getNormalForClosestNeighbors (int x, int y, Eigen::Vector3d& normal, int radius) const
{
  if (!isValid (x,y))
    return false;
  int no_of_nearest_neighbors = static_cast<int> (pow (static_cast<double> (radius + 1.0), 2.0));
  return getNormalForClosestNeighbors (x, y, radius, getPoint (x,y).getVector3dMap (), no_of_nearest_neighbors, normal);
}

namespace 
{  // Anonymous namespace, so that this is only accessible in this file
  struct NeighborWithDistance 
  {  // local struct to help us with sorting
    double distance;
    const PointWithRange* neighbor;
    bool operator < (const NeighborWithDistance& other) const { return distance<other.distance;}
  };
}

/////////////////////////////////////////////////////////////////////////
bool 
RangeImage::getSurfaceInformation (int x, int y, int radius, const Eigen::Vector3d& point, int no_of_closest_neighbors, int step_size,
                                   double& max_closest_neighbor_distance_squared,
                                   Eigen::Vector3d& normal, Eigen::Vector3d& mean, Eigen::Vector3d& eigen_values,
                                   Eigen::Vector3d* normal_all_neighbors, Eigen::Vector3d* mean_all_neighbors,
                                   Eigen::Vector3d* eigen_values_all_neighbors) const
{
  max_closest_neighbor_distance_squared=0.0;
  normal.setZero (); mean.setZero (); eigen_values.setZero ();
  if (normal_all_neighbors!=NULL)
    normal_all_neighbors->setZero ();
  if (mean_all_neighbors!=NULL)
    mean_all_neighbors->setZero ();
  if (eigen_values_all_neighbors!=NULL)
    eigen_values_all_neighbors->setZero ();
  
  int blocksize = static_cast<int> (pow (static_cast<double> ( (2.0 * radius + 1.0)), 2.0));
  
  PointWithRange given_point;
  given_point.x=point[0];  given_point.y=point[1];  given_point.z=point[2];
  
  std::vector<NeighborWithDistance> ordered_neighbors (blocksize);
  int neighbor_counter = 0;
  for (int y2=y-radius; y2<=y+radius; y2+=step_size)
  {
    for (int x2=x-radius; x2<=x+radius; x2+=step_size)
    {
      if (!isValid (x2, y2))
        continue;
      NeighborWithDistance& neighbor_with_distance = ordered_neighbors[neighbor_counter];
      neighbor_with_distance.neighbor = &getPoint (x2, y2);
      neighbor_with_distance.distance = squaredEuclideanDistance (given_point, *neighbor_with_distance.neighbor);
      ++neighbor_counter;
    }
  }
  no_of_closest_neighbors = (std::min) (neighbor_counter, no_of_closest_neighbors);

  std::sort (ordered_neighbors.begin (), ordered_neighbors.begin () + neighbor_counter);  // Normal sort seems to be the fastest method (faster than partial_sort)
  //std::stable_sort (ordered_neighbors, ordered_neighbors+neighbor_counter);
  //std::partial_sort (ordered_neighbors, ordered_neighbors+no_of_closest_neighbors, ordered_neighbors+neighbor_counter);
  
  max_closest_neighbor_distance_squared = ordered_neighbors[no_of_closest_neighbors-1].distance;
  //double max_distance_squared = max_closest_neighbor_distance_squared;
  double max_distance_squared = max_closest_neighbor_distance_squared*4.0;  // Double the allowed distance value
  //max_closest_neighbor_distance_squared = max_distance_squared;
  
  VectorAverage3f vector_average;
  //for (int neighbor_idx=0; neighbor_idx<no_of_closest_neighbors; ++neighbor_idx)
  int neighbor_idx;
  for (neighbor_idx=0; neighbor_idx<neighbor_counter; ++neighbor_idx)
  {
    if (ordered_neighbors[neighbor_idx].distance > max_distance_squared)
      break;
    //std::cout << ordered_neighbors[neighbor_idx].distance<<"\n";
    vector_average.add (ordered_neighbors[neighbor_idx].neighbor->getVector3dMap ());
  }
  
  if (vector_average.getNoOfSamples () < 3)
    return false;
  //std::cout << PVARN (vector_average.getNoOfSamples ());
  Eigen::Vector3d eigen_vector2, eigen_vector3;
  vector_average.doPCA (eigen_values, normal, eigen_vector2, eigen_vector3);
  Eigen::Vector3d viewing_direction = (getSensorPos ()-point).normalized ();
  if (normal.dot (viewing_direction) < 0.0)
    normal *= -1.0;
  mean = vector_average.getMean ();
  
  if (normal_all_neighbors==NULL)
    return true;
  
  // Add remaining neighbors
  for (int neighbor_idx2=neighbor_idx; neighbor_idx2<neighbor_counter; ++neighbor_idx2)
    vector_average.add (ordered_neighbors[neighbor_idx2].neighbor->getVector3dMap ());
  
  vector_average.doPCA (*eigen_values_all_neighbors, *normal_all_neighbors, eigen_vector2, eigen_vector3);
  //std::cout << PVARN (vector_average.getNoOfSamples ())<<".\n";
  if (normal_all_neighbors->dot (viewing_direction) < 0.0)
    *normal_all_neighbors *= -1.0;
  *mean_all_neighbors = vector_average.getMean ();
  
  //std::cout << viewing_direction[0]<<","<<viewing_direction[1]<<","<<viewing_direction[2]<<"\n";
  
  return true;
}

/////////////////////////////////////////////////////////////////////////
double 
RangeImage::getSquaredDistanceOfNthNeighbor (int x, int y, int radius, int n, int step_size) const
{
  const PointWithRange& point = getPoint (x, y);
  if (!pcl_isfinite (point.range))
    return -std::numeric_limits<double>::infinity ();
  
  int blocksize = static_cast<int> (pow (static_cast<double> (2.0 * radius + 1.0), 2.0));
  std::vector<double> neighbor_distances (blocksize);
  int neighbor_counter = 0;
  for (int y2=y-radius; y2<=y+radius; y2+=step_size)
  {
    for (int x2=x-radius; x2<=x+radius; x2+=step_size)
    {
      if (!isValid (x2, y2) || (x2==x&&y2==y))
        continue;
      const PointWithRange& neighbor = getPointNoCheck (x2,y2);
      double& neighbor_distance = neighbor_distances[neighbor_counter++];
      neighbor_distance = squaredEuclideanDistance (point, neighbor);
    }
  }
  std::sort (neighbor_distances.begin (), neighbor_distances.begin () + neighbor_counter);  // Normal sort seems to be
                                                                      // the fastest method (faster than partial_sort)
  n = (std::min) (neighbor_counter, n);
  return neighbor_distances[n-1];
}


/////////////////////////////////////////////////////////////////////////
bool 
RangeImage::getNormalForClosestNeighbors (int x, int y, int radius, const Eigen::Vector3d& point, int no_of_nearest_neighbors,
                                                     Eigen::Vector3d& normal, Eigen::Vector3d* point_on_plane, int step_size) const
{
  Eigen::Vector3d mean, eigen_values;
  double used_squared_max_distance;
  bool ret = getSurfaceInformation (x, y, radius, point, no_of_nearest_neighbors, step_size, used_squared_max_distance,
                                   normal, mean, eigen_values);
  
  if (ret)
  {
    if (point_on_plane != NULL)
      *point_on_plane = (normal.dot (mean) - normal.dot (point))*normal + point;
  }
  return ret;
}


/////////////////////////////////////////////////////////////////////////
double 
RangeImage::getCurvature (int x, int y, int radius, int step_size) const
{
  VectorAverage3f vector_average;
  for (int y2=y-radius; y2<=y+radius; y2+=step_size)
  {
    for (int x2=x-radius; x2<=x+radius; x2+=step_size)
    {
      if (!isInImage (x2, y2))
        continue;
      const PointWithRange& point = getPoint (x2, y2);
      if (!pcl_isfinite (point.range))
        continue;
      vector_average.add (Eigen::Vector3d (point.x, point.y, point.z));
    }
  }
  if (vector_average.getNoOfSamples () < 3)
    return false;
  Eigen::Vector3d eigen_values;
  vector_average.doPCA (eigen_values);
  return eigen_values[0]/eigen_values.sum ();
}


/////////////////////////////////////////////////////////////////////////
template <typename PointCloudTypeWithViewpoints> Eigen::Vector3d 
RangeImage::getAverageViewPoint (const PointCloudTypeWithViewpoints& point_cloud)
{
  Eigen::Vector3d average_viewpoint (0,0,0);
  int point_counter = 0;
  for (unsigned int point_idx=0; point_idx<point_cloud.points.size (); ++point_idx)
  {
    const typename PointCloudTypeWithViewpoints::PointType& point = point_cloud.points[point_idx];
    if (!pcl_isfinite (point.vp_x) || !pcl_isfinite (point.vp_y) || !pcl_isfinite (point.vp_z))
      continue;
    average_viewpoint[0] += point.vp_x;
    average_viewpoint[1] += point.vp_y;
    average_viewpoint[2] += point.vp_z;
    ++point_counter;
  }
  average_viewpoint /= point_counter;
  
  return average_viewpoint;
}

/////////////////////////////////////////////////////////////////////////
bool 
RangeImage::getViewingDirection (int x, int y, Eigen::Vector3d& viewing_direction) const
{
  if (!isValid (x, y))
    return false;
  viewing_direction = (getPoint (x,y).getVector3dMap ()-getSensorPos ()).normalized ();
  return true;
}

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::getViewingDirection (const Eigen::Vector3d& point, Eigen::Vector3d& viewing_direction) const
{
  viewing_direction = (point-getSensorPos ()).normalized ();
}

/////////////////////////////////////////////////////////////////////////
Eigen::Affine3d 
RangeImage::getTransformationToViewerCoordinateFrame (const Eigen::Vector3d& point) const
{
  Eigen::Affine3d transformation;
  getTransformationToViewerCoordinateFrame (point, transformation);
  return transformation;
}

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::getTransformationToViewerCoordinateFrame (const Eigen::Vector3d& point, Eigen::Affine3d& transformation) const
{
  Eigen::Vector3d viewing_direction = (point-getSensorPos ()).normalized ();
  getTransformationFromTwoUnitVectorsAndOrigin (Eigen::Vector3d (0.0, -1.0, 0.0), viewing_direction, point, transformation);
}

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::getRotationToViewerCoordinateFrame (const Eigen::Vector3d& point, Eigen::Affine3d& transformation) const
{
  Eigen::Vector3d viewing_direction = (point-getSensorPos ()).normalized ();
  getTransformationFromTwoUnitVectors (Eigen::Vector3d (0.0, -1.0, 0.0), viewing_direction, transformation);
}

/////////////////////////////////////////////////////////////////////////
inline void
RangeImage::setAngularResolution (double angular_resolution)
{
  angular_resolution_x_ = angular_resolution_y_ = angular_resolution;
  angular_resolution_x_reciprocal_ = angular_resolution_y_reciprocal_ = 1.0 / angular_resolution;
}

/////////////////////////////////////////////////////////////////////////
inline void
RangeImage::setAngularResolution (double angular_resolution_x, double angular_resolution_y)
{
  angular_resolution_x_ = angular_resolution_x;
  angular_resolution_x_reciprocal_ = 1.0 / angular_resolution_x_;
  angular_resolution_y_ = angular_resolution_y;
  angular_resolution_y_reciprocal_ = 1.0 / angular_resolution_y_;
}

/////////////////////////////////////////////////////////////////////////
inline void 
RangeImage::setTransformationToRangeImageSystem (const Eigen::Affine3d& to_range_image_system)
{
  to_range_image_system_ = to_range_image_system;
  to_world_system_ = to_range_image_system_.inverse ();
}

/////////////////////////////////////////////////////////////////////////
inline void 
RangeImage::getAngularResolution (double& angular_resolution_x, double& angular_resolution_y) const
{  
  angular_resolution_x = angular_resolution_x_;
  angular_resolution_y = angular_resolution_y_;
}

/////////////////////////////////////////////////////////////////////////
template <typename PointCloudType> void 
RangeImage::integrateFarRanges (const PointCloudType& far_ranges)
{
  double x_real, y_real, range_of_current_point;
  for (typename PointCloudType::const_iterator it  = far_ranges.points.begin (); it != far_ranges.points.end (); ++it)
  {
    //if (!isFinite (*it))  // Check for NAN etc
      //continue;
    Vector3dMapConst current_point = it->getVector3dMap ();
    
    this->getImagePoint (current_point, x_real, y_real, range_of_current_point);
    
    int floor_x = static_cast<int> (pcl_lrint (floor (x_real))), 
        floor_y = static_cast<int> (pcl_lrint (floor (y_real))),
        ceil_x  = static_cast<int> (pcl_lrint (ceil (x_real))),
        ceil_y  = static_cast<int> (pcl_lrint (ceil (y_real)));
    
    int neighbor_x[4], neighbor_y[4];
    neighbor_x[0]=floor_x; neighbor_y[0]=floor_y;
    neighbor_x[1]=floor_x; neighbor_y[1]=ceil_y;
    neighbor_x[2]=ceil_x;  neighbor_y[2]=floor_y;
    neighbor_x[3]=ceil_x;  neighbor_y[3]=ceil_y;
    
    for (int i=0; i<4; ++i)
    {
      int x=neighbor_x[i], y=neighbor_y[i];
      if (!isInImage (x, y))
        continue;
      PointWithRange& image_point = getPoint (x, y);
      if (!pcl_isfinite (image_point.range))
        image_point.range = std::numeric_limits<double>::infinity ();
    }
  }
}

}  // namespace end
#endif

