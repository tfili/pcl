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
 */

/** \author Bastian Steder */

#include <iostream>
using std::cout;
using std::cerr;

#include <pcl/range_image/range_image_planar.h>

namespace pcl 
{
  /////////////////////////////////////////////////////////////////////////
  RangeImagePlanar::RangeImagePlanar () : RangeImage (), focal_length_x_ (0.0f), focal_length_y_ (0.0f),
                                          focal_length_x_reciprocal_ (0.0f), focal_length_y_reciprocal_ (0.0f),
                                          center_x_ (0.0f), center_y_ (0.0f)
  {
  }

  /////////////////////////////////////////////////////////////////////////
  RangeImagePlanar::~RangeImagePlanar ()
  {
  }

  /////////////////////////////////////////////////////////////////////////
  void
  RangeImagePlanar::setDisparityImage (const double* disparity_image, int di_width, int di_height,
                                       double focal_length, double base_line, double desired_angular_resolution)
  {
    //MEASURE_FUNCTION_TIME;
    reset ();
    
    double original_angular_resolution = atanf (0.5f * static_cast<double> (di_width) / static_cast<double> (focal_length)) / (0.5f * static_cast<double> (di_width));
    int skip = 1;
    if (desired_angular_resolution >= 2.0f*original_angular_resolution)
      skip = static_cast<int> (pcl_lrint (floor (desired_angular_resolution / original_angular_resolution)));

    setAngularResolution (original_angular_resolution * static_cast<double> (skip));
    width  = di_width / skip;
    height = di_height / skip;
    focal_length_x_ = focal_length_y_ = focal_length / static_cast<double> (skip);
    focal_length_x_reciprocal_ = focal_length_y_reciprocal_ = 1.0f / focal_length_x_;
    center_x_ = static_cast<double> (di_width)  / static_cast<double> (2 * skip);
    center_y_ = static_cast<double> (di_height) / static_cast<double> (2 * skip);
    points.resize (width*height);
    
    //cout << PVARN (*this);
    
    double normalization_factor = static_cast<double> (skip) * focal_length_x_ * base_line;
    for (int y=0; y < static_cast<int> (height); ++y)
    {
      for (int x=0; x < static_cast<int> (width); ++x)
      {
        PointWithRange& point = getPointNoCheck (x,y);
        double disparity = disparity_image[ (y*skip)*di_width + x*skip];
        if (disparity <= 0.0f)
        {
          //std::cout << disparity << ", "<<std::flush;
          point = unobserved_point;
          continue;
        }
        point.z = normalization_factor / disparity;
        point.x = (static_cast<double> (x) - center_x_) * point.z * focal_length_x_reciprocal_;
        point.y = (static_cast<double> (y) - center_y_) * point.z * focal_length_y_reciprocal_;
        point.range = point.getVector3dMap ().norm ();
        //cout << point<<std::flush;
        
        //// Just a test:
        //PointWithRange test_point1;
        //calculate3DPoint (double (x), double (y), point.range, test_point1);
        //if ( (point.getVector3dMap ()-test_point1.getVector3dMap ()).norm () > 1e-5)
          //cerr << "Something's wrong here...\n";
        
        //double image_x, image_y;
        //getImagePoint (point.getVector3dMap (), image_x, image_y);
        //if (fabsf (image_x-x)+fabsf (image_y-y)>1e-4)
          //cerr << PVARC (x)<<PVARC (y)<<PVARC (image_x)<<PVARN (image_y);
      }
    }
  }
  
  /////////////////////////////////////////////////////////////////////////
  void
  RangeImagePlanar::setDepthImage (const double* depth_image, int di_width, int di_height,
                                   double di_center_x, double di_center_y,
                                   double di_focal_length_x, double di_focal_length_y,
                                   double desired_angular_resolution)
  {
    //MEASURE_FUNCTION_TIME;
    reset ();
    
    double original_angular_resolution = asinf (0.5f*static_cast<double> (di_width)/static_cast<double> (di_focal_length_x)) / (0.5f*static_cast<double> (di_width));
    int skip = 1;
    if (desired_angular_resolution >= 2.0f*original_angular_resolution)
      skip = static_cast<int> (pcl_lrint (floor (desired_angular_resolution / original_angular_resolution)));

    setAngularResolution (original_angular_resolution * static_cast<double> (skip));
    width  = di_width / skip;
    height = di_height / skip;
    focal_length_x_ = di_focal_length_x / static_cast<double> (skip);
    focal_length_x_reciprocal_ = 1.0f / focal_length_x_;
    focal_length_y_ = di_focal_length_y / static_cast<double> (skip);
    focal_length_y_reciprocal_ = 1.0f / focal_length_y_;
    center_x_ = static_cast<double> (di_center_x) / static_cast<double> (skip);
    center_y_ = static_cast<double> (di_center_y) / static_cast<double> (skip);
    points.resize (width * height);
    
    for (int y=0; y < static_cast<int> (height); ++y)
    {
      for (int x=0; x < static_cast<int> (width); ++x)
      {
        PointWithRange& point = getPointNoCheck (x, y);
        double depth = depth_image[ (y*skip)*di_width + x*skip];
        if (depth <= 0.0f || !pcl_isfinite (depth))
        {
          point = unobserved_point;
          continue;
        }
        point.z = depth;
        point.x = (static_cast<double> (x) - center_x_) * point.z * focal_length_x_reciprocal_;
        point.y = (static_cast<double> (y) - center_y_) * point.z * focal_length_y_reciprocal_;
        point.range = point.getVector3dMap ().norm ();
      }
    }
  }


  /////////////////////////////////////////////////////////////////////////
  void
  RangeImagePlanar::setDepthImage (const unsigned short* depth_image, int di_width, int di_height,
                                   double di_center_x, double di_center_y,
                                   double di_focal_length_x, double di_focal_length_y,
                                   double desired_angular_resolution)
  {
    //MEASURE_FUNCTION_TIME;
    reset ();
    
    double original_angular_resolution = asinf (0.5f*static_cast<double> (di_width)/static_cast<double> (di_focal_length_x)) / (0.5f*static_cast<double> (di_width));
    int skip = 1;
    if (desired_angular_resolution >= 2.0f*original_angular_resolution)
      skip = static_cast<int> (pcl_lrint (floor (desired_angular_resolution/original_angular_resolution)));

    setAngularResolution (original_angular_resolution * static_cast<double> (skip));
    width  = di_width / skip;
    height = di_height / skip;
    focal_length_x_ = di_focal_length_x / static_cast<double> (skip);
    focal_length_x_reciprocal_ = 1.0f / focal_length_x_;
    focal_length_y_ = di_focal_length_y / static_cast<double> (skip);
    focal_length_y_reciprocal_ = 1.0f / focal_length_y_;
    center_x_ = static_cast<double> (di_center_x) / static_cast<double> (skip);
    center_y_ = static_cast<double> (di_center_y) / static_cast<double> (skip);
    points.resize (width * height);
    
    for (int y = 0; y < static_cast<int> (height); ++y)
    {
      for (int x = 0; x < static_cast<int> (width); ++x)
      {
        PointWithRange& point = getPointNoCheck (x, y);
        double depth = depth_image[ (y*skip)*di_width + x*skip] * 0.001f;
        if (depth <= 0.0f || !pcl_isfinite (depth))
        {
          point = unobserved_point;
          continue;
        }
        point.z = depth;
        point.x = (static_cast<double> (x) - center_x_) * point.z * focal_length_x_reciprocal_;
        point.y = (static_cast<double> (y) - center_y_) * point.z * focal_length_y_reciprocal_;
        point.range = point.getVector3dMap ().norm ();
      }
    }
  }
  
  /////////////////////////////////////////////////////////////////////////
  void 
  RangeImagePlanar::getHalfImage (RangeImage& half_image) const
  {
    //std::cout << __PRETTY_FUNCTION__ << " called.\n";
    if (typeid (*this) != typeid (half_image))
    {
      std::cerr << __PRETTY_FUNCTION__<<": Given range image is not a RangeImagePlanar!\n";
      return;
    }
    RangeImagePlanar& ret = * (static_cast<RangeImagePlanar*> (&half_image));
    
    ret.focal_length_x_ = focal_length_x_/2;
    ret.focal_length_x_reciprocal_ = 1.0f/ret.focal_length_x_;
    ret.focal_length_y_ = focal_length_y_/2;
    ret.focal_length_y_reciprocal_ = 1.0f/ret.focal_length_y_;
    ret.center_x_ = center_x_/2;
    ret.center_y_ = center_y_/2;
    BaseClass::getHalfImage (ret);
  }

  /////////////////////////////////////////////////////////////////////////
  void 
  RangeImagePlanar::getSubImage (int sub_image_image_offset_x, int sub_image_image_offset_y, int sub_image_width,
                                 int sub_image_height, int combine_pixels, RangeImage& sub_image) const
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Warning, not tested properly!\n";
    
    if (typeid (*this) != typeid (sub_image))
    {
      std::cerr << __PRETTY_FUNCTION__<<": Given range image is not a RangeImagePlanar!\n";
      return;
    }
    RangeImagePlanar& ret = * (static_cast<RangeImagePlanar*> (&sub_image));
    
    ret.focal_length_x_ = focal_length_x_ / static_cast<double> (combine_pixels);
    ret.focal_length_x_reciprocal_ = 1.0f / ret.focal_length_x_;
    ret.focal_length_y_ = focal_length_x_ / static_cast<double> (combine_pixels);
    ret.focal_length_y_reciprocal_ = 1.0f/ret.focal_length_y_;
    ret.center_x_ = center_x_/2 - static_cast<double> (sub_image_image_offset_x);
    ret.center_y_ = center_y_/2 - static_cast<double> (sub_image_image_offset_y);
    BaseClass::getSubImage (sub_image_image_offset_x, sub_image_image_offset_y, sub_image_width,
                           sub_image_height, combine_pixels, ret);
    ret.image_offset_x_ = ret.image_offset_y_ = 0;
  }

  /////////////////////////////////////////////////////////////////////////
  void
  RangeImagePlanar::copyTo (RangeImage& other) const
  {
    bool ERROR_GIVEN_RANGE_IMAGE_IS_NOT_A_RangeImagePlanar = typeid (*this) == typeid (other);
    if (!ERROR_GIVEN_RANGE_IMAGE_IS_NOT_A_RangeImagePlanar) {
      std::cerr << PVARC(typeid (*this).name())<<PVARN(typeid (other).name());
    }
    assert (ERROR_GIVEN_RANGE_IMAGE_IS_NOT_A_RangeImagePlanar);
    *static_cast<RangeImagePlanar*> (&other) = *this;
  }

}  // namespace end

