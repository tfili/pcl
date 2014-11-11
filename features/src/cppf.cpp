/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Alexandru-Eugen Ichim
 *  Copyright (c) 2012-, Open Perception, Inc.
 *  Copyright (c) 2013, Martin Szarski
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

#include <pcl/features/impl/cppf.hpp>

///////////////////////////////////////////////////////////////////////////////////////////

  inline void 
  RGBtoHSV (const Eigen::Vector4i &in, 
			Eigen::Vector4d &out)
  {
    const unsigned char max = std::max (in[0], std::max (in[1], in[2]));
    const unsigned char min = std::min (in[0], std::min (in[1], in[2]));

    out[2] = static_cast <double> (max) / 255.;

    if (max == 0) // division by zero
    {
      out[1] = 0.;
      out[0] = 0.; // h = -1.;
      return;
    }

    const double diff = static_cast <double> (max - min);
    out[1] = diff / static_cast <double> (max);

    if (min == max) // diff == 0 -> division by zero
    {
      out[0] = 0;
      return;
    }

    if      (max == in[0]) out[0] = 60. * (     static_cast <double> (in[1] - in[2]) / diff);
    else if (max == in[1]) out[0] = 60. * (2. + static_cast <double> (in[2] - in[0]) / diff);
    else                   out[0] = 60. * (4. + static_cast <double> (in[0] - in[1]) / diff); // max == b

    if (out[0] < 0.) out[0] += 360.;
  }
  
bool
pcl::computeCPPFPairFeature (const Eigen::Vector4d &p1, const Eigen::Vector4d &n1, const Eigen::Vector4i &c1,
                            const Eigen::Vector4d &p2, const Eigen::Vector4d &n2, const Eigen::Vector4i &c2,
                            double &f1, double &f2, double &f3, double &f4, double &f5, double &f6, double &f7, double &f8, double &f9, double &f10)
{
  Eigen::Vector4d delta = p2 - p1;
  delta[3] = 0.0;
  // f4 = ||delta||
  f4 = delta.norm ();

  delta /= f4;

  // f1 = n1 dot delta
  f1 = n1[0] * delta[0] + n1[1] * delta[1] + n1[2] * delta[2];
  // f2 = n2 dot delta
  f2 = n2[0] * delta[0] + n2[1] * delta[1] + n2[2] * delta[2];
  // f3 = n1 dot n2
  f3 = n1[0] * n2[0] + n1[1] * n2[1] + n1[2] * n2[2];

  // f5-f7 is hsv component of p1
  // f8-f10 is hsv component of p2
  Eigen::Vector4d hsv1;
  Eigen::Vector4d hsv2;
  
  RGBtoHSV (c1,hsv1);
  RGBtoHSV (c2,hsv2);
  
  f5 = hsv1[0] / 360.0; //normalise to [0-1]
  f6 = hsv1[1];
  f7 = hsv1[2];
  
  f8  = hsv2[0] / 360.0; //normalise to [0-1]
  f9  = hsv2[1];
  f10 = hsv2[2];
  
  return (true);
}

#ifndef PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>
// Instantiations of specific point types
#ifdef PCL_ONLY_CORE_POINT_TYPES
  PCL_INSTANTIATE_PRODUCT(CPPFEstimation, ((pcl::PointXYZRGBA) (pcl::PointXYZRGBNormal))
                        ((pcl::Normal) (pcl::PointNormal)  (pcl::PointXYZRGBNormal))
                        ((pcl::CPPFSignature)))
#else
  PCL_INSTANTIATE_PRODUCT(CPPFEstimation, ((pcl::PointXYZRGBA) (pcl::PointXYZRGBNormal))
                        ((pcl::Normal) (pcl::PointNormal)  (pcl::PointXYZRGBNormal))
                        ((pcl::CPPFSignature)))
#endif
#endif    // PCL_NO_PRECOMPILE

