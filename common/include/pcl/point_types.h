/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
#ifndef PCL_DATA_TYPES_H_
#define PCL_DATA_TYPES_H_

#include <pcl/pcl_macros.h>
#include <bitset>
#include <pcl/register_point_struct.h>
#include <boost/mpl/contains.hpp>
#include <boost/mpl/fold.hpp>
#include <boost/mpl/vector.hpp>

/**
  * \file pcl/point_types.h
  * Defines all the PCL implemented PointT point type structures
  * \ingroup common
  */

// We're doing a lot of black magic with Boost here, so disable warnings in Maintainer mode, as we will never
// be able to fix them anyway
#if defined _MSC_VER
  #pragma warning(disable: 4201)
#endif
//#pragma warning(push, 1)
#if defined __GNUC__
#  pragma GCC system_header
#endif

/** @{*/
namespace pcl
{
  /** \brief Members: double x, y, z
    * \ingroup common
    */
  struct PointXYZ;

  /** \brief Members: rgba
    * \ingroup common
    */
  struct RGB;

  /** \brief Members: intensity (double)
    * \ingroup common
    */
  struct Intensity;

  /** \brief Members: intensity (uint8_t)
    * \ingroup common
    */
  struct Intensity8u;

  /** \brief Members: intensity (uint32_t)
    * \ingroup common
    */
  struct Intensity32u;

  /** \brief Members: double x, y, z, intensity
    * \ingroup common
    */
  struct PointXYZI;

  /** \brief Members: double x, y, z, uin32_t label
    * \ingroup common
    */
  struct PointXYZL;

  /** \brief Members: uint32_t label
    * \ingroup common
    */
  struct Label;

  /** \brief Members: double x, y, z; uint32_t rgba
    * \ingroup common
    */
  struct PointXYZRGBA;

  /** \brief Members: double x, y, z, rgb
    * \ingroup common
    */
  struct PointXYZRGB;

  /** \brief Members: double x, y, z, rgb, uint32_t label
    * \ingroup common
    */
  struct PointXYZRGBL;

  /** \brief Members: double x, y, z, h, s, v
    * \ingroup common
    */
  struct PointXYZHSV;

  /** \brief Members: double x, y
    * \ingroup common
    */
  struct PointXY;

  /** \brief Members: double u, v
    * \ingroup common
    */
  struct PointUV;

  /** \brief Members: double x, y, z, strength
    * \ingroup common
    */
  struct InterestPoint;

  /** \brief Members: double normal[3], curvature
    * \ingroup common
    */
  struct Normal;

  /** \brief Members: double normal[3]
    * \ingroup common
    */
  struct Axis;

  /** \brief Members: double x, y, z; double normal[3], curvature
    * \ingroup common
    */
  struct PointNormal;

  /** \brief Members: double x, y, z, rgb, normal[3], curvature
    * \ingroup common
    */
  struct PointXYZRGBNormal;

  /** \brief Members: double x, y, z, intensity, normal[3], curvature
    * \ingroup common
    */
  struct PointXYZINormal;

  /** \brief Members: double x, y, z, label, normal[3], curvature
    * \ingroup common
    */
  struct PointXYZLNormal;

  /** \brief Members: double x, y, z (union with double point[4]), range
    * \ingroup common
    */
  struct PointWithRange;

  /** \brief Members: double x, y, z, vp_x, vp_y, vp_z
    * \ingroup common
    */
  struct PointWithViewpoint;

  /** \brief Members: double j1, j2, j3
    * \ingroup common
    */
  struct MomentInvariants;

  /** \brief Members: double r_min, r_max
    * \ingroup common
    */
  struct PrincipalRadiiRSD;

  /** \brief Members: uint8_t boundary_point
    * \ingroup common
    */
  struct Boundary;

  /** \brief Members: double principal_curvature[3], pc1, pc2
    * \ingroup common
    */
  struct PrincipalCurvatures;

  /** \brief Members: double descriptor[352], rf[9]
    * \ingroup common
    */
  struct SHOT352;

  /** \brief Members: double descriptor[1344], rf[9]
    * \ingroup common
    */
  struct SHOT1344;

  /** \brief Members: Axis x_axis, y_axis, z_axis
    * \ingroup common
    */
  struct ReferenceFrame;

  /** \brief Members: double descriptor[1980], rf[9]
    * \ingroup common
    */
  struct ShapeContext1980;

  /** \brief Members: double descriptor[1960], rf[9]
    * \ingroup common
    */
  struct UniqueShapeContext1960;

  /** \brief Members: double pfh[125]
    * \ingroup common
    */
  struct PFHSignature125;

  /** \brief Members: double pfhrgb[250]
    * \ingroup common
    */
  struct PFHRGBSignature250;

  /** \brief Members: double f1, f2, f3, f4, alpha_m
    * \ingroup common
    */
  struct PPFSignature;

  /** \brief Members: double f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, alpha_m
    * \ingroup common
    */
  struct CPPFSignature;

  /** \brief Members: double f1, f2, f3, f4, r_ratio, g_ratio, b_ratio, alpha_m
    * \ingroup common
    */
  struct PPFRGBSignature;

  /** \brief Members: double values[12]
    * \ingroup common
    */
  struct NormalBasedSignature12;

  /** \brief Members: double fpfh[33]
    * \ingroup common
    */
  struct FPFHSignature33;
  
  /** \brief Members: double vfh[308]
    * \ingroup common
    */
  struct VFHSignature308;
  
  /** \brief Members: double grsd[21]
    * \ingroup common
    */
  struct GRSDSignature21;
  
  /** \brief Members: double esf[640]
    * \ingroup common
    */
  struct ESFSignature640;

  /** \brief Members: double histogram[16]
    * \ingroup common
    */
  struct GFPFHSignature16;

  /** \brief Members: double scale; double orientation; uint8_t descriptor[64]
    * \ingroup common
    */
  struct BRISKSignature512;

   /** \brief Members: double x, y, z, roll, pitch, yaw; double descriptor[36]
     * \ingroup common
     */
  struct Narf36;

  /** \brief Data type to store extended information about a transition from foreground to backgroundSpecification of the fields for BorderDescription::traits.
    * \ingroup common
    */
  typedef std::bitset<32> BorderTraits;

  /** \brief Specification of the fields for BorderDescription::traits.
    * \ingroup common
    */
  enum BorderTrait
  {
    BORDER_TRAIT__OBSTACLE_BORDER, BORDER_TRAIT__SHADOW_BORDER, BORDER_TRAIT__VEIL_POINT,
    BORDER_TRAIT__SHADOW_BORDER_TOP, BORDER_TRAIT__SHADOW_BORDER_RIGHT, BORDER_TRAIT__SHADOW_BORDER_BOTTOM,
    BORDER_TRAIT__SHADOW_BORDER_LEFT, BORDER_TRAIT__OBSTACLE_BORDER_TOP, BORDER_TRAIT__OBSTACLE_BORDER_RIGHT,
    BORDER_TRAIT__OBSTACLE_BORDER_BOTTOM, BORDER_TRAIT__OBSTACLE_BORDER_LEFT, BORDER_TRAIT__VEIL_POINT_TOP,
    BORDER_TRAIT__VEIL_POINT_RIGHT, BORDER_TRAIT__VEIL_POINT_BOTTOM, BORDER_TRAIT__VEIL_POINT_LEFT
  };

  /** \brief Members: int x, y; BorderTraits traits
    * \ingroup common
    */
  struct BorderDescription;

  /** \brief Members: double gradient[3]
    * \ingroup common
    */
  struct IntensityGradient;

  /** \brief Members: double histogram[N]
    * \ingroup common
    */
  template<int N>
  struct Histogram;

  /** \brief Members: double x, y, z, scale, angle, response, octave
    * \ingroup common
    */
  struct PointWithScale;

  /** \brief Members: double x, y, z, normal[3], rgba, radius, confidence, curvature
    * \ingroup common
    */
  struct PointSurfel;
}

/** @} */

#include <pcl/impl/point_types.hpp>  // Include struct definitions

// ==============================
// =====POINT_CLOUD_REGISTER=====
// ==============================

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::_RGB,
    (uint32_t, rgba, rgba)
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::RGB, pcl::_RGB)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::_Intensity,
    (double, intensity, intensity)
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::Intensity, pcl::_Intensity)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::_Intensity8u,
    (uint8_t, intensity, intensity)
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::Intensity8u, pcl::_Intensity8u)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::_Intensity32u,
    (uint32_t, intensity, intensity)
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::Intensity32u, pcl::_Intensity32u)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::_PointXYZ,
    (double, x, x)
    (double, y, y)
    (double, z, z)
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::PointXYZ, pcl::_PointXYZ)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::_PointXYZRGBA,
    (double, x, x)
    (double, y, y)
    (double, z, z)
    (uint32_t, rgba, rgba)
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::PointXYZRGBA, pcl::_PointXYZRGBA)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::_PointXYZRGB,
    (double, x, x)
    (double, y, y)
    (double, z, z)
    (double, rgb, rgb)
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::PointXYZRGB, pcl::_PointXYZRGB)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::_PointXYZRGBL,
    (double, x, x)
    (double, y, y)
    (double, z, z)
    (uint32_t, rgba, rgba)
    (uint32_t, label, label)
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::PointXYZRGBL, pcl::_PointXYZRGBL)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::_PointXYZHSV,
    (double, x, x)
    (double, y, y)
    (double, z, z)
    (double, h, h)
    (double, s, s)
    (double, v, v)
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::PointXYZHSV, pcl::_PointXYZHSV)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointXY,
    (double, x, x)
    (double, y, y)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointUV,
    (double, u, u)
    (double, v, v)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::InterestPoint,
    (double, x, x)
    (double, y, y)
    (double, z, z)
    (double, strength, strength)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::_PointXYZI,
    (double, x, x)
    (double, y, y)
    (double, z, z)
    (double, intensity, intensity)
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::PointXYZI, pcl::_PointXYZI)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointXYZL,
    (double, x, x)
    (double, y, y)
    (double, z, z)
    (uint32_t, label, label)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::Label,
    (uint32_t, label, label)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::_Normal,
    (double, normal_x, normal_x)
    (double, normal_y, normal_y)
    (double, normal_z, normal_z)
    (double, curvature, curvature)
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::Normal, pcl::_Normal)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::_Axis,
    (double, normal_x, normal_x)
    (double, normal_y, normal_y)
    (double, normal_z, normal_z)
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::Axis, pcl::_Axis)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointNormal,
    (double, x, x)
    (double, y, y)
    (double, z, z)
    (double, normal_x, normal_x)
    (double, normal_y, normal_y)
    (double, normal_z, normal_z)
    (double, curvature, curvature)
)
POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::_PointXYZRGBNormal,
    (double, x, x)
    (double, y, y)
    (double, z, z)
    (double, rgb, rgb)
    (double, normal_x, normal_x)
    (double, normal_y, normal_y)
    (double, normal_z, normal_z)
    (double, curvature, curvature)
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::PointXYZRGBNormal, pcl::_PointXYZRGBNormal)
POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointXYZINormal,
    (double, x, x)
    (double, y, y)
    (double, z, z)
    (double, intensity, intensity)
    (double, normal_x, normal_x)
    (double, normal_y, normal_y)
    (double, normal_z, normal_z)
    (double, curvature, curvature)
)
POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointXYZLNormal,
    (double, x, x)
    (double, y, y)
    (double, z, z)
    (uint32_t, label, label)
    (double, normal_x, normal_x)
    (double, normal_y, normal_y)
    (double, normal_z, normal_z)
    (double, curvature, curvature)
)
POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointWithRange,
    (double, x, x)
    (double, y, y)
    (double, z, z)
    (double, range, range)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::_PointWithViewpoint,
    (double, x, x)
    (double, y, y)
    (double, z, z)
    (double, vp_x, vp_x)
    (double, vp_y, vp_y)
    (double, vp_z, vp_z)
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::PointWithViewpoint, pcl::_PointWithViewpoint)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::MomentInvariants,
    (double, j1, j1)
    (double, j2, j2)
    (double, j3, j3)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PrincipalRadiiRSD,
    (double, r_min, r_min)
    (double, r_max, r_max)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::Boundary,
    (uint8_t, boundary_point, boundary_point)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PrincipalCurvatures,
    (double, principal_curvature_x, principal_curvature_x)
    (double, principal_curvature_y, principal_curvature_y)
    (double, principal_curvature_z, principal_curvature_z)
    (double, pc1, pc1)
    (double, pc2, pc2)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PFHSignature125,
    (double[125], histogram, pfh)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PFHRGBSignature250,
    (double[250], histogram, pfhrgb)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PPFSignature,
    (double, f1, f1)
    (double, f2, f2)
    (double, f3, f3)
    (double, f4, f4)
    (double, alpha_m, alpha_m)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::CPPFSignature,
    (double, f1, f1)
    (double, f2, f2)
    (double, f3, f3)
    (double, f4, f4)
    (double, f5, f5)
    (double, f6, f6)
    (double, f7, f7)
    (double, f8, f8)
    (double, f9, f9)
    (double, f10, f10)
    (double, alpha_m, alpha_m)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PPFRGBSignature,
    (double, f1, f1)
    (double, f2, f2)
    (double, f3, f3)
    (double, f4, f4)
    (double, r_ratio, r_ratio)
    (double, g_ratio, g_ratio)
    (double, b_ratio, b_ratio)
    (double, alpha_m, alpha_m)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::NormalBasedSignature12,
    (double[12], values, values)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::ShapeContext1980,
    (double[1980], descriptor, shape_context)
    (double[9], rf, rf)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::UniqueShapeContext1960,
    (double[1960], descriptor, shape_context)
    (double[9], rf, rf)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::SHOT352,
    (double[352], descriptor, shot)
    (double[9], rf, rf)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::SHOT1344,
    (double[1344], descriptor, shot)
    (double[9], rf, rf)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::FPFHSignature33,
    (double[33], histogram, fpfh)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::BRISKSignature512,
    (double, scale, brisk_scale)
    (double, orientation, brisk_orientation)
    (unsigned char[64], descriptor, brisk_descriptor512)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::VFHSignature308,
    (double[308], histogram, vfh)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::GRSDSignature21,
    (double[21], histogram, grsd)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::ESFSignature640,
    (double[640], histogram, esf)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::Narf36,
    (double[36], descriptor, descriptor)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::GFPFHSignature16,
    (double[16], histogram, gfpfh)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::IntensityGradient,
    (double, gradient_x, gradient_x)
    (double, gradient_y, gradient_y)
    (double, gradient_z, gradient_z)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointWithScale,
    (double, x, x)
    (double, y, y)
    (double, z, z)
    (double, scale, scale)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointSurfel,
    (double, x, x)
    (double, y, y)
    (double, z, z)
    (double, normal_x, normal_x)
    (double, normal_y, normal_y)
    (double, normal_z, normal_z)
    (uint32_t, rgba, rgba)
    (double, radius, radius)
    (double, confidence, confidence)
    (double, curvature, curvature)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::_ReferenceFrame,
    (double[3], x_axis, x_axis)
    (double[3], y_axis, y_axis)
    (double[3], z_axis, z_axis)
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::ReferenceFrame, pcl::_ReferenceFrame)

namespace pcl 
{
  // Allow double 'rgb' data to match to the newer uint32 'rgba' tag. This is so
  // you can load old 'rgb' PCD files into e.g. a PointCloud<PointXYZRGBA>.
  template<typename PointT>
  struct FieldMatches<PointT, fields::rgba>
  {
    bool operator() (const pcl::PCLPointField& field)
    {
      if (field.name == "rgb")
      {
        return (field.datatype == pcl::PCLPointField::FLOAT32 &&
                field.count == 1);
      }
      else
      {
        return (field.name == traits::name<PointT, fields::rgba>::value &&
                field.datatype == traits::datatype<PointT, fields::rgba>::value &&
                field.count == traits::datatype<PointT, fields::rgba>::size);
      }
    }
  };
  template<typename PointT>
  struct FieldMatches<PointT, fields::rgb>
  {
    bool operator() (const pcl::PCLPointField& field)
    {
      if (field.name == "rgba")
      {
        return (field.datatype == pcl::PCLPointField::UINT32 &&
                field.count == 1);
      }
      else
      {
        return (field.name == traits::name<PointT, fields::rgb>::value &&
                field.datatype == traits::datatype<PointT, fields::rgb>::value &&
                field.count == traits::datatype<PointT, fields::rgb>::size);
      }
    }
  };

  namespace traits
  {

    /** \brief Metafunction to check if a given point type has a given field.
     *
     *  Example usage at run-time:
     *
     *  \code
     *  bool curvature_available = pcl::traits::has_field<PointT, pcl::fields::curvature>::value;
     *  \endcode
     *
     *  Example usage at compile-time:
     *
     *  \code
     *  BOOST_MPL_ASSERT_MSG ((pcl::traits::has_field<PointT, pcl::fields::label>::value),
     *                        POINT_TYPE_SHOULD_HAVE_LABEL_FIELD,
     *                        (PointT));
     *  \endcode
     */
    template <typename PointT, typename Field>
    struct has_field : boost::mpl::contains<typename pcl::traits::fieldList<PointT>::type, Field>::type
    { };

    /** Metafunction to check if a given point type has all given fields. */
    template <typename PointT, typename Field>
    struct has_all_fields : boost::mpl::fold<Field,
                                             boost::mpl::bool_<true>,
                                             boost::mpl::and_<boost::mpl::_1,
                                                              has_field<PointT, boost::mpl::_2> > >::type
    { };

    /** Metafunction to check if a given point type has any of the given fields. */
    template <typename PointT, typename Field>
    struct has_any_field : boost::mpl::fold<Field,
                                            boost::mpl::bool_<false>,
                                            boost::mpl::or_<boost::mpl::_1,
                                                            has_field<PointT, boost::mpl::_2> > >::type
    { };

    /** Metafunction to check if a given point type has x, y, and z fields. */
    template <typename PointT>
    struct has_xyz : has_all_fields<PointT, boost::mpl::vector<pcl::fields::x,
                                                               pcl::fields::y,
                                                               pcl::fields::z> >
    { };

    /** Metafunction to check if a given point type has normal_x, normal_y, and
      * normal_z fields. */
    template <typename PointT>
    struct has_normal : has_all_fields<PointT, boost::mpl::vector<pcl::fields::normal_x,
                                                                  pcl::fields::normal_y,
                                                                  pcl::fields::normal_z> >
    { };

    /** Metafunction to check if a given point type has curvature field. */
    template <typename PointT>
    struct has_curvature : has_field<PointT, pcl::fields::curvature>
    { };

    /** Metafunction to check if a given point type has intensity field. */
    template <typename PointT>
    struct has_intensity : has_field<PointT, pcl::fields::intensity>
    { };

    /** Metafunction to check if a given point type has either rgb or rgba field. */
    template <typename PointT>
    struct has_color : has_any_field<PointT, boost::mpl::vector<pcl::fields::rgb,
                                                                pcl::fields::rgba> >
    { };

    /** Metafunction to check if a given point type has label field. */
    template <typename PointT>
    struct has_label : has_field<PointT, pcl::fields::label>
    { };

  }

} // namespace pcl

#if defined _MSC_VER
  #pragma warning(default: 4201)
#endif
//#pragma warning(pop)

#endif  //#ifndef PCL_DATA_TYPES_H_
