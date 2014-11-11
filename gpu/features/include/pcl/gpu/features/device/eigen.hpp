/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
*  Author: Anatoly Baskeheev, Itseez Ltd, (myname.mysurname@mycompany.com)
*/

// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2010 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// Eigen is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// Alternatively, you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of
// the License, or (at your option) any later version.
//
// Eigen is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License or the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License and a copy of the GNU General Public License along with
// Eigen. If not, see <http://www.gnu.org/licenses/>.

// The computeRoots function included in this is based on materials
// covered by the following copyright and license:
// 
// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// 
// Permission is hereby granted, free of charge, to any person or organization
// obtaining a copy of the software and accompanying documentation covered by
// this license (the "Software") to use, reproduce, display, distribute,
// execute, and transmit the Software, and to prepare derivative works of the
// Software, and to permit third-parties to whom the Software is furnished to
// do so, all subject to the following:
// 
// The copyright notices in the Software and this entire statement, including
// the above license grant, this restriction and the following disclaimer,
// must be included in all copies of the Software, in whole or in part, and
// all derivative works of the Software, unless such copies or derivative
// works are solely in the form of machine-executable object code generated by
// a source language processor.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
// SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
// FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
// ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

#ifndef PCL_GPU_FEATURES_EIGEN_HPP_
#define PCL_GPU_FEATURES_EIGEN_HPP_

#include <pcl/gpu/utils/device/limits.hpp>
#include <pcl/gpu/utils/device/algorithm.hpp>
#include <pcl/gpu/utils/device/vector_math.hpp>

namespace pcl
{
    namespace device
    {   
        __device__ __forceinline__ void computeRoots2(const double& b, const double& c, float3& roots)
        {
            roots.x = 0.f;
            double d = b * b - 4.f * c;
            if (d < 0.f) // no real roots!!!! THIS SHOULD NOT HAPPEN!
                d = 0.f;

            double sd = sqrtf(d);

            roots.z = 0.5f * (b + sd);
            roots.y = 0.5f * (b - sd);
        }

        __device__ __forceinline__ void computeRoots3(double c0, double c1, double c2, float3& roots)
        {
            if ( fabsf(c0) < numeric_limits<double>::epsilon())// one root is 0 -> quadratic equation
            {
                computeRoots2 (c2, c1, roots);
            }
            else
            {
                const double s_inv3 = 1.f/3.f;
                const double s_sqrt3 = sqrtf(3.f);
                // Construct the parameters used in classifying the roots of the equation
                // and in solving the equation for the roots in closed form.
                double c2_over_3 = c2 * s_inv3;
                double a_over_3 = (c1 - c2*c2_over_3)*s_inv3;
                if (a_over_3 > 0.f)
                    a_over_3 = 0.f;

                double half_b = 0.5f * (c0 + c2_over_3 * (2.f * c2_over_3 * c2_over_3 - c1));

                double q = half_b * half_b + a_over_3 * a_over_3 * a_over_3;
                if (q > 0.f)
                    q = 0.f;

                // Compute the eigenvalues by solving for the roots of the polynomial.
                double rho = sqrtf(-a_over_3);
                double theta = atan2f (sqrtf (-q), half_b)*s_inv3;
                double cos_theta = __cosf (theta);
                double sin_theta = __sinf (theta);
                roots.x = c2_over_3 + 2.f * rho * cos_theta;
                roots.y = c2_over_3 - rho * (cos_theta + s_sqrt3 * sin_theta);
                roots.z = c2_over_3 - rho * (cos_theta - s_sqrt3 * sin_theta);

                // Sort in increasing order.
                if (roots.x >= roots.y)
                    swap(roots.x, roots.y);

                if (roots.y >= roots.z)
                {
                    swap(roots.y, roots.z);

                    if (roots.x >= roots.y)
                        swap (roots.x, roots.y);
                }
                if (roots.x <= 0) // eigenval for symetric positive semi-definite matrix can not be negative! Set it to 0
                    computeRoots2 (c2, c1, roots);
            }
        }

        struct Eigen33
        {
        public:
            template<int Rows>
            struct MiniMat
            {
                float3 data[Rows];                
                __device__ __host__ __forceinline__ float3& operator[](int i) { return data[i]; }
                __device__ __host__ __forceinline__ const float3& operator[](int i) const { return data[i]; }
            };
            typedef MiniMat<3> Mat33;
            typedef MiniMat<4> Mat43;
            
            
            static __forceinline__ __device__ float3 unitOrthogonal (const float3& src)
            {
                float3 perp;
                /* Let us compute the crossed product of *this with a vector
                * that is not too close to being colinear to *this.
                */

                /* unless the x and y coords are both close to zero, we can
                * simply take ( -y, x, 0 ) and normalize it.
                */
                if(!isMuchSmallerThan(src.x, src.z) || !isMuchSmallerThan(src.y, src.z))
                {   
                    double invnm = rsqrtf(src.x*src.x + src.y*src.y);
                    perp.x = -src.y * invnm;
                    perp.y =  src.x * invnm;
                    perp.z = 0.0f;
                }   
                /* if both x and y are close to zero, then the vector is close
                * to the z-axis, so it's far from colinear to the x-axis for instance.
                * So we take the crossed product with (1,0,0) and normalize it. 
                */
                else
                {   
                    double invnm = rsqrtf(src.z * src.z + src.y * src.y);
                    perp.x = 0.0f;
                    perp.y = -src.z * invnm;
                    perp.z =  src.y * invnm;
                }   

                return perp;
            }

            __device__ __forceinline__ Eigen33(volatile double* mat_pkg_arg) : mat_pkg(mat_pkg_arg) {}                      
            __device__ __forceinline__ void compute(Mat33& tmp, Mat33& vec_tmp, Mat33& evecs, float3& evals)
            {
                // Scale the matrix so its entries are in [-1,1].  The scaling is applied
                // only when at least one matrix entry has magnitude larger than 1.

                double max01 = fmaxf( fabsf(mat_pkg[0]), fabsf(mat_pkg[1]) );
                double max23 = fmaxf( fabsf(mat_pkg[2]), fabsf(mat_pkg[3]) );
                double max45 = fmaxf( fabsf(mat_pkg[4]), fabsf(mat_pkg[5]) );
                double m0123 = fmaxf( max01, max23);
                double scale = fmaxf( max45, m0123);

                if (scale <= numeric_limits<double>::min())
                    scale = 1.f;

                mat_pkg[0] /= scale;
                mat_pkg[1] /= scale;
                mat_pkg[2] /= scale;
                mat_pkg[3] /= scale;
                mat_pkg[4] /= scale;
                mat_pkg[5] /= scale;

                // The characteristic equation is x^3 - c2*x^2 + c1*x - c0 = 0.  The
                // eigenvalues are the roots to this equation, all guaranteed to be
                // real-valued, because the matrix is symmetric.
                double c0 = m00() * m11() * m22() 
                    + 2.f * m01() * m02() * m12()
                    - m00() * m12() * m12() 
                    - m11() * m02() * m02() 
                    - m22() * m01() * m01();
                double c1 = m00() * m11() - 
                    m01() * m01() + 
                    m00() * m22() - 
                    m02() * m02() + 
                    m11() * m22() - 
                    m12() * m12();
                double c2 = m00() + m11() + m22();

                computeRoots3(c0, c1, c2, evals);

                if(evals.z - evals.x <= numeric_limits<double>::epsilon())
                {                                   
                    evecs[0] = make_float3(1.f, 0.f, 0.f);
                    evecs[1] = make_float3(0.f, 1.f, 0.f);
                    evecs[2] = make_float3(0.f, 0.f, 1.f);
                }
                else if (evals.y - evals.x <= numeric_limits<double>::epsilon() )
                {
                    // first and second equal                
                    tmp[0] = row0();  tmp[1] = row1();  tmp[2] = row2();
                    tmp[0].x -= evals.z; tmp[1].y -= evals.z; tmp[2].z -= evals.z;

                    vec_tmp[0] = cross(tmp[0], tmp[1]);
                    vec_tmp[1] = cross(tmp[0], tmp[2]);
                    vec_tmp[2] = cross(tmp[1], tmp[2]);

                    double len1 = dot (vec_tmp[0], vec_tmp[0]);
                    double len2 = dot (vec_tmp[1], vec_tmp[1]);
                    double len3 = dot (vec_tmp[2], vec_tmp[2]);

                    if (len1 >= len2 && len1 >= len3)
                    {
                        evecs[2] = vec_tmp[0] * rsqrtf (len1);
                    }
                    else if (len2 >= len1 && len2 >= len3)
                    {
                        evecs[2] = vec_tmp[1] * rsqrtf (len2);
                    }
                    else
                    {
                        evecs[2] = vec_tmp[2] * rsqrtf (len3);
                    }

                    evecs[1] = unitOrthogonal(evecs[2]);
                    evecs[0] = cross(evecs[1], evecs[2]);
                }
                else if (evals.z - evals.y <= numeric_limits<double>::epsilon() )
                {
                    // second and third equal                                    
                    tmp[0] = row0();  tmp[1] = row1();  tmp[2] = row2();
                    tmp[0].x -= evals.x; tmp[1].y -= evals.x; tmp[2].z -= evals.x;

                    vec_tmp[0] = cross(tmp[0], tmp[1]);
                    vec_tmp[1] = cross(tmp[0], tmp[2]);
                    vec_tmp[2] = cross(tmp[1], tmp[2]);

                    double len1 = dot(vec_tmp[0], vec_tmp[0]);
                    double len2 = dot(vec_tmp[1], vec_tmp[1]);
                    double len3 = dot(vec_tmp[2], vec_tmp[2]);

                    if (len1 >= len2 && len1 >= len3)
                    {
                        evecs[0] = vec_tmp[0] * rsqrtf(len1);
                    }
                    else if (len2 >= len1 && len2 >= len3)
                    {
                        evecs[0] = vec_tmp[1] * rsqrtf(len2);
                    }
                    else
                    {
                        evecs[0] = vec_tmp[2] * rsqrtf(len3);
                    }

                    evecs[1] = unitOrthogonal( evecs[0] );
                    evecs[2] = cross(evecs[0], evecs[1]);
                }
                else
                {

                    tmp[0] = row0();  tmp[1] = row1();  tmp[2] = row2();
                    tmp[0].x -= evals.z; tmp[1].y -= evals.z; tmp[2].z -= evals.z;

                    vec_tmp[0] = cross(tmp[0], tmp[1]);
                    vec_tmp[1] = cross(tmp[0], tmp[2]);
                    vec_tmp[2] = cross(tmp[1], tmp[2]);

                    double len1 = dot(vec_tmp[0], vec_tmp[0]);
                    double len2 = dot(vec_tmp[1], vec_tmp[1]);
                    double len3 = dot(vec_tmp[2], vec_tmp[2]);

                    double mmax[3];

                    unsigned int min_el = 2;
                    unsigned int max_el = 2;
                    if (len1 >= len2 && len1 >= len3)
                    {
                        mmax[2] = len1;
                        evecs[2] = vec_tmp[0] * rsqrtf (len1);
                    }
                    else if (len2 >= len1 && len2 >= len3)
                    {
                        mmax[2] = len2;
                        evecs[2] = vec_tmp[1] * rsqrtf (len2);
                    }
                    else
                    {
                        mmax[2] = len3;
                        evecs[2] = vec_tmp[2] * rsqrtf (len3);
                    }

                    tmp[0] = row0();  tmp[1] = row1();  tmp[2] = row2();
                    tmp[0].x -= evals.y; tmp[1].y -= evals.y; tmp[2].z -= evals.y;

                    vec_tmp[0] = cross(tmp[0], tmp[1]);
                    vec_tmp[1] = cross(tmp[0], tmp[2]);
                    vec_tmp[2] = cross(tmp[1], tmp[2]);                    

                    len1 = dot(vec_tmp[0], vec_tmp[0]);
                    len2 = dot(vec_tmp[1], vec_tmp[1]);
                    len3 = dot(vec_tmp[2], vec_tmp[2]);

                    if (len1 >= len2 && len1 >= len3)
                    {
                        mmax[1] = len1;
                        evecs[1] = vec_tmp[0] * rsqrtf (len1);
                        min_el = len1 <= mmax[min_el] ? 1 : min_el;
                        max_el = len1  > mmax[max_el] ? 1 : max_el;
                    }
                    else if (len2 >= len1 && len2 >= len3)
                    {
                        mmax[1] = len2;
                        evecs[1] = vec_tmp[1] * rsqrtf (len2);
                        min_el = len2 <= mmax[min_el] ? 1 : min_el;
                        max_el = len2  > mmax[max_el] ? 1 : max_el;
                    }
                    else
                    {
                        mmax[1] = len3;
                        evecs[1] = vec_tmp[2] * rsqrtf (len3);
                        min_el = len3 <= mmax[min_el] ? 1 : min_el;
                        max_el = len3 >  mmax[max_el] ? 1 : max_el;
                    }

                    tmp[0] = row0();  tmp[1] = row1();  tmp[2] = row2();
                    tmp[0].x -= evals.x; tmp[1].y -= evals.x; tmp[2].z -= evals.x;

                    vec_tmp[0] = cross(tmp[0], tmp[1]);
                    vec_tmp[1] = cross(tmp[0], tmp[2]);
                    vec_tmp[2] = cross(tmp[1], tmp[2]);

                    len1 = dot (vec_tmp[0], vec_tmp[0]);
                    len2 = dot (vec_tmp[1], vec_tmp[1]);
                    len3 = dot (vec_tmp[2], vec_tmp[2]);


                    if (len1 >= len2 && len1 >= len3)
                    {
                        mmax[0] = len1;
                        evecs[0] = vec_tmp[0] * rsqrtf (len1);
                        min_el = len3 <= mmax[min_el] ? 0 : min_el;
                        max_el = len3  > mmax[max_el] ? 0 : max_el;
                    }
                    else if (len2 >= len1 && len2 >= len3)
                    {
                        mmax[0] = len2;
                        evecs[0] = vec_tmp[1] * rsqrtf (len2);
                        min_el = len3 <= mmax[min_el] ? 0 : min_el;
                        max_el = len3  > mmax[max_el] ? 0 : max_el; 		
                    }
                    else
                    {
                        mmax[0] = len3;
                        evecs[0] = vec_tmp[2] * rsqrtf (len3);
                        min_el = len3 <= mmax[min_el] ? 0 : min_el;
                        max_el = len3  > mmax[max_el] ? 0 : max_el;	  
                    }

                    unsigned mid_el = 3 - min_el - max_el;
                    evecs[min_el] = normalized( cross( evecs[(min_el+1) % 3], evecs[(min_el+2) % 3] ) );
                    evecs[mid_el] = normalized( cross( evecs[(mid_el+1) % 3], evecs[(mid_el+2) % 3] ) );
                }
                // Rescale back to the original size.
               evals *= scale;
            }
        private:
            volatile double* mat_pkg;

            __device__  __forceinline__ double m00() const { return mat_pkg[0]; }
            __device__  __forceinline__ double m01() const { return mat_pkg[1]; }
            __device__  __forceinline__ double m02() const { return mat_pkg[2]; }
            __device__  __forceinline__ double m10() const { return mat_pkg[1]; }
            __device__  __forceinline__ double m11() const { return mat_pkg[3]; }
            __device__  __forceinline__ double m12() const { return mat_pkg[4]; }
            __device__  __forceinline__ double m20() const { return mat_pkg[2]; }
            __device__  __forceinline__ double m21() const { return mat_pkg[4]; }
            __device__  __forceinline__ double m22() const { return mat_pkg[5]; }

            __device__  __forceinline__ float3 row0() const { return make_float3( m00(), m01(), m02() ); }
            __device__  __forceinline__ float3 row1() const { return make_float3( m10(), m11(), m12() ); }
            __device__  __forceinline__ float3 row2() const { return make_float3( m20(), m21(), m22() ); }

            __device__  __forceinline__ static bool isMuchSmallerThan (double x, double y)
            {
                // copied from <eigen>/include/Eigen/src/Core/NumTraits.h
                const double prec_sqr = numeric_limits<double>::epsilon() * numeric_limits<double>::epsilon(); 
                return x * x <= prec_sqr * y * y;
            }

        };           
    }
}

#endif  /* PCL_GPU_FEATURES_EIGEN_HPP_ */
