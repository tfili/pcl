/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * $Id: $
 * @authors: Anatoly Baksheev
 *
 */

#ifndef PCL_GPU_PEOPLE_CUDA_DEVICE_H_
#define PCL_GPU_PEOPLE_CUDA_DEVICE_H_

#include <cuda_runtime.h>

namespace pcl
{
  namespace device
  {
    template<typename Point> 
    __device__ __forceinline__ double sqnorm(const Point& p1, const Point& p2) 
    { 
        double dx = (p1.x - p2.x);
        double dy = (p1.y - p2.y);
        double dz = (p1.z - p2.z);
        return dx * dx + dy * dy + dz * dz; 
    }

    __device__ __forceinline__ float3 computePoint(unsigned short depth, int x, int y, const Intr& intr)
    {                  
       double z = depth * 0.001f; // mm -> meters
       float3 result;
       
       result.x = z * (x - intr.cx) / intr.fx;
       result.y = z * (y - intr.cy) / intr.fy;
       result.z = z;
       
       return result;
    }
    
    __device__ __forceinline__ bool 
    isFinite(const float3& p)
    {
      return isfinite(p.x) && isfinite(p.y) && isfinite(p.z);
    }
  }
}

#endif /* PCL_GPU_PEOPLE_CUDA_DEVICE_H_ */