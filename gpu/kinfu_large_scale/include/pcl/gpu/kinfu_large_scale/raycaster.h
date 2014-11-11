/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 */


#ifndef PCL_KINFU_TSDF_RAYCASTERLS_H_
#define PCL_KINFU_TSDF_RAYCASTERLS_H_


#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/gpu/containers/device_array.h>
#include <pcl/gpu/kinfu_large_scale/pixel_rgb.h>
#include <boost/shared_ptr.hpp>
//#include <boost/graph/buffer_concepts.hpp>
#include <Eigen/Geometry>

#include <pcl/gpu/kinfu_large_scale/tsdf_buffer.h>

namespace pcl
{
  namespace gpu
  {
    namespace kinfuLS
    {
      class TsdfVolume;

      /** \brief Class that performs raycasting for TSDF volume
        * \author Anatoly Baskeheev, Itseez Ltd, (myname.mysurname@mycompany.com)
        */
      struct PCL_EXPORTS RayCaster
      {
      public:
        typedef boost::shared_ptr<RayCaster> Ptr;
        typedef pcl::gpu::DeviceArray2D<double> MapArr;
        typedef pcl::gpu::DeviceArray2D<PixelRGB> View;
        typedef pcl::gpu::DeviceArray2D<unsigned short> Depth;     

        /** \brief Image with height */ 
        const int cols, rows;      
        
        /** \brief Constructor 
          * \param[in] rows image rows
          * \param[in] cols image cols
          * \param[in] fx focal x
          * \param[in] fy focal y
          * \param[in] cx principal point x
          * \param[in] cy principal point y
          */
        RayCaster(int rows = 480, int cols = 640, double fx = 525.f, double fy = 525.f, double cx = -1, double cy = -1);
        
        ~RayCaster();

        /** \brief Sets camera intrinsics */ 
        void
        setIntrinsics(double fx = 525.f, double fy = 525.f, double cx = -1, double cy = -1);
        
        /** \brief Runs raycasting algorithm from given camera pose. It writes results to internal fiels.
          * \param[in] volume tsdf volume container
          * \param[in] camera_pose camera pose
          * \param buffer
          */ 
        void 
        run(const TsdfVolume& volume, const Eigen::Affine3d& camera_pose, tsdf_buffer* buffer);

        /** \brief Generates scene view using data raycasted by run method. So call it before.
          * \param[out] view output array for RGB image        
          */
        void
        generateSceneView(View& view) const;

        /** \brief Generates scene view using data raycasted by run method. So call it before.
          * \param[out] view output array for RGB image
          * \param[in] light_source_pose pose of light source
          */
        void
        generateSceneView(View& view, const Eigen::Vector3d& light_source_pose) const;

        /** \brief Generates depth image using data raycasted by run method. So call it before.
          * \param[out] depth output array for depth image        
          */
        void
        generateDepthImage(Depth& depth) const;
        
        /** \brief Returns raycasterd vertex map. */ 
        MapArr
        getVertexMap() const;

        /** \brief Returns raycasterd normal map. */ 
        MapArr
        getNormalMap() const;

      private:
        /** \brief Camera intrinsics. */ 
        double fx_, fy_, cx_, cy_;
              
        /* Vertext/normal map internal representation example for rows=2 and cols=4
        *  X X X X
        *  X X X X
        *  Y Y Y Y
        *  Y Y Y Y
        *  Z Z Z Z
        *  Z Z Z Z     
        */

        /** \brief vertex map of 3D points*/
        MapArr vertex_map_;
        
        /** \brief normal map of 3D points*/
        MapArr normal_map_;

        /** \brief camera pose from which raycasting was done */
        Eigen::Affine3d camera_pose_;

        /** \brief Last passed volume size */
        Eigen::Vector3d volume_size_;

public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      };
      
      /** \brief Converts from map representation to organized not-dence point cloud. */
      template<typename PointType>
      void convertMapToOranizedCloud(const RayCaster::MapArr& map, pcl::gpu::DeviceArray2D<PointType>& cloud);
    }
  }
}

#endif /* PCL_KINFU_TSDF_RAYCASTER_H_ */
