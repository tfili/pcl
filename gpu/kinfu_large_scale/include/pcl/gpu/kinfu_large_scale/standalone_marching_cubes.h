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
 
 #ifndef PCL_STANDALONE_MARCHING_CUBES_H_
 #define PCL_STANDALONE_MARCHING_CUBES_H_

//General includes and I/O

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_types.h>
#include <stdio.h>
#include <stdarg.h>
#include <pcl/pcl_macros.h>

//Marching cubes includes
#include <pcl/gpu/kinfu_large_scale/marching_cubes.h>
#include <pcl/PolygonMesh.h>

#include <pcl/gpu/containers/device_array.h>

//TSDF volume includes
#include <pcl/gpu/kinfu_large_scale/tsdf_volume.h>

//Eigen Geometry and Transforms
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>
//#include <boost/graph/buffer_concepts.hpp>

namespace pcl
{
  namespace gpu
  {
    namespace kinfuLS
    {
      /** \brief The Standalone Marching Cubes Class provides encapsulated functionality for the Marching Cubes implementation originally by Anatoly Baksheev.
        * \author Raphael Favier
        * \author Francisco Heredia
        */
        
      template <typename PointT>
      class StandaloneMarchingCubes
      {
      public:
          typedef typename pcl::PointCloud<PointT> PointCloud;
          typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
          typedef boost::shared_ptr<pcl::PolygonMesh> MeshPtr;

      /** \brief Constructor        
        */
      StandaloneMarchingCubes (int voxels_x = 512, int voxels_y = 512, int voxels_z = 512, double volume_size = 3.0);
      
      /** \brief Destructor
        */
      ~StandaloneMarchingCubes (){}

      /** \brief Run marching cubes in a TSDF cloud and returns a PolygonMesh. Input X,Y,Z coordinates must be in indices of the TSDF volume grid, output is in meters. 
        * \param[in] cloud TSDF cloud with indices between [0 ... VOXELS_X][0 ... VOXELS_Y][0 ... VOXELS_Z]. Intensity value corresponds to the TSDF value in that coordinate.
        * \return pointer to a PolygonMesh in meters generated by marching cubes.          
        */
      MeshPtr
      getMeshFromTSDFCloud (const PointCloud &cloud);

      /** \brief Runs marching cubes on every pointcloud in the vector. Returns a vector containing the PolygonMeshes. 
        * \param[in] tsdf_clouds Vector of TSDF Clouds
        * \param[in] tsdf_offsets Vector of the offsets for every pointcloud in TsdfClouds. This offset (in indices) indicates the position of the cloud with respect to the absolute origin of the world model
        */
      void
      getMeshesFromTSDFVector (const std::vector<PointCloudPtr> &tsdf_clouds, const std::vector<Eigen::Vector3d> &tsdf_offsets);
      
      /** \brief Returns the associated Tsdf Volume buffer in GPU 
        * \return pointer to the Tsdf Volume buffer in GPU
        */  
      TsdfVolume::Ptr
      tsdfVolumeGPU ();
        
      /** \brief Returns the associated Tsdf Volume buffer in CPU
        * \return the Tsdf Volume buffer in CPU returned returned by reference
        */  
      std::vector<int>&
      tsdfVolumeCPU ();

      protected:

      /** \brief Loads a TSDF Cloud to the TSDF Volume in GPU
        * \param[in] cloud TSDF cloud that will be loaded. X,Y,Z of the cloud will only be loaded if their range is between [0 ... VOXELS_X][0 ... VOXELS_Y][0 ... VOXELS_Z]
        */       
      void
      loadTsdfCloudToGPU (const PointCloud &cloud);

      /** \brief Read the data in the point cloud. Performs a conversion to a suitable format for the TSDF Volume. Loads the converted data to the output vector.
        * \param[in] cloud point cloud to be converted
        * \param[out] output the vector of converted values, ready to be loaded to the GPU.
        */  
      void 
      convertTsdfVectors (const PointCloud &cloud, std::vector<int> &output);

      /** \brief Converts the triangles buffer device to a PolygonMesh.
        * \param[in] triangles the triangles buffer containing the points of the mesh
        * \return pointer to the PolygonMesh egnerated by marchign cubes          
        */  
      MeshPtr
      convertTrianglesToMesh (const pcl::gpu::DeviceArray<pcl::PointXYZ>& triangles);

      /** \brief Runs marching cubes on the data that is contained in the TSDF Volume in GPU.
        * \return param[return] pointer to a PolygonMesh in meters generated by marching cubes.
        */  
      MeshPtr
      runMarchingCubes ();

      private:

      /** The TSDF volume in GPU*/
      TsdfVolume::Ptr tsdf_volume_gpu_;

      /** The TSDF volume in CPU */
      std::vector<int> tsdf_volume_cpu_;
      
      /** Number of voxels in the grid for each axis */
      int voxels_x_;
      int voxels_y_;
      int voxels_z_;

      /** Tsdf volume size in meters. Should match the ones in internal.h */
      double volume_size_;
      
      /** Mesh counter used to name the output meshes */
      int mesh_counter_;
      
      };
    }
  }
}

#define PCL_INSTANTIATE_StandaloneMarchingCubes(PointT) template class PCL_EXPORTS pcl::gpu::kinfuLS::StandaloneMarchingCubes<PointT>;

#endif // PCL_STANDALONE_MARCHING_CUBES_H_
 
