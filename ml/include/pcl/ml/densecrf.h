/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 * Author : Christian Potthast
 * Email  : potthast@usc.edu
 *
 */

#ifndef PCL_DENSE_CRF_H_
#define PCL_DENSE_CRF_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/ml/pairwise_potential.h>

namespace pcl
{
  /** \brief
   * 
   */
  class PCL_EXPORTS DenseCrf
  {
    public:

      /** \brief Constructor for DenseCrf class */
      DenseCrf (int N, int m);

      /** \brief Deconstructor for DenseCrf class */
      ~DenseCrf ();
      
      /** \brief set the input data vector.
       * The input data vector holds the measurements
       * coordinates as ijk of the voxel grid
       */
      void
      setDataVector (const std::vector<Eigen::Vector3i> data);

      /** \brief The associated color of the data
       */
      void
      setColorVector (const std::vector<Eigen::Vector3i> color);

      void
      setUnaryEnergy (const std::vector<double> unary);
 
      /** \brief      */
      void
      addPairwiseEnergy (const std::vector<double> &feature, const int feature_dimension, const double w);
      
      
      /** \brief Add a pairwise gaussian kernel
       * 
       */
      void
      addPairwiseGaussian (double sx, double sy, double sz, double w);
      
      /** \brief Add a bilateral gaussian kernel
       * 
       */
      void
      addPairwiseBilateral (double sx, double sy, double sz, 
                            double sr, double sg, double sb,
                            double w);


      void
      addPairwiseNormals (std::vector<Eigen::Vector3i> &coord,
                          std::vector<Eigen::Vector3d> &normals,
                          double sx, double sy, double sz, 
                          double snx, double sny, double snz,
                          double w);
      

      void
      inference (int n_iterations, std::vector<double> &result, double relax = 1.0f);
 
      void
      mapInference (int n_iterations, std::vector<int> &result, double relax = 1.0f);
      
      void
      expAndNormalize (std::vector<double> &out, const std::vector<double> &in,
                       double scale, double relax = 1.0f);
 
      void
      expAndNormalizeORI ( double* out, const double* in, double scale=1.0f, double relax=1.0f );
      void map ( int n_iterations, std::vector<int> result, double relax=1.0f );
      std::vector<double> runInference( int n_iterations, double relax );
      void startInference();
      void stepInference( double relax );
      

      void
      runInference (double relax);


      void
      getBarycentric (int idx, std::vector<double> &bary);

      void
      getFeatures (int idx, std::vector<double> &features);
      


    protected:

      /** \brief Number of variables and labels */
      int N_, M_;

      /** \brief Data vector */
      std::vector<Eigen::Vector3i> data_;

      /** \brief Color vector */
      std::vector<Eigen::Vector3i> color_;

      /** TODO: double might use to much memory */
      /** \brief CRF unary potentials */
      std::vector<double> unary_;

      std::vector<double> current_;
      std::vector<double> next_;
      std::vector<double> tmp_;

      /** \brief pairwise potentials */
      std::vector<PairwisePotential*> pairwise_potential_;
          
      /** \brief input types */
      bool xyz_, rgb_, normal_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}





#endif
