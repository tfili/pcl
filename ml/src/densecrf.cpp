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

#include <pcl/ml/densecrf.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::DenseCrf::DenseCrf (int N, int m) :
  N_ (N), M_ (m),
  xyz_ (false), rgb_ (false), normal_ (false)
{
  current_.resize (N_ * M_, 0.0f);
  next_.resize (N_ * M_, 0.0f);
  tmp_.resize (2 * N_ * M_, 0.0f);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::DenseCrf::~DenseCrf ()
{
  for(size_t i = 0; i < pairwise_potential_.size (); i++ )
    delete pairwise_potential_[i];
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::DenseCrf::setDataVector (const std::vector<Eigen::Vector3i> data)
{
  xyz_ = true;
  data_ = data;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::DenseCrf::setColorVector (const std::vector<Eigen::Vector3i> color)
{
  rgb_ = true;
  color_ = color;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::DenseCrf::setUnaryEnergy (const std::vector<double> unary)
{
  unary_ = unary;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::DenseCrf::addPairwiseEnergy (const std::vector<double> &feature, const int feature_dimension, const double w)
{
  pairwise_potential_.push_back ( new PairwisePotential (feature, feature_dimension, N_, w) );
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::DenseCrf::addPairwiseGaussian (double sx, double sy, double sz, double w)
{
  // create feature vector
  std::vector<double> feature;
  // reserve space for the three-dimensional Gaussian kernel
  feature.resize (N_ * 3);
  
  // fill the feature vector
  for (size_t i = 0; i < data_.size (); i++)
  {
    feature[i * 3    ] = static_cast<double> (data_[i].x ()) / sx;
    feature[i * 3 + 1] = static_cast<double> (data_[i].y ()) / sy;
    feature[i * 3 + 2] = static_cast<double> (data_[i].z ()) / sz;
  }
  // add kernel
  addPairwiseEnergy (feature, 3, w);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::DenseCrf::addPairwiseBilateral (double sx, double sy, double sz, 
                                     double sr, double sg, double sb,
                                     double w)
{
  // create feature vector
  std::vector<double> feature;
  // reserve space for the six-dimensional Gaussian kernel
  feature.resize (N_ * 6);

  // fill the feature vector
  for (size_t i = 0; i < color_.size (); i++)
  {
    feature[i * 6    ] = static_cast<double> (data_[i].x ()) / sx;
    feature[i * 6 + 1] = static_cast<double> (data_[i].y ()) / sy;
    feature[i * 6 + 2] = static_cast<double> (data_[i].z ()) / sz;
    feature[i * 6 + 3] = static_cast<double> (color_[i].x ()) / sr;
    feature[i * 6 + 4] = static_cast<double> (color_[i].y ()) / sg;
    feature[i * 6 + 5] = static_cast<double> (color_[i].z ()) / sb;
  }
  // add kernel
  addPairwiseEnergy (feature, 6, w);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::DenseCrf::addPairwiseNormals (std::vector<Eigen::Vector3i> &coord,
                                   std::vector<Eigen::Vector3d> &normals,
                                   double sx, double sy, double sz, 
                                   double snx, double sny, double snz,
                                   double w)
{
  std::cout << coord.size () << std::endl;
  std::cout << normals.size () << std::endl;

  // create feature vector
  std::vector<double> feature;
  // reserve space for the six-dimensional Gaussian kernel
  feature.resize (N_ * 6);

  // fill the feature vector
  for (size_t i = 0; i < coord.size (); i++)
  {
    if (pcl_isnan (normals[i].x ()))
    {
      if (i > 0)
      {
        normals[i].x () = normals[i-1].x ();
        normals[i].y () = normals[i-1].y ();
        normals[i].z () = normals[i-1].z ();
      }

      //std::cout << "NaN" << std::endl;
      
    }

    feature[i * 6    ] = static_cast<double> (coord[i].x ()) / sx;
    feature[i * 6 + 1] = static_cast<double> (coord[i].y ()) / sy;
    feature[i * 6 + 2] = static_cast<double> (coord[i].z ()) / sz;
    feature[i * 6 + 3] = static_cast<double> (normals[i].x ()) / snx;
    feature[i * 6 + 4] = static_cast<double> (normals[i].y ()) / sny;
    feature[i * 6 + 5] = static_cast<double> (normals[i].z ()) / snz;
  }
  // add kernel
  
  std::cout << "TEEEEST" << std::endl;

  addPairwiseEnergy (feature, 6, w);

  std::cout << "TEEEEST2" << std::endl;

}




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::DenseCrf::inference (int n_iterations, std::vector<double> &result, double relax)
{
  // Start inference
  // Initialize using the unary energies
  expAndNormalize (current_, unary_, -1);

  for (int i = 0; i < n_iterations; i++)
  {
    runInference (relax);
    std::cout << "iteration: " << i+1 << " - DONE" << std::endl;
  }
  
  // Copy the data into the result vector
  result = current_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::DenseCrf::mapInference (int n_iterations, std::vector<int> &result, double relax)
{
  // Start inference
  // Initialize using the unary energies
  expAndNormalize (current_, unary_, -1);

  for (int i = 0; i < n_iterations; i++)
  {
    runInference (relax);
    std::cout << "iteration: " << i+1 << " - DONE" << std::endl;
  }

  // Find the map
  for (int i = 0; i < N_; i++)
  {
    const int prob_idx = i * M_;
    // Find the max
    double p_label = current_[prob_idx];
    int idx = 0;
    for (int j = 1; j < M_; j++) 
    { 
      if (p_label < current_[prob_idx + j])
      {
        p_label = current_[prob_idx + j];
        idx = j;
      }
    }
    result[i] = idx;
  }
  


/*
	for( int i = 0; i < N_; i++ ){
		const double * p = prob + i*M_;
		// Find the max and subtract it so that the exp doesn't explode
		double mx = p[0];
		int imx = 0;
		for( int j=1; j<M_; j++ )
			if( mx < p[j] ){
				mx = p[j];
				imx = j;
			}
    result[i] = imx
  }
*/

}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::DenseCrf::expAndNormalize (std::vector<double> &out, const std::vector<double> &in,
                                double scale, double relax)
{
  std::vector<double> V (N_ + 10);
	for( int i = 0; i < N_; i++ ){
    int b_idx = i*M_;
		// Find the max and subtract it so that the exp doesn't explode
		double mx = scale * in[b_idx];
		for( int j = 1; j < M_; j++ )
			if( mx < scale * in[b_idx + j] )
				mx = scale * in[b_idx + j];
		double tt = 0;
		for( int  j = 0; j < M_; j++ ){
			V[j] = expf( scale * in[b_idx + j] - mx );
			tt += V[j];
		}
		// Make it a probability
		for( int j = 0; j < M_; j++ )
			V[j] /= tt;
		
    int a_idx = i*M_;
		for( int j = 0; j < M_; j++ )
			if (relax == 1)
				out[a_idx + j] = V[j];
			else
				out[a_idx + j] = (1-relax) * out[a_idx + j] + relax * V[j];
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::DenseCrf::runInference (double relax)
{
  // set the unary potentials
  for (size_t i = 0; i < unary_.size (); i++)
    next_[i] = -unary_[i];

  // Add up all pairwise potentials
	for( unsigned int i = 0; i < pairwise_potential_.size(); i++ )
    pairwise_potential_[i]->compute( next_, current_, tmp_, M_ );

	// Exponentiate and normalize
	expAndNormalize( current_, next_, 1.0, relax );
}

void
pcl::DenseCrf::getBarycentric (int idx, std::vector<double> &bary)
{
  bary = pairwise_potential_[idx]->bary_;
}

void
pcl::DenseCrf::getFeatures (int idx, std::vector<double> &features)
{
  features = pairwise_potential_[idx]->features_;
}
