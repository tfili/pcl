#ifndef PCL_TRACKING_IMPL_DISTANCE_COHERENCE_H_
#define PCL_TRACKING_IMPL_DISTANCE_COHERENCE_H_

#include <Eigen/Dense>
#include <pcl/tracking/distance_coherence.h>

namespace pcl
{
  namespace tracking
  {
    template <typename PointInT> double
    DistanceCoherence<PointInT>::computeCoherence (PointInT &source, PointInT &target)
    {
       Eigen::Vector4d p = source.getVector4dMap ();
       Eigen::Vector4d p_dash = target.getVector4dMap ();
       double d = (p - p_dash).norm ();
       return 1.0 / (1.0 + d * d * weight_);
    }
  }
}

#define PCL_INSTANTIATE_DistanceCoherence(T) template class PCL_EXPORTS pcl::tracking::DistanceCoherence<T>;

#endif
