/*
 * global_nn_classifier.cpp
 *
 *  Created on: Mar 9, 2012
 *      Author: aitor
 */

#include <pcl/apps/3d_rec_framework/pipeline/impl/global_nn_recognizer_crh.hpp>
#include "pcl/apps/3d_rec_framework/utils/metrics.h"

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::Histogram<90>,
    (double[90], histogram, histogram90)
)

//Instantiation
template class pcl::rec_3d_framework::GlobalNNCRHRecognizer<flann::L2, pcl::PointXYZ, pcl::VFHSignature308>;
template class pcl::rec_3d_framework::GlobalNNCRHRecognizer<flann::L1, pcl::PointXYZ, pcl::VFHSignature308>;
template class pcl::rec_3d_framework::GlobalNNCRHRecognizer<Metrics::HistIntersectionUnionDistance, pcl::PointXYZ, pcl::VFHSignature308>;
template class pcl::rec_3d_framework::GlobalNNCRHRecognizer<flann::ChiSquareDistance, pcl::PointXYZ, pcl::VFHSignature308>;
template class pcl::rec_3d_framework::GlobalNNCRHRecognizer<flann::L1, pcl::PointXYZ, pcl::ESFSignature640>;
template class pcl::rec_3d_framework::GlobalNNCRHRecognizer<flann::L2, pcl::PointXYZ, pcl::ESFSignature640>;
