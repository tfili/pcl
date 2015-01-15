#include <pcl/range_image/range_image.h>

int main (int argc, char** argv) {
  pcl::PointCloud<pcl::PointXYZ> pointCloud;
  
  // Generate the data
  for (double y=-0.5; y<=0.5; y+=0.01) {
    for (double z=-0.5; z<=0.5; z+=0.01) {
      pcl::PointXYZ point;
      point.x = 2.0 - y;
      point.y = y;
      point.z = z;
      pointCloud.points.push_back(point);
    }
  }
  pointCloud.width = (uint32_t) pointCloud.points.size();
  pointCloud.height = 1;
  
  // We now want to create a range image from the above point cloud, with a 1deg angular resolution
  double angularResolution = (double) (  1.0 * (M_PI/180.0));  //   1.0 degree in radians
  double maxAngleWidth     = (double) (360.0 * (M_PI/180.0));  // 360.0 degree in radians
  double maxAngleHeight    = (double) (180.0 * (M_PI/180.0));  // 180.0 degree in radians
  Eigen::Affine3d sensorPose = (Eigen::Affine3d)Eigen::Translation3d(0.0, 0.0, 0.0);
  pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
  double noiseLevel=0.00;
  double minRange = 0.0;
  int borderSize = 1;
  
  pcl::RangeImage rangeImage;
  rangeImage.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,
                                  sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
  
  std::cout << rangeImage << "\n";
}
