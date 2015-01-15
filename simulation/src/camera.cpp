#include <iostream>
#include <pcl/simulation/camera.h>

using namespace Eigen;
using namespace pcl::simulation;

void
pcl::simulation::Camera::move (double vx, double vy, double vz)
{
  Vector3d v;
  v << vx, vy, vz;
  pose_.pretranslate (pose_.rotation ()*v);
  x_ = pose_.translation ().x ();
  y_ = pose_.translation ().y ();
  z_ = pose_.translation ().z ();
}

void
pcl::simulation::Camera::updatePose ()
{
  Matrix3d m;
  m = AngleAxisd (yaw_, Vector3d::UnitZ ())
    * AngleAxisd (pitch_, Vector3d::UnitY ())
    * AngleAxisd (roll_, Vector3d::UnitX ());

  pose_.setIdentity ();
  pose_ *= m;
  
  Vector3d v;
  v << x_, y_, z_;
  pose_.translation () = v;
}

void
pcl::simulation::Camera::setParameters (int width, int height,
                                        double fx, double fy,
                                        double cx, double cy,
                                        double z_near, double z_far)
{
  width_ = width;
  height_ = height;
  fx_ = fx;
  fy_ = fy;
  cx_ = cx;
  cy_ = cy;
  z_near_ = z_near;
  z_far_ = z_far;

  double z_nf = (z_near_-z_far_);
  projection_matrix_ <<  2.0*fx_/width_,  0,                 1.0-(2.0*cx_/width_),     0,
                         0,                2.0*fy_/height_,  1.0-(2.0*cy_/height_),    0,
                         0,                0,                (z_far_+z_near_)/z_nf,  2.0*z_near_*z_far_/z_nf,
                         0,                0,                -1.0,                  0;
}

void
pcl::simulation::Camera::initializeCameraParameters ()
{
  setParameters (640, 480,
                 576.09757860, 576.09757860,
                 321.06398107, 242.97676897,
                 0.7, 20.0);
}
