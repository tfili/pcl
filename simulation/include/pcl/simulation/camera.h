#ifndef PCL_SIMULATION_CAMERA_HPP_
#define PCL_SIMULATION_CAMERA_HPP_

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <pcl/pcl_macros.h>

namespace pcl
{
  namespace simulation
  {
    class PCL_EXPORTS Camera
    {
    public:
      typedef boost::shared_ptr<Camera> Ptr;
      typedef boost::shared_ptr<const Camera> ConstPtr;

      Camera () : x_ (0), y_ (0), z_ (0), roll_ (0), pitch_ (0), yaw_ (0)
      {
        updatePose ();
        initializeCameraParameters ();
      }

      Camera (double x, double y, double z,
              double roll, double pitch, double yaw) : x_ (x),
                                                       y_ (y),
                                                       z_ (z),
                                                       roll_ (roll),
                                                       pitch_ (pitch),
                                                       yaw_ (yaw)
      {
        updatePose ();
        initializeCameraParameters ();
      }

      void
      setParameters (int width, int height,
                     double fx, double fy,
                     double cx, double cy,
                     double z_near, double z_far);

      Eigen::Matrix4d
      getProjectionMatrix () { return projection_matrix_; }

      double getX () const { return x_; }
      void setX (double x) { x_ = x; updatePose (); }

      double getY () const { return y_; }
      void setY (double y) { y_ = y; updatePose(); }

      double getZ () const { return z_; }
      void setZ (double z) { z_ = z; updatePose(); }

      double
      getRoll () const { return roll_; }
      void
      setRoll (double roll) { roll_ = roll; updatePose (); }

      double
      getPitch() const {return pitch_;}
      void
      setPitch(double pitch) { pitch_ = pitch; updatePose (); }

      double
      getYaw () const { return yaw_; }
      void
      setYaw (double yaw) { yaw_ = yaw; updatePose (); }

      Eigen::Isometry3d
      getPose () const { return pose_; }

      void set (double x, double y, double z, double roll, double pitch, double yaw)
      {
        x_ = x; y_ = y; z_ = z;
        roll_ = roll; pitch_ = pitch; yaw_ = yaw;
        updatePose();
      }

      void move (double vx, double vy, double vz);

      // Return the pose of the camera:
      Eigen::Vector3d getYPR ()
      {
        return Eigen::Vector3d (yaw_, pitch_, roll_);
      }

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
      void
      updatePose ();

      void
      initializeCameraParameters ();

      void
      updateProjectionMatrix ();
    
      double x_,y_,z_;
      double roll_,pitch_,yaw_;
    
      Eigen::Isometry3d pose_;

      // Camera Intrinsic Parameters
      int width_;
      int height_;
      double fx_;
      double fy_;
      double cx_;
      double cy_;

      // min and max range of the camera
      double z_near_;
      double z_far_;

      Eigen::Matrix4d projection_matrix_;
    };
  } // namespace - simulation
} // namespace - pcl

#endif /* PCL_SIMULATION_CAMERA_HPP_ */
