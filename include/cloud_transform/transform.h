/**
 *  scan_matching_benchmark - 3D Scan Matching Benchmark
 *  Alejandro Mora - alejandromoramar@gmail.com
 *
 *  License: <>
 */

/**
 *  cloud_transform - transform
 *
 *  Utilities to apply rigid transformations to point clouds.
 */

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

namespace cloud_transform {

/**
 *  \brief Apply a rigid transformation to a whole point cloud (rotation
 *  expressed in Euler angles). Input and output clouds can be the same.
 */
template <typename PCLPointT, typename T>
void transformCloud(pcl::PointCloud<PCLPointT>& input_cloud,
                    const Eigen::Matrix<T, 3, 1> translation,
                    const Eigen::Matrix<T, 3, 1> euler_angles,
                    pcl::PointCloud<PCLPointT>& output_cloud)
{
  // Build an Eigen::Affine3 transform object.
  Eigen::Transform<T, 3, Eigen::Affine> transformation;
  // Translation provided as: [t_x, t_y, t_z]
  transformation.translation() = translation;
  // Rotation provided as: [roll (R_x), pitch (R_y), yaw (R_z)]
  Eigen::Matrix<T, 3, 3> R;
  R = Eigen::AngleAxis<T>(euler_angles(0), Eigen::Matrix<T, 3, 1>::UnitX()) *
      Eigen::AngleAxis<T>(euler_angles(1), Eigen::Matrix<T, 3, 1>::UnitY()) *
      Eigen::AngleAxis<T>(euler_angles(2), Eigen::Matrix<T, 3, 1>::UnitZ());
  transformation.matrix().block(0, 0, 3, 3) = R;

  // Apply transform.
  pcl::transformPointCloud<pcl::PointXYZ>(input_cloud, output_cloud, transformation);
};

/**
 *  \brief Apply a rigid transformation to a whole point cloud (rotation
 *  expressed in Euler angles). TRANSLATE first, then rotate.
 */
template <typename PCLPointT, typename T>
void transformCloudTranslasteFirst(pcl::PointCloud<PCLPointT>& input_cloud,
                                   const Eigen::Matrix<T, 3, 1> translation,
                                   const Eigen::Matrix<T, 3, 1> euler_angles,
                                   pcl::PointCloud<PCLPointT>& output_cloud)
{
  // Build an Eigen::Affine3 transform object.
  Eigen::Transform<T, 3, Eigen::Affine> transformation1
    = Eigen::Transform<T, 3, Eigen::Affine>::Identity();

  Eigen::Transform<T, 3, Eigen::Affine> transformation2
    = Eigen::Transform<T, 3, Eigen::Affine>::Identity();

  // Translation provided as: [t_x, t_y, t_z]
  transformation1.translation() = translation;

  // Rotation provided as: [roll (R_x), pitch (R_y), yaw (R_z)]
  Eigen::Matrix<T, 3, 3> R;
  R = Eigen::AngleAxis<T>(euler_angles(0), Eigen::Matrix<T, 3, 1>::UnitX()) *
      Eigen::AngleAxis<T>(euler_angles(1), Eigen::Matrix<T, 3, 1>::UnitY()) *
      Eigen::AngleAxis<T>(euler_angles(2), Eigen::Matrix<T, 3, 1>::UnitZ());
  transformation2.matrix().block(0, 0, 3, 3) = R;

  // Apply transform.
  pcl::transformPointCloud<pcl::PointXYZ>(input_cloud, output_cloud, transformation1);
  pcl::transformPointCloud<pcl::PointXYZ>(output_cloud, output_cloud, transformation2);

};

}
