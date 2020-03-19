/**
 *  scan_matching_benchmark - 3D Scan Matching Benchmark
 *  Alejandro Mora - alejandromoramar@gmail.com
 *
 *  License: <>
 */

#include <gtest/gtest.h>
#include "cloud_transform/transform.h"

TEST(transform, rigid_transform_3D)
{
  // 0. Generate test point cloud.
  pcl::PointCloud<pcl::PointXYZ> cloud0;
  cloud0.points.push_back(pcl::PointXYZ(0.0, 0.0, 0.0));
  cloud0.points.push_back(pcl::PointXYZ(0.0, 0.0, 1.0));
  cloud0.points.push_back(pcl::PointXYZ(1.0, 0.0, 0.0));
  cloud0.points.push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
  cloud0.points.push_back(pcl::PointXYZ(1.0, 1.0, 1.0));

  // 1. Generate test transform:
  // --> Translation = [0.0, 1.0, -1.0]
  // --> Rotation = [0.0, 0.0, M_PI/2]
  Eigen::Vector3d t(0.0, 1.0, -1.0);
  Eigen::Vector3d R(0.0, 0.0, M_PI/2.0);

  // 2. Apply transform. Translate first, rotate after.
  pcl::PointCloud<pcl::PointXYZ> cloud1;
  cloud_transform::transformCloud<pcl::PointXYZ, double>(cloud0, t, Eigen::Vector3d::Zero(), cloud1);
  cloud_transform::transformCloud<pcl::PointXYZ, double>(cloud1, Eigen::Vector3d::Zero(), R, cloud1);

  // 3. Assert resulting points position.
  ASSERT_NEAR(cloud1.points[0].x, -1.0, 1e-6);
  ASSERT_NEAR(cloud1.points[0].y, 0.0, 1e-6);
  ASSERT_NEAR(cloud1.points[0].z, -1.0, 1e-6);

  ASSERT_NEAR(cloud1.points[1].x, -1.0, 1e-6);
  ASSERT_NEAR(cloud1.points[1].y, 0.0, 1e-6);
  ASSERT_NEAR(cloud1.points[1].z, 0.0, 1e-6);

  ASSERT_NEAR(cloud1.points[2].x, -1.0, 1e-6);
  ASSERT_NEAR(cloud1.points[2].y, 1.0, 1e-6);
  ASSERT_NEAR(cloud1.points[2].z, -1.0, 1e-6);

  ASSERT_NEAR(cloud1.points[3].x, -2.0, 1e-6);
  ASSERT_NEAR(cloud1.points[3].y, 0.0, 1e-6);
  ASSERT_NEAR(cloud1.points[3].z, -1.0, 1e-6);

  ASSERT_NEAR(cloud1.points[4].x, -2.0, 1e-6);
  ASSERT_NEAR(cloud1.points[4].y, 1.0, 1e-6);
  ASSERT_NEAR(cloud1.points[4].z, 0.0, 1e-6);
}

TEST(transform, rigid_transform_translate_first)
{
  // 0. Generate test point cloud.
  pcl::PointCloud<pcl::PointXYZ> cloud0;
  cloud0.points.push_back(pcl::PointXYZ(0.0, 0.0, 0.0));
  cloud0.points.push_back(pcl::PointXYZ(0.0, 0.0, 1.0));
  cloud0.points.push_back(pcl::PointXYZ(1.0, 0.0, 0.0));
  cloud0.points.push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
  cloud0.points.push_back(pcl::PointXYZ(1.0, 1.0, 1.0));

  // 1. Generate test transform:
  // --> Translation = [0.0, 1.0, -1.0]
  // --> Rotation = [0.0, 0.0, M_PI/2]
  Eigen::Vector3d t(0.0, 1.0, -1.0);
  Eigen::Vector3d R(0.0, 0.0, M_PI/2.0);

  // 2. Apply transform. Translate first, rotate after.
  pcl::PointCloud<pcl::PointXYZ> cloud1;
  cloud_transform::transformCloud<pcl::PointXYZ, double>(cloud0, t, Eigen::Vector3d::Zero(), cloud1);
  cloud_transform::transformCloud<pcl::PointXYZ, double>(cloud1, Eigen::Vector3d::Zero(), R, cloud1);

  // 2.1. Alternative way.
  pcl::PointCloud<pcl::PointXYZ> cloud2;
  cloud_transform::transformCloudTranslasteFirst<pcl::PointXYZ, double>(cloud0, t, R, cloud2);

  // 3. Assert resulting points position.
  ASSERT_NEAR(cloud1.points[0].x, cloud2.points[0].x, 1e-6);
  ASSERT_NEAR(cloud1.points[0].y, cloud2.points[0].y, 1e-6);
  ASSERT_NEAR(cloud1.points[0].z, cloud2.points[0].z, 1e-6);

  ASSERT_NEAR(cloud1.points[1].x, cloud2.points[1].x, 1e-6);
  ASSERT_NEAR(cloud1.points[1].y, cloud2.points[1].y, 1e-6);
  ASSERT_NEAR(cloud1.points[1].z, cloud2.points[1].z, 1e-6);

  ASSERT_NEAR(cloud1.points[2].x, cloud2.points[2].x, 1e-6);
  ASSERT_NEAR(cloud1.points[2].y, cloud2.points[2].y, 1e-6);
  ASSERT_NEAR(cloud1.points[2].z, cloud2.points[2].z, 1e-6);

  ASSERT_NEAR(cloud1.points[3].x, cloud2.points[3].x, 1e-6);
  ASSERT_NEAR(cloud1.points[3].y, cloud2.points[3].y, 1e-6);
  ASSERT_NEAR(cloud1.points[3].z, cloud2.points[3].z, 1e-6);

  ASSERT_NEAR(cloud1.points[4].x, cloud2.points[4].x, 1e-6);
  ASSERT_NEAR(cloud1.points[4].y, cloud2.points[4].y, 1e-6);
  ASSERT_NEAR(cloud1.points[4].z, cloud2.points[4].z, 1e-6);
}
