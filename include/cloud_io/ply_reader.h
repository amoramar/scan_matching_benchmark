/**
 *  scan_matching_benchmark - 3D Scan Matching Benchmark
 *  Alejandro Mora - alejandromoramar@gmail.com
 *
 *  License: <>
 */

/**
 *  cloud_io - ply_reader
 *
 *  Utilities to read '.ply' files and parse them as pcl::PointCloud 's.
 */

#pragma once

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

/**
 *  \brief Read a '.ply' file. Copy its contents in a pcl::PointCloud
 *  provided by the user, of a certain templated point type.
 *  \param[in] ply_path Absolute path to the '.ply' file.
 *  \param[out] cloud Cloud object provided by the user.
 *  \return Boolean indicating if read operation was successful.
 */
template <typename PCLpointT>
bool readPLYInPlace(const std::string ply_path,
                    const typename pcl::PointCloud<PCLpointT>& cloud)
{
  // Instantiate a buffer object.
  typename pcl::PointCloud<PCLpointT>::Ptr buffer(new pcl::PointCloud<PCLpointT>);

  // Check that a file in the specified path exists.
  if (pcl::io::loadPLYFile<PCLpointT> (ply_path, *buffer) == -1)
    return (false);

  // If so, copy the retrieved contents in the cloud object provided.
  pcl::copyPointCloud(*buffer, cloud);
  return (true);
}

/**
 *  \brief Read a '.ply' file. Copy its contents in a pcl::PointCloud
 *  provided by the user, of a certain templated point type.
 *  \param[in] ply_path Absolute path to the '.ply' file.
 *  \param[out] cloud Cloud object provided by the user.
 */
template <typename PCLpointT>
void readPLY(const std::string ply_path,
             const typename pcl::PointCloud<PCLpointT>& cloud)
{
  // Create a pcl::PLYReader object, and provide it the necessary data.
  pcl::PLYReader ply_reader;
  ply_reader.read<PCLpointT>(ply_path, cloud);
}
