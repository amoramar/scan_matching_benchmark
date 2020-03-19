/**
 *  scan_matching_benchmark - 3D Scan Matching Benchmark
 *  Alejandro Mora - alejandromoramar@gmail.com
 *
 *  License: <>
 */

/**
 *  cloud_io - pcd_reader
 *
 *  Utilities to write '.pcd' files.
 */

#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

/**
 *  \brief Write a '.pcd' file. Saved in the current working directory!
 *  \param[in] file_name Name given to the '.pcd' file.
 *  \param[in] cloud Cloud object with the contents to be saved.
 *  \return Boolean indicating if read operation was successful.
 */
template <typename PCLpointT>
bool writePCD(const std::string file_name,
              const typename pcl::PointCloud<PCLpointT>& cloud)
{
  // Make use of the 'pcl::PCDWriter' class.
  pcl::PCDWriter pcd_writer;
  bool status = pcd_writer.writeASCII(file_name, cloud);

  // Return a boolean indicating the successfulness of the operation.
  if (status == -1)
    return(false);
  else
    return(true);
}
