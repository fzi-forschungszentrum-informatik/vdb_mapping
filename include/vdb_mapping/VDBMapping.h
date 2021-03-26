// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2021 FZI Forschungszentrum Informatik
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Marvin Große Besselmann grosse@fzi.de
 * \date    2020-12-23
 *
 */
//----------------------------------------------------------------------
#ifndef VDB_MAPPING_VDB_MAPPING_H_INCLUDED
#define VDB_MAPPING_VDB_MAPPING_H_INCLUDED

#include <openvdb/Types.h>
#include <openvdb/math/DDA.h>
#include <openvdb/math/Ray.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/Morphology.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <eigen3/Eigen/Geometry>

/*!
 * \brief Main Mapping class which handles all data integration
 */
class VDBMapping
{
public:
  using PointT      = pcl::PointXYZ;
  using PointCloudT = pcl::PointCloud<PointT>;

  using RayT  = openvdb::math::Ray<double>;
  using Vec3T = RayT::Vec3Type;
  using DDAT  = openvdb::math::DDA<RayT, 0>;

  using GridT = openvdb::FloatGrid;

  /*!
   * \brief Accumulation of configuration parameters
   */
  struct Config
  {
    double max_range;
    double prob_hit;
    double prob_miss;
    double prob_thres_min;
    double prob_thres_max;
  };

  VDBMapping() = delete;

  /*!
   * \brief Construktur creates a new VDBMapping objekt with parametrizable grid resolution
   *
   * \param resolution Resolution of the VDB Grid
   */
  VDBMapping(const double resolution);
  virtual ~VDBMapping(){};

  /*!
   * \brief Handles the integration of new PointCloud data into the VDB data structure.
   * All datapoints are raycasted starting from the origin position
   *
   * \param cloud Input cloud in map coordinates
   * \param origin Sensor position in map coordinates
   *
   * \returns Was the insertion of the new pointcloud successful
   */
  bool insertPointCloud(const PointCloudT::ConstPtr& cloud,
                        const Eigen::Matrix<double, 3, 1> origin);


  /*!
   * \brief Returns a pointer to the VDB map structure
   *
   * \returns Map pointer
   */
  GridT::Ptr getMap() const { return m_vdb_grid; }


  /*!
   * \brief Handles changing the mapping config
   *
   * \param config Configuration structure
   */
  void setConfig(const Config config);

private:
  /*!
   * \brief VDB grid pointer
   */
  GridT::Ptr m_vdb_grid;
  /*!
   * \brief Maximum raycasting distance
   */
  double m_max_range;
  /*!
   * \brief Grid resolution of the map
   */
  double m_resolution;
  /*!
   * \brief Probability update value for passing an obstacle
   */
  double m_logodds_hit;
  /*!
   * \brief Probability update value for passing free space
   */
  double m_logodds_miss;
  /*!
   * \brief Upper occupancy probability threshold
   */
  double m_logodds_thres_min;
  /*!
   * \brief Lower occupancy probability threshold
   */
  double m_logodds_thres_max;
  /*!
   * \brief Flag checking wether a valid config was already loaded
   */
  bool m_config_set;
};

#endif /* VDB_MAPPING_VDB_MAPPING_H_INCLUDED */
