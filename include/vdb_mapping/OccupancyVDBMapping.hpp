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
 * \author  Lennart Puck puck@fzi.de
 * \date    2021-04-29
 *
 */
//----------------------------------------------------------------------
#ifndef VDB_MAPPING_OCCUPANCY_VDB_MAPPING_H_INCLUDED
#define VDB_MAPPING_OCCUPANCY_VDB_MAPPING_H_INCLUDED

#include "vdb_mapping/VDBMapping.hpp"

namespace vdb_mapping {

/*!
 * \brief Accumulation of configuration parameters
 */
struct Config : BaseConfig
{
  double prob_hit;
  double prob_miss;
  double prob_thres_min;
  double prob_thres_max;
};

class OccupancyVDBMapping : public VDBMapping<float, Config>
{
public:
  OccupancyVDBMapping(const double resolution)
    : VDBMapping<float, Config>(resolution)
  {
  }

  /*!
   * \brief Handles changing the mapping config
   *
   * \param config Configuration structure
   */
  inline void setConfig(const Config& config) override
  {
    // call base class function
    VDBMapping::setConfig(config);

    // Sanity Check for input config
    if (config.prob_miss > 0.5)
    {
      std::cerr << "Probability for a miss should be below 0.5 but is " << config.prob_miss
                << std::endl;
      return;
    }
    if (config.prob_hit < 0.5)
    {
      std::cerr << "Probability for a hit should be above 0.5 but is " << config.prob_hit
                << std::endl;
      return;
    }

    // Store probabilities as log odds
    m_logodds_miss = static_cast<float>(log(config.prob_miss) - log(1 - config.prob_miss));
    m_logodds_hit  = static_cast<float>(log(config.prob_hit) - log(1 - config.prob_hit));
    m_logodds_thres_min =
      static_cast<float>(log(config.prob_thres_min) - log(1 - config.prob_thres_min));
    m_logodds_thres_max =
      static_cast<float>(log(config.prob_thres_max) - log(1 - config.prob_thres_max));
    // Values to clamp the logodds in order to prevent non dynamic map behavior
    m_max_logodds = static_cast<float>(log(0.99) - log(0.01));
    m_min_logodds = static_cast<float>(log(0.01) - log(0.99));
    m_config_set  = true;
  }

protected:
  inline bool updateFreeNode(float& voxel_value, bool& active) override
  {
    voxel_value += m_logodds_miss;
    if (voxel_value < m_logodds_thres_min)
    {
      active = false;
      if (voxel_value < m_min_logodds)
      {
        voxel_value = m_min_logodds;
      }
    }
    return true;
  }
  inline bool updateOccupiedNode(float& voxel_value, bool& active) override
  {
    voxel_value += m_logodds_hit;
    if (voxel_value > m_logodds_thres_max)
    {
      active = true;
      if (voxel_value > m_max_logodds)
      {
        voxel_value = m_max_logodds;
      }
    }
    return true;
  }
  inline bool setNodeToFree(float& voxel_value, bool& active) override
  {
    voxel_value = m_min_logodds;
    active      = false;
    return true;
  }
  inline bool setNodeToOccupied(float& voxel_value, bool& active) override
  {
    voxel_value = m_max_logodds;
    active      = true;
    return true;
  }
  inline bool setNodeState(float& voxel_value, bool& active) override
  {
    active = voxel_value > m_logodds_thres_max;
    return true;
  }

  inline void createMapFromPointCloud(const PointCloudT::Ptr& cloud,
                                      const bool set_background,
                                      const bool clear_map) override
  {
    if (clear_map)
    {
      m_vdb_grid->clear();
    }

    typename GridT::Accessor acc = m_vdb_grid->getAccessor();

    for (auto point : cloud->points)
    {
      openvdb::Vec3d index_coord =
        m_vdb_grid->worldToIndex(openvdb::Vec3d(point.x, point.y, point.z));
      acc.setValueOn(openvdb::Coord::floor(index_coord), m_max_logodds);
    }
    openvdb::CoordBBox bbox = m_vdb_grid->evalActiveVoxelBoundingBox();

    if (set_background)
    {
      for (int x = bbox.min().x(); x <= bbox.max().x(); ++x)
      {
        for (int y = bbox.min().y(); y <= bbox.max().y(); ++y)
        {
          for (int z = bbox.min().z(); z <= bbox.max().z(); ++z)
          {
            openvdb::Coord index_coord = openvdb::Coord(x, y, z);
            if (!acc.isValueOn(index_coord))
            {
              acc.setValueOff(index_coord, m_min_logodds);
            }
          }
        }
      }
    }

    m_vdb_grid->pruneGrid();
  }

  /*!
   * \brief Probability update value for passing an obstacle
   */
  float m_logodds_hit;
  /*!
   * \brief Probability update value for passing free space
   */
  float m_logodds_miss;
  /*!
   * \brief Upper occupancy probability threshold
   */
  float m_logodds_thres_min;
  /*!
   * \brief Lower occupancy probability threshold
   */
  float m_logodds_thres_max;
  /*!
   * \brief Maximum clamping point for logodds
   */
  float m_max_logodds;
  /*!
   * \brief Minimum clamping point for logodds
   */
  float m_min_logodds;
};


} // namespace vdb_mapping

#include "OccupancyVDBMapping.hpp"
#endif /* VDB_MAPPING_OCCUPANCY_VDB_MAPPING_H_INCLUDED */
