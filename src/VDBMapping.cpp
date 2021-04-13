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

#include <vdb_mapping/VDBMapping.h>

#include <iostream>

VDBMapping::VDBMapping(const double resolution)
  : m_resolution(resolution)
  , m_config_set(false)
{
  // Initialize Grid
  m_vdb_grid = createVDBMap(m_resolution);
}

void VDBMapping::resetMap()
{
  m_vdb_grid->clear();
  m_vdb_grid = createVDBMap(m_resolution);
}

VDBMapping::GridT::Ptr VDBMapping::createVDBMap(double resolution)
{
  GridT::Ptr new_map = GridT::create(0.0);
  new_map->setTransform(openvdb::math::Transform::createLinearTransform(m_resolution));
  new_map->setGridClass(openvdb::GRID_LEVEL_SET);
  return new_map;
}

bool VDBMapping::insertPointCloud(const PointCloudT::ConstPtr& cloud,
                                  const Eigen::Matrix<double, 3, 1>& origin)
{
  // Check if a valid configuration was loaded
  if (!m_config_set)
  {
    std::cerr << "Map not properly configured. Did you call setConfig method?" << std::endl;
    return false;
  }

  RayT ray;
  DDAT dda;

  // Ray origin in world coordinates
  openvdb::Vec3d ray_origin_world(origin.x(), origin.y(), origin.z());
  // Ray origin in index coordinates
  const Vec3T ray_origin_index(m_vdb_grid->worldToIndex(ray_origin_world));
  // Ray end point in world coordinates
  openvdb::Vec3d ray_end_world;
  // Direction the ray is point towards
  openvdb::Vec3d ray_direction;
  bool max_range_ray;

  GridT::Accessor acc = m_vdb_grid->getAccessor();

  // Creating a temporary grid in which the new data is casted. This way we prevent the computation
  // of redundant probability updates in the actual map
  GridT::Ptr temp_grid     = GridT::create(0.0);
  GridT::Accessor temp_acc = temp_grid->getAccessor();

  openvdb::Vec3d x;
  double ray_length;

  // Raycasting of every point in the input cloud
  for (const PointT& pt : *cloud)
  {
    max_range_ray = false;
    ray_end_world = openvdb::Vec3d(pt.x, pt.y, pt.z);
    if (m_max_range > 0.0 && (ray_end_world - ray_origin_world).length() > m_max_range)
    {
      ray_end_world = ray_origin_world + (ray_end_world - ray_origin_world).unit() * m_max_range;
      max_range_ray = true;
    }

    ray_direction = m_vdb_grid->worldToIndex(ray_end_world - ray_origin_world);

    ray.setEye(ray_origin_index);
    ray.setDir(ray_direction);
    dda.init(ray);

    ray_length = ray_direction.length();
    ray_direction.normalize();

    // The signed distance is calculated for each DDA step to determine, when the endpoint is
    // reached.
    double signed_distance = 1;
    while (signed_distance > 0)
    {
      x = openvdb::Vec3d(dda.voxel().x(), dda.voxel().y(), dda.voxel().z()) - ray_origin_index;
      // Signed distance in grid coordinates for faster processing(not scaled with the grid
      // resolution!!!)
      signed_distance = ray_length - ray_direction.dot(x);
      temp_acc.setActiveState(dda.voxel(), true);
      dda.step();
    }

    // Set the last passed voxel as occupied if the ray wasn't longer than the maximum raycast range
    if (!max_range_ray)
    {
      temp_acc.setValueOn(dda.voxel(), -1);
    }
  }

  // Probability update lambda for free space grid elements
  auto miss = [&prob_miss      = m_logodds_miss,
               &prob_thres_min = m_logodds_thres_min](float& voxel_value, bool& active) {
    voxel_value += prob_miss;
    if (voxel_value < prob_thres_min)
    {
      active = false;
    }
  };

  // Probability update lambda for occupied grid elements
  auto hit = [&prob_hit = m_logodds_hit, &prob_thres_max = m_logodds_thres_max](float& voxel_value,
                                                                                bool& active) {
    voxel_value += prob_hit;
    if (voxel_value > prob_thres_max)
    {
      active = true;
    }
  };

  // Integrating the data of the temporary grid into the map using the probability update functions
  for (GridT::ValueOnCIter iter = temp_grid->cbeginValueOn(); iter; ++iter)
  {
    if (*iter == -1)
    {
      acc.modifyValueAndActiveState(iter.getCoord(), hit);
    }
    else
    {
      acc.modifyValueAndActiveState(iter.getCoord(), miss);
    }
  }
  return true;
}

void VDBMapping::setConfig(const Config config)
{
  // Sanity Check for input config
  if (config.prob_miss > 0.5)
  {
    std::cerr << "Probability for a miss should be below 0.5 but is " << config.prob_miss
              << std::endl;
    return;
  }
  if (config.prob_hit < 0.5)
  {
    std::cerr << "Probability for a hit should be above 0.5 but is " << config.prob_miss
              << std::endl;
    return;
  }

  if (config.max_range < 0.0)
  {
    std::cerr << "Max range of " << config.max_range << " invalid. Range cannot be negative."
              << config.prob_miss << std::endl;
    return;
  }
  m_max_range = config.max_range;
  // Store probabilities as log odds
  m_logodds_miss      = log(config.prob_miss) - log(1 - config.prob_miss);
  m_logodds_hit       = log(config.prob_hit) - log(1 - config.prob_hit);
  m_logodds_thres_min = log(config.prob_thres_min) - log(1 - config.prob_thres_min);
  m_logodds_thres_max = log(config.prob_thres_max) - log(1 - config.prob_thres_max);
  m_config_set        = true;
}
