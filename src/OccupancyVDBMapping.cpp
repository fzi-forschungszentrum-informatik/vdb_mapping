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
 * \author  Lennart Puck puck@fzi.de
 * \date    2021-04-29
 *
 */
//----------------------------------------------------------------------


#include "vdb_mapping/OccupancyVDBMapping.h"


bool OccupancyVDBMapping::insertPointCloud(const PointCloudT::ConstPtr& cloud,
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

  typename GridT::Accessor acc = m_vdb_grid->getAccessor();

  // Creating a temporary grid in which the new data is casted. This way we prevent the computation
  // of redundant probability updates in the actual map
  //typename GridT::Ptr temp_grid     = GridT::create(0.0);
  //typename GridT::Accessor temp_acc = temp_grid->getAccessor();
  typename openvdb::FloatGrid::Ptr temp_grid     = openvdb::FloatGrid::create(0.0);
  typename openvdb::FloatGrid::Accessor temp_acc = temp_grid->getAccessor();

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
  auto miss = [& prob_miss     = m_logodds_miss,
               &prob_thres_min = m_logodds_thres_min](DataNodeT& node, bool& active) {
    node.updateNode(node.getData() + prob_miss);
    if (node.getData() < prob_thres_min)
    {
      active = false;
    }
  };

  // Probability update lambda for occupied grid elements
  auto hit = [& prob_hit = m_logodds_hit, &prob_thres_max = m_logodds_thres_max](DataNodeT& node,
                                                                                 bool& active) {
    node.updateNode(node.getData() + prob_hit);
    if (node.getData() > prob_thres_max)
    {
      active = true;
    }
  };

  // Integrating the data of the temporary grid into the map using the probability update functions
  //for (typename GridT::ValueOnCIter iter = temp_grid->cbeginValueOn(); iter; ++iter)
  for (typename openvdb::FloatGrid::ValueOnCIter iter = temp_grid->cbeginValueOn(); iter; ++iter)
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

