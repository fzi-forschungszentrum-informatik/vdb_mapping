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
 * \date    2020-12-23
 *
 */
//----------------------------------------------------------------------


#include <iostream>

template <typename DataT, typename ConfigT>
VDBMapping<DataT, ConfigT>::VDBMapping(const double resolution)
  : m_resolution(resolution)
  , m_config_set(false)
{
  // Initialize Grid
  openvdb::initialize();
  if (!GridT::isRegistered())
  {
    GridT::registerGrid();
  }
  m_vdb_grid = createVDBMap(m_resolution);
}

template <typename DataT, typename ConfigT>
void VDBMapping<DataT, ConfigT>::resetMap()
{
  m_vdb_grid->clear();
  m_vdb_grid = createVDBMap(m_resolution);
}


template <typename DataT, typename ConfigT>
bool VDBMapping<DataT, ConfigT>::saveMap()
{
  auto timestamp     = std::chrono::system_clock::now();
  std::time_t now_tt = std::chrono::system_clock::to_time_t(timestamp);
  std::tm tm         = *std::localtime(&now_tt);
  std::stringstream sstime;
  sstime << std::put_time(&tm, "%Y-%m-%d_%H:%M:%S");

  std::string map_name = m_map_directory_path + sstime.str() + "_map.vdb";
  std::cout << map_name << std::endl;
  openvdb::io::File fileHandle(map_name);
  openvdb::GridPtrVec grids;
  grids.push_back(m_vdb_grid);
  fileHandle.write(grids);
  fileHandle.close();

  return true;
}

template <typename DataT, typename ConfigT>
bool VDBMapping<DataT, ConfigT>::loadMap(std::string file_path)
{
  openvdb::io::File fileHandle(file_path);
  fileHandle.open();
  openvdb::GridBase::Ptr baseGrid;
  for (openvdb::io::File::NameIterator nameIter = fileHandle.beginName();
       nameIter != fileHandle.endName();
       ++nameIter)
  {
    baseGrid = fileHandle.readGrid(nameIter.gridName());
    m_vdb_grid->clear();
    m_vdb_grid = openvdb::gridPtrCast<GridT>(baseGrid);
  }
  fileHandle.close();


  return true;
}

template <typename DataT, typename ConfigT>
typename VDBMapping<DataT, ConfigT>::GridT::Ptr
VDBMapping<DataT, ConfigT>::createVDBMap(double resolution)
{
  typename GridT::Ptr new_map = GridT::create(DataT());
  new_map->setTransform(openvdb::math::Transform::createLinearTransform(m_resolution));
  new_map->setGridClass(openvdb::GRID_LEVEL_SET);
  return new_map;
}

template <typename DataT, typename ConfigT>
bool VDBMapping<DataT, ConfigT>::insertPointCloud(const PointCloudT::ConstPtr& cloud,
                                                  const Eigen::Matrix<double, 3, 1>& origin)
{
  return updateMap(createUpdate(cloud, origin));
}

template <typename DataT, typename ConfigT>
bool VDBMapping<DataT, ConfigT>::insertPointCloud(const PointCloudT::ConstPtr& cloud,
                                                  const Eigen::Matrix<double, 3, 1>& origin,
                                                  typename GridT::Ptr& update_grid)
{
  update_grid = createUpdate(cloud, origin);
  return updateMap(update_grid);
}

template <typename DataT, typename ConfigT>
openvdb::FloatGrid::Ptr
VDBMapping<DataT, ConfigT>::createUpdate(const PointCloudT::ConstPtr& cloud,
                                         const Eigen::Matrix<double, 3, 1>& origin) const
{
  // Creating a temporary grid in which the new data is casted. This way we prevent the computation
  // of redundant probability updates in the actual map
  openvdb::FloatGrid::Ptr temp_grid     = openvdb::FloatGrid::create(0.0);
  openvdb::FloatGrid::Accessor temp_acc = temp_grid->getAccessor();

  // Check if a valid configuration was loaded
  if (!m_config_set)
  {
    std::cerr << "Map not properly configured. Did you call setConfig method?" << std::endl;
    return temp_grid;
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
    while (signed_distance >= 0)
    {
      x = openvdb::Vec3d(dda.voxel().x(), dda.voxel().y(), dda.voxel().z()) - ray_origin_index;
      // Signed distance in grid coordinates for faster processing(not scaled with the grid
      // resolution!!!)
      // The main idea of the dot product is to first get the center of the current voxel and then
      // add half the ray direction to gain the outer boundary of this voxel
      signed_distance = ray_length - ray_direction.dot(x + 0.5 + ray_direction / 2.0);
      if (signed_distance >= 0)
      {
        temp_acc.setActiveState(dda.voxel(), true);
        dda.step();
      }
      else
      {
        // Set the last passed voxel as occupied if the ray wasn't longer than the maximum raycast
        // range
        if (!max_range_ray)
        {
          temp_acc.setValueOn(dda.voxel(), -1);
        }
      }
    }
  }
  return temp_grid;
}

template <typename DataT, typename ConfigT>
bool VDBMapping<DataT, ConfigT>::updateMap(const openvdb::FloatGrid::Ptr& temp_grid)
{
  if (temp_grid->empty())
  {
    std::cout << "Update grid is empty. Cannot integrate it into the map." << std::endl;
    return false;
  }

  typename GridT::Accessor acc = m_vdb_grid->getAccessor();
  // Probability update lambda for free space grid elements
  auto miss = [this](DataT& voxel_value, bool& active) { updateFreeNode(voxel_value, active); };

  // Probability update lambda for occupied grid elements
  auto hit = [this](DataT& voxel_value, bool& active) { updateOccupiedNode(voxel_value, active); };

  // Integrating the data of the temporary grid into the map using the probability update functions
  for (openvdb::FloatGrid::ValueOnCIter iter = temp_grid->cbeginValueOn(); iter; ++iter)
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

template <typename DataT, typename ConfigT>
void VDBMapping<DataT, ConfigT>::setConfig(const ConfigT& config)
{
  if (config.max_range < 0.0)
  {
    std::cerr << "Max range of " << config.max_range << " invalid. Range cannot be negative."
              << std::endl;
    return;
  }
  m_max_range          = config.max_range;
  m_map_directory_path = config.map_directory_path;
  m_config_set         = true;
}
