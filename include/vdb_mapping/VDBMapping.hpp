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
  if (!UpdateGridT::isRegistered())
  {
    UpdateGridT::registerGrid();
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
bool VDBMapping<DataT, ConfigT>::saveMap() const
{
  auto timestamp     = std::chrono::system_clock::now();
  std::time_t now_tt = std::chrono::system_clock::to_time_t(timestamp);
  std::tm tm         = *std::localtime(&now_tt);
  std::stringstream sstime;
  sstime << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");

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
bool VDBMapping<DataT, ConfigT>::loadMap(const std::string file_path)
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
typename VDBMapping<DataT, ConfigT>::UpdateGridT::Ptr
VDBMapping<DataT, ConfigT>::getMapSection(const double min_x,
                                          const double min_y,
                                          const double min_z,
                                          const double max_x,
                                          const double max_y,
                                          const double max_z) const
{
  typename UpdateGridT::Ptr temp_grid     = UpdateGridT::create(false);
  typename UpdateGridT::Accessor temp_acc = temp_grid->getAccessor();
  typename GridT::Accessor acc            = m_vdb_grid->getAccessor();

  openvdb::Vec3d world_min_pt(min_x, min_y, min_z);
  openvdb::Vec3d world_max_pt(max_x, max_y, max_z);
  openvdb::Vec3d index_min_pt = m_vdb_grid->worldToIndex(world_min_pt);
  openvdb::Vec3d index_max_pt = m_vdb_grid->worldToIndex(world_max_pt);


  openvdb::CoordBBox bouding_box(index_min_pt.x(),
                                 index_min_pt.y(),
                                 index_min_pt.z(),
                                 index_max_pt.x(),
                                 index_max_pt.y(),
                                 index_max_pt.z());

  for(auto iter = m_vdb_grid->cbeginValueOn(); iter; ++iter)
  {
    if(bouding_box.isInside(iter.getCoord()))
    {
      temp_acc.setValueOn(iter.getCoord(), true);
    }
  }

  return temp_grid;
}

template <typename DataT, typename ConfigT>
bool VDBMapping<DataT, ConfigT>::insertPointCloud(const PointCloudT::ConstPtr& cloud,
                                                  const Eigen::Matrix<double, 3, 1>& origin)
{
  //return updateMap(createUpdate(cloud, origin));
  updateMap(createUpdate(cloud, origin));
  return true;
}

template <typename DataT, typename ConfigT>
bool VDBMapping<DataT, ConfigT>::insertPointCloud(const PointCloudT::ConstPtr& cloud,
                                                  const Eigen::Matrix<double, 3, 1>& origin,
                                                  UpdateGridT::Ptr& update_grid,
                                                  UpdateGridT::Ptr& overwrite_grid,
                                                  const bool reduce_data)
{
  UpdateGridT::Ptr raycast_update_grid;

  // TODO change into compression on/off
  // and send the overwrite either way


  if (!reduce_data)
  {
    update_grid    = raycastPointCloud(cloud, origin);
    overwrite_grid = updateMap(update_grid);
  }
  else
  {
    update_grid         = pointCloudToUpdateGrid(cloud, origin);
    raycast_update_grid = raycastUpdateGrid(update_grid);
    overwrite_grid      = updateMap(raycast_update_grid);
  }
  return true;
}

template <typename DataT, typename ConfigT>
VDBMapping<DataT, ConfigT>::UpdateGridT::Ptr
VDBMapping<DataT, ConfigT>::createUpdate(const PointCloudT::ConstPtr& cloud,
                                         const Eigen::Matrix<double, 3, 1>& origin) const
{
  // Creating a temporary grid in which the new data is casted. This way we prevent the computation
  // of redundant probability updates in the actual map
  UpdateGridT::Ptr temp_grid     = UpdateGridT::create(false);
  UpdateGridT::Accessor temp_acc = temp_grid->getAccessor();

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
          temp_acc.setValueOn(dda.voxel(), true);
        }
      }
    }
  }
  return temp_grid;
}


template <typename DataT, typename ConfigT>
VDBMapping<DataT, ConfigT>::UpdateGridT::Ptr
VDBMapping<DataT, ConfigT>::raycastPointCloud(const PointCloudT::ConstPtr& cloud,
                                              const Eigen::Matrix<double, 3, 1>& origin) const
{
  // Creating a temporary grid in which the new data is casted. This way we prevent the computation
  // of redundant probability updates in the actual map
  UpdateGridT::Ptr temp_grid     = UpdateGridT::create(false);
  UpdateGridT::Accessor temp_acc = temp_grid->getAccessor();

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
  Vec3T ray_origin_index(m_vdb_grid->worldToIndex(ray_origin_world));
  // Ray end point in world coordinates
  openvdb::Vec3d ray_end_world;

  // Raycasting of every point in the input cloud
  for (const PointT& pt : *cloud)
  {
    ray_end_world = openvdb::Vec3d(pt.x, pt.y, pt.z);
    castRayIntoGrid(ray_origin_world, ray_origin_index, ray_end_world, temp_acc);
  }
  return temp_grid;
}

template <typename DataT, typename ConfigT>
VDBMapping<DataT, ConfigT>::UpdateGridT::Ptr
VDBMapping<DataT, ConfigT>::pointCloudToUpdateGrid(const PointCloudT::ConstPtr& cloud,
                                                   const Eigen::Matrix<double, 3, 1>& origin) const
{
  UpdateGridT::Ptr temp_grid     = UpdateGridT::create(false);
  UpdateGridT::Accessor temp_acc = temp_grid->getAccessor();
  if (!m_config_set)
  {
    std::cerr << "Map not properly configured. Did you call setConfig method?" << std::endl;
    return temp_grid;
  }

  openvdb::Vec3d origin_world(origin.x(), origin.y(), origin.z());
  for (const PointT& pt : *cloud)
  {
    openvdb::Vec3d end_world(pt.x, pt.y, pt.z);
    if (m_max_range > 0.0 && (end_world - origin_world).length() > m_max_range)
    {
      end_world =
        origin_world + (end_world - origin_world).unit() * (m_max_range + 2 * m_resolution);
    }
    openvdb::Vec3d index_buffer = m_vdb_grid->worldToIndex(end_world);
    openvdb::Coord end_index(index_buffer.x(), index_buffer.y(), index_buffer.z());
    temp_acc.setValueOn(end_index, true);
  }

  temp_grid->insertMeta("origin", openvdb::Vec3DMetadata(origin_world));
  return temp_grid;
}

template <typename DataT, typename ConfigT>
VDBMapping<DataT, ConfigT>::UpdateGridT::Ptr
VDBMapping<DataT, ConfigT>::raycastUpdateGrid(const UpdateGridT::Ptr& grid) const
{
  UpdateGridT::Ptr temp_grid     = UpdateGridT::create(false);
  UpdateGridT::Accessor temp_acc = temp_grid->getAccessor();
  // Check if a valid configuration was loaded
  if (!m_config_set)
  {
    std::cerr << "Map not properly configured. Did you call setConfig method?" << std::endl;
    return temp_grid;
  }

  RayT ray;
  DDAT dda;

  // Ray origin in world coordinates
  openvdb::Vec3d ray_origin_world = grid->metaValue<openvdb::Vec3d>("origin");
  // Ray origin in index coordinates
  Vec3T ray_origin_index(m_vdb_grid->worldToIndex(ray_origin_world));

  // Ray end point in world coordinates
  openvdb::Vec3d ray_end_world;

  for (UpdateGridT::ValueOnCIter iter = grid->cbeginValueOn(); iter; ++iter)
  {
    ray_end_world = m_vdb_grid->indexToWorld(iter.getCoord());
    castRayIntoGrid(ray_origin_world, ray_origin_index, ray_end_world, temp_acc);
  }
  return temp_grid;
}


template <typename DataT, typename ConfigT>
void VDBMapping<DataT, ConfigT>::castRayIntoGrid(openvdb::Vec3d& ray_origin_world,
                                                 Vec3T& ray_origin_index,
                                                 openvdb::Vec3d& ray_end_world,
                                                 UpdateGridT::Accessor& update_grid_acc) const
{
  RayT ray;
  DDAT dda;
  // Direction the ray is point towards
  openvdb::Vec3d ray_direction;

  // TODO rename
  openvdb::Vec3d x;
  double ray_length;
  bool max_range_ray = false;

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
      update_grid_acc.setActiveState(dda.voxel(), true);
      dda.step();
    }
    else
    {
      // Set the last passed voxel as occupied if the ray wasn't longer than the maximum raycast
      // range
      if (!max_range_ray)
      {
        update_grid_acc.setValueOn(dda.voxel(), true);
      }
    }
  }
}

template <typename DataT, typename ConfigT>
VDBMapping<DataT, ConfigT>::UpdateGridT::Ptr
VDBMapping<DataT, ConfigT>::updateMap(const UpdateGridT::Ptr& temp_grid)
{
  UpdateGridT::Ptr change          = UpdateGridT::create(false);
  UpdateGridT::Accessor change_acc = change->getAccessor();
  if (temp_grid->empty())
  {
    std::cout << "Update grid is empty. Cannot integrate it into the map." << std::endl;
    return change;
  }

  bool state_changed           = false;
  typename GridT::Accessor acc = m_vdb_grid->getAccessor();
  // Probability update lambda for free space grid elements
  auto miss = [&](DataT& voxel_value, bool& active) {
    bool last_state = active;
    updateFreeNode(voxel_value, active);
    if (last_state != active)
    {
      state_changed = true;
    }
  };

  // Probability update lambda for occupied grid elements
  auto hit = [&](DataT& voxel_value, bool& active) {
    bool last_state = active;
    updateOccupiedNode(voxel_value, active);
    if (last_state != active)
    {
      state_changed = true;
    }
  };

  // Integrating the data of the temporary grid into the map using the probability update functions
  for (UpdateGridT::ValueOnCIter iter = temp_grid->cbeginValueOn(); iter; ++iter)
  {
    state_changed = false;
    if (*iter == true)
    {
      acc.modifyValueAndActiveState(iter.getCoord(), hit);
      if (state_changed)
      {
        change_acc.setValueOn(iter.getCoord(), true);
      }
    }
    else
    {
      acc.modifyValueAndActiveState(iter.getCoord(), miss);
      if (state_changed)
      {
        change_acc.setActiveState(iter.getCoord(), true);
      }
    }
  }
  return change;
}

template <typename DataT, typename ConfigT>
void VDBMapping<DataT, ConfigT>::overwriteMap(const UpdateGridT::Ptr& update_grid)
{
  typename GridT::Accessor acc = m_vdb_grid->getAccessor();
  for (UpdateGridT::ValueOnCIter iter = update_grid->cbeginValueOn(); iter; ++iter)
  {
    if (*iter == true)
    {
      acc.setActiveState(iter.getCoord(), true);
    }
    else
    {
      acc.setActiveState(iter.getCoord(), false);
    }
  }
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
