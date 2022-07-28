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
openvdb::BBoxd VDBMapping<DataT, ConfigT>::createWorldBoundingBox(
  const Eigen::Matrix<double, 3, 1> min_boundary,
  const Eigen::Matrix<double, 3, 1> max_boundary,
  const Eigen::Matrix<double, 4, 4> map_to_reference_tf) const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr corners(new pcl::PointCloud<pcl::PointXYZ>());
  corners->points.push_back(pcl::PointXYZ(min_boundary.x(), min_boundary.y(), min_boundary.z()));
  corners->points.push_back(pcl::PointXYZ(min_boundary.x(), min_boundary.y(), max_boundary.z()));
  corners->points.push_back(pcl::PointXYZ(min_boundary.x(), max_boundary.y(), min_boundary.z()));
  corners->points.push_back(pcl::PointXYZ(min_boundary.x(), max_boundary.y(), max_boundary.z()));
  corners->points.push_back(pcl::PointXYZ(max_boundary.x(), min_boundary.y(), min_boundary.z()));
  corners->points.push_back(pcl::PointXYZ(max_boundary.x(), min_boundary.y(), max_boundary.z()));
  corners->points.push_back(pcl::PointXYZ(max_boundary.x(), max_boundary.y(), min_boundary.z()));
  corners->points.push_back(pcl::PointXYZ(max_boundary.x(), max_boundary.y(), max_boundary.z()));
  pcl::transformPointCloud(*corners, *corners, map_to_reference_tf);
  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(*corners, min_pt, max_pt);

  return openvdb::BBoxd(openvdb::Vec3d(min_pt.x, min_pt.y, min_pt.z),
                        openvdb::Vec3d(max_pt.x, max_pt.y, max_pt.z));
}

template <typename DataT, typename ConfigT>
openvdb::CoordBBox VDBMapping<DataT, ConfigT>::createIndexBoundingBox(
  const Eigen::Matrix<double, 3, 1> min_boundary,
  const Eigen::Matrix<double, 3, 1> max_boundary,
  const Eigen::Matrix<double, 4, 4> map_to_reference_tf) const
{
  openvdb::BBoxd world_bb = createWorldBoundingBox(min_boundary, max_boundary, map_to_reference_tf);


  openvdb::Vec3d min_index = m_vdb_grid->worldToIndex(world_bb.min());
  openvdb::Vec3d max_index = m_vdb_grid->worldToIndex(world_bb.max());

  return openvdb::CoordBBox(openvdb::Coord(min_index.x(), min_index.y(), min_index.z()),
                            openvdb::Coord(max_index.x(), max_index.y(), max_index.z()));
}

template <typename DataT, typename ConfigT>
typename VDBMapping<DataT, ConfigT>::UpdateGridT::Ptr
VDBMapping<DataT, ConfigT>::getMapSectionUpdateGrid(
  const Eigen::Matrix<double, 3, 1> min_boundary,
  const Eigen::Matrix<double, 3, 1> max_boundary,
  const Eigen::Matrix<double, 4, 4> map_to_reference_tf) const
{
  return getMapSection<typename VDBMapping<DataT, ConfigT>::UpdateGridT>(
    min_boundary, max_boundary, map_to_reference_tf);
}

template <typename DataT, typename ConfigT>
typename VDBMapping<DataT, ConfigT>::GridT::Ptr VDBMapping<DataT, ConfigT>::getMapSectionGrid(
  const Eigen::Matrix<double, 3, 1> min_boundary,
  const Eigen::Matrix<double, 3, 1> max_boundary,
  const Eigen::Matrix<double, 4, 4> map_to_reference_tf) const
{
  return getMapSection<typename VDBMapping<DataT, ConfigT>::GridT>(
    min_boundary, max_boundary, map_to_reference_tf);
}

template <typename DataT, typename ConfigT>
template <typename ResultGridT>
typename ResultGridT::Ptr VDBMapping<DataT, ConfigT>::getMapSection(
  const Eigen::Matrix<double, 3, 1> min_boundary,
  const Eigen::Matrix<double, 3, 1> max_boundary,
  const Eigen::Matrix<double, 4, 4> map_to_reference_tf) const
{
  typename ResultGridT::Ptr temp_grid = ResultGridT::create(false);
  temp_grid->setTransform(openvdb::math::Transform::createLinearTransform(m_resolution));

  typename ResultGridT::Accessor temp_acc = temp_grid->getAccessor();

  openvdb::CoordBBox bounding_box(
    createIndexBoundingBox(min_boundary, max_boundary, map_to_reference_tf));

  for (auto iter = m_vdb_grid->cbeginValueOn(); iter; ++iter)
  {
    if (bounding_box.isInside(iter.getCoord()))
    {
      temp_acc.setValueOn(iter.getCoord(), true);
    }
  }

  return temp_grid;
}

template <typename DataT, typename ConfigT>
[[deprecated]] bool
VDBMapping<DataT, ConfigT>::insertPointCloud(const PointCloudT::ConstPtr& cloud,
                                             const Eigen::Matrix<double, 3, 1>& origin)
{
  UpdateGridT::Ptr update_grid;
  UpdateGridT::Ptr overwrite_grid;

  return insertPointCloud(cloud, origin, update_grid, overwrite_grid, false);
}

template <typename DataT, typename ConfigT>
bool VDBMapping<DataT, ConfigT>::insertPointCloud(const PointCloudT::ConstPtr& cloud,
                                                  const Eigen::Matrix<double, 3, 1>& origin,
                                                  UpdateGridT::Ptr& update_grid,
                                                  UpdateGridT::Ptr& overwrite_grid,
                                                  const bool reduce_data)
{
  UpdateGridT::Ptr raycast_update_grid;

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
    ray_end_world      = openvdb::Vec3d(pt.x, pt.y, pt.z);
    bool max_range_ray = false;

    if (m_max_range > 0.0 && (ray_end_world - ray_origin_world).length() > m_max_range)
    {
      ray_end_world = ray_origin_world + (ray_end_world - ray_origin_world).unit() * m_max_range;
      max_range_ray = true;
    }

    openvdb::Coord ray_end_index =
      castRayIntoGrid(ray_origin_world, ray_origin_index, ray_end_world, temp_acc);

    if (!max_range_ray)
    {
      temp_acc.setValueOn(ray_end_index, true);
    }
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
    bool max_range_ray = false;
    if (m_max_range > 0.0 && (end_world - origin_world).length() > m_max_range)
    {
      end_world     = origin_world + (end_world - origin_world).unit() * m_max_range;
      max_range_ray = true;
    }
    openvdb::Vec3d index_buffer = m_vdb_grid->worldToIndex(end_world);
    openvdb::Coord end_index(index_buffer.x(), index_buffer.y(), index_buffer.z());
    temp_acc.setValueOn(end_index, !max_range_ray);
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
    openvdb::Coord ray_end_index =
      castRayIntoGrid(ray_origin_world, ray_origin_index, ray_end_world, temp_acc);

    if (iter.getValue())
    {
      temp_acc.setValueOn(ray_end_index, true);
    }
  }
  return temp_grid;
}


template <typename DataT, typename ConfigT>
openvdb::Coord
VDBMapping<DataT, ConfigT>::castRayIntoGrid(const openvdb::Vec3d& ray_origin_world,
                                            const Vec3T& ray_origin_index,
                                            const openvdb::Vec3d& ray_end_world,
                                            UpdateGridT::Accessor& update_grid_acc) const
{
  openvdb::Vec3d ray_direction = (ray_end_world - ray_origin_world);

  const RayT ray(ray_origin_index, ray_direction);
  DDAT dda(ray);

  openvdb::Coord ray_end_index = openvdb::Coord::floor(m_vdb_grid->worldToIndex(ray_end_world));

  while (dda.voxel() != ray_end_index)
  {
    update_grid_acc.setActiveState(dda.voxel(), true);
    dda.step();
  }
  return dda.voxel();
}

template <typename DataT, typename ConfigT>
bool VDBMapping<DataT, ConfigT>::raytrace(const openvdb::Vec3d& ray_origin_world,
                                          const openvdb::Vec3d& ray_direction,
                                          const double max_ray_length,
                                          openvdb::Vec3d& trace)
{
  typename GridT::Accessor acc    = m_vdb_grid->getAccessor();
  openvdb::Vec3d ray_origin_index = m_vdb_grid->worldToIndex(ray_origin_world);
  const RayT ray(ray_origin_index, ray_direction);
  DDAT dda(ray);
  while (true)
  {
    const double distance =
      m_resolution * std::sqrt(std::pow((dda.voxel().x() - ray_origin_index.x()), 2) +
                               std::pow((dda.voxel().y() - ray_origin_index.y()), 2) +
                               std::pow((dda.voxel().z() - ray_origin_index.z()), 2));
    if (distance < max_ray_length)
    {
      if (acc.isValueOn(dda.voxel()))
      {
        trace.x() = dda.voxel().x();
        trace.y() = dda.voxel().y();
        trace.z() = dda.voxel().z();
        trace = m_vdb_grid->indexToWorld(trace);
        return true;
      }
      else
      {
        dda.step();
      }
    }
    else
    {
      return false;
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
