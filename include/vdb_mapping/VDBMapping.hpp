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
#ifndef VDB_MAPPING_VDB_MAPPING_H_INCLUDED
#define VDB_MAPPING_VDB_MAPPING_H_INCLUDED


#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <chrono>
#include <eigen3/Eigen/Geometry>
#include <iostream>

#include <openvdb/Types.h>
#include <openvdb/io/Stream.h>
#include <openvdb/math/DDA.h>
#include <openvdb/math/Ray.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/Clip.h>
#include <openvdb/tools/Morphology.h>

#include <zstd.h>

namespace vdb_mapping {


/*!
 * \brief Accumulation of configuration parameters
 */
struct BaseConfig
{
  double max_range;
  std::string map_directory_path;
};
/*!
 * \brief Main Mapping class which handles all data integration
 */
template <typename TData, typename TConfig = BaseConfig>
class VDBMapping
{
public:
  using PointT      = pcl::PointXYZ;
  using PointCloudT = pcl::PointCloud<PointT>;

  using RayT  = openvdb::math::Ray<double>;
  using Vec3T = RayT::Vec3Type;
  using DDAT  = openvdb::math::DDA<RayT, 0>;

  using GridT       = openvdb::Grid<typename openvdb::tree::Tree4<TData, 5, 4, 3>::Type>;
  using UpdateGridT = openvdb::Grid<openvdb::tree::Tree4<bool, 1, 4, 3>::Type>;


  VDBMapping()                  = delete;
  VDBMapping(const VDBMapping&) = delete;

  VDBMapping& operator=(const VDBMapping&) = delete;

  /*!
   * \brief Construktur creates a new VDBMapping objekt with parametrizable grid resolution
   *
   * \param resolution Resolution of the VDB Grid
   */
  VDBMapping(const double resolution)
    : m_resolution(resolution)
    , m_config_set(false)
    , m_artificial_areas_present(false)
  {
    m_map_mutex                    = std::make_shared<std::shared_mutex>();
    m_update_grid_mutex            = std::make_shared<std::shared_mutex>();
    m_consumable_update_grid_mutex = std::make_shared<std::shared_mutex>();
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
    m_vdb_grid               = createVDBMap(m_resolution);
    m_update_grid            = UpdateGridT::create(false);
    m_consumable_update_grid = UpdateGridT::create(false);
    m_artificial_area_grid   = UpdateGridT::create(false);
  }
  virtual ~VDBMapping() = default;

  /*!
   * \brief Creates a new VDB Grid
   *
   * \param resolution Resolution of the grid
   *
   * \returns Grid shared pointer
   */
  typename GridT::Ptr createVDBMap(double resolution)
  {
    typename GridT::Ptr new_map = GridT::create(TData());
    new_map->setTransform(openvdb::math::Transform::createLinearTransform(m_resolution));
    new_map->setGridClass(openvdb::GRID_LEVEL_SET);
    return new_map;
  }

  /*!
   * \brief Reset the current map
   */
  void resetMap()
  {
    std::unique_lock map_lock(*m_map_mutex);
    m_vdb_grid->clear();
    m_vdb_grid = createVDBMap(m_resolution);
    map_lock.unlock();
    std::unique_lock update_grid_lock(*m_update_grid_mutex);
    m_update_grid = UpdateGridT::create(false);
    update_grid_lock.unlock();
    std::unique_lock consumable_update_grid_lock(*m_consumable_update_grid_mutex);
    m_consumable_update_grid = UpdateGridT::create(false);
  }

  /*!
   * \brief Saves the current map
   */
  bool saveMap() const
  {
    auto timestamp     = std::chrono::system_clock::now();
    std::time_t now_tt = std::chrono::system_clock::to_time_t(timestamp);
    std::tm tm         = *std::localtime(&now_tt);
    std::stringstream sstime;
    sstime << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");

    std::string map_name = m_map_directory_path + sstime.str() + "_map.vdb";
    std::cout << map_name << std::endl;
    openvdb::io::File file_handle(map_name);
    openvdb::GridPtrVec grids;
    std::shared_lock map_lock(*m_map_mutex);
    grids.push_back(m_vdb_grid);
    file_handle.write(grids);
    file_handle.close();
    map_lock.unlock();
    return true;
  }

  /*!
   * \brief Saves the active values of the current map as PCD file
   *
   * \returns Saving pcd successfull
   */
  bool saveMapToPCD()
  {
    auto timestamp     = std::chrono::system_clock::now();
    std::time_t now_tt = std::chrono::system_clock::to_time_t(timestamp);
    std::tm tm         = *std::localtime(&now_tt);
    std::stringstream sstime;
    sstime << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    std::string pcd_path = m_map_directory_path + sstime.str() + "_active_values_map.pcd";

    PointCloudT::Ptr cloud(new PointCloudT);

    std::shared_lock map_lock(*m_map_mutex);
    cloud->points.reserve(m_vdb_grid->activeVoxelCount());

    for (typename GridT::ValueOnCIter iter = m_vdb_grid->cbeginValueOn(); iter; ++iter)
    {
      openvdb::Vec3d world_coord = m_vdb_grid->indexToWorld(iter.getCoord());

      world_coord += 0.5 * m_resolution;
      PointT point;
      point.x = static_cast<float>(world_coord.x());
      point.y = static_cast<float>(world_coord.y());
      point.z = static_cast<float>(world_coord.z());
      cloud->points.push_back(point);
    }
    map_lock.unlock();

    cloud->points.shrink_to_fit();
    cloud->width  = cloud->points.size();
    cloud->height = 1;

    if (pcl::io::savePCDFile(pcd_path, *cloud) != 0)
    {
      PCL_ERROR("Could not write PCD file.");
      return false;
    }
    std::cout << "Wrote pcd to: " << pcd_path << std::endl;
    return true;
  }

  /*!
   * \brief Loads a stored map
   */
  bool loadMap(const std::string& file_path)
  {
    openvdb::io::File file_handle(file_path);
    file_handle.open();
    openvdb::GridBase::Ptr base_grid;

    std::unique_lock map_lock(*m_map_mutex);
    for (openvdb::io::File::NameIterator name_iter = file_handle.beginName();
         name_iter != file_handle.endName();
         ++name_iter)
    {
      base_grid = file_handle.readGrid(name_iter.gridName());
      m_vdb_grid->clear();
      m_vdb_grid = openvdb::gridPtrCast<GridT>(base_grid);
    }
    file_handle.close();
    map_lock.unlock();

    return true;
  }

  /*!
   * \brief Loads a stored map from a pcd file
   *
   * \param file_path Path to pcd file
   * \param set_background Specifies if the background should be set
   * \param clear_map Specifies if the map has to be cleared before inserting data
   *
   * \returns Loading of map successfull
   */
  bool loadMapFromPCD(const std::string& file_path, const bool set_background, const bool clear_map)
  {
    PointCloudT::Ptr cloud(new PointCloudT);
    if (pcl::io::loadPCDFile<PointT>(file_path, *cloud) == -1)
    {
      PCL_ERROR("Could not open PCD file");
      return false;
    }
    std::unique_lock map_lock(*m_map_mutex);
    createMapFromPointCloud(cloud, set_background, clear_map);
    map_lock.unlock();
    return true;
  }

  /*!
   * \brief Accumulates a new sensor point cloud to the update grid
   *
   * \param cloud Input cloud in map coordinates
   * \param origin Sensor position in map coordinates
   * \param max_range Maximum raycasting range of this measurement
   */
  void accumulateUpdate(const PointCloudT::ConstPtr& cloud,
                        const Eigen::Matrix<double, 3, 1>& origin,
                        const double& max_range)
  {
    std::unique_lock update_grid_lock(*m_update_grid_mutex);
    UpdateGridT::Accessor update_grid_acc = m_update_grid->getAccessor();
    if (max_range > 0)
    {
      raycastPointCloud(cloud, origin, max_range, update_grid_acc);
    }
    else
    {
      raycastPointCloud(cloud, origin, update_grid_acc);
    }
    update_grid_lock.unlock();
  }

  /*!
   * \brief Integrates the accumulated updates into the map
   *
   * \param update_grid Update grid
   * \param overwrite_grid Overwrite grid
   */
  void integrateUpdate(UpdateGridT::Ptr& update_grid, UpdateGridT::Ptr& overwrite_grid)
  {
    std::shared_lock update_grid_lock(*m_update_grid_mutex);
    std::unique_lock consumable_update_grid_lock(*m_consumable_update_grid_mutex);
    m_consumable_update_grid = m_update_grid;
    update_grid_lock.unlock();
    resetUpdate();
    overwrite_grid = updateMap(m_consumable_update_grid);
    update_grid    = m_consumable_update_grid;
    consumable_update_grid_lock.unlock();
  }

  /*!
   * \brief Resets the updates grid
   */
  void resetUpdate()
  {
    std::unique_lock update_grid_lock(*m_update_grid_mutex);
    m_update_grid = UpdateGridT::create(false);
    // m_update_grid.reset(UpdateGridT::create(false));
    update_grid_lock.unlock();
  }

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
                        const Eigen::Matrix<double, 3, 1>& origin)
  {
    UpdateGridT::Ptr update_grid;
    UpdateGridT::Ptr overwrite_grid;

    return insertPointCloud(cloud, origin, update_grid, overwrite_grid);
  }

  /*!
   * \brief Handles the integration of new PointCloud data into the VDB data structure.
   * All datapoints are raycasted starting from the origin position
   *
   * \param cloud Input cloud in map coordinates
   * \param origin Sensor position in map coordinates
   * \param update_grid Update grid that was created internally while mapping
   * \param overwrite_grid Overwrite grid containing all changed voxel indices
   *
   * \returns Was the insertion of the new pointcloud successful
   */
  bool insertPointCloud(const PointCloudT::ConstPtr& cloud,
                        const Eigen::Matrix<double, 3, 1>& origin,
                        UpdateGridT::Ptr& update_grid,
                        UpdateGridT::Ptr& overwrite_grid)
  {
    accumulateUpdate(cloud, origin, m_max_range);
    integrateUpdate(update_grid, overwrite_grid);
    resetUpdate();
    return true;
  }

  bool removePointsFromGrid(const PointCloudT::ConstPtr& cloud)
  {
    typename GridT::Accessor acc = m_vdb_grid->getAccessor();

    auto set_node = [&](TData& voxel_value, bool& active) { setNodeToFree(voxel_value, active); };


    for (const PointT& pt : *cloud)
    {
      openvdb::Vec3d world_pt(pt.x, pt.y, pt.z);
      openvdb::Coord index_pt = openvdb::Coord::floor(m_vdb_grid->worldToIndex(world_pt));
      acc.modifyValueAndActiveState(index_pt, set_node);
    }
    return true;
  }

  bool addPointsToGrid(const PointCloudT::ConstPtr& cloud)
  {
    typename GridT::Accessor acc = m_vdb_grid->getAccessor();
    auto set_node                = [&](TData& voxel_value, bool& active) {
      setNodeToOccupied(voxel_value, active);
    };
    for (const PointT& pt : *cloud)
    {
      openvdb::Vec3d world_pt(pt.x, pt.y, pt.z);
      openvdb::Coord index_pt = openvdb::Coord::floor(m_vdb_grid->worldToIndex(world_pt));
      acc.modifyValueAndActiveState(index_pt, set_node);
    }
    return true;
  }

  /*!
   * \brief  Raycasts a Pointcloud into an update Grid
   *
   * For each point in the input pointcloud, a raycast is performed from the origin.
   * All cells along these rays are marked as active.
   *
   * All points are clipped according to the config's max_range parameter. If a point is within
   * this range, its corresponding cell value in the update grid is set to true and will be handled
   * as a sensor hit in the map update.
   *
   * \param cloud Input sensor point cloud
   * \param origin Origin of the sensor measurement
   * \param update_grid_acc Accessor to the grid in which the raycasting takes place
   *
   * \returns Raycasted update grid
   */
  bool raycastPointCloud(const PointCloudT::ConstPtr& cloud,
                         const Eigen::Matrix<double, 3, 1>& origin,
                         UpdateGridT::Accessor& update_grid_acc)
  {
    return raycastPointCloud(cloud, origin, m_max_range, update_grid_acc);
  }

  /*!
   * \brief  Raycasts a Pointcloud into an update Grid
   * For each point in the input pointcloud, a raycast is performed from the origin.
   * All cells along these rays are marked as active.
   *
   * All points are clipped according to the config's max_range parameter. If a point is within
   * this range, its corresponding cell value in the update grid is set to true and will be handled
   * as a sensor hit in the map update.
   *
   * \param cloud Input sensor point cloud
   * \param origin Origin of the sensor measurement
   * \param raycast_range Maximum raycasting range
   * \param update_grid_acc Accessor to the grid in which the raycasting takes place
   *
   * \returns Raycasted update grid
   */
  bool raycastPointCloud(const PointCloudT::ConstPtr& cloud,
                         const Eigen::Matrix<double, 3, 1>& origin,
                         const double raycast_range,
                         UpdateGridT::Accessor& update_grid_acc)
  {
    // Creating a temporary grid in which the new data is casted. This way we prevent the
    // computation of redundant probability updates in the actual map

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
    openvdb::Coord ray_origin_index = this->worldToIndex(ray_origin_world);
    // Ray end point in world coordinates
    openvdb::Vec3d ray_end_world;

    // Raycasting of every point in the input cloud
    for (const PointT& pt : *cloud)
    {
      ray_end_world      = openvdb::Vec3d(pt.x, pt.y, pt.z);
      bool max_range_ray = false;


      if (std::isnan(ray_end_world.x()) || std::isnan(ray_end_world.y()) ||
          std::isnan(ray_end_world.z()) || std::isnan(ray_origin_world.x()) ||
          std::isnan(ray_origin_world.y()) || std::isnan(ray_origin_world.z()))
      {
        continue;
      }

      if (raycast_range > 0.0 && (ray_end_world - ray_origin_world).length() > raycast_range)
      {
        ray_end_world =
          ray_origin_world + (ray_end_world - ray_origin_world).unit() * raycast_range;
        max_range_ray = true;
      }

      openvdb::Coord ray_end_index = this->worldToIndex(ray_end_world);
      castRayIntoGrid(ray_origin_index, ray_end_index, update_grid_acc);

      if (!max_range_ray)
      {
        update_grid_acc.setValueOn(ray_end_index, true);
      }
    }
    return true;
  }

  /*!
   * \brief Casts a single ray into an update grid structure
   *
   * Each cell along the ray is marked as active.
   *
   * \param ray_origin_world Ray origin in world coordinates
   * \param ray_origin_index Ray origin in index coordinates
   * \param ray_end_world Ray endpoint in world coordinates
   * \param update_grid_acc Accessor to the update grid
   *
   * \returns Final visited index coordinate
   */
  void castRayIntoGrid(const openvdb::Coord& ray_origin_index,
                       const openvdb::Coord& ray_end_index,
                       UpdateGridT::Accessor& update_grid_acc) const
  {
    openvdb::Vec3d ray_direction = (ray_end_index.asVec3d() - ray_origin_index);

    // Starting the ray from the center of the voxel
    RayT ray(ray_origin_index.asVec3d() + 0.5, ray_direction, 0, 1);
    DDAT dda(ray, 0);
    if (ray_end_index != ray_origin_index)
    {
      do
      {
        update_grid_acc.setActiveState(dda.voxel(), true);
      } while (dda.step());
    }
  }

  openvdb::Coord worldToIndex(const openvdb::Vec3d& world_coordinate) const
  {
    openvdb::Vec3d adjusted_world_coordinate = world_coordinate;
    if (std::fmod(adjusted_world_coordinate.x(), m_resolution))
    {
      adjusted_world_coordinate.x() = adjusted_world_coordinate.x() + (m_resolution / 2.0);
    }
    if (std::fmod(adjusted_world_coordinate.y(), m_resolution))
    {
      adjusted_world_coordinate.y() = adjusted_world_coordinate.y() + (m_resolution / 2.0);
    }
    if (std::fmod(adjusted_world_coordinate.z(), m_resolution))
    {
      adjusted_world_coordinate.z() = adjusted_world_coordinate.z() + (m_resolution / 2.0);
    }
    openvdb::Coord index_coordinate =
      openvdb::Coord::floor(m_vdb_grid->worldToIndex(adjusted_world_coordinate));

    return index_coordinate;
  }

  bool raytrace(const openvdb::Vec3d& ray_origin_world,
                const openvdb::Vec3d& ray_direction,
                const double max_ray_length,
                openvdb::Vec3d& end_point)
  {
    typename GridT::Accessor acc    = m_vdb_grid->getAccessor();
    openvdb::Vec3d ray_origin_index = m_vdb_grid->worldToIndex(ray_origin_world);
    RayT ray(ray_origin_index, ray_direction);
    DDAT dda(ray);
    while (true)
    {
      double distance =
        m_resolution * std::sqrt(std::pow((dda.voxel().x() - ray_origin_index.x()), 2) +
                                 std::pow((dda.voxel().y() - ray_origin_index.y()), 2) +
                                 std::pow((dda.voxel().z() - ray_origin_index.z()), 2));
      if (distance < max_ray_length)
      {
        if (acc.isValueOn(dda.voxel()))
        {
          end_point = m_vdb_grid->indexToWorld(dda.voxel());
          return true;
        }
        dda.step();
      }
      else
      {
        return false;
      }
    }
  }

  /*!
   * \brief Overwrites the active states of a map given an update grid
   *
   * \param update_grid Update Grid containing all states that changed during the last update
   */
  void overwriteMap(const UpdateGridT::Ptr& update_grid)
  {
    std::unique_lock map_lock(*m_map_mutex);
    typename GridT::Accessor acc = m_vdb_grid->getAccessor();
    for (UpdateGridT::ValueOnCIter iter = update_grid->cbeginValueOn(); iter; ++iter)
    {
      if (*iter)
      {
        acc.setActiveState(iter.getCoord(), true);
      }
      else
      {
        acc.setActiveState(iter.getCoord(), false);
      }
    }
    map_lock.unlock();
  }

  /*!
   * \brief Incorporates the information of an update grid to the internal map. This will update the
   * probabilities of all cells specified by the update grid.
   *
   * \param temp_grid Grid containing all cells which shall be updated
   *
   * \returns Was the insertion of the pointcloud successuff
   */
  UpdateGridT::Ptr updateMap(const UpdateGridT::Ptr& temp_grid)
  {
    UpdateGridT::Ptr change          = UpdateGridT::create(false);
    UpdateGridT::Accessor change_acc = change->getAccessor();
    if (temp_grid->empty())
    {
      return change;
    }

    bool state_changed = false;
    std::unique_lock map_lock(*m_map_mutex);
    typename GridT::Accessor acc = m_vdb_grid->getAccessor();
    // Probability update lambda for free space grid elements
    auto miss = [&](TData& voxel_value, bool& active) {
      bool last_state = active;
      updateFreeNode(voxel_value, active);
      if (last_state != active)
      {
        state_changed = true;
      }
    };

    // Probability update lambda for occupied grid elements
    auto hit = [&](TData& voxel_value, bool& active) {
      bool last_state = active;
      updateOccupiedNode(voxel_value, active);
      if (last_state != active)
      {
        state_changed = true;
      }
    };

    // Integrating the data of the temporary grid into the map using the probability update
    // functions
    for (UpdateGridT::ValueOnCIter iter = temp_grid->cbeginValueOn(); iter; ++iter)
    {
      state_changed = false;
      if (*iter)
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

    // Set all artificial values to active
    for (UpdateGridT::ValueOnCIter iter = m_artificial_area_grid->cbeginValueOn(); iter; ++iter)
    {
      acc.setActiveState(iter.getCoord(), true);
    }
    map_lock.unlock();

    return change;
  }

  /*!
   * \brief Returns a pointer to the VDB map structure
   *
   * \returns Map pointer
   */
  typename GridT::Ptr getGrid() const { return m_vdb_grid; }

  /*!
   * \brief Creates a world coordinate bounding box around a transform
   *
   * \param min_boundary Minimum boundary of box
   * \param max_boundary Maximum boundary of box
   * \param map_to_reference_tf Transform from map to reference frame
   *
   * \returns World coordinate bounding box
   */
  openvdb::BBoxd
  createWorldBoundingBox(const Eigen::Matrix<double, 3, 1>& min_boundary,
                         const Eigen::Matrix<double, 3, 1>& max_boundary,
                         const Eigen::Matrix<double, 4, 4>& map_to_reference_tf) const
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr corners(new pcl::PointCloud<pcl::PointXYZ>());
    corners->points.emplace_back(static_cast<float>(min_boundary.x()),
                                 static_cast<float>(min_boundary.y()),
                                 static_cast<float>(min_boundary.z()));
    corners->points.emplace_back(static_cast<float>(min_boundary.x()),
                                 static_cast<float>(min_boundary.y()),
                                 static_cast<float>(max_boundary.z()));
    corners->points.emplace_back(static_cast<float>(min_boundary.x()),
                                 static_cast<float>(max_boundary.y()),
                                 static_cast<float>(min_boundary.z()));
    corners->points.emplace_back(static_cast<float>(min_boundary.x()),
                                 static_cast<float>(max_boundary.y()),
                                 static_cast<float>(max_boundary.z()));
    corners->points.emplace_back(static_cast<float>(max_boundary.x()),
                                 static_cast<float>(min_boundary.y()),
                                 static_cast<float>(min_boundary.z()));
    corners->points.emplace_back(static_cast<float>(max_boundary.x()),
                                 static_cast<float>(min_boundary.y()),
                                 static_cast<float>(max_boundary.z()));
    corners->points.emplace_back(static_cast<float>(max_boundary.x()),
                                 static_cast<float>(max_boundary.y()),
                                 static_cast<float>(min_boundary.z()));
    corners->points.emplace_back(static_cast<float>(max_boundary.x()),
                                 static_cast<float>(max_boundary.y()),
                                 static_cast<float>(max_boundary.z()));
    pcl::transformPointCloud(*corners, *corners, map_to_reference_tf);
    pcl::PointXYZ min_pt;
    pcl::PointXYZ max_pt;
    pcl::getMinMax3D(*corners, min_pt, max_pt);

    return openvdb::BBoxd(openvdb::Vec3d(min_pt.x, min_pt.y, min_pt.z),
                          openvdb::Vec3d(max_pt.x, max_pt.y, max_pt.z));
  }
  /*!
   * \brief Creates an index coordinate bounding box around a transform
   *
   * \param min_boundary Minimum boundary of box
   * \param max_boundary Maximum boundary of box
   * \param map_to_reference_tf Transform from map to reference frame
   *
   * \returns Index coordinate bounding box
   */
  openvdb::CoordBBox
  createIndexBoundingBox(const Eigen::Matrix<double, 3, 1>& min_boundary,
                         const Eigen::Matrix<double, 3, 1>& max_boundary,
                         const Eigen::Matrix<double, 4, 4>& map_to_reference_tf) const
  {
    openvdb::BBoxd world_bb =
      createWorldBoundingBox(min_boundary, max_boundary, map_to_reference_tf);

    std::shared_lock map_lock(*m_map_mutex);
    openvdb::Vec3d min_index = m_vdb_grid->worldToIndex(world_bb.min());
    openvdb::Vec3d max_index = m_vdb_grid->worldToIndex(world_bb.max());
    map_lock.unlock();

    return {openvdb::Coord::floor(min_index), openvdb::Coord::floor(max_index)};
  }

  /*!
   * \brief Generates an update grid from a bouding box and a reference frame
   *
   * \param min_boundary Minimum boundary of the box
   * \param max_boundary Maximum boundary of the box
   * \param map_to_reference_tf Transform from map to reference frame
   * \param full_grid Specifies whether the entire grid or just the active values should be return
   *
   * \returns Update Grid containing the information within the bounding box
   */
  typename UpdateGridT::Ptr
  getMapSectionUpdateGrid(const Eigen::Matrix<double, 3, 1>& min_boundary,
                          const Eigen::Matrix<double, 3, 1>& max_boundary,
                          const Eigen::Matrix<double, 4, 4>& map_to_reference_tf,
                          const bool full_grid = false) const
  {
    return getMapSection<typename VDBMapping<TData, TConfig>::UpdateGridT>(
      min_boundary, max_boundary, map_to_reference_tf, full_grid);
  }
  /*!
   * \brief Generates a grid from a bounding box and a reference frame
   *
   * \param min_boundary Minimum boundary of the box
   * \param max_boundary Maximum boundary of the box
   * \param map_to_reference_tf Transform from map to reference frame
   * \param full_grid Specifies whether the entire grid or just the active values should be return
   *
   * \returns Grid containing the information within the bounding box
   */
  typename GridT::Ptr getMapSectionGrid(const Eigen::Matrix<double, 3, 1>& min_boundary,
                                        const Eigen::Matrix<double, 3, 1>& max_boundary,
                                        const Eigen::Matrix<double, 4, 4>& map_to_reference_tf,
                                        const bool full_grid = false) const
  {
    return getMapSection<typename VDBMapping<TData, TConfig>::GridT>(
      min_boundary, max_boundary, map_to_reference_tf, full_grid);
  }
  /*!
   * \brief Generates a grid or update grid from a bounding box and a reference frame
   *
   * @tparam TResultGrid Resulting Grid Type
   * \param min_boundary Minimum boundary of the box
   * \param max_boundary Maximum boundary of the box
   * \param map_to_reference_tf Transform from map to reference frame
   * \param full_grid Specifies whether the entire grid or just the active values should be return
   *
   * \returns Grid/UpdateGrid containing the information within the bounding box
   */
  template <typename TResultGrid>
  typename TResultGrid::Ptr getMapSection(const Eigen::Matrix<double, 3, 1>& min_boundary,
                                          const Eigen::Matrix<double, 3, 1>& max_boundary,
                                          const Eigen::Matrix<double, 4, 4>& map_to_reference_tf,
                                          const bool full_grid = false) const
  {
    typename TResultGrid::Ptr temp_grid = TResultGrid::create(0);
    temp_grid->setTransform(openvdb::math::Transform::createLinearTransform(m_resolution));

    typename TResultGrid::Accessor temp_acc = temp_grid->getAccessor();

    openvdb::CoordBBox bounding_box(
      createIndexBoundingBox(min_boundary, max_boundary, map_to_reference_tf));

    std::shared_lock map_lock(*m_map_mutex);
    for (auto leaf_iter = m_vdb_grid->tree().cbeginLeaf(); leaf_iter; ++leaf_iter)
    {
      openvdb::CoordBBox bbox;
      bbox = leaf_iter.getLeaf()->getNodeBoundingBox();

      if (bbox.hasOverlap(bounding_box))
      {
        if (full_grid)
        {
          for (auto iter = leaf_iter->cbeginValueAll(); iter; ++iter)
          {
            if (bounding_box.isInside(iter.getCoord()))
            {
              if (iter.isValueOn())
              {
                temp_acc.setValueOn(iter.getCoord(), iter.getValue());
              }
              else
              {
                temp_acc.setValueOff(iter.getCoord(), iter.getValue());
              }
            }
          }
        }
        else
        {
          for (auto iter = leaf_iter->cbeginValueOn(); iter; ++iter)
          {
            if (bounding_box.isInside(iter.getCoord()))
            {
              temp_acc.setValueOn(iter.getCoord(), true);
            }
          }
        }
      }
    }
    map_lock.unlock();

    openvdb::Vec3d min(bounding_box.min().x(), bounding_box.min().y(), bounding_box.min().z());
    openvdb::Vec3d max(bounding_box.max().x(), bounding_box.max().y(), bounding_box.max().z());
    temp_grid->insertMeta("bb_min", openvdb::Vec3DMetadata(min));
    temp_grid->insertMeta("bb_max", openvdb::Vec3DMetadata(max));
    return temp_grid;
  }

  /*!
   * \brief Applies a map section grid to the map
   *
   * \param section Section grid containing the information about part of the map. The boundary box
   * of the section is encoded in the grids meta information
   *
   */
  void applyMapSectionGrid(const typename GridT::Ptr section)
  {
    typename GridT::Accessor section_acc = section->getAccessor();
    std::unique_lock map_lock(*m_map_mutex);
    typename GridT::Accessor acc = m_vdb_grid->getAccessor();
    for (auto iter = section->cbeginValueAll(); iter; ++iter)
    {
      openvdb::Coord coord = iter.getCoord();
      if (section_acc.isValueOn(coord))
      {
        acc.setValueOn(coord, section_acc.getValue(coord));
      }
      else
      {
        acc.setValueOff(coord, section_acc.getValue(coord));
      }
    }
    map_lock.unlock();
  }

  /*!
   * \brief Applies a map section update grid to the map
   *
   * \param section Section grid containing the information about part of the map. The boundary box
   * of the section is encoded in the grids meta information
   *
   */
  void applyMapSectionUpdateGrid(const typename UpdateGridT::Ptr section)
  {
    typename UpdateGridT::Accessor section_acc = section->getAccessor();
    std::unique_lock map_lock(*m_map_mutex);
    typename GridT::Accessor acc = m_vdb_grid->getAccessor();
    openvdb::Vec3d min           = section->template metaValue<openvdb::Vec3d>("bb_min");
    openvdb::Vec3d max           = section->template metaValue<openvdb::Vec3d>("bb_max");
    openvdb::CoordBBox bbox(openvdb::Coord::floor(min), openvdb::Coord::floor(max));

    for (auto iter = m_vdb_grid->cbeginValueOn(); iter; ++iter)
    {
      if (bbox.isInside(iter.getCoord()))
      {
        acc.setActiveState(iter.getCoord(), false);
      }
    }
    for (auto iter = section->cbeginValueOn(); iter; ++iter)
    {
      acc.setActiveState(iter.getCoord(), true);
    }
    map_lock.unlock();
  }


  void restoreMapIntegrity()
  {
    // Set each voxel to its previous state
    auto restoreState = [&](TData& voxel_value, bool& active) {
      setNodeState(voxel_value, active);
    };
    typename GridT::Accessor acc = m_vdb_grid->getAccessor();
    for (auto iter = m_artificial_area_grid->cbeginValueOn(); iter; ++iter)
    {
      acc.modifyValueAndActiveState(iter.getCoord(), restoreState);
    }
    // Clear artificial grid
    m_artificial_area_grid->clear();
    m_artificial_areas_present = false;
  }

  void
  addArtificialAreas(const std::vector<std::vector<Eigen::Matrix<double, 4, 1> > > artificial_areas,
                     const double negative_height,
                     const double positive_height)
  {
    // Restore map integrity by removing all artificial walls
    restoreMapIntegrity();
    m_artificial_areas_present = true;

    // Add all artificial areas
    for (auto& artificial_area : artificial_areas)
    {
      addArtificialPolygon(artificial_area, negative_height, positive_height);
    }
  }

  void addArtificialPolygon(const std::vector<Eigen::Matrix<double, 4, 1> > polygon,
                            const double negative_height,
                            const double positive_height)
  {
    for (size_t i = 0; i < polygon.size(); i++)
    {
      addArtificialWall(
        polygon[i], polygon[(i + 1) % polygon.size()], negative_height, positive_height);
    }
  }

  void addArtificialWall(const Eigen::Matrix<double, 4, 1> start,
                         const Eigen::Matrix<double, 4, 1> end,
                         const double negative_height,
                         const double positive_height)
  {
    UpdateGridT::Accessor artificial_area_grid_acc = m_artificial_area_grid->getAccessor();
    openvdb::Coord start_index =
      this->worldToIndex(openvdb::Vec3d(start.x(), start.y(), start.z()));
    openvdb::Coord end_index = this->worldToIndex(openvdb::Vec3d(end.x(), end.y(), end.z()));

    int negative_index = (int)(negative_height / m_resolution);
    int positive_index = (int)(positive_height / m_resolution);

    for (int i = negative_index; i < positive_index; i++)
    {
      castRayIntoGrid(start_index + openvdb::Coord(0, 0, i),
                      end_index + openvdb::Coord(0, 0, i),
                      artificial_area_grid_acc);
    }
  }


  std::vector<uint8_t> compressString(const std::string& string) const
  {
    auto uncompressed = std::vector<uint8_t>(string.begin(), string.end());

    // Create buffer with enough size for worst case scenario
    size_t len = ZSTD_compressBound(uncompressed.size());
    std::vector<uint8_t> compressed(len);

    int ret = ZSTD_compress(
      compressed.data(), len, uncompressed.data(), uncompressed.size(), m_compression_level);


    if (ZSTD_isError(ret))
    {
      std::cerr << "Compression using ZSTD failed: " << ZSTD_getErrorName(ret)
                << " , sending uncompressed byte array" << std::endl;
      return uncompressed;
    }

    // Resize compressed buffer to actual compressed size
    compressed.resize(ret);
    return compressed;
  }

  std::string decompressByteArray(const std::vector<uint8_t>& byte_array) const
  {
    std::size_t len = ZSTD_getDecompressedSize(byte_array.data(), byte_array.size());
    std::vector<uint8_t> uncompressed(len);

    std::size_t size =
      ZSTD_decompress(uncompressed.data(), len, byte_array.data(), byte_array.size());


    std::string map_str;
    if (ZSTD_isError(size))
    {
      std::cerr << "Could not decompress map using ZSTD failed: " << ZSTD_getErrorName(size)
                << " , returning raw data" << std::endl;
      map_str = std::string(byte_array.begin(), byte_array.end());
    }
    else
    {
      map_str = std::string(uncompressed.begin(), uncompressed.end());
    }

    return map_str;
  }

  template <typename TGrid>
  std::vector<uint8_t> gridToByteArray(typename TGrid::Ptr grid)
  {
    openvdb::GridPtrVec grids;
    grids.push_back(grid);
    std::ostringstream oss(std::ios_base::binary);
    openvdb::io::Stream(oss).write(grids);
    return compressString(oss.str());
  }

  template <typename TGrid>
  typename TGrid::Ptr byteArrayToGrid(std::vector<uint8_t> byte_array)
  {
    std::istringstream iss(decompressByteArray(byte_array));
    openvdb::io::Stream strm(iss);
    openvdb::GridPtrVecPtr grids;
    grids = strm.getGrids();
    // This cast might fail if different VDB versions are used.
    // Corresponding error messages are generated by VDB directly
    typename TGrid::Ptr update_grid = openvdb::gridPtrCast<TGrid>(grids->front());
    return update_grid;
  }

  std::shared_ptr<std::shared_mutex> getMapMutex() { return m_map_mutex; }
  std::shared_ptr<std::shared_mutex> getUpdateGridMutex() { return m_map_mutex; }
  std::shared_ptr<std::shared_mutex> getConsumableUpdateGridMutex() { return m_map_mutex; }

  /*!
   * \brief Handles changing the mapping config
   *
   * \param config Configuration structure
   */
  virtual void setConfig(const TConfig& config)
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

protected:
  virtual bool updateFreeNode(TData& voxel_value, bool& active) { return false; }
  virtual bool updateOccupiedNode(TData& voxel_value, bool& active) { return false; }
  virtual bool setNodeToFree(TData& voxel_value, bool& active) { return false; }
  virtual bool setNodeToOccupied(TData& voxel_value, bool& active) { return false; }
  virtual bool setNodeState(TData& voxel_value, bool& active) { return false; }

  virtual void createMapFromPointCloud(const PointCloudT::Ptr& cloud,
                                       const bool set_background,
                                       const bool clear_map)
  {
    std::cerr << "Not implemented for data type" << std::endl;
  }

  /*!
   * \brief VDB grid pointer
   */
  typename GridT::Ptr m_vdb_grid;

  typename UpdateGridT::Ptr m_artificial_area_grid;
  /*!
   * \brief Maximum raycasting distance
   */
  double m_max_range;
  /*!
   * \brief Grid resolution of the map
   */
  double m_resolution;
  /*!
   * \brief path where the maps will be stored
   */
  std::string m_map_directory_path;
  /*!
   * \brief Flag checking wether a valid config was already loaded
   */
  bool m_config_set;

  UpdateGridT::Ptr m_update_grid;

  unsigned int m_compression_level = 1;
  bool m_artificial_areas_present;

  UpdateGridT::Ptr m_consumable_update_grid;

  mutable std::shared_ptr<std::shared_mutex> m_map_mutex;
  mutable std::shared_ptr<std::shared_mutex> m_update_grid_mutex;
  mutable std::shared_ptr<std::shared_mutex> m_consumable_update_grid_mutex;
};

#include "VDBMapping.hpp"

} // namespace vdb_mapping

#endif /* VDB_MAPPING_VDB_MAPPING_H_INCLUDED */
