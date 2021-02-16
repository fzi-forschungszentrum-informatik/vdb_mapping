#include <vdb_mapping/VDBMapping.h>

#include <iostream>

VDBMapping::VDBMapping(double resolution)
  : m_resolution(resolution)
  , m_config_set(false)
{
  m_vdb_grid = GridT::create(0.0);
  m_vdb_grid->setTransform(openvdb::math::Transform::createLinearTransform(m_resolution));
  m_vdb_grid->setGridClass(openvdb::GRID_LEVEL_SET);
}


void VDBMapping::insertPointCloud(const PointCloudT::ConstPtr& cloud,
                                  Eigen::Matrix<double, 3, 1> origin)
{
  if(!m_config_set)
  {
    std::cout << "Map not properly configured. Did you call setConfig method?" << std::endl;
  }

  RayT ray;
  DDAT dda;

  openvdb::Vec3d ray_origin_world(origin.x(), origin.y(), origin.z());
  const Vec3T ray_origing_index(m_vdb_grid->worldToIndex(ray_origin_world));
  openvdb::Vec3d ray_end_world;
  openvdb::Vec3d ray_direction;
  bool max_range_ray;

  GridT::Accessor acc      = m_vdb_grid->getAccessor();
  GridT::Ptr temp_grid     = GridT::create(0.0);
  GridT::Accessor temp_acc = temp_grid->getAccessor();

  openvdb::Vec3d x;
  double ray_length;

  for (const PointT& pt : *cloud)
  {
    max_range_ray = false;
    ray_end_world = openvdb::Vec3d(pt.x, pt.y, pt.z);
    if (m_max_range > 0.0 && (ray_end_world - ray_origin_world).length() > m_max_range)
    {
      ray_end_world = ray_origin_world + (ray_end_world - ray_origin_world).unit() * m_max_range;
      max_range_ray = true;
    }

    openvdb::Vec3d buffer = m_vdb_grid->worldToIndex(ray_end_world);

    ray_direction = buffer - ray_origing_index;
    ray.setEye(ray_origing_index);
    ray.setDir(ray_direction);
    dda.init(ray);

    ray_length = ray_direction.length();
    ray_direction.normalize();

    double signed_distance = -1;
    while (signed_distance < 0)
    {
      x = openvdb::Vec3d(dda.voxel().x(), dda.voxel().y(), dda.voxel().z()) - ray_origing_index;
      // Signed distance in grid coordinates for faster processing(not scaled with the grid resolution!!!)
      signed_distance = ray_direction.dot(x) - ray_length;
      temp_acc.setActiveState(dda.voxel(), true);
      dda.step();
    }

    if (!max_range_ray)
    {
      temp_acc.setValueOn(dda.voxel(), -1);
    }
  }

  auto miss = [&prob_miss = m_prob_miss, &prob_thres_min = m_prob_thres_min](float& f, bool& b) {
    f += prob_miss;
    if (f < prob_thres_min)
      f = prob_thres_min;
    b = f > 0;
  };

  auto hit = [&prob_hit = m_prob_hit, &prob_thres_max = m_prob_thres_max](float& f, bool& b) {
    f += prob_hit;
    if (f > prob_thres_max)
      f = prob_thres_max;
    b = f > 0;
  };

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
}
