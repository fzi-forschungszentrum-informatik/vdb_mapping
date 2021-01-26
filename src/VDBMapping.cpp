#include <vdb_mapping/VDBMapping.h>

#include <iostream>

VDBMapping::VDBMapping()
{
  // TODO remove hard coded variables
  m_resolution = 0.1;
  m_prob_hit   = 0.7;
  m_prob_miss  = 0.4;
  double p_min = 0.12;
  double p_max = 0.97;
  m_max_range  = 15.0;

  m_thres_min = log(p_min) - log(1 - p_min);
  m_thres_max = log(p_max) - log(1 - p_max);
  m_l_miss    = log(m_prob_miss) - log(1 - m_prob_miss);
  m_l_hit     = log(m_prob_hit) - log(1 - m_prob_hit);

  m_vdb_grid = GridT::create(0.0);
  m_vdb_grid->setTransform(openvdb::math::Transform::createLinearTransform(m_resolution));
  m_vdb_grid->setGridClass(openvdb::GRID_LEVEL_SET);

  std::cout << "Initialization complete" << std::endl;
}


void VDBMapping::insertPointCloud(const PointCloudT::ConstPtr& cloud,
                                  Eigen::Matrix<double, 3, 1> origin)
{
  RayT ray;
  DDAT dda;

  openvdb::Vec3d ray_origin_world(origin.x(), origin.y(), origin.z());
  const Vec3T ray_origing_index(m_vdb_grid->worldToIndex(ray_origin_world));

  openvdb::Vec3d ray_end_world;
  openvdb::Coord ray_end_index;

  openvdb::Vec3d ray_direction;

  GridT::Accessor acc = m_vdb_grid->getAccessor();


  bool max_range_ray;

  GridT::Ptr temp_grid     = GridT::create(0.0);
  GridT::Accessor temp_acc = temp_grid->getAccessor();

  auto miss = [&l_miss = m_l_miss, &thres_min = m_thres_min](float& f, bool& b) {
    f += l_miss;
    if (f < thres_min)
      f = thres_min;
    b = f > 0;
  };

  auto hit = [&l_hit = m_l_hit, &thres_max = m_thres_max](float& f, bool& b) {
    f += l_hit;
    if (f > thres_max)
      f = thres_max;
    b = f > 0;
  };

  openvdb::Vec3d w, x;
  double s;

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
    ray_end_index         = openvdb::Coord(buffer[0], buffer[1], buffer[2]);

    ray.setEye(ray_origing_index);
    ray.setDir(buffer - ray_origing_index);
    dda.init(ray);

    w = buffer - ray_origing_index;
    s = w.length();
    w.normalize();

    auto calcSD = [&w, &x, &s, &resolution = m_resolution]() {
      return (((w.x() * x.x() + w.y() * x.y() + w.z() * x.z()) - s));
    };

    double sd = -1;
    while (sd < 0)
    {
      x = openvdb::Vec3d(dda.voxel().x(), dda.voxel().y(), dda.voxel().z()) - ray_origing_index;
      sd = calcSD();
      temp_acc.setActiveState(dda.voxel(), true);
      dda.step();
    }

    if (!max_range_ray)
    {
      temp_acc.setValueOn(dda.voxel(), -1);
    }
  }

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
