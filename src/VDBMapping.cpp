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

  m_vdb_grid = GridT::create(0.0);
  m_vdb_grid->setTransform(openvdb::math::Transform::createLinearTransform(m_resolution));
  m_vdb_grid->setGridClass(openvdb::GRID_LEVEL_SET);

  std::cout << "Initialization complete" << std::endl;
}


void VDBMapping::insertPointCloud(const PointCloudT::ConstPtr& cloud,
                                  Eigen::Matrix<double, 3, 1> origin)
{
  openvdb::Vec3d start(origin.x(), origin.y(), origin.z());

  const Vec3T eye(m_vdb_grid->worldToIndex(start));

  openvdb::Vec3d end;
  openvdb::Coord index_end;
  GridT::Accessor acc = m_vdb_grid->getAccessor();
  openvdb::Vec3d dir;

  RayT ray;
  DDAT dda;

  bool ray_too_long = false;

  GridT::Ptr temp_grid     = GridT::create(0.0);
  GridT::Accessor temp_acc = temp_grid->getAccessor();

  // TODO not nice
  double thres_min = m_thres_min;
  double thres_max = m_thres_max;

  // Log Odds calculation
  double l_miss = log(m_prob_miss) - log(1 - m_prob_miss);
  double l_hit  = log(m_prob_hit) - log(1 - m_prob_hit);

  // TODO which of these lambdas is still in use
  auto miss = [&l_miss, &thres_min](float& f, bool& b) {
    // Adding Log Odds for miss, Prior is currently not inserted
    f += l_miss;
    // Capping all values
    if (f < thres_min)
      f = thres_min;
    b = f > 0;
  };

  auto hit = [&l_hit, &thres_max](float& f, bool& b) {
    // l_current+=log10(occ)-log10(1-occ)-l_start
    f += l_hit;
    if (f > thres_max)
      f = thres_max;
    b = f > 0;
  };

  for (const PointT& pt : *cloud)
  {
    ray_too_long = false;
    end          = openvdb::Vec3d(pt.x, pt.y, pt.z);
    // TODO this should be an absolute value in the length
    if (m_max_range > 0.0 && (end - start).length() > m_max_range)
    {
      end          = start + (end - start).unit() * m_max_range;
      ray_too_long = true;
    }

    openvdb::Vec3d buffer = m_vdb_grid->worldToIndex(end);
    index_end             = openvdb::Coord(buffer[0], buffer[1], buffer[2]);

    ray.setEye(eye);
    ray.setDir(buffer - eye);
    dda.init(ray);


    // TODO Better compare function
    auto compare = [](openvdb::Coord a, openvdb::Coord b) {
      int dist = std::abs(a.x() - b.x()) + std::abs(a.y() - b.y()) + std::abs(a.z() - b.z());
      return (dist <= 3);
    };

    while (!compare(dda.voxel(), index_end))
    {
      temp_acc.setActiveState(dda.voxel(), true);
      dda.step();
    }
    if(!ray_too_long)
    {
      temp_acc.setValueOn(dda.voxel(), -1);
    }
  }

  for(GridT::ValueOnCIter iter = temp_grid->cbeginValueOn(); iter; ++iter)
  {
    if(*iter == -1)
    {
      acc.modifyValueAndActiveState(iter.getCoord(), hit);
    }
    else
    {
      acc.modifyValueAndActiveState(iter.getCoord(), miss);
    }
  }
}
