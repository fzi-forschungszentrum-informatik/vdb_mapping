// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Marvin Große Besselmann grosse@fzi.de
 * \date    2020-12-23
 *
 */
//----------------------------------------------------------------------
#ifndef VDB_MAPPING_VDB_MAPPING_H_INCLUDED
#define VDB_MAPPING_VDB_MAPPING_H_INCLUDED

#include <openvdb/Types.h>
#include <openvdb/math/DDA.h>
#include <openvdb/math/Ray.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/Morphology.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <eigen3/Eigen/Geometry>

class VDBMapping
{
  using PointT      = pcl::PointXYZ;
  using PointCloudT = pcl::PointCloud<PointT>;

  using RayT  = openvdb::math::Ray<double>;
  using Vec3T = RayT::Vec3Type;
  using DDAT  = openvdb::math::DDA<RayT, 0>;

  using GridT = openvdb::FloatGrid;

  // TODO vdb grid type hier auch später mal veränderbar machen

public:
  VDBMapping();
  virtual ~VDBMapping(){};

  void insertPointCloud(const PointCloudT::ConstPtr& cloud, Eigen::Matrix<double, 3, 1> origin);

  GridT::Ptr getMap() { return m_vdb_grid; }

private:
  GridT::Ptr m_vdb_grid;
  double m_max_range;
  double m_resolution;
  double m_prob_hit;
  double m_prob_miss;
  double m_thres_min;
  double m_thres_max;
  double m_l_miss;
  double m_l_hit;
};

#endif /* VDB_MAPPING_VDB_MAPPING_H_INCLUDED */
