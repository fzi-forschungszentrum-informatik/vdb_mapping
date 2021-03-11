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

public:

  /*!
   * \brief Accumulation of configuration parameters
   */
  struct Config
  {
    double max_range;
    double prob_hit;
    double prob_miss;
    double prob_thres_min;
    double prob_thres_max;
  };

  VDBMapping(double resolution);
  virtual ~VDBMapping(){};

  /*!
   * \brief Handles the integration of new PointCloud data into the VDB data structure.
   * All datapoints are raycasted starting from the origin position
   *
   * \param cloud Input cloud in map coordinates
   * \param origin Sensor position in map coordinates
   */
  void insertPointCloud(const PointCloudT::ConstPtr& cloud, Eigen::Matrix<double, 3, 1> origin);


  /*!
   * \brief Returns a pointer to the VDB map structure
   *
   * \returns Map pointer
   */
  GridT::Ptr getMap() { return m_vdb_grid; }


  /*!
   * \brief Handles changing the mapping config
   *
   * \param config Configuration structure
   */
  void setConfig(Config config)
  {
    m_max_range = config.max_range;
    // Store probabilities as log odds
    m_prob_miss      = log(config.prob_miss) - log(1 - config.prob_miss);
    m_prob_hit       = log(config.prob_hit) - log(1 - config.prob_hit);
    m_prob_thres_min = log(config.prob_thres_min) - log(1 - config.prob_thres_min);
    m_prob_thres_max = log(config.prob_thres_max) - log(1 - config.prob_thres_max);

    m_config_set = true;
  }

private:
  /*!
   * \brief VDB grid pointer
   */
  GridT::Ptr m_vdb_grid;
  /*!
   * \brief Maximum raycasting distance
   */
  double m_max_range;
  /*!
   * \brief Grid resolution of the map
   */
  double m_resolution;
  /*!
   * \brief Probability update value for passing an obstacle
   */
  double m_prob_hit;
  /*!
   * \brief Probability update value for passing free space
   */
  double m_prob_miss;
  /*!
   * \brief Upper occupancy probability threshold
   */
  double m_prob_thres_min;
  /*!
   * \brief Lower occupancy probability threshold
   */
  double m_prob_thres_max;
  /*!
   * \brief Flag checking wether a valid config was already loaded
   */
  bool m_config_set;
};

#endif /* VDB_MAPPING_VDB_MAPPING_H_INCLUDED */
