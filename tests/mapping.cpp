#include "gtest/gtest.h"
#include <vdb_mapping/VDBMapping.h>

namespace {

TEST(Mapping, SetConfig)
{
  VDBMapping map(1);
  VDBMapping::PointCloudT::Ptr cloud(new VDBMapping::PointCloudT);
  cloud->points.push_back(VDBMapping::PointT(0, 0, 1));
  Eigen::Matrix<double, 3, 1> origin(0, 0, 0);
  map.insertPointCloud(cloud, origin);
  VDBMapping::GridT::Accessor acc = map.getMap()->getAccessor();
  openvdb::Coord coord(0, 0, 1);
  EXPECT_EQ(acc.getValue(coord), 0.0);
  VDBMapping::Config conf;
  conf.max_range      = 10;
  conf.prob_hit       = 0.9;
  conf.prob_miss      = 0.1;
  conf.prob_thres_max = 0.51;
  conf.prob_thres_min = 0.49;
  map.setConfig(conf);
  float log_hit  = log(conf.prob_hit) - log(1 - conf.prob_hit);
  float log_miss = log(conf.prob_miss) - log(1 - conf.prob_miss);
  map.insertPointCloud(cloud, origin);
  EXPECT_EQ(acc.getValue(openvdb::Coord(0, 0, 0)), log_miss);
  EXPECT_EQ(acc.getValue(openvdb::Coord(0, 0, 1)), log_hit);
}

TEST(Mapping, InsertPositivePoint)
{
  VDBMapping map(1);
  VDBMapping::Config conf;
  conf.max_range      = 10;
  conf.prob_hit       = 0.9;
  conf.prob_miss      = 0.1;
  conf.prob_thres_max = 0.51;
  conf.prob_thres_min = 0.49;
  map.setConfig(conf);

  float log_hit  = log(conf.prob_hit) - log(1 - conf.prob_hit);
  float log_miss = log(conf.prob_miss) - log(1 - conf.prob_miss);

  VDBMapping::PointCloudT::Ptr cloud(new VDBMapping::PointCloudT);
  cloud->points.push_back(VDBMapping::PointT(0, 0, 5));
  Eigen::Matrix<double, 3, 1> origin(0, 0, 0);
  map.insertPointCloud(cloud, origin);
  VDBMapping::GridT::Accessor acc = map.getMap()->getAccessor();
  for (int i = 0; i < 5; ++i)
  {
    openvdb::Coord coord(0, 0, i);
    EXPECT_EQ(acc.getValue(coord), log_miss);
    EXPECT_FALSE(acc.isValueOn(coord));
  }
  openvdb::Coord coord(0, 0, 5);
  EXPECT_EQ(acc.getValue(coord), log_hit);
  EXPECT_TRUE(acc.isValueOn(coord));
}

TEST(Mapping, InsertNegativePoint)
{
  VDBMapping map(1);
  VDBMapping::Config conf;
  conf.max_range      = 10;
  conf.prob_hit       = 0.9;
  conf.prob_miss      = 0.1;
  conf.prob_thres_max = 0.51;
  conf.prob_thres_min = 0.49;
  map.setConfig(conf);

  float log_hit  = log(conf.prob_hit) - log(1 - conf.prob_hit);
  float log_miss = log(conf.prob_miss) - log(1 - conf.prob_miss);

  VDBMapping::PointCloudT::Ptr cloud(new VDBMapping::PointCloudT);
  cloud->points.push_back(VDBMapping::PointT(0, 0, -5));
  Eigen::Matrix<double, 3, 1> origin(0, 0, 0);
  map.insertPointCloud(cloud, origin);
  VDBMapping::GridT::Accessor acc = map.getMap()->getAccessor();
  for (int i = 1; i < 6; ++i)
  {
    openvdb::Coord coord(0, 0, -1 * i);
    EXPECT_EQ(acc.getValue(coord), log_miss);
    EXPECT_FALSE(acc.isValueOn(coord));
  }
  openvdb::Coord coord(0, 0, -6);
  EXPECT_EQ(acc.getValue(coord), log_hit);
  EXPECT_TRUE(acc.isValueOn(coord));
}

TEST(Mapping, InsertMaxRangePoint)
{
  VDBMapping map(1);
  VDBMapping::Config conf;
  conf.max_range      = 5;
  conf.prob_hit       = 0.9;
  conf.prob_miss      = 0.1;
  conf.prob_thres_max = 0.51;
  conf.prob_thres_min = 0.49;
  map.setConfig(conf);

  float log_miss = log(conf.prob_miss) - log(1 - conf.prob_miss);

  VDBMapping::PointCloudT::Ptr cloud(new VDBMapping::PointCloudT);
  cloud->points.push_back(VDBMapping::PointT(0, 0, 7));
  Eigen::Matrix<double, 3, 1> origin(0, 0, 0);
  map.insertPointCloud(cloud, origin);
  VDBMapping::GridT::Accessor acc = map.getMap()->getAccessor();
  for (int i = 0; i < 5; ++i)
  {
    openvdb::Coord coord(0, 0, i);
    EXPECT_EQ(acc.getValue(coord), log_miss);
    EXPECT_FALSE(acc.isValueOn(coord));
  }
  openvdb::Coord coord(0, 0, 5);
  EXPECT_EQ(acc.getValue(coord), 0);
  EXPECT_FALSE(acc.isValueOn(coord));
}

TEST(Mapping, ResetMap)
{
  VDBMapping map(1);
  VDBMapping::Config conf;
  conf.max_range      = 10;
  conf.prob_hit       = 0.9;
  conf.prob_miss      = 0.1;
  conf.prob_thres_max = 0.51;
  conf.prob_thres_min = 0.49;
  map.setConfig(conf);

  float log_hit = log(conf.prob_hit) - log(1 - conf.prob_hit);

  VDBMapping::PointCloudT::Ptr cloud(new VDBMapping::PointCloudT);
  cloud->points.push_back(VDBMapping::PointT(0, 0, 1));
  Eigen::Matrix<double, 3, 1> origin(0, 0, 0);
  map.insertPointCloud(cloud, origin);
  VDBMapping::GridT::Accessor acc = map.getMap()->getAccessor();
  EXPECT_EQ(acc.getValue(openvdb::Coord(0, 0, 1)), log_hit);
  map.resetMap();
  acc = map.getMap()->getAccessor();
  EXPECT_EQ(acc.getValue(openvdb::Coord(0, 0, 1)), 0.0);
}

} // namespace

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
