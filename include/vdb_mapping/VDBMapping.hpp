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
 * \date    2020-12-23
 *
 */
//----------------------------------------------------------------------


#include <iostream>


template <class T>
VDBMapping<T>::VDBMapping(const double resolution)
  : m_resolution(resolution)
  , m_config_set(false)
{
  // Initialize Grid
  m_vdb_grid = createVDBMap(m_resolution);
}

template <class T>
void VDBMapping<T>::resetMap()
{
  m_vdb_grid->clear();
  m_vdb_grid = createVDBMap(m_resolution);
}

template <class T>
typename VDBMapping<T>::GridT::Ptr VDBMapping<T>::createVDBMap(double resolution)
{
  typename GridT::Ptr new_map = GridT::create(0.0);
  new_map->setTransform(openvdb::math::Transform::createLinearTransform(m_resolution));
  new_map->setGridClass(openvdb::GRID_LEVEL_SET);
  return new_map;
}


template <class T>
void VDBMapping<T>::setConfig(const Config config)
{
  // Sanity Check for input config
  if (config.prob_miss > 0.5)
  {
    std::cerr << "Probability for a miss should be below 0.5 but is " << config.prob_miss
              << std::endl;
    return;
  }
  if (config.prob_hit < 0.5)
  {
    std::cerr << "Probability for a hit should be above 0.5 but is " << config.prob_miss
              << std::endl;
    return;
  }

  if (config.max_range < 0.0)
  {
    std::cerr << "Max range of " << config.max_range << " invalid. Range cannot be negative."
              << config.prob_miss << std::endl;
    return;
  }
  m_max_range = config.max_range;
  // Store probabilities as log odds
  m_logodds_miss      = log(config.prob_miss) - log(1 - config.prob_miss);
  m_logodds_hit       = log(config.prob_hit) - log(1 - config.prob_hit);
  m_logodds_thres_min = log(config.prob_thres_min) - log(1 - config.prob_thres_min);
  m_logodds_thres_max = log(config.prob_thres_max) - log(1 - config.prob_thres_max);
  m_config_set        = true;
}
