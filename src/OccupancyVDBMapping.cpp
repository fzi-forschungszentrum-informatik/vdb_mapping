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
 * \date    2021-04-29
 *
 */
//----------------------------------------------------------------------


#include "vdb_mapping/OccupancyVDBMapping.h"

bool OccupancyVDBMapping::updateFreeNode(float& voxel_value, bool& active)
{
  voxel_value += m_logodds_miss;
  if (voxel_value < m_logodds_thres_min)
  {
    active = false;
  }
  return true;
}

bool OccupancyVDBMapping::updateOccupiedNode(float& voxel_value, bool& active)
{
  voxel_value += m_logodds_hit;
  if (voxel_value > m_logodds_thres_max)
  {
    active = true;
  }
  return true;
}


void OccupancyVDBMapping::setConfig(const Config config)
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
