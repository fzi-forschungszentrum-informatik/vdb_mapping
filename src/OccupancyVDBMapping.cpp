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
 * \author  Lennart Puck puck@fzi.de
 * \date    2021-04-29
 *
 */
//----------------------------------------------------------------------


#include "vdb_mapping/OccupancyVDBMapping.h"


bool OccupancyVDBMapping::updateNode(openvdb::FloatGrid::Ptr& temp_grid)
{
  typename GridT::Accessor acc = m_vdb_grid->getAccessor();
  // Probability update lambda for free space grid elements
  auto miss = [& prob_miss = m_logodds_miss, &prob_thres_min = m_logodds_thres_min](DataNodeT& node,
                                                                                    bool& active) {
    node.updateNode(node.getData() + prob_miss);
    if (node.getData() < prob_thres_min)
    {
      active = false;
    }
  };

  // Probability update lambda for occupied grid elements
  auto hit = [& prob_hit = m_logodds_hit, &prob_thres_max = m_logodds_thres_max](DataNodeT& node,
                                                                                 bool& active) {
    node.updateNode(node.getData() + prob_hit);
    if (node.getData() > prob_thres_max)
    {
      active = true;
    }
  };

  // Integrating the data of the temporary grid into the map using the probability update functions
  // for (typename GridT::ValueOnCIter iter = temp_grid->cbeginValueOn(); iter; ++iter)
  for (typename openvdb::FloatGrid::ValueOnCIter iter = temp_grid->cbeginValueOn(); iter; ++iter)
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
