/**
 * Copyright (C) 2017 Chalmers Revere
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
 * USA.
 */

#ifndef SURFACECOLLECTOR_HPP
#define SURFACECOLLECTOR_HPP

#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include "opendlv-standard-message-set.hpp"
#include "holder.hpp"
#include "cluon-complete.hpp"


class SurfaceCollector{
  public:
    SurfaceCollector(Holder &holder, int, int, int);
    ~SurfaceCollector() = default;
    void CollectSurfaces(cluon::data::Envelope data);
    void InitializeCollection();
    void GetCompleteFrame();
    void SendFrame();

  private:
    cluon::data::TimeStamp m_currentFrameTime = {};
    std::map<int,opendlv::logic::perception::GroundSurfaceArea> m_currentFrame = {};
    std::map<int,int> m_envelopeCount = {};
    bool m_newFrame = true;
    bool m_processing = false;
    uint32_t m_messageCount = 0;
    Holder &m_module;
    uint32_t m_packetSize;
    int m_timeOutMs;
    int m_separationTimeMs;
    std::chrono::time_point<std::chrono::system_clock> m_timeReceived;
    uint32_t m_numberOfItems = 1;
};

#endif
