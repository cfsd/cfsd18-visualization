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

#ifndef HOLDER_HPP
#define HOLDER_HPP

#include <iostream>
#include <cmath>
#include <vector>
#include <chrono>
#include <Eigen/Dense>
#include "opendlv-standard-message-set.hpp"
#include "holder.hpp"
#include "cluon-complete.hpp"


typedef std::tuple<opendlv::logic::perception::ObjectDirection,opendlv::logic::perception::ObjectDistance,opendlv::logic::perception::ObjectType> ConePackage;

class Holder{
    public: 
        Holder();
        ~Holder() = default;
        void receiveSurfaceMessage(std::map<int,opendlv::logic::perception::GroundSurfaceArea>);
        void receiveDetectCone(std::map<int,ConePackage> currentFrame);
        void receiveAttention(std::map<int,ConePackage> currentFrame);

        Eigen::MatrixXd getAttention();
        Eigen::MatrixXd getDetectCone();
        std::vector<opendlv::logic::perception::GroundSurfaceArea> getSurfaces();

    private:
        std::mutex m_surfaceMutex = {};
        std::mutex m_colorConeMutex = {};
        std::mutex m_attentionMutex = {};
        Eigen::MatrixXd m_colorcones = {};
        Eigen::MatrixXd m_attentionCones = {};
        std::vector<opendlv::logic::perception::GroundSurfaceArea> m_surfaces = {};
};

#endif