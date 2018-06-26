/**
* Copyright (C) 2018 Chalmers Revere
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

#include "holder.hpp"

Holder::Holder(){

}

void Holder::receiveSurfaceMessage(std::map<int,opendlv::logic::perception::GroundSurfaceArea> currentFrame){
    std::lock_guard<std::mutex> lock(m_surfaceMutex);
    m_surfaces.clear();
    std::reverse_iterator<std::map<int,opendlv::logic::perception::GroundSurfaceArea>::iterator> it;
    it = currentFrame.rbegin();
    while(it != currentFrame.rend()){
        m_surfaces.push_back(it->second);
        it++;
    }
}

void Holder::receiveDetectCone(std::map<int,ConePackage> currentFrame){
    std::lock_guard<std::mutex> lock(m_colorConeMutex);
    m_colorcones = Eigen::MatrixXd::Zero(4,currentFrame.size());
    std::map<int,ConePackage>::iterator it;
    it = currentFrame.begin();
    int coneIndex = 0;
    while(it != currentFrame.end()){
        auto direction = std::get<0>(it->second);
        auto distance = std::get<1>(it->second);
        auto type = std::get<2>(it->second);
        m_colorcones(0,coneIndex) = direction.azimuthAngle();
        m_colorcones(1,coneIndex) = direction.zenithAngle();
        m_colorcones(2,coneIndex) = distance.distance();
        m_colorcones(3,coneIndex) = type.type();
        it++;
        coneIndex++;
    }
}

void Holder::receiveAttention(std::map<int,ConePackage> currentFrame){
    std::lock_guard<std::mutex> lock(m_attentionMutex);
    m_attentionCones = Eigen::MatrixXd::Zero(3,currentFrame.size());
    std::map<int,ConePackage>::iterator it;
    it = currentFrame.begin();
    int coneIndex = 0;
    while(it != currentFrame.end()){
        auto direction = std::get<0>(it->second);
        auto distance = std::get<1>(it->second);
        m_attentionCones(0,coneIndex) = direction.azimuthAngle();
        m_attentionCones(1,coneIndex) = direction.zenithAngle();
        m_attentionCones(2,coneIndex) = distance.distance();
        it++;
        coneIndex++;
    }
}

Eigen::MatrixXd Holder::getAttention(){
    std::lock_guard<std::mutex> lock(m_attentionMutex);
    return m_attentionCones;
}

Eigen::MatrixXd Holder::getDetectCone(){
    std::lock_guard<std::mutex> lock(m_colorConeMutex);
    return m_colorcones;
}

std::vector<opendlv::logic::perception::GroundSurfaceArea> Holder::getSurfaces(){
    std::lock_guard<std::mutex> lock(m_surfaceMutex);
    return m_surfaces;
}
