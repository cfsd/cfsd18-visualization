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

#include "surfacecollector.hpp"

SurfaceCollector::SurfaceCollector(Holder &holder,int timeOutMs, int separationTimeMs, int packetSize) :
    m_module(holder),
    m_packetSize(packetSize),
    m_timeOutMs(timeOutMs),
    m_separationTimeMs(separationTimeMs),
    m_timeReceived()
{
}

void SurfaceCollector::CollectSurfaces(cluon::data::Envelope data){
    cluon::data::TimeStamp ts = data.sampleTimeStamp();
    int64_t delta = cluon::time::deltaInMicroseconds(ts,m_currentFrameTime);
    m_timeReceived = std::chrono::system_clock::now();
    if(std::abs(delta)<1){
        if(data.dataType() == opendlv::logic::perception::GroundSurfaceArea::ID()){
            opendlv::logic::perception::GroundSurfaceArea surfaceArea = cluon::extractMessage<opendlv::logic::perception::GroundSurfaceArea>(std::move(data));
            uint32_t id = surfaceArea.surfaceId();
            std::map<int,opendlv::logic::perception::GroundSurfaceArea>::iterator it;
            it = m_currentFrame.find(id);
            if(it!=m_currentFrame.end()){
                it->second = surfaceArea;
                m_envelopeCount[id]++;
            }
            else{
                m_currentFrame[id] = surfaceArea;
                m_envelopeCount[id] = 1;
            }
            m_numberOfItems = (m_numberOfItems<=id)?(id+1):(m_numberOfItems);
        }
        m_messageCount++;

    }
    else if(m_newFrame)
    {
        m_numberOfItems = 1;
        m_currentFrame.clear();
        m_envelopeCount.clear();
        m_messageCount = 1;
        m_currentFrameTime = data.sampleTimeStamp();
        m_newFrame = false;
        if(data.dataType() == opendlv::logic::perception::GroundSurfaceArea::ID()){
            opendlv::logic::perception::GroundSurfaceArea surfaceArea = cluon::extractMessage<opendlv::logic::perception::GroundSurfaceArea>(std::move(data));
            uint32_t id = surfaceArea.surfaceId();
            m_currentFrame[id] = surfaceArea;
            m_envelopeCount[id] = 1;
            m_numberOfItems = (m_numberOfItems<=id)?(id+1):(m_numberOfItems);
        }

        std::thread surfaceCollector (&SurfaceCollector::InitializeCollection,this); //just sleep instead maybe since this is unclear how it works
        surfaceCollector.detach();
    }
    else{
        std::cout << "Leaking frames wtf!!!" << std::endl;
    }

}

void SurfaceCollector::InitializeCollection(){
bool sleep = true;
auto start = std::chrono::system_clock::now();

  while(sleep)
  {
    auto now = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - start);
    std::chrono::duration<double> dur = now-m_timeReceived;
    if(m_messageCount == m_numberOfItems*m_packetSize){
        sleep = false;
    }
    if(elapsed.count() > m_timeOutMs*1000){
        std::cout << "Timed out" << std::endl;
        sleep = false;
    }
    if (dur.count()>m_separationTimeMs*0.001) {
      std::cout << "Separation time exceeded: " <<" dur.count(): "<<dur.count()<<" m_separationTimeMs*0.001 "<<m_separationTimeMs*0.001<<std::endl;
      sleep = false;
    }
  }
  GetCompleteFrame();
  SendFrame();
  m_newFrame = true;
}

void SurfaceCollector::GetCompleteFrame(){
    std::map<int,int>::iterator it2 = m_envelopeCount.begin();
    std::map<int,opendlv::logic::perception::GroundSurfaceArea> currentFrameCopy = m_currentFrame;
    while(it2 != m_envelopeCount.end()){
        if(it2->second != static_cast<int>(m_packetSize)){
            currentFrameCopy.erase(it2->first);
            std::cout << "Incomplete frame with id " << it2->first << " removed" << std::endl;
        }
        it2++;
    }
    if (m_currentFrame.size() != currentFrameCopy.size()) {
      m_currentFrame = currentFrameCopy;
    }
}

void SurfaceCollector::SendFrame(){
    std::cout << "sending " << m_currentFrame.size() << " surfaces" << std::endl;
    m_module.receiveSurfaceMessage(m_currentFrame);
}
