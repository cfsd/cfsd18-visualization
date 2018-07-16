/*
 * Copyright (C) 2018  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "collector.hpp"
#include "holder.hpp"
#include "surfacecollector.hpp"
#include "drawer.hpp"
#include "viewer.hpp"
#include <Eigen/Dense>
#include <cstdint>
#include <tuple>
#include <utility>
#include <iostream>
#include <string>
#include <thread>
#include <tuple>
typedef std::tuple<opendlv::logic::perception::ObjectDirection,opendlv::logic::perception::ObjectDistance,opendlv::logic::perception::ObjectType> ConePackage;

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{0};
  std::map<std::string, std::string> commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (commandlineArguments.size()<=0) {
    std::cerr << argv[0] << " is a visualizer for the CFSD18 project using pangolin." << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDaVINCI session> [--id=<Identifier in case of simulated units>] [--verbose] [Module specific parameters....]" << std::endl;
    std::cerr << "Example: " << argv[0] << "--cid=111 --id=211 --gatheringTimeMs=20 --separationTimeMs=10" <<  std::endl;
    retCode = 1;
  } else {

    cluon::data::Envelope data;
    cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
    Holder holder;
    Drawer drawer(commandlineArguments,holder);
    Viewer viewer(commandlineArguments,drawer);
    std::thread viewThread (&Viewer::Run,viewer);
    int gatheringTimeMs = (commandlineArguments.count("gatheringTimeMs")>0)?(std::stoi(commandlineArguments["gatheringTimeMs"])):(30);
    int separationTimeMs = (commandlineArguments.count("separationTimeMs")>0)?(std::stoi(commandlineArguments["separationTimeMs"])):(5);
    Collector attentionCollector(holder,gatheringTimeMs,separationTimeMs,2);
    Collector detectConeCollector(holder,gatheringTimeMs,separationTimeMs,3);
    SurfaceCollector surfaceCollector(holder,gatheringTimeMs,separationTimeMs,1);
    uint32_t detectconeStamp = (commandlineArguments.count("detectConeId")>0)?(static_cast<uint32_t>(std::stoi(commandlineArguments["detectConeId"]))):(118);
    uint32_t attentionStamp = (commandlineArguments.count("attentionId")>0)?(static_cast<uint32_t>(std::stoi(commandlineArguments["attentionId"]))):(116);
    uint32_t detectconelaneStamp = (commandlineArguments.count("detectConeLaneId")>0)?(static_cast<uint32_t>(std::stoi(commandlineArguments["detectConeLaneId"]))):(211);
    uint32_t trackStamp = (commandlineArguments.count("trackId")>0)?(static_cast<uint32_t>(std::stoi(commandlineArguments["trackId"]))):(221);
    uint32_t localTrackStamp = (commandlineArguments.count("localTrackId")>0)?(static_cast<uint32_t>(std::stoi(commandlineArguments["localTrackId"]))):(666);
    auto coneEnvelope{[detectconeStamp,attentionStamp,&detectConeCollector,&attentionCollector](cluon::data::Envelope &&envelope)
      {
        if(envelope.senderStamp() == detectconeStamp){
          detectConeCollector.CollectCones(envelope);
        }
        else if(envelope.senderStamp() == attentionStamp)
          attentionCollector.CollectCones(envelope);
      }
    };

    auto surfaceEnvelope{[senderStamp = detectconelaneStamp,&surfaceCollector](cluon::data::Envelope &&envelope)
      {
        if(envelope.senderStamp() == senderStamp){
          surfaceCollector.CollectSurfaces(envelope);
        }
      }
    };

    auto aimPointEnvelope{[trackStamp,localTrackStamp,&holder](cluon::data::Envelope &&envelope)
      {
        if(envelope.senderStamp() == trackStamp){
          holder.receiveAimPoint(envelope);
        }
        if(envelope.senderStamp() == localTrackStamp){
          holder.receiveLocalAimPoint(envelope);
        }
      }
    };
    od4.dataTrigger(opendlv::logic::perception::ObjectDirection::ID(),coneEnvelope);
    od4.dataTrigger(opendlv::logic::perception::ObjectDistance::ID(),coneEnvelope);
    od4.dataTrigger(opendlv::logic::perception::ObjectType::ID(),coneEnvelope);
    od4.dataTrigger(opendlv::logic::perception::GroundSurfaceArea::ID(),surfaceEnvelope);
    od4.dataTrigger(opendlv::logic::action::AimPoint::ID(),aimPointEnvelope);

    // Just sleep as this microservice is data driven.
    using namespace std::literals::chrono_literals;
    while (od4.isRunning()) {
      std::this_thread::sleep_for(1s);
      std::chrono::system_clock::time_point tp;
    }
  }
  return retCode;
}
