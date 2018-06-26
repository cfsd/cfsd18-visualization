


#ifndef DRAWER_HPP
#define DRAWER_HPP
#include <pangolin/pangolin.h>
#include <Eigen/Dense>
#include "holder.hpp"




class Drawer{
    public:
        Drawer(std::map<std::string,std::string> commandlineArgs, Holder &holder);
        void drawAttention();
        void drawDetectCone();
        void drawSurfaces(bool,bool);
        void drawGraph();

    private:
        Holder& holder;
        std::vector<opendlv::logic::perception::GroundSurfaceArea> m_surfaces = {};
        Eigen::MatrixXd m_attentionCones = {};
        Eigen::MatrixXd m_colorCones = {};
        void drawPath(opendlv::logic::perception::GroundSurfaceArea surface);
        void drawFinalCones(opendlv::logic::perception::GroundSurfaceArea surface);
        Eigen::MatrixXd Spherical2Cartesian(double, double, double);
        const double DEG2RAD = 0.017453292522222; // PI/180.0

};
#endif