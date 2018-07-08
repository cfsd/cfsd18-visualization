#include "drawer.hpp"


Drawer::Drawer(std::map<std::string,std::string> commandlineArgs, Holder &a_holder):
holder(a_holder)
{
    std::cout << commandlineArgs.count("cid") << std::endl;
}


void Drawer::drawAttention(){
    m_attentionCones = holder.getAttention();
    uint32_t nPoints = static_cast<unsigned int>(m_attentionCones.cols());
    if(nPoints == 0){
        return; 
    }
    glPointSize(10);
    glBegin(GL_POINTS);
    for(uint32_t i = 0; i<nPoints; i++){
        Eigen::MatrixXd localCone;
        localCone = Spherical2Cartesian(m_attentionCones(0,i),0.0,m_attentionCones(2,i));
        float x = static_cast<float>(localCone(0,0)/5);
        float y = static_cast<float>(localCone(1,0)/5);
        float z = 0.0f;
        glColor3f(0.5,0.5,0.5);
        glVertex3f(x,y,z);
    }
    glEnd();
}

void Drawer::drawDetectCone(){
    m_colorCones = holder.getDetectCone();
    uint32_t nPoints = static_cast<unsigned int>(m_colorCones.cols());
    if(nPoints == 0){
        return; 
    }
    glPointSize(15);
    glBegin(GL_POINTS);
    for(uint32_t i = 0; i<nPoints; i++){
        Eigen::MatrixXd localCone;
        localCone = Spherical2Cartesian(m_colorCones(0,i),0.0,m_colorCones(2,i));
        float x = static_cast<float>(localCone(0,0)/5);
        float y = static_cast<float>(localCone(1,0)/5);
        float z = 0.0f;
        if(static_cast<int>(m_colorCones(3,i)) == 2){
            glColor3f(1.0,0.75f,0.0);//yellow
        }
        else if(static_cast<int>(m_colorCones(3,i)) == 1){
            glColor3f(0.0,0.0,1.0);//blue
        }
        else if(static_cast<int>(m_colorCones(3,i)) == 3){
            glColor3f(1.0,0.5,0.0);//little orange
        }
        else if(static_cast<int>(m_colorCones(3,i)) == 4){
            glColor3f(1.0,0.5,0.0);//big orange
            glPointSize(20);
        }
        else{
            continue;
        }
        glVertex3f(x,y,z);
    }
    glEnd();
}

void Drawer::drawSurfaces(bool finalCones,bool path){
    m_surfaces = holder.getSurfaces();
    uint32_t nSurfaces = m_surfaces.size();
    if(nSurfaces == 0){
        return;
    }
    for(uint32_t i = 0; i < nSurfaces; i++){
        if(finalCones){
            drawFinalCones(m_surfaces[i]);    
        }
        if(path){
            drawPath(m_surfaces[i]);
        }
    }
}

void Drawer::drawFinalCones(opendlv::logic::perception::GroundSurfaceArea surface){
    double x1 = surface.x1()/5;
    double x2 = surface.x2()/5;
    double x3 = surface.x3()/5;
    double x4 = surface.x4()/5;
    double y1 = surface.y1()/5;
    double y2 = surface.y2()/5;
    double y3 = surface.y3()/5;
    double y4 = surface.y4()/5;
    glPointSize(5);
    glBegin(GL_POINTS);
    if(y1<y2){
        glColor4f(1.0,0.75f,0.0,1.0);//yellow
        glVertex3f(static_cast<float>(x1),static_cast<float>(y1),0.0f);
        glVertex3f(static_cast<float>(x3),static_cast<float>(y3),0.0f);
        glColor4f(0.0,0.0,1.0,1.0);//blue
        glPointSize(5);
        glVertex3f(static_cast<float>(x2),static_cast<float>(y2),0.0f);
        glVertex3f(static_cast<float>(x4),static_cast<float>(y4),0.0f);
    }
    else{
        glColor4f(0.0,0.0,1.0,1.0);
        glPointSize(5);
        glVertex3f(static_cast<float>(x1),static_cast<float>(y1),0.0f);
        glVertex3f(static_cast<float>(x3),static_cast<float>(y3),0.0f);
        glColor4f(1.0,0.75f,0.0,1.0);//yellow
        glPointSize(5);
        glVertex3f(static_cast<float>(x2),static_cast<float>(y2),0.0f);
        glVertex3f(static_cast<float>(x4),static_cast<float>(y4),0.0f);

    }
    glEnd();
}

void Drawer::drawPath(opendlv::logic::perception::GroundSurfaceArea surface){
    double x1 = surface.x1()/5;
    double x2 = surface.x2()/5;
    double x3 = surface.x3()/5;
    double x4 = surface.x4()/5;
    double y1 = surface.y1()/5;
    double y2 = surface.y2()/5;
    double y3 = surface.y3()/5;
    double y4 = surface.y4()/5;
    glPointSize(15);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);
    glVertex3f(0.0,0.0,0.0);
    glEnd();
    glLineWidth(2);
    glColor4f(0.5f,0.5f,0.5f,1.0f);
    glBegin(GL_LINES);
    x1 = (x1+x2)/2;
    y1 = (y1+y2)/2;
    x2 = (x3+x4)/2;
    y2 = (y3+y4)/2;
    glVertex3f(static_cast<float>(x1),static_cast<float>(y1),0.0f);
    glVertex3f(static_cast<float>(x2),static_cast<float>(y2),0.0f);
    glEnd();
}

void Drawer::drawAimPoint(){
    m_aimPoint = holder.getAimPoint();
    //std::cout << "Aimpoint azimuth: " << m_aimPoint.azimuthAngle() <<", distance: " << m_aimPoint.distance() << std::endl;
    Eigen::MatrixXd aimPoint = Spherical2Cartesian(m_aimPoint.azimuthAngle()*static_cast<double>(RAD2DEG),0.0,m_aimPoint.distance());
    glPointSize(15);
    glBegin(GL_POINTS);
    glColor3f(0.0,1.0,0.0);
    glVertex3f(static_cast<float>(aimPoint(0)/5),static_cast<float>(aimPoint(1)/5),0.0);
    glEnd();
}

Eigen::MatrixXd Drawer::Spherical2Cartesian(double azimuth, double zenimuth, double distance)
{
  double xData = distance * cos(zenimuth * static_cast<double>(DEG2RAD))*cos(azimuth * static_cast<double>(DEG2RAD));
  double yData = distance * cos(zenimuth * static_cast<double>(DEG2RAD))*sin(azimuth * static_cast<double>(DEG2RAD));
  Eigen::MatrixXd recievedPoint = Eigen::MatrixXd::Zero(2,1);
  recievedPoint << xData,
                   yData;
  return recievedPoint;
} // End of Spherical2Cartesian