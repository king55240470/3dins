#include "distanceconstructor.h"

DistanceConstructor::DistanceConstructor() {}
CEntity* DistanceConstructor::create(QVector<CEntity*>& entitylist){
    QVector<CPoint>point;
    QVector<CPlane>plane;
    for(int i=0;i<entitylist.size();i++){
        CEntity* entity=entitylist[i];
        if(!entity->IsSelected())continue;
        if(entity->GetUniqueType()==enPoint){
            CPoint * Point=(CPoint*)entity;
            point.push_back(*Point);
        }else if(entity->GetUniqueType()==enPlane){
            CPlane* Plane=(CPlane*)entity;
            plane.push_back(*Plane);
        }
    }
    if(point.size()==1&&plane.size()==1){

        return createDistance(point[0],plane[0],0.0,0.0);
    }
    return nullptr;
}
CDistance* DistanceConstructor::setDistance(CPosition p1,CPlane plane,double uptolerance=0.0,double undertolerance=0.0){
    m_distance->setbegin(p1);
    m_distance->setplane(plane);
    m_distance->setUndertolerance(undertolerance);
    m_distance->setUptolerance(uptolerance);
    return m_distance;
}
CDistance* DistanceConstructor::setDistance(CPoint p1,CPlane plane,double uptolerance=0.0,double undertolerance=0.0){
    CPosition pt1=p1.GetPt();
    return setDistance(pt1,plane,uptolerance,undertolerance);
}
CDistance* DistanceConstructor::getDistance(){
    return m_distance;
}

CDistance *DistanceConstructor::createDistance(CPosition p1, CPlane plane, double uptolerance, double undertolerance)
{
    m_distance->setbegin(p1);
    m_distance->setplane(plane);
    CDistance *newdistance =new CDistance();
    newdistance->setbegin(p1);
    newdistance->setplane(plane);
    return newdistance;
}

CDistance *DistanceConstructor::createDistance(CPoint p1, CPlane plane, double uptolerance, double undertolerance)
{
    CPosition pt1=p1.GetPt();
    m_distance->setbegin(pt1);
    m_distance->setplane(plane);
    CDistance *newdistance =new CDistance();
    newdistance->setbegin(pt1);
    newdistance->setplane(plane);
    return newdistance;
}
