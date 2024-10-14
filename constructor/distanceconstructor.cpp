#include "distanceconstructor.h"

DistanceConstructor::DistanceConstructor() {}
CEntity* DistanceConstructor::create(QVector<CEntity*>& entitylist){
    //QVector<CPosition>positions;//存储有效点
    QVector<CPoint*>points;
    QVector<CPlane*>planes;
    for(int i=0;i<entitylist.size();i++){
        CEntity* entity=entitylist[i];
        if(!entity->IsSelected())continue;
        if(entity->GetUniqueType()==enPoint){
            CPoint * point=(CPoint*)entity;
            points.push_back(point);
        }
        if(entity->GetUniqueType()==enPlane){
            CPlane* plane=(CPlane*)entity;
            planes.push_back(plane);
        }
    }
    if(points.size()==1&&planes.size()==1){

        return createDistance(points[0],planes[0]);
    }
    return nullptr;
}
/*CDistance* DistanceConstructor::setDistance(CPosition p1,CPlane plane,double uptolerance=0.0,double undertolerance=0.0){
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
}*/

/*CDistance *DistanceConstructor::createDistance(CPosition p1, CPlane plane)
{
    m_distance.setbegin(p1);
    m_distance.setplane(plane);
    CDistance *newdistance =new CDistance();
    newdistance->setbegin(p1);
    newdistance->setplane(plane);
    return newdistance;
}*/

CDistance *DistanceConstructor::createDistance(CPoint* p1, CPlane* plane)
{
    CPosition pt1=p1->GetPt();
    m_distance.setbegin(pt1);
    m_distance.setplane(*plane);
    CDistance *newdistance =new CDistance();
    newdistance->setbegin(pt1);
    newdistance->setplane(*plane);
    return newdistance;
}

void DistanceConstructor::createTolerance(double up, double under)
{
    m_distance.setUptolerance(up);
    m_distance.setUndertolerance(under);
}
