#include "distanceconstructor.h"

DistanceConstructor::DistanceConstructor() {}
bool DistanceConstructor::setDistance(QVector<CEntity*>& entitylist){
    QVector<CPoint*>points;
    for(int i=0;i<entitylist.size();i++){
        CEntity* entity=entitylist[i];
        if(!entity->IsSelected()||entity->GetUniqueType()!=enPoint){
            continue;
        }
        CPoint * point=(CPoint*)entity;
        points.push_back(point);
    }
    if(points.size()>=2){
        bool make=false;
        for(int i=0;i<points.size();i++){
            for(int j=i+1;j<points.size();j++){
                if(setDistance(*points[i],*points[j],0,0))
                    make=true;
            }
        }
        return make;
    }
    return false;
}
bool DistanceConstructor::setDistance(CPosition p1,CPosition p2,double uptolerance=0.0,double undertolerance=0.0){
    m_distance.setbegin(p1);
    m_distance.setend(p2);
    m_distance.setUndertolerance(undertolerance);
    m_distance.setUptolerance(uptolerance);
    return true;
}
bool DistanceConstructor::setDistance(CPoint p1,CPoint p2,double uptolerance=0.0,double undertolerance=0.0){
    CPosition pt1=p1.GetPt();
    CPosition pt2=p2.GetPt();
    return setDistance(pt1,pt2,uptolerance,undertolerance);
}
CDistance DistanceConstructor::getDistance(){
    return m_distance;
}
