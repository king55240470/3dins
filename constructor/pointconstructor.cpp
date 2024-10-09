#include "pointconstructor.h"

PointConstructor::PointConstructor() {}

bool PointConstructor::setPoint(QVector<CEntity*>& entitylist){
    for(int i=0;i<entitylist.size();i++){
        CEntity* entity=entitylist[i];
        if(!entity->IsSelected()||entity->GetUniqueType()!=enPoint){
            continue;
        }
        CPoint * point=(CPoint*)entity;
        return setPoint(*point);
    }
    return false;
}
bool PointConstructor:: setPoint(CPosition point){
     m_point.SetPosition(point);
    return true;
}
bool PointConstructor::setPoint(CPoint point){
     m_point=point;
    return true;
}
CPoint PointConstructor:: getPoint(){
    return  m_point;
}
