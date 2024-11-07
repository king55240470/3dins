#include "pointconstructor.h"

PointConstructor::PointConstructor() {}
CEntity* PointConstructor::create(QVector<CEntity*>& entitylist){
    Constructor::create(entitylist);
    positions=Constructor::getPositions();//存储有效点
    if(positions.size()>=1){
        CPoint*point=createPoint(positions[0]);
        return point;
    }
    return nullptr;
}

CPoint* PointConstructor:: createPoint(CPosition point){
    CPoint* newPoint=new CPoint();
    newPoint->SetPosition(point);
    return newPoint;
}
CPoint* PointConstructor::createPoint(CPoint& point){
    CPoint* newPoint=new CPoint();
    newPoint->SetPosition(point.GetPt());
    return newPoint;
}
QVector<CPosition>& PointConstructor::getPositions(){
    return positions;
}
CPoint* PointConstructor::createPoint(double x,double y,double z){
    CPosition p(x,y,z);
    return createPoint(p);
}
CEntity* PointConstructor::create(QVector<CPosition>& chosenlist){
    positions=chosenlist;
    if(positions.size()>=1){
        return createPoint(positions[0]);
    }
    return nullptr;
}
