#include "constructor.h"

Constructor::Constructor() {}
CEntity* Constructor::create(QVector<CEntity*>& entitylist){
    positions.clear();
    //基类的create负责读取所有有效点
    for(int i=0;i<entitylist.size();i++){
        CEntity* entity=entitylist[i];
        if(!entity->IsSelected())continue;
        if(entity->GetUniqueType()==enPoint){
            CPoint * point=(CPoint*)entity;
            positions.push_back(point->GetPt());
        }else if(entity->GetUniqueType()==enCircle){
            CCircle* circle=(CCircle*)entity;
            positions.push_back(circle->getCenter());
        }else if(entity->GetUniqueType()==enSphere){
            CSphere* sphere=(CSphere*)entity;
            positions.push_back(sphere->getCenter());
        }else if(entity->GetUniqueType()==enPlane){
            CPlane* plane=(CPlane*)entity;
            positions.push_back(plane->getCenter());
        }else if(entity->GetUniqueType()==enCone){
            CCone* cone=(CCone*)entity;
            positions.push_back(cone->getVertex());
        }else if(entity->GetUniqueType()==enCylinder){
            CCylinder* cylinder=(CCylinder*)entity;
            positions.push_back(cylinder->getBtm_center());
        }else if(entity->GetUniqueType()==enLine){
            CLine* line=(CLine*)entity;
            positions.push_back(line->getBegin());
            positions.push_back(line->getEnd());
        }
        ;
    }
    return nullptr;
}
QVector<CPosition>& Constructor::getPositions(){
    return positions;
}
