#include "lineconstructor.h"

LineConstructor::LineConstructor() {}
CEntity* LineConstructor::create(QVector<CEntity*>& entitylist){
    QVector<CPosition>positions;//存储有效点
    QVector<CPoint*>points;//存储图形点
    QVector<CCircle*>ciecles;//存储图形圆
    for(int i=0;i<entitylist.size();i++){
        CEntity* entity=entitylist[i];
        if(!entity->IsSelected())continue;
        if(entity->GetUniqueType()==enPoint){
            CPoint * point=(CPoint*)entity;
            positions.push_back(point->GetPt());
        }else if(entity->GetUniqueType()==enCircle){
            CCircle* circle=(CCircle*)entity;
            positions.push_back(circle->getCenter());
        }
        ;
    }
    if(positions.size()==2){
        return createLine(positions[0],positions[1]);
    }
    return nullptr;
}

CLine* LineConstructor::createLine(CPosition begin,CPosition end){
    if(begin.x==end.x
        &&begin.y==end.y
        &&begin.z==end.z)
        return nullptr;
    m_line.SetPosition(begin,end);
    CLine * newLine=new CLine();
    newLine->setBegin(begin);
    newLine->setEnd(end);
    return newLine;
}

CLine* LineConstructor::createLine(CPoint begin,CPoint end){
    CPosition pt1=begin.GetPt();
    CPosition pt2=end.GetPt();
    if(pt1.x==pt2.x
        &&pt1.y==pt2.y
        &&pt1.z==pt2.z)
        return nullptr;
     CLine * newLine=new CLine();
    newLine->setBegin(pt1);
    newLine->setEnd(pt2);
    return newLine;
}
CLine* LineConstructor::createLine(CCircle* Circle1,CCircle*Circle2){
    CPosition p1=Circle1->getCenter();
    CPosition p2=Circle2->getCenter();
    return createLine(p1,p2);
}
