#include "lineconstructor.h"

LineConstructor::LineConstructor() {}
CEntity* LineConstructor::create(QVector<CEntity*>& entitylist){
    Constructor::create(entitylist);
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
        CLine*line=createLine(points[0],planes[0]);
        if(line==nullptr){

            return nullptr;
        }

        line->setCurrentId();
        line->parent.push_back(points[0]);
        line->parent.push_back(planes[0]);
        return line;
    }
    if(points.size()==2){
        CLine*line=createLine(points[0],points[1]);
        if(line==nullptr){

            return nullptr;
        }
        line->setCurrentId();
        qDebug()<<points.size();
        line->parent.push_back(points[0]);
        line->parent.push_back(points[1]);
        return line;
    }else{
        if(points.size()>2){
            setWrongInformation(PointTooMuch);
        }
        if(points.size()<2){
            setWrongInformation(PointTooLess);
        }
    }
    QVector<CPosition>&positions=Constructor::getPositions();//存储有效点
    if(positions.size()==2){
        CLine*line=createLine(positions[0],positions[1]);
        setWrongInformation(Null);
        return line;
    }else{
        setWrongInformation(PointTooLess);
    }
    return nullptr;
}

CLine* LineConstructor::createLine(CPosition begin,CPosition end){
    if(begin.x==end.x
        &&begin.y==end.y
        &&begin.z==end.z)
    {
        setWrongInformation(PointTooClose);
        return nullptr;
    }
    m_line.SetPosition(begin,end);
    CLine * newLine=new CLine();
    newLine->setCurrentId();
    newLine->setBegin(begin);
    newLine->setEnd(end);
    return newLine;
}

CLine* LineConstructor::createLine(CPoint *begin,CPoint *end){
    CPosition pt1=begin->GetPt();
    CPosition pt2=end->GetPt();
    if(pt1.x==pt2.x
        &&pt1.y==pt2.y
        &&pt1.z==pt2.z)
    {
        setWrongInformation(PointTooClose);
        return nullptr;
    }
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
CLine * LineConstructor::createLine(CPoint* p,CPlane* plane){

    // 点的坐标
    CPosition position=p->GetPt();
    CPosition center=plane->getCenter();
    QVector4D normal_=plane->getNormal();
    double point[3]={position.x,position.y,position.z};

    // 计算垂足（点到平面的垂线段的另一端点）
    double t;
    double origin[3]={center.x,center.y,center.z};


    double normal[3]={normal_[0],normal_[1],normal_[2]};


    // 计算垂足的参数t
    t = (normal[0] * (origin[0] - point[0]) + normal[1] * (origin[1] - point[1]) + normal[2] * (origin[2] - point[2])) /
        (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);

    // 垂足的坐标
    double foot[3];
    foot[0] = point[0] + t * normal[0];
    foot[1] = point[1] + t * normal[1];
    foot[2] = point[2] + t * normal[2];

    CPosition foot_(foot[0],foot[1],foot[2]);
    return createLine(position,foot_);

}
