#include "distanceconstructor.h"

DistanceConstructor::DistanceConstructor() {}
CEntity* DistanceConstructor::create(QVector<CEntity*>& entitylist){
    //QVector<CPosition>positions;//存储有效点
    QVector<CPoint*>points;
    QVector<CPlane*>planes;
    QVector<CCircle*>circles;
    QVector<CLine*>lines;
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
        if(entity->GetUniqueType()==enCircle){
            CCircle* circle=(CCircle*)entity;
            circles.push_back(circle);
        }
        if(entity->GetUniqueType()==enLine){
            CLine* line=(CLine*)entity;
            lines.push_back(line);
        }
    }
    if(points.size()==1&&planes.size()==1){
        CDistance*dis=createDistance(points[0],planes[0]);
        dis->parent.push_back(points[0]);
        dis->parent.push_back(planes[0]);
        return dis;
    }
    if(points.size()==2){
        CDistance*dis=createDistance(points[0],points[1]);
        dis->parent.push_back(points[0]);
        dis->parent.push_back(points[1]);
        return dis;
    }
    if(points.size()==1&&circles.size()==1){
        CDistance*dis=createDistance(points[0],circles[0]);
        dis->parent.push_back(points[0]);
        dis->parent.push_back(circles[0]);
         return dis;
    }
    if(points.size()==1&&lines.size()==1){
        CDistance*dis=createDistance(points[0],lines[0]);
        dis->parent.push_back(points[0]);
        dis->parent.push_back(lines[0]);
        return dis;
    }
    return nullptr;
}

CDistance *DistanceConstructor::createDistance(CPoint* p1, CPlane* plane)
{
    CPosition pt1=p1->GetPt();
    m_distance.setbegin(pt1);
    m_distance.setplane(*plane);
    CDistance *newdistance =new CDistance();
    newdistance->setbegin(pt1);
    newdistance->setplane(*plane);
    newdistance->setdistance(newdistance->getdistanceplane());
    return newdistance;
}

CDistance *DistanceConstructor::createDistance(CPoint *p1, CPoint *point)
{
    CPosition pt1=p1->GetPt();
    CPosition pt2=point->GetPt();
    m_distance.setbegin(pt1);
    m_distance.setend(pt2);
    CDistance *newdistance =new CDistance();
    newdistance->setbegin(pt1);
    newdistance->setend(pt2);
    newdistance->setdistance(newdistance->getdistancepoint());
    return newdistance;
}

CDistance *DistanceConstructor::createDistance(CPoint *p1, CCircle *circle)
{
    CPosition pt1=p1->GetPt();
    m_distance.setbegin(pt1);
    m_distance.setcircle(*circle);
    CDistance *newdistance =new CDistance();
    newdistance->setbegin(pt1);
    newdistance->setcircle(*circle);
    newdistance->setdistance(newdistance->getdistancecircle());
    return newdistance;
}

CDistance *DistanceConstructor::createDistance(CPoint *p1, CLine *line)
{
    CPosition pt1=p1->GetPt();
    m_distance.setbegin(pt1);
    m_distance.setline(*line);
    CDistance *newdistance =new CDistance();
    newdistance->setbegin(pt1);
    newdistance->setline(*line);
    newdistance->setdistance(newdistance->getdistanceline());
    return newdistance;
}


void DistanceConstructor::createTolerance(double up, double under)
{
    m_distance.setUptolerance(up);
    m_distance.setUndertolerance(under);
}
