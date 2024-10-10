#include "cylinderconstructor.h"

static auto isPointInCircle=[](const CPosition &circleCenter, double radius, const CPosition &point) {
    double distanceSquared = (point.x - circleCenter.x) * (point.x - circleCenter.x) +
                             (point.y - circleCenter.y) * (point.y - circleCenter.y)+
                             (point.z - circleCenter.z) * (point.z - circleCenter.z);
    return (distanceSquared <= radius * radius);
};
CylinderConstructor::CylinderConstructor() {}
bool CylinderConstructor::setCylinder(QVector<CEntity*>& entitylist){
    QVector<CPoint*>points;
    for(int i=0;i<entitylist.size();i++){
        CEntity* entity=entitylist[i];
        if(!entity->IsSelected()||entity->GetUniqueType()!=enPoint){
            continue;
        }
        CPoint * point=(CPoint*)entity;
        points.push_back(point);
    }
    if(points.size()==4){
        return setCylinder(*points[0],*points[1],*points[2],*points[3]);
    }
    return false;
}
bool CylinderConstructor::setCylinder(CPosition p1,CPosition p2,CPosition p3,CPosition p4){
    CPosition A=p1;
    CPosition B=p2;
    CPosition C=p3;
    CPosition D=p4;

    CircleInfor Circle=calculateCircle(A,B,C);
    if(Circle.radius<1e-6){
        return false;
    }
    double radius=Circle.radius;
    QVector4D circleCenter=Circle.center;
    QVector4D normal = Circle.normal;

    // 计算底面圆心和第四个点的距离
    double distanceToD =distanceToPlane(toQVector4D(D),circleCenter,normal);

    // 计算法线并沿法线延伸距离
    QVector4D topCircleCenter=circleCenter+normal*distanceToD;
    QVector4D middleCenter=circleCenter+normal*(distanceToD/2);

    if(isPointInCircle(toCPosition(topCircleCenter),radius,D)){

        m_cylinder.setBtm_center(toCPosition(middleCenter));
        m_cylinder.setDiameter(2*radius);
        m_cylinder.setAxis(normal);
        m_cylinder.setHeight(fabs(distanceToD));
        return true;
    }else{
        return false;
    }

}
bool CylinderConstructor::setCylinder(CPoint p1,CPoint p2,CPoint p3,CPoint p4){
    CPosition pt1=p1.GetPt();
    CPosition pt2=p2.GetPt();
    CPosition pt3=p3.GetPt();
    CPosition pt4=p4.GetPt();
    return setCylinder(pt1,pt2,pt3,pt4);
}
CCylinder CylinderConstructor::getCylinder(){
    return m_cylinder;
}
