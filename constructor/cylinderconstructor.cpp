#include "cylinderconstructor.h"

static auto isPointInCircle=[](const CPosition &circleCenter, double radius, const CPosition &point) {
    double distanceSquared = (point.x - circleCenter.x) * (point.x - circleCenter.x) +
                             (point.y - circleCenter.y) * (point.y - circleCenter.y)+
                             (point.z - circleCenter.z) * (point.z - circleCenter.z);
    return (distanceSquared <= radius * radius);
};
CylinderConstructor::CylinderConstructor() {}
CEntity* CylinderConstructor::create(QVector<CEntity*>& entitylist){
    //Constructor::create(entitylist);

    QVector<CPosition> positions;
    QVector<CPoint*>points;
    QVector<CCircle*>circles;
    for(int i=0;i<entitylist.size();i++){
        CEntity* entity=entitylist[i];
        if(!entity->IsSelected())continue;
        if(entity->GetUniqueType()==enPoint){
            CPoint * point=(CPoint*)entity;
            points.push_back(point);
            positions.push_back(point->GetPt());
        }
        if(entity->GetUniqueType()==enCircle){
            CCircle* circle=(CCircle*)entity;
            circles.push_back(circle);
        }
    }
    //四个点构造一个圆柱
    if(points.size()==4&&positions.size()==4){
        CCylinder*cylinder=createCylinder(positions[0],positions[1],positions[2],positions[3]);
        if(cylinder==nullptr){
            return nullptr;
        }
        cylinder->parent.push_back(points[0]); 
        cylinder->parent.push_back(points[1]);  
        cylinder->parent.push_back(points[2]);         
        cylinder->parent.push_back(points[3]);

        return cylinder;
    }
    //一个圆一个点构造一个圆柱
    if(points.size()==1&&circles.size()==1){
        CCylinder*cylinder=createCylinder(positions[0],circles[0]);
        if(cylinder==nullptr){
            setWrongInformation(PointDontMatch);
            return nullptr;
        }
        cylinder->parent.push_back(points[0]);
        cylinder->parent.push_back(circles[0]);
        return cylinder;
    }

    if(points.size()>4){
        setWrongInformation(PointTooMuch);
    }else if(points.size()<4){
        setWrongInformation(PointTooLess);
    }else{
         setWrongInformation(PointDontMatch);
    }
    return nullptr;

}
CCylinder* CylinderConstructor::createCylinder(CPosition p1,CCircle* circle){
    double radius=circle->getDiameter()/2;
    QVector4D circleCenter=toQVector4D(circle->getCenter());
    QVector4D normal = circle->normal;


    // 计算底面圆心和第四个点的距离
    double distanceToD =distanceToPlane(toQVector4D(p1),circleCenter,normal);

    // 计算法线并沿法线延伸距离
    QVector4D topCircleCenter=circleCenter+normal*distanceToD;
    QVector4D middleCenter=circleCenter+normal*(distanceToD/2);

    if(isPointInCircle(toCPosition(topCircleCenter),radius,p1)){
        return createCylinder(toCPosition(middleCenter),normal,fabs(distanceToD),2*radius);
    }else{
        setWrongInformation(PointDontMatch);
        return nullptr;
    }

}

CCylinder* CylinderConstructor::createCylinder(CPosition p1,CPosition p2,CPosition p3,CPosition p4){
    CircleConstructor circleConstructor;
    CCircle* circle=circleConstructor.createCircle(p1,p2,p3);
    CPosition A=p1;
    CPosition B=p2;
    CPosition C=p3;
    CPosition D=p4;

    CircleInfor Circle=calculateCircle(A,B,C);
    if(Circle.radius<1e-6){
        setWrongInformation(PointTooClose);
        return nullptr;
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
        return createCylinder(toCPosition(middleCenter),normal,fabs(distanceToD),2*radius);
    }else{
         setWrongInformation(PointDontMatch);
        return nullptr;
    }

}

CCylinder* CylinderConstructor::createCylinder(CPosition pos, QVector4D vec, double height, double diametre){
    CCylinder* newCylinder=new CCylinder();
    newCylinder->setCurrentId();
    newCylinder->setBtm_center(pos);
    newCylinder->setDiameter(diametre);
    newCylinder->setAxis(vec);
    newCylinder->setHeight(height);
    return newCylinder;
}
