#include "cylinderconstructor.h"

static auto isPointInCircle=[](const CPosition &circleCenter, double radius, const CPosition &point) {
    double distanceSquared = (point.x - circleCenter.x) * (point.x - circleCenter.x) +
                             (point.y - circleCenter.y) * (point.y - circleCenter.y)+
                             (point.z - circleCenter.z) * (point.z - circleCenter.z);
    return (distanceSquared <= radius * radius);
};
CylinderConstructor::CylinderConstructor() {}
CEntity* CylinderConstructor::create(QVector<CEntity*>& entitylist){
    Constructor::create(entitylist);
    QVector<CPosition>& positions=Constructor::getPositions();
    if(positions.size()==4){
        return createCylinder(positions[0],positions[1],positions[2],positions[3]);
    }
    return nullptr;

}
CCylinder* CylinderConstructor::createCylinder(CPosition p1,CPosition p2,CPosition p3,CPosition p4){
    CPosition A=p1;
    CPosition B=p2;
    CPosition C=p3;
    CPosition D=p4;

    CircleInfor Circle=calculateCircle(A,B,C);
    if(Circle.radius<1e-6){
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
        return nullptr;
    }

}

CCylinder* CylinderConstructor::createCylinder(CPosition pos, QVector4D vec, double height, double diametre){
    CCylinder* newCylinder=new CCylinder();
    newCylinder->setBtm_center(pos);
    newCylinder->setDiameter(diametre);
    newCylinder->setAxis(vec);
    newCylinder->setHeight(height);
    return newCylinder;
}
