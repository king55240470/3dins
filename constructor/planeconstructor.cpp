#include "planeconstructor.h"
static QVector4D crossProduct(const QVector4D& a, const QVector4D& b) {
    return QVector4D(
        a.y() * b.z() - a.z() * b.y(),
        a.z() * b.x() - a.x() * b.z(),
        a.x() * b.y() - a.y() * b.x(),
        0 // 对于三维向量，w 组件通常设为 0
        );
};
static auto distance(const CPosition& p1, const CPosition& p2) {
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2) + pow(p2.z - p1.z, 2));
};
PlaneConstructor::PlaneConstructor() {}
CEntity* PlaneConstructor::create(QVector<CEntity*>& entitylist){
    Constructor::create(entitylist);
    QVector<CPoint*>points;
    for(int i=0;i<entitylist.size();i++){
        CEntity* entity=entitylist[i];
        if(!entity->IsSelected())continue;
        if(entity->GetUniqueType()==enPoint){
            CPoint * point=(CPoint*)entity;
            points.push_back(point);
        }
    }
    QVector<CPosition>&positions=Constructor::getPositions();//存储有效点
    if(positions.size()==3){
        CPlane*plane=createPlane(positions[0],positions[1],positions[2]);
        plane->parent.push_back(points[0]);
        plane->parent.push_back(points[1]);
        plane->parent.push_back(points[2]);
        return plane;
    }
    return nullptr;
}
CPlane* PlaneConstructor::createPlane(CPosition p1,CPosition p2,CPosition p3){
    CPosition A=p1;
    CPosition B=p2;
    CPosition C=p3;

    QVector4D vecA(A.x, A.y, A.z, 1.0);
    QVector4D vecB(B.x, B.y, B.z, 1.0);
    QVector4D vecC(C.x, C.y, C.z, 1.0);

    // 计算法向量
    QVector4D AB = vecB - vecA; // 向量 AB
    QVector4D AC = vecC - vecA; // 向量 AC
    QVector4D BC=  vecC - vecB;
    CPosition center; // 平面的中心点
    QVector4D normal; // 平面的法向量
    QVector4D direction;

    normal = crossProduct(AB,AC); // 计算法向量并归一化

    // 如果法向量的长度为零，说明三点共线
    if (normal.length() == 0) {
        return  nullptr;
    }

    // 计算平面的中心
    center = CPosition((A.x + B.x + C.x) / 3, (A.y + B.y + C.y) / 3, (A.z + B.z + C.z) / 3);

    // 最长边向量
    direction = QVector4D();// 方向向量可以根据具体应用调整

    double lenAB = distance(A, B);
    double lenAC = distance(A, C);
    double lenBC = distance(B, C);

    double width=qMin(lenAB,lenAC);
    width=qMin(width,lenBC);

    double length=qMax(lenAB,lenAC);
    length=qMax(length,lenBC);

    if(length==lenAB){
        direction=AB;
    }else if(length==lenAC){
        direction=AC;
    }else{
        direction=BC;
    }

    CPlane* newplane=new CPlane();
    newplane->setCenter(center);
    newplane->setNormal(normal);
    newplane->setDir_long_edge(direction);
    newplane->setLength(length);
    newplane->setWidth(width);
    return newplane;
}
CPlane* PlaneConstructor::createPlane(CPoint p1,CPoint p2,CPoint p3){
    CPosition pt1=p1.GetPt();
    CPosition pt2=p2.GetPt();
    CPosition pt3=p3.GetPt();
    return createPlane(pt1,pt2,pt3);
}
CPlane* PlaneConstructor::createPlane(CPosition posCenter, QVector4D normal, QVector4D direction, double length, double width){
    CPlane* newplane=new CPlane();
    newplane->setCenter(posCenter);
    newplane->setNormal(normal);
    newplane->setDir_long_edge(direction);
    newplane->setLength(length);
    newplane->setWidth(width);
    return newplane;
}
