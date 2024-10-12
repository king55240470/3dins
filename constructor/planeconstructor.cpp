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
bool PlaneConstructor::setPlane(QVector<CEntity*>& entitylist){
    QVector<CPoint*>points;
    for(int i=0;i<entitylist.size();i++){
        CEntity* entity=entitylist[i];
        if(!entity->IsSelected()||entity->GetUniqueType()!=enPoint){
            continue;
        }
        CPoint * point=(CPoint*)entity;
        points.push_back(point);
    }
    if(points.size()==3){
        return setPlane(*points[0],*points[1],*points[2]);
    }
    return false;
}
bool PlaneConstructor::setPlane(CPosition p1,CPosition p2,CPosition p3){
    CPosition A=p1;
    CPosition B=p2;
    CPosition C=p3;

    QVector4D vecA(A.x, A.y, A.z, 1.0);
    QVector4D vecB(B.x, B.y, B.z, 1.0);
    QVector4D vecC(C.x, C.y, C.z, 1.0);

    // 计算法向量
    QVector4D AB = vecB - vecA; // 向量 AB
    QVector4D AC = vecC - vecA; // 向量 AC

    CPosition center; // 平面的中心点
    QVector4D normal; // 平面的法向量
    QVector4D direction;

    normal = crossProduct(AB,AC); // 计算法向量并归一化

    // 如果法向量的长度为零，说明三点共线
    if (normal.length() == 0) {
        return  false;
    }

    // 计算平面的中心
    center = CPosition((A.x + B.x + C.x) / 3, (A.y + B.y + C.y) / 3, (A.z + B.z + C.z) / 3);

    // 方向可以由任一向量定义，这里使用 X 轴的正方向作为方向
    direction = QVector4D(1, 0, 0, 0); // 方向向量可以根据具体应用调整

    double lenAB = distance(A, B);
    double lenAC = distance(A, C);
    double lenBC = distance(B, C);

    double width=qMax(lenAB,lenAC);
    width=qMax(width,lenBC);
    double length=qMin(lenAB,lenAC);
    length=qMin(length,lenBC);

    m_plane.setCenter(center);
    m_plane.setNormal(normal);
    m_plane.setDir_long_edge(direction);
    m_plane.setLength(length);
    m_plane.setWidth(width);
    return true;
}
bool PlaneConstructor::setPlane(CPoint p1,CPoint p2,CPoint p3){
    CPosition pt1=p1.GetPt();
    CPosition pt2=p2.GetPt();
    CPosition pt3=p3.GetPt();
    return setPlane(pt1,pt2,pt3);
}
bool PlaneConstructor::setPlane(CPlane plane){
    m_plane=plane;
    return true;
}
CPlane PlaneConstructor::getPlane(){
    return m_plane;
}
