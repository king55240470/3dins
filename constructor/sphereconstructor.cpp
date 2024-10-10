#include "sphereconstructor.h"

struct SphereInfo {
    QVector4D center;
    double radius;
    SphereInfo(QVector4D center,double radius){
        this->center=center;
        this->radius=radius;
    }
};

static auto checkCollinearity=[](const QVector4D& p1, const QVector4D& p2, const QVector4D& p3) {
    QVector4D v1 = p2 - p1;
    QVector4D v2 = p3 - p1;
    return qFabs(QVector4D::dotProduct(v1, v2) / (v1.length() * v2.length())) == 1.0; // 若夹角为0或180度则共线
};

static auto canFormSphere=[](const QVector4D& p1, const QVector4D& p2, const QVector4D& p3, const QVector4D& p4) {
    return !checkCollinearity(p1, p2, p3); // 至少需要三个不共线的点
};

static auto calculateSphere=[](const QVector4D& p1, const QVector4D& p2, const QVector4D& p3, const QVector4D& p4) {
    QVector4D v1 = p2 - p1;
    QVector4D v2 = p3 - p1;
    QVector4D v3 = p4 - p1;

    double a = QVector4D::dotProduct(v1, v1);
    double b = QVector4D::dotProduct(v2, v2);
    double c = QVector4D::dotProduct(v3, v3);

    double d = 2 * (v1.x() * v2.y() - v2.x() * v1.y() + v1.x() * v3.y() - v3.x() * v1.y() + v2.x() * v3.y() - v2.y() * v3.x());

    if (qFabs(d) < 1e-5) {
        return SphereInfo(QVector4D(0,0,0,0),0);
    }

    QVector4D center;
    center.setX(((v1.x() * v1.x() + v1.y() * v1.y()) * (v2.y() - v3.y()) +
                 (v2.x() * v2.x() + v2.y() * v2.y()) * (v3.y() - v1.y()) +
                 (v3.x() * v3.x() + v3.y() * v3.y()) * (v1.y() - v2.y())) / d);

    center.setY(((v1.x() * v1.x() + v1.y() * v1.y()) * (v3.x() - v2.x()) +
                 (v2.x() * v2.x() + v2.y() * v2.y()) * (v1.x() - v3.x()) +
                 (v3.x() * v3.x() + v3.y() * v3.y()) * (v2.x() - v1.x())) / d);

    center.setZ(0); // 这里假设球在xy平面上

    // 计算半径
    double radius = (center - p1).length();

    return SphereInfo(center,radius);
};
SphereConstructor::SphereConstructor() {}
bool SphereConstructor::setSphere(QVector<CEntity*>& entitylist){
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
        return setSphere(*points[0],*points[1],*points[2],*points[3]);
    }
    return false;
}
bool SphereConstructor::setSphere(CPosition pt1,CPosition pt2,CPosition pt3,CPosition pt4){
    CPosition A=pt1;
    CPosition B=pt2;
    CPosition C=pt3;
    CPosition D=pt4;
    QVector<QVector4D> points;
    QVector4D p1(A.x, A.y, A.z, 1.0);
    QVector4D p2(B.x, B.y, B.z, 1.0);
    QVector4D p3(C.x, C.y, C.z, 1.0);
    QVector4D p4(D.x, D.y, D.z, 1.0);

    if (canFormSphere(p1, p2, p3, p4)) {
        SphereInfo sphereInfo = calculateSphere(p1, p2, p3, p4);
        if(sphereInfo.radius==0){
            return false;
        }else{
            CPosition center(sphereInfo.center.x(),sphereInfo.center.y(),sphereInfo.center.z());
            m_sphere.setCenter(center);
            m_sphere.setDiameter(sphereInfo.radius*2);
            return true;
        }

    }
    return false;


}
bool SphereConstructor::setSphere(CPoint p1,CPoint p2,CPoint p3,CPoint p4){
    CPosition pt1=p1.GetPt();
    CPosition pt2=p2.GetPt();
    CPosition pt3=p3.GetPt();
    CPosition pt4=p4.GetPt();
    return setSphere(pt1,pt2,pt3,pt4);
}
CSphere SphereConstructor::getSphere(){
    return m_sphere;
}
