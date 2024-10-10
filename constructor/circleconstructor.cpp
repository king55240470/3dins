#include "circleconstructor.h"
 QVector4D  toQVector4D(CPosition P){
    return QVector4D(P.x,P.y,P.z,1.0);
}
 CPosition toCPosition(QVector4D P){
    return CPosition(P.x(),P.y(),P.z());
}
 QVector4D crossProduct(const QVector4D& a, const QVector4D& b) {
    return QVector4D(
        a.y() * b.z() - a.z() * b.y(),
        a.z() * b.x() - a.x() * b.z(),
        a.x() * b.y() - a.y() * b.x(),
        0 // 对于三维向量，w 组件通常设为 0
        );
};
  double distanceToPlane(const QVector4D& point, const QVector4D& planePoint, const QVector4D& normal) {
     // 计算从平面上的点到外部点的向量
     QVector4D d = point - planePoint;

     // 计算法向量的单位向量
     QVector4D unitNormal = normal.normalized();

     // 计算点到平面的距离
     double distance = QVector4D::dotProduct(d, unitNormal);

     return distance;
 }
CircleInfor centerCircle3d(CPosition p1,CPosition p2,CPosition p3)
{
    double x1=p1.x;
    double y1=p1.y;
    double z1=p1.z;
    double x2=p2.x;
    double y2=p2.y;
    double z2=p2.z;
    double x3=p3.x;
    double y3=p3.y;
    double z3=p3.z;
    double x,y,z,radius;
    double a1 = (y1*z2 - y2*z1 - y1*z3 + y3*z1 + y2*z3 - y3*z2),
        b1 = -(x1*z2 - x2*z1 - x1*z3 + x3*z1 + x2*z3 - x3*z2),
        c1 = (x1*y2 - x2*y1 - x1*y3 + x3*y1 + x2*y3 - x3*y2),
        d1 = -(x1*y2*z3 - x1*y3*z2 - x2*y1*z3 + x2*y3*z1 + x3*y1*z2 - x3*y2*z1);

    double a2 = 2 * (x2 - x1),
        b2 = 2 * (y2 - y1),
        c2 = 2 * (z2 - z1),
        d2 = x1*x1 + y1*y1 + z1*z1 - x2*x2 - y2*y2 - z2*z2;

    double a3 = 2 * (x3 - x1),
        b3 = 2 * (y3 - y1),
        c3 = 2 * (z3 - z1),
        d3 = x1*x1 + y1*y1 + z1*z1 - x3*x3 - y3*y3 - z3*z3;

    x = -(b1*c2*d3 - b1*c3*d2 - b2*c1*d3 + b2*c3*d1 + b3*c1*d2 - b3*c2*d1)
        / (a1*b2*c3 - a1*b3*c2 - a2*b1*c3 + a2*b3*c1 + a3*b1*c2 - a3*b2*c1);
    y = (a1*c2*d3 - a1*c3*d2 - a2*c1*d3 + a2*c3*d1 + a3*c1*d2 - a3*c2*d1)
        / (a1*b2*c3 - a1*b3*c2 - a2*b1*c3 + a2*b3*c1 + a3*b1*c2 - a3*b2*c1);
    z = -(a1*b2*d3 - a1*b3*d2 - a2*b1*d3 + a2*b3*d1 + a3*b1*d2 - a3*b2*d1)
        / (a1*b2*c3 - a1*b3*c2 - a2*b1*c3 + a2*b3*c1 + a3*b1*c2 - a3*b2*c1);
    radius = sqrt((x1 - x)*(x1 - x) + (y1 - y)*(y1 - y) + (z1 - z)*(z1 - z));
    return CircleInfor(QVector4D(x,y,z,1.0),QVector4D(0,0,0,0),radius);
}
CircleInfor calculateCircle(const CPosition &A, const CPosition &B, const CPosition &C){
    CircleInfor Circle=centerCircle3d(A,B,C);
    QVector4D A_vec(A.x, A.y, A.z, 1);
    QVector4D B_vec(B.x, B.y, B.z, 1);
    QVector4D C_vec(C.x, C.y, C.z, 1);

    // 计算向量 AB 和 AC
    QVector4D AB = B_vec - A_vec;
    QVector4D AC = C_vec - A_vec;


    // 计算法向量 N

    QVector4D N = crossProduct(AB, AC);
    if(N.length()<=1e-6){
        Circle.radius=0;
    }

    QVector4D dirN = N.normalized();

    // 更新圆心
    Circle.normal=dirN;

    return Circle;
}

CircleConstructor::CircleConstructor() {}
CEntity* CircleConstructor::create(QVector<CEntity*>& entitylist){
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
        return createCircle(*points[0],*points[1],*points[2]);
    }
    return nullptr;
}

CCircle* CircleConstructor::createCircle(CPosition p1,CPosition p2,CPosition p3){
    CircleInfor Circle=calculateCircle(p1,p2,p3);
    if(Circle.radius<1e-5)
        return nullptr;
    return createCircle(toCPosition(Circle.center),Circle.radius*2);
}
CCircle* CircleConstructor::createCircle(CPoint p1,CPoint p2,CPoint p3){
    CPosition pt1=p1.GetPt();
    CPosition pt2=p2.GetPt();
    CPosition pt3=p3.GetPt();
    return createCircle(pt1,pt2,pt3);
}
CCircle* CircleConstructor::createCircle(CPosition center,double diameter){
    CCircle* newCircle=new CCircle();
    newCircle->SetCenter(center);
    newCircle->SetDiameter(diameter);
    return newCircle;
}
QVector4D CircleConstructor::getNormal(){
    return m_normal;
}
