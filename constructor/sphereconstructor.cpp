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
CEntity* SphereConstructor::create(QVector<CEntity*>& entitylist){
    QVector<CPosition>positions;//存储有效点
    QVector<CPoint*>points;
    QVector<CCircle*>circles;
    CCircle* Circle=nullptr;//存储圆
    for(int i=0;i<entitylist.size();i++){
        CEntity* entity=entitylist[i];
        if(!entity->IsSelected())continue;
        if(entity->GetUniqueType()==enPoint){
            CPoint * point=(CPoint*)entity;
            points.push_back(point);
            positions.push_back(point->GetPt());
        }else if(entity->GetUniqueType()==enCircle){
            CCircle* circle=(CCircle*)entity;
            circles.push_back(circle);
            positions.push_back(circle->getCenter());
            Circle=circle;
        }else if(entity->GetUniqueType()==enSphere){
            CSphere* sphere=(CSphere*)entity;
            positions.push_back(sphere->getCenter());
        }else if(entity->GetUniqueType()==enPlane){
            CPlane* plane=(CPlane*)entity;
            positions.push_back(plane->getCenter());
        }else if(entity->GetUniqueType()==enCone){
            CCone* cone=(CCone*)entity;
            positions.push_back(cone->getVertex());
        }else if(entity->GetUniqueType()==enCylinder){
            CCylinder* cylinder=(CCylinder*)entity;
            positions.push_back(cylinder->getBtm_center());
        }else if(entity->GetUniqueType()==enLine){
            CLine* line=(CLine*)entity;
            positions.push_back(line->getBegin());
            positions.push_back(line->getEnd());
        }
        ;
    }
    if(Circle==nullptr){
        qDebug()<<"构造球读取圆失败";
    }
    if(positions.size()==4){
        CSphere*sphere=createSphere(positions[0],positions[1],positions[2],positions[3]);
        sphere->parent.push_back(points[0]);
        sphere->parent.push_back(points[1]);
        sphere->parent.push_back(points[2]);
        sphere->parent.push_back(points[3]);
        return sphere;
    }
    if(Circle){
        CSphere*sphere=createSphere(Circle->getCenter(),Circle->getDiameter()/2);
        sphere->parent.push_back(circles[0]);
        return sphere;
    }
    return nullptr;
}
CSphere* SphereConstructor::createSphere(CPosition pt1,CPosition pt2,CPosition pt3,CPosition pt4){
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
            return nullptr;
        }else{
            CPosition center(sphereInfo.center.x(),sphereInfo.center.y(),sphereInfo.center.z());
            return createSphere(center,sphereInfo.radius);
        }

    }
    return nullptr;
}
CSphere* SphereConstructor::createSphere(CPosition center,double radius){
    CSphere* newSphere=new CSphere();
    newSphere->setCenter(center);
    newSphere->setDiameter(radius*2);
    return newSphere;
}
