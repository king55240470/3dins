#include "coneconstructor.h"

void calculateCircleCenterAndNormal(const QVector3D& p1, const QVector3D& p2, const QVector3D& p3, QVector3D& center, QVector3D& normal) {
    QVector3D v1 = p2 - p1;
    QVector3D v2 = p3 - p1;

    // 叉积计算法向量
    normal = QVector3D::crossProduct(v1, v2).normalized();

    // 计算中点
    QVector3D mid1 = (p1 + p2) / 2.0f;
    QVector3D mid2 = (p1 + p3) / 2.0f;

    // 计算中点的法向量
    QVector3D normal1 = QVector3D::crossProduct(v1, normal).normalized();
    QVector3D normal2 = QVector3D::crossProduct(v2, normal).normalized();

    // 计算直线交点（圆心）
    float a1 = normal1.x(), b1 = normal1.y(), c1 = normal1.z();
    float a2 = normal2.x(), b2 = normal2.y(), c2 = normal2.z();
    float d1 = -(a1 * mid1.x() + b1 * mid1.y() + c1 * mid1.z());
    float d2 = -(a2 * mid2.x() + b2 * mid2.y() + c2 * mid2.z());

    float det = a1 * b2 - a2 * b1;
    if (det == 0) {
        std::cerr << "Points are collinear!" << std::endl;
        return;
    }

    center.setX((b1 * d2 - b2 * d1) / det);
    center.setY((a2 * d1 - a1 * d2) / det);
    center.setZ(-(a1 * center.x() + b1 * center.y() + d1) / c1);
}

// 计算圆锥的参数
void calculateConeParameters(const QVector3D& p1, const QVector3D& p2, const QVector3D& p3, const QVector3D& vertex,
                             QVector3D& center, QVector3D& normal, float& height, float& halfAngle) {
    // 计算底面圆的中心和法向量
    calculateCircleCenterAndNormal(p1, p2, p3, center, normal);

    // 计算顶点到底面圆的高度
    height = QVector3D::dotProduct(vertex - center, normal) / normal.length();

    // 计算底面半径（使用第一个点和中心点的距离）
    float radius = (p1 - center).length();

    // 计算半角
    halfAngle = atan(radius / height);
}

ConeConstructor::ConeConstructor() {}
CEntity* ConeConstructor::create(QVector<CEntity*>& entitylist){
    Constructor::create(entitylist);
    QVector<CPoint*>points;
    QVector<CPosition> positions;
    for(int i=0;i<entitylist.size();i++){
        CEntity* entity=entitylist[i];
        if(!entity->IsSelected())continue;
        if(entity->GetUniqueType()==enPoint){
            CPoint * point=(CPoint*)entity;
            points.push_back(point);
            positions.push_back(point->GetPt());
        }
    }

    if(positions.size()==4){
        CCone*cone=createCone(positions[0],positions[1],positions[2],positions[3]);
        cone->setCurrentId();
        cone->parent.push_back(points[0]);
        cone->parent.push_back(points[1]);
        cone->parent.push_back(points[2]);
        cone->parent.push_back(points[3]);
        return cone;
    }
    if(points.size()<4){
        setWrongInformation(PointTooLess);
    }else if(points.size()>4){
        setWrongInformation(PointTooMuch);
    }
    return nullptr;
}
CCone* ConeConstructor::createCone(CPosition p1,CPosition p2,CPosition p3,CPosition p4){
    QVector3D P1(p1.x, p1.y, p1.z);
    QVector3D P2(p2.x, p2.y, p2.z);
    QVector3D P3(p3.x, p3.y, p3.z);
    QVector3D vertex(p4.x, p4.y, p4.z);

    QVector3D center, normal;
    float height, halfAngle;

    // 计算圆锥的参数
    calculateConeParameters(P1, P2, P3, vertex, center, normal, height, halfAngle);
    CPosition posCenter(center.x(),center.y(),center.z());
    QVector4D axisVector(normal.x(), normal.y(),  normal.z(), 0);
    qDebug()<<"posCenter";
    qDebug()<<posCenter.x<<posCenter.y<<posCenter.z;
    qDebug()<<"axisVector";
    qDebug()<<axisVector.x()<<axisVector.y()<<axisVector.z();
    qDebug()<<"height";
    qDebug()<<height;
    qDebug()<<"halAngle*2";
    qDebug()<<halfAngle*2;
    return createCone(posCenter,axisVector,height,height,halfAngle*2);
}
CCone* ConeConstructor::createCone(CPosition posCenter, QVector4D axis, double partH, double fullH, double angle){
    CCone* newCone=new CCone();
    newCone->setCurrentId();
    newCone->setCone_height(partH);
    newCone->setHeight(fullH);
    newCone->setAxis(axis);
    newCone->setRadian(angle);
    newCone->setVertex(posCenter);
    return newCone;

}
