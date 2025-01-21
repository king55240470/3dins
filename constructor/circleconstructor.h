#ifndef CIRCLECONSTRUCTOR_H
#define CIRCLECONSTRUCTOR_H
#include<QVector4D>
#include"geometry/centitytypes.h"
#include"constructor.h"
//通用工具类 其他的constructor也要用到
class CircleInfor{
public:
    QVector4D center;
    QVector4D normal;
    double radius;

    CircleInfor(){}
    CircleInfor(QVector4D center,QVector4D normal,double radius){
        this->center=center;
        this->radius=radius;
        this->normal=normal;
    }
};
CPosition toCPosition(QVector4D P);
CircleInfor centerCircle3d(CPosition p1,CPosition p2,CPosition p3);
CircleInfor calculateCircle(const CPosition &A, const CPosition &B, const CPosition &C);
QVector4D crossProduct(const QVector4D& a, const QVector4D& b);
QVector4D  toQVector4D(CPosition P);
double distanceToPlane(const QVector4D& point, const QVector4D& planePoint, const QVector4D& normal);


class CircleConstructor:public Constructor
{
private:
    CCircle m_circle;
    QVector4D m_normal;
public:
    CircleConstructor();
    CEntity* create(QVector<CEntity*>& entitylist)override;
    CCircle* createCircle(CPosition p1,CPosition p2,CPosition p3);
    CCircle* createCircle(CPoint *p1,CPoint *p2,CPoint *p3);
    CCircle* createCircle(CPosition center,double diameter,QVector4D normal);
    QVector4D getNormal();

};

#endif // CIRCLECONSTRUCTOR_H
