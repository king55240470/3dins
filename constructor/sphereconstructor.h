#ifndef SPHERECONSTRUCTOR_H
#define SPHERECONSTRUCTOR_H
#include"geometry/centitytypes.h"
#include"constructor.h"
class SphereConstructor
{
private:
    CSphere m_sphere;
public:
    SphereConstructor();
    bool setSphere(QVector<CEntity*>& entitylist);
    bool setSphere(CPosition p1,CPosition p2,CPosition p3,CPosition p4);
    bool setSphere(CPoint p1,CPoint p2,CPoint p3,CPoint p4);
    CSphere getSphere();

};

#endif // SPHERECONSTRUCTOR_H
