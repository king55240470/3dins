#ifndef SPHERECONSTRUCTOR_H
#define SPHERECONSTRUCTOR_H
#include"geometry/centitytypes.h"
#include"constructor.h"
class SphereConstructor:public Constructor
{
private:
    CSphere m_sphere;
public:
    SphereConstructor();
    CEntity* create(QVector<CEntity*>& entitylist)override;
    CSphere* createSphere(CPosition p1,CPosition p2,CPosition p3,CPosition p4);

    CSphere* createSphere(CPosition center,double radius);
};

#endif // SPHERECONSTRUCTOR_H
