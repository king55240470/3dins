#ifndef CONECONSTRUCTOR_H
#define CONECONSTRUCTOR_H
#include"geometry/centitytypes.h"
#include"circleconstructor.h"
#include"constructor.h"
class ConeConstructor:public Constructor
{
private:
    CCone m_cone;
public:
    ConeConstructor();
    CEntity* create(QVector<CEntity*>& entitylist)override;
    CCone* createCone(CPosition p1,CPosition p2,CPosition p3,CPosition p4);
    CCone* createCone(CPosition posCenter, QVector4D axis, double partH, double fullH, double angle);

};

#endif // CONECONSTRUCTOR_H
