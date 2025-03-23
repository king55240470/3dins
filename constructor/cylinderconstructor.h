#ifndef CYLINDERCONSTRUCTOR_H
#define CYLINDERCONSTRUCTOR_H
#include"geometry/centitytypes.h"
#include"circleconstructor.h"
#include"constructor.h"
class CylinderConstructor:public Constructor
{
private:
    CCylinder m_cylinder;
public:
    CylinderConstructor();
    CEntity* create(QVector<CEntity*>& entitylist)override;
    CCylinder* createCylinder(CPosition p1,CPosition p2,CPosition p3,CPosition p4);
    CCylinder* createCylinder(CPosition pos, QVector4D vec, double height, double diametre);
    CCylinder* createCylinder(CPosition p1,CCircle* ciecle);
};

#endif // CYLINDERCONSTRUCTOR_H
