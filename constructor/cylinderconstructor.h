#ifndef CYLINDERCONSTRUCTOR_H
#define CYLINDERCONSTRUCTOR_H
#include"geometry/centitytypes.h"
#include"circleconstructor.h"
#include"constructor.h"
class CylinderConstructor
{
private:
    CCylinder m_cylinder;
public:
    CylinderConstructor();
    bool setCylinder(QVector<CEntity*>& entitylist);
    bool setCylinder(CPosition p1,CPosition p2,CPosition p3,CPosition p4);
    bool setCylinder(CPoint p1,CPoint p2,CPoint p3,CPoint p4);
    CCylinder getCylinder();
};

#endif // CYLINDERCONSTRUCTOR_H
