#ifndef PLANECONSTRUCTOR_H
#define PLANECONSTRUCTOR_H
#include"geometry/centitytypes.h"
#include"constructor.h"
class PlaneConstructor
{
private:
    CPlane m_plane;
public:
    PlaneConstructor();
    bool setPlane(QVector<CEntity*>& entitylist);
    bool setPlane(CPosition p1,CPosition p2,CPosition p3);
    bool setPlane(CPoint p1,CPoint p2,CPoint p3);
    bool setPlane(CPlane plane);
    CPlane getPlane();
};

#endif // PLANECONSTRUCTOR_H
