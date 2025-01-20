#ifndef PLANECONSTRUCTOR_H
#define PLANECONSTRUCTOR_H
#include"geometry/centitytypes.h"
#include"constructor.h"
class PlaneConstructor:public Constructor
{
private:
    CPlane m_plane;
public:
    PlaneConstructor();
    CEntity* create(QVector<CEntity*>& entitylist)override;
    CPlane* createPlane(CPosition p1,CPosition p2,CPosition p3);
    CPlane* createPlane(CPoint p1,CPoint p2,CPoint p3);
    CPlane* createPlane(CPlane* pl1,CPlane*pl2,double weight1_pl1=0.5);
    CPlane* createPlane(CPosition posCenter, QVector4D normal, QVector4D direction, double length, double width);
};

#endif // PLANECONSTRUCTOR_H
