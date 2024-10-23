#ifndef RECTANGLECONSTRUCTOR_H
#define RECTANGLECONSTRUCTOR_H
#include"geometry/centitytypes.h"
#include"planeconstructor.h"
#include"constructor.h"
class RectangleConstructor:public Constructor
{
private:
    CPlane m_rectangle;
public:
    RectangleConstructor();
    CEntity* create(QVector<CEntity*>& entitylist)override;
    CPlane* createRectangle(CPosition p1,CPosition p2,CPosition p3);
    CPlane* createRectangle(CPosition p1,CPosition p2,CPosition p3,CPosition p4);
    CPlane* createRectangle(CPosition posCenter, QVector4D normal, QVector4D direction, double length, double width);
};

#endif // RECTANGLECONSTRUCTOR_H
