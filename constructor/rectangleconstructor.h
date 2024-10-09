#ifndef RECTANGLECONSTRUCTOR_H
#define RECTANGLECONSTRUCTOR_H
#include"geometry/centitytypes.h"
#include"planeconstructor.h"
#include"constructor.h"
class RectangleConstructor
{
private:
    CPlane m_rectangle;
public:
    RectangleConstructor();
    bool setRectangle(QVector<CEntity*>& entitylist);
    bool setRectangle(CPosition p1,CPosition p2,CPosition p3);
    bool setRectangle(CPoint p1,CPoint p2,CPoint p3);
    bool setRectangle(CPosition p1,CPosition p2,CPosition p3,CPosition p4);
    bool setRectangle(CPoint p1,CPoint p2,CPoint p3,CPoint p4);
    CPlane getRectabgle();

};

#endif // RECTANGLECONSTRUCTOR_H
