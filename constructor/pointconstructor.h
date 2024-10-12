#ifndef POINTCONSTRUCTOR_H
#define POINTCONSTRUCTOR_H
#include"geometry/centitytypes.h"
#include"constructor.h"
class PointConstructor
{
private:
    CPoint m_point;

public:
    PointConstructor();
    bool setPoint(QVector<CEntity*>& );
    bool setPoint(CPosition);
    bool setPoint(CPoint);
    CPoint getPoint();
};

#endif // POINTCONSTRUCTOR_H
