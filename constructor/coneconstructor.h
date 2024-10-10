#ifndef CONECONSTRUCTOR_H
#define CONECONSTRUCTOR_H
#include"geometry/centitytypes.h"
#include"circleconstructor.h"
#include"constructor.h"
class ConeConstructor
{
private:
    CCone m_cone;
public:
    ConeConstructor();
    bool setCone(QVector<CEntity*>& entitylist);
    bool setCone(CPosition p1,CPosition p2,CPosition p3);
    bool setCone(CPoint p1,CPoint p2,CPoint p3);
    CCone getCone();
};

#endif // CONECONSTRUCTOR_H
