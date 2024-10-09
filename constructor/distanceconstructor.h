#ifndef DISTANCECONSTRUCTOR_H
#define DISTANCECONSTRUCTOR_H
#include"geometry/centitytypes.h"
#include"constructor.h"
class DistanceConstructor
{
private:
    CDistance m_distance;
public:
    DistanceConstructor();
    bool setDistance(QVector<CEntity*>& entitylist);
    bool setDistance(CPosition p1,CPosition p2,double uptolerance,double undertolerance);
    bool setDistance(CPoint p1,CPoint p2,double uptolerance,double undertolerance);
    CDistance getDistance();
};

#endif // DISTANCECONSTRUCTOR_H
