#ifndef DISTANCECONSTRUCTOR_H
#define DISTANCECONSTRUCTOR_H
#include"geometry/centitytypes.h"
#include"constructor.h"
class DistanceConstructor:public Constructor
{
private:
    CDistance m_distance;
public:
    DistanceConstructor();
    CEntity* create(QVector<CEntity*>& entitylist)override;
    CDistance* createDistance(CPoint *p1,CPlane *plane);
    void createTolerance(double up,double under);
};

#endif // DISTANCECONSTRUCTOR_H
