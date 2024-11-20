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
    CDistance* createDistance(CPoint *p1,CPoint *point);
    CDistance* createDistance(CPoint *p1,CCircle *circle);
    CDistance* createDistance(CPoint *p1,CLine *line);
    CDistance* createDistance(CPlane* plane1, CPlane* plane2);
    void createTolerance(double up,double under);
};

#endif // DISTANCECONSTRUCTOR_H
