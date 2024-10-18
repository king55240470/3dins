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
    //CDistance* setDistance(CPosition p1,CPlane plane,double uptolerance,double undertolerance);
    //CDistance* setDistance(CPoint p1,CPlane plane,double uptolerance,double undertolerance);
    //CDistance* getDistance();
    //CDistance* createDistance(CPosition p1,CPlane plane);
    CDistance* createDistance(CPoint *p1,CPlane *plane);
    CDistance* createDistance(CPoint *p1,CPoint *point);
    CDistance* createDistance(CPoint *p1,CCircle *circle);
    CDistance* createDistance(CPoint *p1,CLine *line);
    void createTolerance(double up,double under);
};

#endif // DISTANCECONSTRUCTOR_H
