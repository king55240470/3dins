#ifndef POINTCONSTRUCTOR_H
#define POINTCONSTRUCTOR_H
#include"geometry/centitytypes.h"
#include"constructor.h"
class PointConstructor:public Constructor
{
private:
    QVector<CPosition>positions;//存储有效点

public:
    PointConstructor();
    CEntity* create(QVector<CPosition>& chosenlist);
    CEntity* create(QVector<CEntity*>& entitylist)override;
    CPoint* createPoint(CPosition);
    CPoint* createPoint(CPoint&);
    CPoint* createPoint(double x,double y,double z);
    QVector<CPosition>& getPositions();
};

#endif // POINTCONSTRUCTOR_H
