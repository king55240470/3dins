#ifndef ANGLECONSTRUCTOR_H
#define ANGLECONSTRUCTOR_H
#include"geometry/centitytypes.h"
#include"constructor.h"
class AngleConstructor:public Constructor
{
public:
    AngleConstructor(){}
    CAngle* createAngle(CPoint* p1,CPoint* p2,CPoint*p3);
    CAngle* createAngle(CLine* line1,CLine* line2);
    CAngle* createAngle(CLine* line1,CLine* line2,double angleValue);
    CAngle* createAngle(CLine* line1,CLine* line2,double angleValue,CPosition vertex);
    CAngle* createAngle(CPlane* plane1,CPlane* plane2);
    CAngle* createAngle(CLine* line1,CPlane* plane1);
    CEntity* create(QVector<CEntity*>& entitylist)override;
};

#endif // ANGLECONSTRUCTOR_H
