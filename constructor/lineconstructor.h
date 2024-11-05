#ifndef LINECONSTRUCTOR_H
#define LINECONSTRUCTOR_H
#include"geometry/centitytypes.h"
#include"constructor.h"
class LineConstructor:public Constructor
{
private:
    CLine m_line;
public:
    LineConstructor();
    CEntity* create(QVector<CEntity*>& entitylist)override;
    CLine* createLine(CPosition begin,CPosition end);
    CLine* createLine(CPoint *begin,CPoint *end);
    CLine* createLine(CCircle* ,CCircle*);
    CLine * createLine(CPoint*,CPlane*);
};

#endif // LINECONSTRUCTOR_H
