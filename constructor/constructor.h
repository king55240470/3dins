#ifndef CONSTRUCTOR_H
#define CONSTRUCTOR_H
#include"geometry/centitytypes.h"
#include "geometry/centity.h"
class Constructor
{
private:
    QVector<CPosition>positions;//记录有效点
public:
    virtual CEntity* create(QVector<CEntity*>& entitylist);
    Constructor();
    QVector<CPosition>& getPositions();
};

#endif // CONSTRUCTOR_H
