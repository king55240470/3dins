#ifndef CONSTRUCTOR_H
#define CONSTRUCTOR_H
#include"geometry/centitytypes.h"
#include "geometry/centity.h"
class Constructor
{
public:
    virtual CEntity* create(QVector<CEntity*>& entitylist){return nullptr;}
    Constructor();
};

#endif // CONSTRUCTOR_H
