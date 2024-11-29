#ifndef CONSTRUCTOR_H
#define CONSTRUCTOR_H
#include"geometry/centitytypes.h"
#include "geometry/centity.h"
enum WrongInformation{
    PointTooLess,
    PointTooMuch,
    PointInOneLine,
    PointInOnePlane,
    PointTooClose,
    Unkown,
    PointDontMatch,
    Null,
    SourceCloudNull,
    SourceEntityLess,
    SourceEntityMuch
};

class Constructor
{
protected:
    QVector<CPosition>positions;//记录有效点
    WrongInformation m_wrongInfo;
public:
    virtual CEntity* create(QVector<CEntity*>& entitylist);
    Constructor();
    QVector<CPosition>& getPositions();
    void setWrongInformation(WrongInformation wrongInfo){
        m_wrongInfo= wrongInfo;
    }
    WrongInformation getWrongInformation(){
        return m_wrongInfo;
    }
};

#endif // CONSTRUCTOR_H
