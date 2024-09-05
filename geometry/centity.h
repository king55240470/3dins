#ifndef CENTITY_H
#define CENTITY_H
#include <QVector>
#include "cobject.h"

class CEntity : public CObject
{
public:
    CEntity(QVector<CObject*> List,QString name,QString Autoname,int id=0);
    QVector<CObject*>GetConstructList();
private:
    static QVector<CObject*> m_ConstructList;

};

#endif // CENTITY_H
