#include "centity.h"

#include <QString>

QList<CObject*> CEntity::m_ConstructList;
CEntity::CEntity(QVector<CObject *> List,QString name,QString Autoname,int id):CObject(name,Autoname,id)
{
    m_ConstructList=List;
}



QVector<CObject*> CEntity::GetConstructList(){
    return m_ConstructList;
}
