#ifndef COBJECTMGR_H
#define COBJECTMGR_H
#include "geometry/cobject.h"
#include "component/elementlistwidget.h"
class CObjectMgr
{
private:
    QVector<CObject*> m_objectList;
public:
    CObjectMgr();
    void Add(CObject*);
    void Insert(CObject*, int index);
    int FindIndex(CObject*);

    //void FindObjectByType(ENTITY_TYPE,  QVector<CObject*>);

    CObject* GetAt(int);
    int GetCount();
    //int GetInsertPos();
    bool IsEmpty();
    //bool IsHideInvalid();
    void RemoveAll();
    void SelectAll(bool);
    CObject* findObject();
};

#endif // COBJECTMGR_H
