#ifndef COBJECTMGR_H
#define COBJECTMGR_H
#include "geometry/cobject.h"
#include "component/elementlistwidget.h"

class CObjectMgr
{
public:
    QVector<CObject*> m_objectList;
    int m_nInsertPos=-1;
    bool m_bHideInvalid;
    int m_colorGroup[0x2];
    int m_nGroupIndex;
public:
    CObjectMgr();
    void Add(CObject*);
    void Insert(CObject*, int index);
    int FindIndex(CObject*);

    //void FindObjectByType(ENTITY_TYPE,  QVector<CObject*>);

    CObject* GetAt(int);
    int GetCount();
    int GetInsertPos();
    bool IsEmpty();
    //bool IsHideInvalid();
    void RemoveAll();
    void SelectAll(bool);
    CObject* findObject();
    void upadteelementlist();
};

#endif // COBJECTMGR_H
