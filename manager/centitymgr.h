#ifndef CENTITYMGR_H
#define CENTITYMGR_H

#include <geometry/centity.h>
#include <vtkActor.h>
#include <vtkSmartPointer.h>

class CEntityMgr : public CEntity
{
private:
    QVector<CEntity*> m_entityList; // 封装centity对象的数组

    int m_nSize;
    int m_nCount;
    bool m_bRedraw;
    int m_nRedrawIndex;
public:
    CEntityMgr();
    //virtual void action() const = 0;
    virtual ~CEntityMgr() {}
    void Add(CEntity*);

    int FindEntity(CEntity*);
    CEntity* GetAt(int);
    CEntity* FindEntityById(int);
    CEntity* FindEntityByName(QString);
    int GetCount();
    void RemoveAll();

    // 获取m_entityList中新加入的元素
    CEntity* getLatestEntity();
};

#endif // CENTITYMGR_H
