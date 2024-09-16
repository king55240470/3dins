#ifndef CENTITYMGR_H
#define CENTITYMGR_H

#include <geometry/centity.h>
#include <vtkActor.h>
#include <vtkSmartPointer.h>

class CEntityMgr : public CEntity
{
private:


    int m_nSize;
    int m_nCount;
    bool m_bRedraw;
    int m_nRedrawIndex;
public:
    CEntityMgr();
     // 封装centity对象的数组
    //virtual void action() const = 0;
    virtual ~CEntityMgr() {}
    void Add(CEntity*);
    QVector<CEntity*> m_entityList;
    int FindEntity(CEntity*);
    CEntity* GetAt(int);
    CEntity* FindEntityById(int);
    CEntity* FindEntityByName(QString);
    int GetCount();
    void RemoveAll();
    QVector<CEntity*> getEntityList();
    // 遍历m_entityList重新绘制
    void reDraw();
};

#endif // CENTITYMGR_H
