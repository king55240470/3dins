#ifndef CENTITYMGR_H
#define CENTITYMGR_H

#include <geometry/centity.h>
#include "component/elementlistwidget.h"
#include "component/vtkwidget.h"

class CEntityMgr : public CEntity
{
private:
    static QVector<CEntity*> m_entityList; // 封装centity对象的数组

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

    // 遍历m_entityList重新绘制
    static void reDraw();
};

#endif // CENTITYMGR_H
