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
    QVector<bool> marklist;//filemanagerwidget标记是否隐藏entitylist里的元素
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
    QVector<CEntity*>& getEntityList();
    QVector<bool> &getMarkList();
    QString getCEntityInfo() override;
    friend QDataStream& operator<<(QDataStream& out, const CEntityMgr& mgr);
    friend QDataStream& operator>>(QDataStream& in, CEntityMgr& mgr);
};

#endif // CENTITYMGR_H
