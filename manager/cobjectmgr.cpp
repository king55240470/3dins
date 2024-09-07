#include "cobjectmgr.h"
CObjectMgr::CObjectMgr() {}
void CObjectMgr::Add(CObject* pObject)
{
    m_objectList.push_back(pObject);
    pObject->AddObjectCount();
    pObject->SetID(m_objectList.size());
}
void CObjectMgr::Insert(CObject* newObject, int index) {
    if (index >= 0 && index <= m_objectList.size()) {
        m_objectList.insert(index, newObject);
        newObject->AddObjectCount();
        newObject->SetID(m_objectList.size());
    }
}
int CObjectMgr::FindIndex(CObject *obj)
{
    return m_objectList.indexOf(obj);
}



CObject *CObjectMgr::GetAt(int idx)
{
    return m_objectList.at(idx);
}

int CObjectMgr::GetCount()
{
    return m_objectList.count();
}



bool CObjectMgr::IsEmpty()
{
    return m_objectList.isEmpty();
}



void CObjectMgr::RemoveAll()
{
    for (CObject *pObj : m_objectList)
        delete pObj;
    m_objectList.clear();
}

void CObjectMgr::SelectAll(bool bVal)
{
    for (CObject *pObj : m_objectList)
        pObj->m_bSel= bVal;
}

