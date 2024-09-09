#include "centitymgr.h"

CEntityMgr::CEntityMgr() {}
void CEntityMgr::Add(CEntity* pEntity){

    m_entityList.push_back(pEntity);
    ElementListWidget *element;
    element->judgetype(pEntity);
}


int CEntityMgr::FindEntity(CEntity *pEntity)
{
    int idx = m_entityList.indexOf(pEntity);

    return idx;
}

CEntity *CEntityMgr::GetAt(int idx)
{
    return m_entityList.at(idx);
}

CEntity *CEntityMgr::FindEntityById(int nID)
{
    CEntity *pEntity = NULL;
    for (CEntity *pEntityTemp : m_entityList) {
        if(pEntityTemp->m_nObjectID == nID)
        {
            pEntity = pEntityTemp;
            break;
        }
    }
    return pEntity;
}

CEntity* CEntityMgr::FindEntityByName(QString name)
{
    CEntity *pEntity = NULL;
    for (CEntity *pEntityTemp : m_entityList) {
        if(pEntityTemp->m_strAutoName == name)
        {
            pEntity = pEntityTemp;
            break;
        }
    }
    return pEntity;
}

int CEntityMgr::GetCount()
{
    return m_entityList.count();
}
void CEntityMgr::RemoveAll()
{
    for (CEntity *pEntityTemp : m_entityList) {
        delete pEntityTemp;
    }
    m_entityList.clear();
}
