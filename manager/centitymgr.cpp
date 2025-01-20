#include "centitymgr.h"
#include "geometry/centitytypes.h"
#include "geometry/globes.h"

CEntityMgr::CEntityMgr() {

}

void CEntityMgr::Add(CEntity* pEntity){
    m_entityList.push_back(pEntity);
    marklist.push_back(false);
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

QVector<CEntity *>& CEntityMgr::getEntityList()
{
    return m_entityList;
}

QVector<bool> &CEntityMgr::getMarkList()
{
    return marklist;
}

QString CEntityMgr::getCEntityInfo()
{
    return 0;
}

QDataStream& operator<<(QDataStream& out, const CEntityMgr& mgr) {
    out << mgr.m_nSize << mgr.m_nCount << mgr.m_bRedraw << mgr.m_nRedrawIndex;

    // 序列化 m_entityList
    out << static_cast<int>(mgr.m_entityList.size());
    for (const auto& entity : mgr.m_entityList) {
        int typeInt = entity->GetUniqueType();  // 获取对象的类型标识符
        ENTITY_TYPE type=static_cast<ENTITY_TYPE>(typeInt);
        out << type;  // 序列化类型标识符
        entity->serialize(out);  // 调用对象的序列化方法（通过多态性处理具体类型）
    }

    // // 序列化 m_SelList
    // out << static_cast<int>(mgr.m_SelList.size());
    // for (const auto& entity : mgr.m_SelList) {
    //     ENTITY_TYPE type = entity->GetEntityType();  // 获取对象的类型标识符
    //     out << type;  // 序列化类型标识符
    //     entity->serialize(out);  // 调用对象的序列化方法
    // }
    // qDebug()<<"m_entityList.size and m_SelList.size"<<mgr.m_entityList.size()<<mgr.m_SelList.size();
    CPointCloud::haveSaved=true;
    return out;
}

QDataStream& operator>>(QDataStream& in, CEntityMgr& mgr) {
    int entityListSize, selListSize;

    in >> mgr.m_nSize >> mgr.m_nCount >> mgr.m_bRedraw >> mgr.m_nRedrawIndex;

    // 反序列化 m_entityList
    in >> entityListSize;
    mgr.m_entityList.clear();  // 清空现有数据
    for (int i = 0; i < entityListSize; ++i) {
        int typeInt;
        in >> typeInt;
        ENTITY_TYPE type = static_cast<ENTITY_TYPE>(typeInt);

        CEntity* entity = nullptr;
        switch (type) {
        case enCircle:
            entity = new CCircle();
            break;
        case enLine:
            entity = new CLine();
            break;
        case enPoint:
            entity=new CPoint();
            break;
        case enPlane:
            entity=new CPlane();
            break;
        case enSphere:
            entity=new CSphere();
            break;
        case enCylinder:
            entity=new CCylinder();
            break;
        case enCone:
            entity=new CCone();
            break;
        case enCuboid:
            entity=new CCuboid();
            break;
        case enDistance:
            entity=new CDistance();
            break;
        case enPointCloud:
            entity=new CPointCloud();
            break;
        case enAngle:
            entity=new CAngle();
            break;
        default:
            entity = new CEntity();
            break;
        }

        if (entity) {
            entity->deserialize(in);  // 调用对象的反序列化方法
            mgr.m_entityList.append(entity);
            // mgr.Add(entity);
        }
    }

    CPointCloud::haveOpened=true;

    // // 反序列化 m_SelList
    // in >> selListSize;
    // mgr.m_SelList.clear();  // 清空现有数据
    // for (int i = 0; i < selListSize; ++i) {
    //     int typeInt;
    //     in >> typeInt;
    //     ENTITY_TYPE type = static_cast<ENTITY_TYPE>(typeInt);

    //     CEntity* entity = nullptr;
    //     switch (type) {
    //     case enCircle:
    //         entity = new CCircle();
    //         break;
    //     // 添加其他类型的处理逻辑
    //     case enLine:
    //         // entity = new CLine();
    //         break;
    //     // ...其他类型
    //     default:
    //         entity = new CEntity();
    //         break;
    //     }

    //     if (entity) {
    //         entity->deserialize(in);  // 调用对象的反序列化方法
    //         mgr.m_SelList.append(entity);
    //     }
    // }

    // qDebug() << "m_entityList.size and m_SelList.size" << entityListSize << selListSize;
    return in;
}



