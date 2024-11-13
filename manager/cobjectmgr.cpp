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
int CObjectMgr::GetInsertPos()
{
    return m_nInsertPos;
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

QVector<CObject *> &CObjectMgr::getObjectList()
{
    return m_objectList;
}

QDataStream& operator<<(QDataStream& out, const CObjectMgr& mgr){
    // out<<mgr.m_objectList.count();
    // qDebug()<<"mgr.m_objectList.count:"<<mgr.m_objectList.count();

    // for(CObject* obj:mgr.m_objectList){
    //     if(obj){
    //         obj->serialize(out);
    //     }
    // }

    out << mgr.m_nInsertPos << mgr.m_bHideInvalid
        << mgr.m_colorGroup[0] << mgr.m_colorGroup[1] << mgr.m_nGroupIndex;

    out << static_cast<int>(mgr.m_objectList.size());  // 序列化对象列表的大小
    for (const auto& object : mgr.m_objectList) {
        int type = object->GetUniqueType();  // 获取对象的类型标识符
        out << type;  // 序列化类型标识符
        object->serialize(out);  // 调用对象的序列化方法（通过多态性处理具体类型）
    }

    return out;
}

QDataStream& operator>>(QDataStream& in, CObjectMgr& mgr){
    // mgr.RemoveAll();

    // qsizetype objectCount;
    // in >> objectCount;
    // qDebug()<<"objectcount:"<<objectCount;

    // // 反序列化每个对象
    // for (int i = 0; i < objectCount; ++i)
    // {
    //     // QString type;
    //     // in>>type;// "圆"

    //     // if(type.){
    //     //     CCircle* cir=new CCircle();
    //     //     cir->deserialize(in);
    //     // }

    //     //pcs->object
    //     CEntity* obj = new CEntity();
    //     obj->deserialize(in);
    //     mgr.Add(obj);
    // }

    in >> mgr.m_nInsertPos >> mgr.m_bHideInvalid
        >> mgr.m_colorGroup[0] >> mgr.m_colorGroup[1] >> mgr.m_nGroupIndex;

    int objectListSize;
    in >> objectListSize;
    mgr.RemoveAll();
    for (int i = 0; i < objectListSize; ++i) {
        int type;
        in >> type;

        CObject* object = new CEntity();
        switch (type) {
        case enCircle:
            object = new CCircle();
            break;
        // 其他类型处理逻辑...
        default:
            // object = new CObject();
            break;
        }

        object->deserialize(in);  // 调用反序列化函数处理特有数据
        mgr.m_objectList.append(object);
        // mgr.Add(object);

    }

    return in;
}



