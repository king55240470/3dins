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

    qsizetype size=mgr.m_objectList.size();
    qDebug()<<"objectListSize:"<<mgr.m_objectList.size();
    out << size;  // 序列化对象列表的大小
    for (const auto& object : mgr.m_objectList) {
        int typeInt = object->GetUniqueType();  // 获取对象的类型标识符
        ENTITY_TYPE type = static_cast<ENTITY_TYPE>(typeInt);
        out << type;  // 序列化类型标识符
        // qDebug()<<"type:"<<type;
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

    qsizetype objectListSize;
    in >> objectListSize;
    qDebug()<<"objectListSize:"<<objectListSize;
    mgr.RemoveAll();
    for (qsizetype i = 0; i < objectListSize; ++i) {
        int typeInt;
        in >> typeInt;
        ENTITY_TYPE type = static_cast<ENTITY_TYPE>(typeInt);
        // qDebug()<<"type:"<<type;

        CObject* object = nullptr;
        switch (type) {
        case enCircle:
            object = new CCircle();
            break;
        case enLine:
            object=new CLine();
            break;
        case enPoint:
            object=new CPoint();
            break;
        case enPlane:
            object=new CPlane();
            break;
        case enSphere:
            object=new CSphere();
            break;
        case enCylinder:
            object=new CCylinder();
            break;
        case enCone:
            object=new CCone();
            break;
        case enCuboid:
            object=new CCuboid();
            break;
        case enDistance:
            object=new CDistance();
            break;
        case enPointCloud:
            object=new CPointCloud();
            break;
        default:
            object = new CObject();
            break;
        }

        object->deserialize(in);  // 调用反序列化函数处理特有数据
        mgr.m_objectList.append(object);
        // mgr.Add(object);

    }

    return in;
}



