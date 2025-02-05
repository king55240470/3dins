#include "centity.h"
#include <QString>
#include "manager/cpcsmgr.h"

//QList<CEntity*> CEntity::m_ConstructList;
CEntity::CEntity() {
    m_pRefCoord=CPcsMgr::m_pPcsCurrent;
    m_pCurCoord=CPcsMgr::m_pPcsCurrent;
    m_pExtCoord=CPcsMgr::m_pPcsCurrent;

    m_bShowCNameLabel=false;
    m_nRef=0;
    m_bRemeasure=false;
}

CPcs *CEntity::GetRefCoord(){
    return m_pRefCoord;
}
CPcs *CEntity::GetCurCoord(){
    return m_pCurCoord;
}
CPcs *CEntity::GetExtCoord(){
    return m_pExtCoord;
}
void CEntity::SetRefCoord(CPcs *pRefCs){
    m_pRefCoord = pRefCs;
}
void CEntity::SetCurCoord(CPcs *pCurCs){
    m_pCurCoord = pCurCs;
}
void CEntity::SetExtCoord(CPcs *pExtCs){
    m_pExtCoord = pExtCs;
}

QVector<CEntity*> CEntity::GetConstructList(){
    return m_ConstructList;
}

/*void CEntity::SetImageToolList(QVector<CImageTool *> toolList)
{
    m_ToolArray = toolList;
}*/
/*QVector<CImageTool *> CEntity::GetImageToolList(){
    return m_ToolArray;
}*/

bool CEntity::IsShowCNameLabel(){
    return m_bShowCNameLabel;
}

int CEntity::AddRef(CShape*){
    m_nRef++;
    return m_nRef;
}
int CEntity::DelAllRefShape(){
    m_nRef = 0;
    return 0;
}

void CEntity::SetRemeasure(bool bRemeasure){
    m_bRemeasure = bRemeasure;
}

bool CEntity::IsCanAutoMeasure(){
    return true;
}
bool CEntity::IsMeasureByImage(){
    return true;
}
bool CEntity::IsMeasureByLaser(){
    return false;
}
bool CEntity::IsMeasureByProbe(){
    return false;
}

bool CEntity::IsType(ENTITY_TYPE type){
    if(type==m_EntityType){
        return true;
    }else{
        return false;
    }
}

void CEntity::setEntityType(ENTITY_TYPE type){
    m_EntityType = type;
}
ENTITY_TYPE CEntity::getEntityType() const{
    return m_EntityType;
}

void CEntity::SetCompenType(COMPENSATE_TYPE type){
    m_CompenType = type;
}
COMPENSATE_TYPE CEntity::GetCompenType(){
    return m_CompenType;
}

void CEntity::SetCreateForm(CREATE_FORM form){
    m_CreateForm = form;
}
CREATE_FORM CEntity::GetCreateForm(){
    return m_CreateForm;
}

//序列化
QDataStream& CEntity::serialize(QDataStream& out) const {
    CObject::serialize(out);//序列化基类部分
    out<<*m_pRefCoord<<*m_pCurCoord<<*m_pExtCoord;

    // out<<static_cast<int>(m_ConstructList.size());
    // qDebug()<<"constructListSize:"<<m_ConstructList.size();
    // for(const auto* entity:m_ConstructList){//auto让编译器自动推断元素的类型
    //     out<<*entity;
    // }

    out<<m_bShowCNameLabel
        <<m_nRef
        <<m_bRemeasure;
    return out;
}



//反序列化
QDataStream& CEntity::deserialize(QDataStream& in) {
    CObject::deserialize(in);//反序列化基类部分
    in>>*m_pRefCoord>>*m_pCurCoord>>*m_pExtCoord;//解引用‘*’将数据流中的内容读取到已经存在的对象中，确保将读取的数据填充到对象内部

    // int constructListSize;
    // in>>constructListSize;
    // qDebug()<<"constructListSize:"<<constructListSize;
    // m_ConstructList.clear();
    // for(int i=0;i<constructListSize;i++){
    //     CEntity *entity=new CEntity();
    //     in>>*entity;
    //     m_ConstructList.append(entity);
    // }

    in>>m_bShowCNameLabel
        >>m_nRef
        >>m_bRemeasure;
    return in;
}
