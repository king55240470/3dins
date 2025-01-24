#include "cobject.h"

CObject::CObject(){
    m_bMeasured=false;
    m_bChecked=false;
    m_bValid=false;
    m_bBreak=false;
    m_nCsForm=0;
    m_nObjectID=0;
}

void CObject::AddObjectCount()
{
    ++m_nObjectCount;
}

void CObject::SetID(int id)
{
    m_nObjectID=id;
}

bool CObject::IsMeasured(){
    return m_bMeasured;
}
bool CObject::IsChecked(){
    return m_bChecked;
}
bool CObject::IsValid(){
    return m_bValid;
}
bool CObject::IsBreak(){
    return m_bBreak;
}

void CObject::SetMeasured(bool bMeasured){
    m_bMeasured = bMeasured;
}
void CObject::SetChecked(bool){

}
void CObject::SetValid(bool bVal){
    m_bValid = bVal;
}
void CObject::SetBreak(bool bBreak){
    m_bBreak = bBreak;
}

QString CObject::GetObjectCName(){
    return m_strCName;
}
QString CObject::GetObjectAutoName(){
    return m_strAutoName;
}

void CObject::SetObjectCName(QString str){
    m_strCName = str;
}
void CObject::SetObjectAutoName(QString str){
    m_strAutoName = str;
}

int CObject::GetObjectID(){
    return m_nObjectID;
}

bool CObject::IsFormAxis(){
    if(m_nCsForm == 1)
        return true;
    else
        return false;
}
bool CObject::IsFormOrigin(){
    if(m_nCsForm == 2)
        return true;
    else
        return false;
}
bool CObject::IsFormPlanar(){
    if(m_nCsForm == 3)
        return true;
    else
        return false;
}

int CObject::GetState()
{
    // return nominal_base::GetEntityState;
    return 0;
}
int CObject::IsNeedPause()
{
    // return nominal_base::GetEntityIsNeedPause;
    return 0;
}
int CObject::InOOCRange()
{
    // return nominal_base::GetEntityIsInOOCRange;
    return 0;
}

QDataStream& CObject::serialize(QDataStream& out) const{
    out<<static_cast<const CShape&>(*this);//序列化基类部分
    out<<m_bMeasured
        <<m_bChecked
        <<m_bValid
        <<m_bBreak
        <<m_nCsForm
        <<m_strCName
        <<m_strAutoName
        <<m_nObjectID
        <<Form;

    // out<<static_cast<int>(parent.size());
    // qDebug()<<"parentSize:"<<parent.size();
    // for(const auto* obj:parent){//auto让编译器自动推断元素的类型
    //     out<<*obj;
    // }

    return out;
}
//反序列化（反序列化通常不使用const成员函数，因为反序列化过程需要修改对象的状态以恢复其原始数据）
QDataStream& CObject::deserialize(QDataStream& in){
    in>>static_cast<CShape&>(*this);//反序列化基类部分
    in>>m_bMeasured
        >>m_bChecked
        >>m_bValid
        >>m_bBreak
        >>m_nCsForm
        >>m_strCName
        >>m_strAutoName
        >>m_nObjectID
        >>Form;

    // int parentSize;
    // in>>parentSize;
    // qDebug()<<"parentSize:"<<parentSize;
    // parent.clear();
    // for(int i=0;i<parentSize;i++){
    //     CObject *obj=new CObject();
    //     in>>*obj;
    //     parent.append(obj);
    // }

    return in;
}

