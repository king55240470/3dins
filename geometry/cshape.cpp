#include "cshape.h"

CShape::CShape()
{
    m_bUnderCursor=false;
    m_bSel=false;
    m_bDeleted=false;
    m_bShow=false;
    m_bGroup=false;
}

bool CShape::IsUnderCursor(){
    return m_bUnderCursor;
}
bool CShape::IsSelected(){
    return m_bSel;
}
bool CShape::IsDeleted(){
    return m_bDeleted;
}
bool CShape::IsShow(){
    return m_bShow;
}
bool CShape::IsGroup(){
    return m_bGroup;
}

void CShape::setUnderCursor(bool bUnderCursor){
    m_bUnderCursor = bUnderCursor;
}
void CShape::SetSelected(bool bSel){
    m_bSel = bSel;
}
void CShape::SetDeleted(bool bDel){
    m_bDeleted = bDel;
}
void CShape::setShow(bool bShow){
    m_bShow = bShow;
}
void CShape::SetGroup(bool bGroup){
    m_bGroup = bGroup;
}

void CShape::setDwAddress(uintptr_t newDwAddress){
    m_dwAddress = newDwAddress;
}

void CShape::SetstrInformation(QString str){
    m_strInformation = str;
}

//bool CShape::getUnderCursor() const{}
bool CShape::getSelected() const{
    return m_bSel;
}
bool CShape::getDeleted() const{
    return m_bDeleted;
}
//bool CShape::getShow() const{}
//bool CShape::getGroup() const{}

uintptr_t CShape::getdwAddress() const{
    return m_dwAddress;
}

void CShape::update()
{
    return;
}

CPcs *CShape::GetRefCoord(){
    return NULL;
}
CPcs *CShape::GetCurCoord(){
    return NULL;
}
CPcs *CShape::GetExtCoord(){
    return NULL;
}

//序列化
QDataStream& operator<<(QDataStream& out,const CShape& shape){
    out<<shape.m_bUnderCursor
        <<shape.m_bSel
        <<shape.m_bDeleted
        <<shape.m_bShow
        <<shape.m_bGroup;
        // <<static_cast<quint64>(shape.m_dwAddress)//转换为64位整数以确保跨平台一致性
        // <<shape.m_strInformation;
    // qDebug()<<"address:"<<static_cast<quint64>(shape.m_dwAddress);
    return out;
}
//反序列化
QDataStream& operator>>(QDataStream& in,CShape& shape){
    // quint64 address;
    in>>shape.m_bUnderCursor
        >>shape.m_bSel
        >>shape.m_bDeleted
        >>shape.m_bShow
        >>shape.m_bGroup;
        // >>address//先读取为 64 位整数
        // >>shape.m_strInformation;
    // qDebug()<<"address:"<<address;
    // shape.m_dwAddress=static_cast<uintptr_t>(address);//转换回uintptr_t类型
    return in;
}
