#include "cshape.h"

CShape::CShape()
{}

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
