#include "cobject.h"


CObject::~CObject() {}

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

