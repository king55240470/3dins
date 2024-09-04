#include "centity.h"

CEntity::CEntity() {

}
CEntity::~CEntity() {

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
