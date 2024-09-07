#include "cpcsmgr.h"

CPcsMgr::CPcsMgr()
{
    m_nCount=0;
    m_pNodeTemporary=NULL;
    m_bTempPcsNodeInUse=false;
}

void CPcsMgr::Initialize()
{
    //初始化机械坐标系（只有一个）
    CPcs* csMechanical= new CPcs();
    csMechanical->m_mat.setToIdentity(); //单位矩阵
    csMechanical->m_poso=CPosition(0,0,0); //机械坐标系原点
    csMechanical->m_nPcsID=1;
    csMechanical->m_nRef=-1;

    m_pPcsCurrent=csMechanical;

    //初始化一个CPcsNode
    CPcsNode::nPcsNodeCount=1;
    CPcsNode* pcsNode=new CPcsNode();
    pcsNode->pPcs=csMechanical;
    pcsNode->nPcsNodeId =1;
    pcsNode->nRefID=-1;

    m_nCount++;

    //将pcsNode加入列表中
    m_pHeadPcsNode=pcsNode;
    m_pTailPcsNode=pcsNode;
    m_PcsNodeList.push_back(pcsNode);
}

void CPcsMgr::AddCoordSys(CPcsNode* pPcsNode)
{
    m_PcsNodeList.push_back(pPcsNode);
    m_pPcsCurrent=pPcsNode->pPcs;
    m_pTailPcsNode=pPcsNode;
    m_nCount++;
}

CPosition CPcsMgr::GetLocalPosOfCertainPcs(CPosition pos,CPcs* pcs)
{
    QVector4D vec(pos.x,pos.y,pos.z,1); //全局坐标
    //坐标转换
    bool bres=false;
    QVector4D vec2=pcs->m_mat.inverted(&bres)*vec; //局部坐标
    pos.x=vec2.x(); pos.y = vec2.y();pos.z = vec2.z();
    return pos;
}

bool CPcsMgr::Erase(QString)
{
    return true;
}

void CPcsMgr::SetCurCoordSystem(int nPcsID, bool)
{
    for(CPcsNode* pPcsNode: m_PcsNodeList){
        if(pPcsNode->pPcs->m_nPcsID==nPcsID){
            m_pPcsCurrent=pPcsNode->pPcs;
        }
    }

    //更新实体当前坐标系
}

//未实现查找坐标系名称
void CPcsMgr::SetCurCoordSystem(QString)
{

    //更新实体当前坐标系
}

void CPcsMgr::SetCurCoordSystem(CPcs* pPcs)
{
    m_pPcsCurrent=pPcs;

    //更新实体当前坐标系
}

CPcs* CPcsMgr::GetBaseCoordSystem()
{
    return m_pHeadPcsNode->pPcs;
}

CPcs* CPcsMgr::GetCurCoordSystem()
{
    return m_pPcsCurrent;
}

CPcs* CPcsMgr::Find(int nPcsID)
{
    for(CPcsNode* pPcsNode:m_PcsNodeList){
        if(pPcsNode->pPcs->m_nPcsID==nPcsID)
            return pPcsNode->pPcs;
    }
    return nullptr;
}

// CPcs* CPcsMgr::Find(QString)
// {
//     //按名称查找坐标系
// }

CPcsNode* CPcsMgr::FindNode(int nPcsID)
{
    for(CPcsNode* pPcsNode:m_PcsNodeList){
        if(pPcsNode->pPcs->m_nPcsID==nPcsID)
            return pPcsNode;
    }
    return nullptr;
}

CPcsNode* CPcsMgr::FindNode(CPcs const* pPcs)
{
    for(CPcsNode* pPcsNode:m_PcsNodeList){
        if(pPcsNode->pPcs==pPcs)
            return pPcsNode;
    }
    return nullptr;
}

// CPcsNode* CPcsMgr::FindNode(QString)
// {}

// CPcsNode* CPcsMgr::GetAt(int)
// {}

CPcsNode* CPcsMgr::findPreviousNode(CPcsNode* currentNode)
{
    if(m_PcsNodeList.empty()||currentNode==nullptr){
        return nullptr;
    }

    std::list<CPcsNode*>::iterator it=m_PcsNodeList.begin();//迭代器
    CPcsNode* preNode=nullptr;
    while(it!=m_PcsNodeList.end()){
        if(*it==currentNode){
            if(preNode!=nullptr){ //&&!preNode->m_bDeleted
                return preNode;
            }
            break;
        }
        //更新前一个节点（该节点未被删除）
        if(*it!=nullptr){ //&&!(*it)->m_bDeleted
            preNode=*it;
        }
        it++;
    }
    return nullptr;
}

CPcsNode* CPcsMgr::GetHeadNode()
{
    return m_pHeadPcsNode;
}

CPcsNode* CPcsMgr::GetTailNode()
{
    return m_pTailPcsNode;
}

CPcsNode* CPcsMgr::GetTempNode()
{
    return m_pNodeTemporary;
}

//pos为移动的距离
void CPcsMgr::MoveCurrentCoordSystem(CPosition pos)
{
    if(m_pPcsCurrent!=nullptr){
        m_pPcsCurrent->m_poso.x +=pos.x;
        m_pPcsCurrent->m_poso.y +=pos.y;
        m_pPcsCurrent->m_poso.z +=pos.z;

        //平移矩阵
        QMatrix4x4 transMatrix;
        transMatrix.setToIdentity();
        transMatrix.translate(pos.x,pos.y,pos.z);
        m_pPcsCurrent->m_mat=transMatrix*m_pPcsCurrent->m_mat; //平移矩阵*原矩阵
    }
}

void CPcsMgr::RotateCurrentCoordSystem(double angle,QString coordinateAxis)
{
    if(m_pPcsCurrent!=nullptr){
        QMatrix4x4 rotatMatrix;
        rotatMatrix.setToIdentity();
        if(coordinateAxis=="z"){
            rotatMatrix.rotate(angle,0,0,1);
        }else if(coordinateAxis=="y"){
            rotatMatrix.rotate(angle,0,1,0);
        }else{
            rotatMatrix.rotate(angle,1,0,0);
        }
        m_pPcsCurrent->m_mat=rotatMatrix*m_pPcsCurrent->m_mat;
    }
}

// void CPcsMgr::SaveCurrentCoord()
// {}

// void CPcsMgr::SelectAll(int)
// {}

bool CPcsMgr::IsMultiCoordSys() const
{
    return m_bMultiCoordSys;
}

bool CPcsMgr::bTempPcsNodeInUse() const
{
    return m_bTempPcsNodeInUse;
}

void CPcsMgr::setBTempPcsNodeInUse(bool newBTempPcsNodeInUse)
{
    m_bTempPcsNodeInUse=newBTempPcsNodeInUse;
}




