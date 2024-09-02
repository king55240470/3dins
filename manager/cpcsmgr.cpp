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
    pos.x=vec.x(); pos.y = vec2.y();pos.z = vec2.z();
    return pos;
}

bool CPcsMgr::Erase(QString)
{

    return true;
}

//
void CPcsMgr::SetCurCoordSystem(int nPcsID, bool)
{
    m_pPcsCurrent->m_nPcsID=nPcsID;
}

//
void CPcsMgr::SetCurCoordSystem(CPcs* pPcs)
{
    m_pPcsCurrent=pPcs;
}


