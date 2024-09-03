#ifndef CPCSMGR_H
#define CPCSMGR_H

#include"geometry/cpcsnode.h"
#include<list>

class CPcsMgr
{
public:
    CPcsMgr();

    CPcs* m_pPcsCurrent; //当前坐标系
    CPcsNode* m_pNodeTemporary; //临时坐标系节点
    bool m_bTempPcsNodeInUse; //临时坐标系节点是否使用
    CPcsNode* m_pHeadPcsNode; //头节点
    CPcsNode* m_pTailPcsNode; //尾节点
    std::list<CPcsNode*> m_PcsNodeList; //存储坐标系节点的链表
    int m_nCount;
    int m_nID;
    bool m_bMultiCoordSys; //多坐标系

    //初始化
    void Initialize();

    //坐标系管理
    void AddCoordSys(CPcsNode* pPcsNode); //新增坐标系节点
    CPosition GetLocalPosOfCertainPcs(CPosition pos,CPcs* pcs); //获取局部坐标（工件坐标系）
    bool Erase(QString); //？根据名称判断是否删除坐标系
    void SetCurCoordSystem(int, bool);
    void SetCurCoordSystem(QString);
    void SetCurCoordSystem(CPcs*);
    CPcs* GetBaseCoordSystem();
    CPcs* GetCurCoordSystem();
    //CPcs* GetWorldCoordSystem();
    CPcs* Find(int);
    CPcs* Find(QString);

    //节点管理
    CPcsNode* FindNode(int); //ID
    CPcsNode* FindNode(CPcs const*);
    CPcsNode* FindNode(QString);
    // CPcsNode* GetAt(int); //索引
    CPcsNode* findPreviousNode(CPcsNode* currentNode);
    CPcsNode* GetHeadNode(); //获得头节点
    CPcsNode* GetTailNode(); //获得尾节点
    CPcsNode* GetTempNode(); //获得临时节点

    //坐标系转换
    void MoveCurrentCoordSystem(CPosition); //移动
    void RotateCurrentCoordSystem(double,QString); //旋转
    void SaveCurrentCoord(); //保存
    void SelectAll(int); //选中所有与当前坐标系相关的实体

    //其他
    bool IsMultiCoordSys() const; //多坐标系判断
    bool bTempPcsNodeInUse() const; //临时坐标系节点是否使用
    void setBTempPcsNodeInUse(bool newBTempPcsNodeInUse); //设置使用


};

#endif // CPCSMGR_H
