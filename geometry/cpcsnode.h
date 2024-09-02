#ifndef CPCSNODE_H
#define CPCSNODE_H

#include"cpcs.h"

class CPcsNode
{
public:
    CPcsNode();

    static int nPcsNodeCount; //PcsNode的实例数量
    int nPcsNodeId;

    CPcs* pPcs; //指向坐标系对象
    int nID;
    unsigned int dwAddressPcs; //存储CPcs地址
    int nRefID;
    bool UpdateElement; //
    double rotationAngle=0; //旋转角度

    int getId()
    {
        return nPcsNodeId;
    }

    CPcs* getPcs()
    {
        return pPcs;
    }
};

#endif // CPCSNODE_H
