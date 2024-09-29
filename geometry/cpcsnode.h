#ifndef CPCSNODE_H
#define CPCSNODE_H

#include"cpcs.h"
#include "cobject.h"

#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkCamera.h>
#include <vtkAxesActor.h>
#include <vtkTextProperty.h>
#include <vtkCaptionActor2D.h>

class CPcsNode : public CObject
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

    // 坐标器的draw()方法
    vtkSmartPointer<vtkAxesActor> draw();
};

#endif // CPCSNODE_H
