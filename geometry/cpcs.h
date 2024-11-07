#ifndef CPCS_H
#define CPCS_H

#include<QMatrix4x4>
#include"globes.h"

class CPcs
{
public:
    CPcs();

    QMatrix4x4 m_mat;
    CPosition m_poso; //坐标系原点
    int m_nRef; //坐标系被几个参考
    int m_nPcsID;

    //序列化/反序列化CPcs对象
    friend QDataStream& operator<<(QDataStream& out, const CPcs& pcs);

    friend QDataStream& operator>>(QDataStream& in, CPcs& pcs);

    //ID
    int nPcsID() const;
    void setNPcsID(int newNPcsID);
    CPosition poso() const;


public:
    // double GetAngleOnXoY(){
    //     float a = m_mat(0, 0); // X轴变换后的X分量
    //     float e = m_mat(1, 0); // X轴变换后的Y分量
    //     double angle2 = qRadiansToDegrees(qAtan2(e, a));

    //     // 确保返回的角度是正数
    //     return angle2 >= 0 ? angle2 : angle2 + 360;
    // }

    void PlanarRotateXinXY(double angle){
        m_mat.rotate(angle, 0,0,1);
    }

    void PlanarRotateYinXY(double angle){
        m_mat.rotate(0, angle,0,1);
    }

};


#endif // CPCS_H
