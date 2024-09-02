#ifndef CPCS_H
#define CPCS_H

#include<QMatrix4x4>
#include"cposition.h"

class CPcs
{
public:
    CPcs();

    QMatrix4x4 m_mat;
    CPosition m_poso; //坐标系原点
    int m_nRef; //坐标系被几个参考
    int m_nPcsID;

    //序列化/反序列化CPcs对象
    friend QDataStream& operator<<(QDataStream& out, const CPcs& pcs) {
        out << pcs.m_mat << pcs.m_poso << pcs.m_nRef << pcs.m_nPcsID;
        return out;
    }


    friend QDataStream& operator>>(QDataStream& in, CPcs& pcs) {
        in >> pcs.m_mat >> pcs.m_poso >> pcs.m_nRef >> pcs.m_nPcsID;
        return in;
    }

    //ID
    int nPcsID() const;
    void setNPcsID(int newNPcsID);
    CPosition poso() const;


};

#endif // CPCS_H
