#include "cpcs.h"

CPcs::CPcs() {
    m_nRef=0;
    m_nPcsID=-1;
}

QDataStream& operator<<(QDataStream& out, const CPcs& pcs) {
    out << pcs.m_mat << pcs.m_poso << pcs.m_nRef << pcs.m_nPcsID;
    return out;
}


QDataStream& operator>>(QDataStream& in, CPcs& pcs) {
    in >> pcs.m_mat >> pcs.m_poso >> pcs.m_nRef >> pcs.m_nPcsID;
    return in;
}
