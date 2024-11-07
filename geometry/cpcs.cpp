#include "cpcs.h"

CPcs::CPcs() {}

QDataStream& operator<<(QDataStream& out, const CPcs& pcs) {
    out << pcs.m_mat << pcs.m_poso << pcs.m_nRef << pcs.m_nPcsID;
    return out;
}


QDataStream& operator>>(QDataStream& in, CPcs& pcs) {
    in >> pcs.m_mat >> pcs.m_poso >> pcs.m_nRef >> pcs.m_nPcsID;
    return in;
}
