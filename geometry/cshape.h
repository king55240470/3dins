#ifndef CSHAPE_H
#define CSHAPE_H

#include <QString>
#include <cstdint>//定义uintptr_t（一种无符号整数类型，用于存储指针）
#include <QDataStream>

#include "CPcs.h"

class CShape
{
public:
    CShape();

    // 获取centity子类具体类型接口
    virtual int GetUniqueType() {return -1;};

    bool m_bUnderCursor=false;//是否在光标下
    bool m_bSel=false;//是否被选中
    bool m_bDeleted=false;//是否被删除
    bool m_bShow=false;//是否显示
    bool m_bGroup=false;//是否属于某个组

    uintptr_t m_dwAddress;//存储与形状相关联的地址

    QString m_strInformation;//存储与形状相关的信息

    bool IsUnderCursor();
    bool IsSelected();
    bool IsDeleted();
    bool IsShow();
    bool IsGroup();

    void setUnderCursor(bool);
    virtual void SetSelected(bool);
    virtual void SetDeleted(bool);
    void setShow(bool);
    void SetGroup(bool);

    void setDwAddress(uintptr_t newDwAddress);

    void SetstrInformation(QString);

    //bool getUnderCursor() const;
    bool getSelected() const;
    bool getDeleted() const;
    //bool getShow() const;
    //bool getGroup() const;

    uintptr_t getdwAddress() const;

    //QString getstrInformation() const;

    virtual void SetCurCoord(CPcs *){};
    virtual void SetRefCoord(CPcs *){};
    virtual void SetExtCoord(CPcs *){};

    virtual CPcs* GetRefCoord();
    virtual CPcs* GetCurCoord();
    virtual CPcs* GetExtCoord();

    virtual void update();

    //序列化
    friend QDataStream& operator<<(QDataStream& out,const CShape& shape);
    //反序列化
    friend QDataStream& operator>>(QDataStream& in,CShape& shape);
};

#endif // CSHAPE_H
