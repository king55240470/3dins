#ifndef CENTITY_H
#define CENTITY_H

#include "cobject.h"
#include "globes.h"
#include "CPcs.h"
#include <QVector>
#include <vtkSmartPointer.h>
#include <vtkActor.h>

class CEntity:public CObject
{
public:
    CPcs* m_pRefCoord;//参考坐标系
    CPcs* m_pCurCoord;//当前坐标系
    CPcs* m_pExtCoord;

    QVector<CEntity*> m_ConstructList;//存储该实体的一些信息
    //QVector<CImageTool *>  m_ToolArray;//存储该实体使用的工具

    bool m_bShowCNameLabel;//是否显示该实体的名称

    int  m_nRef;//表示参考哪个坐标系，m_nRef表示坐标系的ID

    bool m_bRemeasure;//是否需要重新测量

    enum CREATE_FORM m_CreateForm;
    enum ENTITY_TYPE m_EntityType;
    enum COMPENSATE_TYPE m_CompenType;
public:
    CEntity();
    ~CEntity();
    CPcs* GetRefCoord() override;//使用override关键字时，如果派生类的函数签名与基类的虚函数不匹配，编译器会报错,这有助于确保正确地重写基类函数
    CPcs* GetCurCoord() override;
    CPcs* GetExtCoord() override;

    void SetRefCoord(CPcs *)override;
    void SetCurCoord(CPcs *)override;
    void SetExtCoord(CPcs *)override;

    QVector<CEntity*>GetConstructList();
    //void SetImageToolList(QVector<CImageTool *>);
    //QVector<CImageTool *> GetImageToolList();

    bool IsShowCNameLabel();

    int AddRef(CShape*);
    int DelAllRefShape();

    void SetRemeasure(bool);

    bool IsCanAutoMeasure();//是否自动测量
    bool IsMeasureByImage();//是否通过图像测量
    bool IsMeasureByLaser();//是否通过激光测量
    bool IsMeasureByProbe();//是否通过探针测量

    bool IsType(ENTITY_TYPE);
    // 当遍历entitylist进行绘制时，传入子类的指针，自动调用它们自己的draw
    virtual vtkSmartPointer<vtkActor> draw() {return nullptr;};

    void setEntityType(ENTITY_TYPE);
    ENTITY_TYPE getEntityType() const;

    void SetCompenType(COMPENSATE_TYPE);
    COMPENSATE_TYPE GetCompenType();

    void SetCreateForm(CREATE_FORM);
    CREATE_FORM GetCreateForm();


    //序列化
    QDataStream& serialize(QDataStream& out) const override{
        CObject::serialize(out);//序列化基类部分
        out<<m_pRefCoord<<m_pCurCoord<<m_pExtCoord;//将指针的值（地址）写入数据流中，不涉及对象内容的读写?
        out<<static_cast<int>(m_ConstructList.size());
        for(const auto& entity:m_ConstructList){//auto让编译器自动推断元素的类型
            out<<*entity;
        }
        /*out<<static_cast<int>(m_ToolArray.size());
        for(const auto& entity:m_ToolArray){
            out<<*entity;
        }*/
        out<<m_bShowCNameLabel
            <<static_cast<int>(m_CompenType)
            <<m_nRef
            <<m_bRemeasure;
        return out;
    }



    //反序列化
    QDataStream& deserialize(QDataStream& in) override{
        CObject::deserialize(in);//反序列化基类部分
        in>>*m_pRefCoord>>*m_pCurCoord>>*m_pExtCoord;//解引用‘*’将数据流中的内容读取到已经存在的对象中，确保将读取的数据填充到对象内部

        int constructListSize,toolArraySize;

        in>>constructListSize;
        m_ConstructList.clear();
        for(int i=0;i<constructListSize;i++){
            CEntity *entity=new CEntity();
            in>>*entity;
            m_ConstructList.append(entity);
        }

        /*in>>toolArraySize;
        m_ToolArray.clear();
        for(int i=0;i<toolArraySize;i++){
            CImageTool *tool=new CImageTool();
            in>>*tool;
            m_ToolArray.append(tool);
        }*/

        in>>m_bShowCNameLabel
            >>reinterpret_cast<int&>(m_CompenType)
            >>m_nRef
            >>m_bRemeasure;
        return in;
    }
};

#endif // CENTITY_H
