#include "datawidget.h"
#include <QHeaderView>
#include <QVector4D>
#include "manager/cpcsmgr.h"

DataWidget::DataWidget(QWidget *parent)
    : QWidget{parent}
{
    m_pMainWin=(MainWindow*)parent;
    vlayout=new QVBoxLayout(this);
    hlayout=new QHBoxLayout();

    label1=new QLabel("目前元素:");
    label2=new QLabel();
    label2->setStyleSheet("border: 1px solid grey");
    label3=new QLabel("坐标系:");
    box=new QComboBox();
    box->addItem("机械坐标系");
    //box->addItem("参考依赖坐标系");

    hlayout->addWidget(label1);
    hlayout->addWidget(label2);
    hlayout->addWidget(label3);
    hlayout->addWidget(box);

    table=new QTableWidget();
    table->setColumnCount(3);
    table->setHorizontalHeaderLabels(QStringList()<<"数据项"<<"量测值"<<"状态");
    table->setRowCount(6);
    table->verticalHeader()->setHidden(true);//隐藏列号
    table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);//使table内部表格随边框的大小而改变

    vlayout->addLayout(hlayout);
    vlayout->addWidget(table);

}

void DataWidget::getentityindex(int entityindex)
{
    index=entityindex;
}

void DataWidget::updateinfo()
{
    table->clear();
    if(index==-1){
        table->clear();
        return;
    }
    for(CObject *obj:m_pMainWin->m_ObjectListMgr->getObjectList()){
        if(obj->GetObjectCName().left(5)=="工件坐标系"){
            box->addItem(obj->GetObjectCName());
        }
    }
    if(m_pMainWin->m_EntityListMgr->getEntityList().empty()){
        return;
    }
        CEntity* entity=m_pMainWin->m_EntityListMgr->m_entityList[index];
        //table->setItem(count, 0, new QTableWidgetItem(entity->m_strCName));
        label2->setText(entity->m_strCName);
        if(entity->GetUniqueType()==enPoint){
            CPoint* point = dynamic_cast<CPoint*>(entity);
            CPosition position;
            if(m_pMainWin->m_nRelyOnWhichCs==csRef){
                //即为点存储的坐标（出生时参考的坐标）
                //不做改变
                position.x=point->m_pt.x;
                position.y=point->m_pt.y;
                position.z=point->m_pt.z;
            }else{
                //参考当前坐标系
                //转换为全局坐标
                QVector4D vec=point->m_pRefCoord->m_mat*QVector4D(point->m_pt.x,point->m_pt.y,point->m_pt.z,1);
                CPosition gloPosition(vec.x(),vec.y(),vec.z());
                //当前参考的坐标系下的坐标值
                position=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(gloPosition,point->m_pCurCoord);
            }
            if (point != nullptr){
                //double xx=point->GetPt().x;
                table->setItem(0, 0, new QTableWidgetItem("X"));
                table->setItem(0, 1, new QTableWidgetItem(QString::number(position.x,'f',6)));
                table->setItem(0, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(1, 0, new QTableWidgetItem("Y"));
                table->setItem(1, 1, new QTableWidgetItem(QString::number(position.y,'f',6)));
                table->setItem(1, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(2, 0, new QTableWidgetItem("Z"));
                table->setItem(2, 1, new QTableWidgetItem(QString::number(position.z,'f',6)));
                table->setItem(2, 2, new QTableWidgetItem(""));
            }
        }
        if(entity->GetUniqueType()==enLine){
            CLine* line = dynamic_cast<CLine*>(entity);
            CPosition position1;
            if(m_pMainWin->m_nRelyOnWhichCs==csRef){
                //即为点存储的坐标（出生时参考的坐标）
                //不做改变
                position1.x=line->getPosition1().x;
                position1.y=line->getPosition1().y;
                position1.z=line->getPosition1().z;
            }else{
                //参考当前坐标系
                //转换为全局坐标
                QVector4D vec1=line->m_pRefCoord->m_mat*QVector4D(line->getPosition1().x,line->getPosition1().y,line->getPosition1().z,1);
                CPosition gloPosition1(vec1.x(),vec1.y(),vec1.z());
                //当前参考的坐标系下的坐标值
                position1=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(gloPosition1,line->m_pCurCoord);
            }
            CPosition position2;
            if(m_pMainWin->m_nRelyOnWhichCs==csRef){
                //即为点存储的坐标（出生时参考的坐标）
                //不做改变
                position2.x=line->getPosition2().x;
                position2.y=line->getPosition2().y;
                position2.z=line->getPosition2().z;
            }else{
                //参考当前坐标系
                //转换为全局坐标
                QVector4D vec2=line->m_pRefCoord->m_mat*QVector4D(line->getPosition2().x,line->getPosition2().y,line->getPosition2().z,1);
                CPosition gloPosition2(vec2.x(),vec2.y(),vec2.z());
                //当前参考的坐标系下的坐标值
                position2=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(gloPosition2,line->m_pCurCoord);
            }
            if (line != nullptr){
                table->setItem(0, 0, new QTableWidgetItem("X1"));
                table->setItem(0, 1, new QTableWidgetItem(QString::number(position1.x,'f',6)));
                table->setItem(0, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(1, 0, new QTableWidgetItem("Y1"));
                table->setItem(1, 1, new QTableWidgetItem(QString::number(position1.y,'f',6)));
                table->setItem(1, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(2, 0, new QTableWidgetItem("Z1"));
                table->setItem(2, 1, new QTableWidgetItem(QString::number(position1.z,'f',6)));
                table->setItem(2, 2, new QTableWidgetItem(""));

                table->setItem(3, 0, new QTableWidgetItem("X2"));
                table->setItem(3, 1, new QTableWidgetItem(QString::number(position2.x,'f',6)));
                table->setItem(3, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(4, 0, new QTableWidgetItem("Y2"));
                table->setItem(4, 1, new QTableWidgetItem(QString::number(position2.y,'f',6)));
                table->setItem(4, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(5, 0, new QTableWidgetItem("Z2"));
                table->setItem(5, 1, new QTableWidgetItem(QString::number(position2.z,'f',6)));
                table->setItem(5, 2, new QTableWidgetItem(""));

            }
        }
        if(entity->GetUniqueType()==enCircle){
            CCircle* circle = dynamic_cast<CCircle*>(entity);
            CPosition position;
            if(m_pMainWin->m_nRelyOnWhichCs==csRef){
                //即为点存储的坐标（出生时参考的坐标）
                //不做改变
                position.x=circle->m_pt.x;
                position.y=circle->m_pt.y;
                position.z=circle->m_pt.z;
            }else{
                //转换为全局坐标
                QVector4D vec=circle->m_pRefCoord->m_mat*QVector4D(circle->m_pt.x,circle->m_pt.y,circle->m_pt.z,1);
                CPosition gloPosition(vec.x(),vec.y(),vec.z());
                //当前参考的坐标系下的坐标值
                position=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(gloPosition,circle->m_pCurCoord);
            }
            if (circle != nullptr){
                table->setItem(0, 0, new QTableWidgetItem("X"));
                table->setItem(0, 1, new QTableWidgetItem(QString::number(position.x,'f',6)));
                table->setItem(0, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(1, 0, new QTableWidgetItem("Y"));
                table->setItem(1, 1, new QTableWidgetItem(QString::number(position.y,'f',6)));
                table->setItem(1, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(2, 0, new QTableWidgetItem("Z"));
                table->setItem(2, 1, new QTableWidgetItem(QString::number(position.z,'f',6)));
                table->setItem(2, 2, new QTableWidgetItem(""));

                table->setItem(3, 0, new QTableWidgetItem("D"));
                table->setItem(3, 1, new QTableWidgetItem(QString::number(circle->getDiameter(),'f',6)));
                table->setItem(3, 2, new QTableWidgetItem(""));
            }
        }
        if(entity->GetUniqueType()==enPlane){
            CPlane* plane = dynamic_cast<CPlane*>(entity);
            CPosition position;
            if(m_pMainWin->m_nRelyOnWhichCs==csRef){
                //即为点存储的坐标（出生时参考的坐标）
                //不做改变
                position.x=plane->getCenter().x;
                position.y=plane->getCenter().y;
                position.z=plane->getCenter().z;
            }else{
                //转换为全局坐标
                QVector4D vec=plane->m_pRefCoord->m_mat*QVector4D(plane->getCenter().x,plane->getCenter().y,plane->getCenter().z,1);
                CPosition gloPosition(vec.x(),vec.y(),vec.z());
                //当前参考的坐标系下的坐标值
                position=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(gloPosition,plane->m_pCurCoord);
            }
            if (plane != nullptr){
                table->setItem(0, 0, new QTableWidgetItem("X"));
                table->setItem(0, 1, new QTableWidgetItem(QString::number(position.x,'f',6)));
                table->setItem(0, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(1, 0, new QTableWidgetItem("Y"));
                table->setItem(1, 1, new QTableWidgetItem(QString::number(position.y,'f',6)));
                table->setItem(1, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(2, 0, new QTableWidgetItem("Z"));
                table->setItem(2, 1, new QTableWidgetItem(QString::number(position.z,'f',6)));
                table->setItem(2, 2, new QTableWidgetItem(""));

                table->setItem(3, 0, new QTableWidgetItem("Length"));
                table->setItem(3, 1, new QTableWidgetItem(QString::number(plane->getLength(),'f',6)));
                table->setItem(3, 2, new QTableWidgetItem(""));

                table->setItem(4, 0, new QTableWidgetItem("Width"));
                table->setItem(4, 1, new QTableWidgetItem(QString::number(plane->getWidth(),'f',6)));
                table->setItem(4, 2, new QTableWidgetItem(""));
            }
        }
        if(entity->GetUniqueType()==enSphere){
            CSphere* sphere = dynamic_cast<CSphere*>(entity);
            CPosition position;
            if(m_pMainWin->m_nRelyOnWhichCs==csRef){
                //即为点存储的坐标（出生时参考的坐标）
                //不做改变
                position.x=sphere->getCenter().x;
                position.y=sphere->getCenter().y;
                position.z=sphere->getCenter().z;
            }else{
                //转换为全局坐标
                QVector4D vec=sphere->m_pRefCoord->m_mat*QVector4D(sphere->getCenter().x,sphere->getCenter().y,sphere->getCenter().z,1);
                CPosition gloPosition(vec.x(),vec.y(),vec.z());
                //当前参考的坐标系下的坐标值
                position=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(gloPosition,sphere->m_pCurCoord);
            }
            if (sphere != nullptr){
                table->setItem(0, 0, new QTableWidgetItem("X"));
                table->setItem(0, 1, new QTableWidgetItem(QString::number(position.x,'f',6)));
                table->setItem(0, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(1, 0, new QTableWidgetItem("Y"));
                table->setItem(1, 1, new QTableWidgetItem(QString::number(position.y,'f',6)));
                table->setItem(1, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(2, 0, new QTableWidgetItem("Z"));
                table->setItem(2, 1, new QTableWidgetItem(QString::number(position.z,'f',6)));
                table->setItem(2, 2, new QTableWidgetItem(""));

                table->setItem(3, 0, new QTableWidgetItem("R"));
                table->setItem(3, 1, new QTableWidgetItem(QString::number(sphere->getDiameter(),'f',6)));
                table->setItem(3, 2, new QTableWidgetItem(""));
            }
        }
        if(entity->GetUniqueType()==enCylinder){
            CCylinder* cylinder = dynamic_cast<CCylinder*>(entity);
            CPosition position;
            if(m_pMainWin->m_nRelyOnWhichCs==csRef){
                //即为点存储的坐标（出生时参考的坐标）
                //不做改变
                position.x=cylinder->getBtm_center().x;
                position.y=cylinder->getBtm_center().y;
                position.z=cylinder->getBtm_center().z;
            }else{
                //转换为全局坐标
                QVector4D vec=cylinder->m_pRefCoord->m_mat*QVector4D(cylinder->getBtm_center().x,cylinder->getBtm_center().y,cylinder->getBtm_center().z,1);
                CPosition gloPosition(vec.x(),vec.y(),vec.z());
                //当前参考的坐标系下的坐标值
                position=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(gloPosition,cylinder->m_pCurCoord);
            }
            if (cylinder != nullptr){
                table->setItem(0, 0, new QTableWidgetItem("X"));
                table->setItem(0, 1, new QTableWidgetItem(QString::number(position.x,'f',6)));
                table->setItem(0, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(1, 0, new QTableWidgetItem("Y"));
                table->setItem(1, 1, new QTableWidgetItem(QString::number(position.y,'f',6)));
                table->setItem(1, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(2, 0, new QTableWidgetItem("Z"));
                table->setItem(2, 1, new QTableWidgetItem(QString::number(position.z,'f',6)));
                table->setItem(2, 2, new QTableWidgetItem(""));

                table->setItem(3, 0, new QTableWidgetItem("R"));
                table->setItem(3, 1, new QTableWidgetItem(QString::number(cylinder->getDiameter(),'f',6)));
                table->setItem(3, 2, new QTableWidgetItem(""));

                table->setItem(4, 0, new QTableWidgetItem("Height"));
                table->setItem(4, 1, new QTableWidgetItem(QString::number(cylinder->getHeight(),'f',6)));
                table->setItem(4, 2, new QTableWidgetItem(""));
            }
        }
        if(entity->GetUniqueType()==enCone){
            CCone* cone = dynamic_cast<CCone*>(entity);
            CPosition position;
            if(m_pMainWin->m_nRelyOnWhichCs==csRef){
                //即为点存储的坐标（出生时参考的坐标）
                //不做改变
                position.x=cone->GetObjectCenterLocalPoint().x;
                position.y=cone->GetObjectCenterLocalPoint().y;
                position.z=cone->GetObjectCenterLocalPoint().z;
            }else{
                //转换为全局坐标
                QVector4D vec=cone->m_pRefCoord->m_mat*QVector4D(cone->GetObjectCenterLocalPoint().x,cone->GetObjectCenterLocalPoint().y,cone->GetObjectCenterLocalPoint().z,1);
                CPosition gloPosition(vec.x(),vec.y(),vec.z());
                //当前参考的坐标系下的坐标值
                position=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(gloPosition,cone->m_pCurCoord);
            }
            if (cone != nullptr){
                table->setItem(0, 0, new QTableWidgetItem("X"));
                table->setItem(0, 1, new QTableWidgetItem(QString::number(position.x,'f',6)));
                table->setItem(0, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(1, 0, new QTableWidgetItem("Y"));
                table->setItem(1, 1, new QTableWidgetItem(QString::number(position.y,'f',6)));
                table->setItem(1, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(2, 0, new QTableWidgetItem("Z"));
                table->setItem(2, 1, new QTableWidgetItem(QString::number(position.z,'f',6)));
                table->setItem(2, 2, new QTableWidgetItem(""));

                table->setItem(2, 0, new QTableWidgetItem("H"));
                table->setItem(2, 1, new QTableWidgetItem(QString::number(cone->getHeight(),'f',6)));
                table->setItem(2, 2, new QTableWidgetItem(""));
            }
        }
}
