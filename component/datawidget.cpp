#include "datawidget.h"
#include <QHeaderView>
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
    box->addItem("参考依赖坐标系");

    hlayout->addWidget(label1);
    hlayout->addWidget(label2);
    hlayout->addWidget(label3);
    hlayout->addWidget(box);

    table=new QTableWidget();
    table->setColumnCount(3);
    table->setHorizontalHeaderLabels(QStringList()<<"数据项"<<"量测值"<<"状态");
    table->setRowCount(5);
    table->verticalHeader()->setHidden(true);//隐藏列号
    table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);//使table内部表格随边框的大小而改变

    vlayout->addLayout(hlayout);
    vlayout->addWidget(table);

}

void DataWidget::updateele(QList<int> idex)
{
    table->clear();
    int count=0;
    for(int i:idex)
    {
        CEntity* entity=m_pMainWin->m_EntityListMgr->m_entityList[i];
        table->setItem(count, 0, new QTableWidgetItem(entity->m_strCName));
        if(entity->GetUniqueType()==enPoint){
            CPoint* point = dynamic_cast<CPoint*>(entity);
            if (point != nullptr){
                QString String = QString("(%1, %2, %3)").arg(point->GetPt().x).arg(point->GetPt().y).arg(point->GetPt().z);
                table->setItem(count,1,new QTableWidgetItem(String));
            }
        }
        if(entity->GetUniqueType()==enLine){
            CLine* line = dynamic_cast<CLine*>(entity);
            if (line != nullptr){
                QString String = QString("(%1, %2, %3),(%4,%5,%6)").arg(line->getPosition1().x).arg(line->getPosition1().y).arg(line->getPosition1().z).arg(line->getPosition2().x).arg(line->getPosition2().y).arg(line->getPosition2().z);
                table->setItem(count,1,new QTableWidgetItem(String));
            }
        }
        if(entity->GetUniqueType()==enCircle){
            CCircle* circle = dynamic_cast<CCircle*>(entity);
            if (circle != nullptr){
                QString String = QString("(%1, %2, %3),%4").arg(circle->getCenter().x).arg(circle->getCenter().y).arg(circle->getCenter().z).arg(circle->getDiameter());
                table->setItem(count,1,new QTableWidgetItem(String));
            }
        }
        if(entity->GetUniqueType()==enPlane){
            CPlane* plane = dynamic_cast<CPlane*>(entity);
            if (plane != nullptr){
                QString String = QString("(%1, %2, %3),length:%4,width:%5").arg(plane->getCenter().x).arg(plane->getCenter().y).arg(plane->getCenter().z).arg(plane->getLength()).arg(plane->getWidth());
                table->setItem(count,1,new QTableWidgetItem(String));
            }
        }
        if(entity->GetUniqueType()==enSphere){
            CSphere* sphere = dynamic_cast<CSphere*>(entity);
            if (sphere != nullptr){
                QString String = QString("(%1, %2, %3),%4").arg(sphere->getCenter().x).arg(sphere->getCenter().y).arg(sphere->getCenter().z).arg(sphere->getDiameter());
                table->setItem(count,1,new QTableWidgetItem(String));
            }
        }
        if(entity->GetUniqueType()==enCylinder){
            CCylinder* cylinder = dynamic_cast<CCylinder*>(entity);
            if (cylinder != nullptr){
                QString String = QString("(%1, %2, %3),diameter:%4,hight:%5").arg(cylinder->getBtm_center().x).arg(cylinder->getBtm_center().y).arg(cylinder->getBtm_center().z).arg(cylinder->getDiameter()).arg(cylinder->getHeight());
                table->setItem(count,1,new QTableWidgetItem(String));
            }
        }
        if(entity->GetUniqueType()==enCone){
            CCone* cone = dynamic_cast<CCone*>(entity);
            if (cone != nullptr){
                QString String = QString("(%1, %2, %3)").arg(cone->getVertex().x).arg(cone->getVertex().y).arg(cone->getVertex().z);
                table->setItem(count,1,new QTableWidgetItem(String));
            }
        }
        count++;
    }

}
