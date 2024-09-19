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
    table->setRowCount(6);
    table->verticalHeader()->setHidden(true);//隐藏列号
    table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);//使table内部表格随边框的大小而改变

    vlayout->addLayout(hlayout);
    vlayout->addWidget(table);

}

void DataWidget::updateele(int i)
{
    table->clear();
        CEntity* entity=m_pMainWin->m_EntityListMgr->m_entityList[i];
        //table->setItem(count, 0, new QTableWidgetItem(entity->m_strCName));
        label2->setText(entity->m_strCName);
        if(entity->GetUniqueType()==enPoint){
            CPoint* point = dynamic_cast<CPoint*>(entity);
            if (point != nullptr){
                table->setItem(0, 0, new QTableWidgetItem("X"));
                table->setItem(0, 1, new QTableWidgetItem(QString::number(point->GetPt().x)));
                table->setItem(0, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(1, 0, new QTableWidgetItem("Y"));
                table->setItem(1, 1, new QTableWidgetItem(QString::number(point->GetPt().y)));
                table->setItem(1, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(2, 0, new QTableWidgetItem("Z"));
                table->setItem(2, 1, new QTableWidgetItem(QString::number(point->GetPt().z)));
                table->setItem(2, 2, new QTableWidgetItem(""));
            }
        }
        if(entity->GetUniqueType()==enLine){
            CLine* line = dynamic_cast<CLine*>(entity);
            if (line != nullptr){
                table->setItem(0, 0, new QTableWidgetItem("X1"));
                table->setItem(0, 1, new QTableWidgetItem(QString::number(line->getPosition1().x)));
                table->setItem(0, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(1, 0, new QTableWidgetItem("Y1"));
                table->setItem(1, 1, new QTableWidgetItem(QString::number(line->getPosition1().y)));
                table->setItem(1, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(2, 0, new QTableWidgetItem("Z1"));
                table->setItem(2, 1, new QTableWidgetItem(QString::number(line->getPosition1().z)));
                table->setItem(2, 2, new QTableWidgetItem(""));

                table->setItem(3, 0, new QTableWidgetItem("X2"));
                table->setItem(3, 1, new QTableWidgetItem(QString::number(line->getPosition2().x)));
                table->setItem(3, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(4, 0, new QTableWidgetItem("Y2"));
                table->setItem(4, 1, new QTableWidgetItem(QString::number(line->getPosition2().y)));
                table->setItem(4, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(5, 0, new QTableWidgetItem("Z2"));
                table->setItem(5, 1, new QTableWidgetItem(QString::number(line->getPosition2().z)));
                table->setItem(5, 2, new QTableWidgetItem(""));

            }
        }
        if(entity->GetUniqueType()==enCircle){
            CCircle* circle = dynamic_cast<CCircle*>(entity);
            if (circle != nullptr){
                table->setItem(0, 0, new QTableWidgetItem("X"));
                table->setItem(0, 1, new QTableWidgetItem(QString::number(circle->getCenter().x)));
                table->setItem(0, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(1, 0, new QTableWidgetItem("Y"));
                table->setItem(1, 1, new QTableWidgetItem(QString::number(circle->getCenter().y)));
                table->setItem(1, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(2, 0, new QTableWidgetItem("Z"));
                table->setItem(2, 1, new QTableWidgetItem(QString::number(circle->getCenter().z)));
                table->setItem(2, 2, new QTableWidgetItem(""));

                table->setItem(3, 0, new QTableWidgetItem("D"));
                table->setItem(3, 1, new QTableWidgetItem(QString::number(circle->getDiameter())));
                table->setItem(3, 2, new QTableWidgetItem(""));
            }
        }
        if(entity->GetUniqueType()==enPlane){
            CPlane* plane = dynamic_cast<CPlane*>(entity);
            if (plane != nullptr){
                table->setItem(0, 0, new QTableWidgetItem("X"));
                table->setItem(0, 1, new QTableWidgetItem(QString::number(plane->getCenter().x)));
                table->setItem(0, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(1, 0, new QTableWidgetItem("Y"));
                table->setItem(1, 1, new QTableWidgetItem(QString::number(plane->getCenter().y)));
                table->setItem(1, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(2, 0, new QTableWidgetItem("Z"));
                table->setItem(2, 1, new QTableWidgetItem(QString::number(plane->getCenter().z)));
                table->setItem(2, 2, new QTableWidgetItem(""));

                table->setItem(3, 0, new QTableWidgetItem("Length"));
                table->setItem(3, 1, new QTableWidgetItem(QString::number(plane->getLength())));
                table->setItem(3, 2, new QTableWidgetItem(""));

                table->setItem(4, 0, new QTableWidgetItem("Width"));
                table->setItem(4, 1, new QTableWidgetItem(QString::number(plane->getWidth())));
                table->setItem(4, 2, new QTableWidgetItem(""));
            }
        }
        if(entity->GetUniqueType()==enSphere){
            CSphere* sphere = dynamic_cast<CSphere*>(entity);
            if (sphere != nullptr){
                table->setItem(0, 0, new QTableWidgetItem("X"));
                table->setItem(0, 1, new QTableWidgetItem(QString::number(sphere->getCenter().x)));
                table->setItem(0, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(1, 0, new QTableWidgetItem("Y"));
                table->setItem(1, 1, new QTableWidgetItem(QString::number(sphere->getCenter().y)));
                table->setItem(1, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(2, 0, new QTableWidgetItem("Z"));
                table->setItem(2, 1, new QTableWidgetItem(QString::number(sphere->getCenter().z)));
                table->setItem(2, 2, new QTableWidgetItem(""));

                table->setItem(3, 0, new QTableWidgetItem("R"));
                table->setItem(3, 1, new QTableWidgetItem(QString::number(sphere->getDiameter())));
                table->setItem(3, 2, new QTableWidgetItem(""));
            }
        }
        if(entity->GetUniqueType()==enCylinder){
            CCylinder* cylinder = dynamic_cast<CCylinder*>(entity);
            if (cylinder != nullptr){
                table->setItem(0, 0, new QTableWidgetItem("X"));
                table->setItem(0, 1, new QTableWidgetItem(QString::number(cylinder->getBtm_center().x)));
                table->setItem(0, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(1, 0, new QTableWidgetItem("Y"));
                table->setItem(1, 1, new QTableWidgetItem(QString::number(cylinder->getBtm_center().y)));
                table->setItem(1, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(2, 0, new QTableWidgetItem("Z"));
                table->setItem(2, 1, new QTableWidgetItem(QString::number(cylinder->getBtm_center().z)));
                table->setItem(2, 2, new QTableWidgetItem(""));

                table->setItem(3, 0, new QTableWidgetItem("R"));
                table->setItem(3, 1, new QTableWidgetItem(QString::number(cylinder->getDiameter())));
                table->setItem(3, 2, new QTableWidgetItem(""));

                table->setItem(4, 0, new QTableWidgetItem("Height"));
                table->setItem(4, 1, new QTableWidgetItem(QString::number(cylinder->getHeight())));
                table->setItem(4, 2, new QTableWidgetItem(""));
            }
        }
        if(entity->GetUniqueType()==enCone){
            CCone* cone = dynamic_cast<CCone*>(entity);
            if (cone != nullptr){
                table->setItem(0, 0, new QTableWidgetItem("X"));
                table->setItem(0, 1, new QTableWidgetItem(QString::number(cone->GetObjectCenterLocalPoint().x)));
                table->setItem(0, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(1, 0, new QTableWidgetItem("Y"));
                table->setItem(1, 1, new QTableWidgetItem(QString::number(cone->GetObjectCenterLocalPoint().y)));
                table->setItem(1, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(2, 0, new QTableWidgetItem("Z"));
                table->setItem(2, 1, new QTableWidgetItem(QString::number(cone->GetObjectCenterLocalPoint().z)));
                table->setItem(2, 2, new QTableWidgetItem(""));

                table->setItem(2, 0, new QTableWidgetItem("H"));
                table->setItem(2, 1, new QTableWidgetItem(QString::number(cone->getHeight())));
                table->setItem(2, 2, new QTableWidgetItem(""));
            }
        }
}
