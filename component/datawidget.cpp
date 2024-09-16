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

void DataWidget::updateele(int index)
{
    CEntity* entity=m_pMainWin->m_EntityListMgr->m_entityList[index];
    table->setItem(count, 0, new QTableWidgetItem(entity->m_strCName));
    if(entity->GetUniqueType()==enPoint){
        CPoint* point = dynamic_cast<CPoint*>(entity);
        QString String = QString("(%1, %2, %3)").arg(point->GetPt().x).arg(point->GetPt().y).arg(point->GetPt().z);
        table->setItem(count,1,new QTableWidgetItem(String));
    }

    count++;
}
