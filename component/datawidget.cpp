#include "datawidget.h"
#include <QHeaderView>
#include <QVector4D>
#include <QTimer>
#include "manager/cpcsmgr.h"
#include <QMessageBox>

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
    table->setRowCount(9);
    table->verticalHeader()->setHidden(true);//隐藏列号
    table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);//使table内部表格随边框的大小而改变

    vlayout->addLayout(hlayout);
    vlayout->addWidget(table);

}

void DataWidget::getobjindex(int objindex)
{
    index=objindex;
}

void DataWidget::updateinfo()
{
    table->blockSignals(true); // 暂停信号

    table->clear();
    if(index==-1){
        table->clear();
        return;
    }

    for(CObject *obj:m_pMainWin->m_ObjectListMgr->getObjectList()){
        QString objectName = obj->GetObjectCName();
        if(m_pMainWin->m_pcsListMgr->bTempPcsNodeInUse()){
            if(tempIn==true)break;
            if(objectName.left(5) == "临时坐标系"){
                tempIn=true;
                box->addItem(objectName);
            }
        }else{
            if(tempIn){
                if(objectName.left(5) == "工件坐标系"){
                    box->removeItem(box->findText("临时坐标系"));
                    tempIn=false;
                    if (box->findText(objectName) == -1) {  // -1表示未找到重复
                        box->addItem(objectName);
                    }
                }
            }else{
                if(objectName.left(5) == "工件坐标系"){
                    if (box->findText(objectName) == -1) {  // -1表示未找到重复
                        box->addItem(objectName);
                    }
                }
            }
        }
    }

    if(m_pMainWin->m_EntityListMgr->getEntityList().empty()){
        return;
    }
    CObject* obj=m_pMainWin->getObjectListMgr()->getObjectList()[index];

        //CEntity* entity=m_pMainWin->m_EntityListMgr->m_entityList[index];
        //table->setItem(count, 0, new QTableWidgetItem(entity->m_strCName));
        label2->setText(obj->m_strCName);
        if(obj->GetUniqueType()==enPoint){
            CPoint* point = dynamic_cast<CPoint*>(obj);
            qDebug()<<"data里的point"<<point->GetPt().x;
            CPosition position;
            //转换为全局坐标
            QVector4D vec=point->m_pRefCoord->m_mat*QVector4D(point->GetPt().x,point->GetPt().y,point->GetPt().z,1);
            CPosition gloPosition(vec.x(),vec.y(),vec.z());

            if(m_pMainWin->m_nRelyOnWhichCs==csRef){
                box->setEnabled(true);
                box->setCurrentIndex(box->findText((m_pMainWin->m_pcsListMgr->FindNode(point->GetRefCoord()))->GetObjectCName())); // 设置box当前数据显示参考的坐标系
                //即为点存储的坐标（出生时参考的坐标）

                point->SetExtCoord(point->GetRefCoord());

                //当切换扩展坐标系时检测到变化
                connect(box, &QComboBox::currentIndexChanged, this, [=](){
                    table->blockSignals(true); // 暂停信号
                    // QString currentText =box->currentText();
                    // qDebug() << "当前选中的文本是: " << currentText;
                    if(box->currentText()=="机械坐标系"){
                        point->SetExtCoord(m_pMainWin->m_pcsListMgr->GetBaseCoordSystem());
                    }else{
                        for(CObject *obj:m_pMainWin->m_ObjectListMgr->getObjectList()){
                            if(box->currentText()==obj->GetObjectCName()){
                                CPcsNode* node=dynamic_cast<CPcsNode*>(obj);
                                point->SetExtCoord(node->getPcs());
                                // qDebug() << "Current Text: " << box->currentText();
                                // qDebug() << "Object Name: " << obj->GetObjectCName();
                                break;
                            }
                        }
                    }

                    CPosition temposition=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(gloPosition,point->m_pExtCoord);

                    table->setItem(0, 1, new QTableWidgetItem(QString::number(temposition.x,'f',6)));
                    table->setItem(1, 1, new QTableWidgetItem(QString::number(temposition.y,'f',6)));
                    table->setItem(2, 1, new QTableWidgetItem(QString::number(temposition.z,'f',6)));

                    table->viewport()->update();
                });

            }else{
                box->setCurrentIndex(box->findText((m_pMainWin->m_pcsListMgr->FindNode(point->GetCurCoord()))->GetObjectCName())); // 设置box当前数据显示参考的坐标系
                box->setEnabled(false);
                //参考当前坐标系
                //将扩展坐标系设置为当前坐标系
                point->SetExtCoord(point->GetCurCoord());
            }

            //数据显示用扩展坐标系计算
            position=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(gloPosition,point->m_pExtCoord);

            if (point != nullptr){
                //double xx=point->GetPt().x;
                table->setItem(0, 0, new QTableWidgetItem("_x"));
                table->setItem(0, 1, new QTableWidgetItem(QString::number(position.x,'f',6)));
                table->setItem(0, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(1, 0, new QTableWidgetItem("_y"));
                table->setItem(1, 1, new QTableWidgetItem(QString::number(position.y,'f',6)));
                table->setItem(1, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(2, 0, new QTableWidgetItem("_z"));
                table->setItem(2, 1, new QTableWidgetItem(QString::number(position.z,'f',6)));
                table->setItem(2, 2, new QTableWidgetItem(""));
            }
        }
        if(obj->GetUniqueType()==enLine){
            CLine* line = dynamic_cast<CLine*>(obj);
            CPosition position1;
            CPosition position2;
            //转换为全局坐标
            QVector4D vec1=line->m_pRefCoord->m_mat*QVector4D(line->getPosition1().x,line->getPosition1().y,line->getPosition1().z,1);
            CPosition gloPosition1(vec1.x(),vec1.y(),vec1.z());
            QVector4D vec2=line->m_pRefCoord->m_mat*QVector4D(line->getPosition2().x,line->getPosition2().y,line->getPosition2().z,1);
            CPosition gloPosition2(vec2.x(),vec2.y(),vec2.z());
            if(m_pMainWin->m_nRelyOnWhichCs==csRef){
                box->setEnabled(true);
                box->setCurrentIndex(box->findText((m_pMainWin->m_pcsListMgr->FindNode(line->GetRefCoord()))->GetObjectCName())); // 设置box当前数据显示参考的坐标系
                //即为点存储的坐标（出生时参考的坐标）

                line->SetExtCoord(line->GetRefCoord());

                //当切换扩展坐标系时检测到变化
                connect(box, &QComboBox::currentIndexChanged, this, [=](){
                    table->blockSignals(true); // 暂停信号
                    // QString currentText =box->currentText();
                    // qDebug() << "当前选中的文本是: " << currentText;
                    if(box->currentText()=="机械坐标系"){
                        line->SetExtCoord(m_pMainWin->m_pcsListMgr->GetBaseCoordSystem());
                    }else{
                        for(CObject *obj:m_pMainWin->m_ObjectListMgr->getObjectList()){
                            if(box->currentText()==obj->GetObjectCName()){
                                CPcsNode* node=dynamic_cast<CPcsNode*>(obj);
                                line->SetExtCoord(node->getPcs());
                                // qDebug() << "Current Text: " << box->currentText();
                                // qDebug() << "Object Name: " << obj->GetObjectCName();
                                break;
                            }
                        }
                    }

                    CPosition temposition1=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(gloPosition1,line->m_pExtCoord);
                    CPosition temposition2=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(gloPosition2,line->m_pExtCoord);

                    table->setItem(0, 1, new QTableWidgetItem(QString::number(temposition1.x,'f',6)));
                    table->setItem(1, 1, new QTableWidgetItem(QString::number(temposition1.y,'f',6)));
                    table->setItem(2, 1, new QTableWidgetItem(QString::number(temposition1.z,'f',6)));
                    table->setItem(0, 1, new QTableWidgetItem(QString::number(temposition2.x,'f',6)));
                    table->setItem(1, 1, new QTableWidgetItem(QString::number(temposition2.y,'f',6)));
                    table->setItem(2, 1, new QTableWidgetItem(QString::number(temposition2.z,'f',6)));

                    table->viewport()->update();
                });

            }else{
                box->setCurrentIndex(box->findText((m_pMainWin->m_pcsListMgr->FindNode(line->GetCurCoord()))->GetObjectCName()));
                box->setEnabled(false);

                //参考当前坐标系
                line->SetExtCoord(line->GetCurCoord());
            }

            position1=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(gloPosition1,line->m_pExtCoord);
            position2=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(gloPosition2,line->m_pExtCoord);

            if (line != nullptr){
                table->setItem(0, 0, new QTableWidgetItem("起点_x"));
                table->setItem(0, 1, new QTableWidgetItem(QString::number(position1.x,'f',6)));
                table->setItem(0, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(1, 0, new QTableWidgetItem("起点_y"));
                table->setItem(1, 1, new QTableWidgetItem(QString::number(position1.y,'f',6)));
                table->setItem(1, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(2, 0, new QTableWidgetItem("起点_z"));
                table->setItem(2, 1, new QTableWidgetItem(QString::number(position1.z,'f',6)));
                table->setItem(2, 2, new QTableWidgetItem(""));

                table->setItem(3, 0, new QTableWidgetItem("终点_x"));
                table->setItem(3, 1, new QTableWidgetItem(QString::number(position2.x,'f',6)));
                table->setItem(3, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(4, 0, new QTableWidgetItem("终点_y"));
                table->setItem(4, 1, new QTableWidgetItem(QString::number(position2.y,'f',6)));
                table->setItem(4, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(5, 0, new QTableWidgetItem("终点_z"));
                table->setItem(5, 1, new QTableWidgetItem(QString::number(position2.z,'f',6)));
                table->setItem(5, 2, new QTableWidgetItem(""));

            }
        }
        if(obj->GetUniqueType()==enCircle){
            CCircle* circle = dynamic_cast<CCircle*>(obj);
            CPosition position;
            //转换为全局坐标
            QVector4D vec=circle->m_pRefCoord->m_mat*QVector4D(circle->m_pt.x,circle->m_pt.y,circle->m_pt.z,1);
            CPosition gloPosition(vec.x(),vec.y(),vec.z());
            if(m_pMainWin->m_nRelyOnWhichCs==csRef){
                box->setEnabled(true);
                box->setCurrentIndex(box->findText((m_pMainWin->m_pcsListMgr->FindNode(circle->GetRefCoord()))->GetObjectCName()));
                //即为点存储的坐标（出生时参考的坐标）

                circle->SetExtCoord(circle->GetRefCoord());

                //当切换扩展坐标系时检测到变化
                connect(box, &QComboBox::currentIndexChanged, this, [=](){
                    table->blockSignals(true); // 暂停信号
                    // QString currentText =box->currentText();
                    // qDebug() << "当前选中的文本是: " << currentText;
                    if(box->currentText()=="机械坐标系"){
                        circle->SetExtCoord(m_pMainWin->m_pcsListMgr->GetBaseCoordSystem());
                    }else{
                        for(CObject *obj:m_pMainWin->m_ObjectListMgr->getObjectList()){
                            if(box->currentText()==obj->GetObjectCName()){
                                CPcsNode* node=dynamic_cast<CPcsNode*>(obj);
                                circle->SetExtCoord(node->getPcs());
                                // qDebug() << "Current Text: " << box->currentText();
                                // qDebug() << "Object Name: " << obj->GetObjectCName();
                                break;
                            }
                        }
                    }

                    CPosition temposition=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(gloPosition,circle->m_pExtCoord);

                    table->setItem(0, 1, new QTableWidgetItem(QString::number(temposition.x,'f',6)));
                    table->setItem(1, 1, new QTableWidgetItem(QString::number(temposition.y,'f',6)));
                    table->setItem(2, 1, new QTableWidgetItem(QString::number(temposition.z,'f',6)));

                    table->viewport()->update();
                });

            }else{
                box->setCurrentIndex(box->findText((m_pMainWin->m_pcsListMgr->FindNode(circle->GetCurCoord()))->GetObjectCName()));
                box->setEnabled(false);

                circle->SetExtCoord(circle->GetCurCoord());
            }

            position=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(gloPosition,circle->m_pExtCoord);

            if (circle != nullptr){
                table->setItem(0, 0, new QTableWidgetItem("圆心_x"));
                table->setItem(0, 1, new QTableWidgetItem(QString::number(position.x,'f',6)));
                table->setItem(0, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(1, 0, new QTableWidgetItem("圆心_y"));
                table->setItem(1, 1, new QTableWidgetItem(QString::number(position.y,'f',6)));
                table->setItem(1, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(2, 0, new QTableWidgetItem("圆心_z"));
                table->setItem(2, 1, new QTableWidgetItem(QString::number(position.z,'f',6)));
                table->setItem(2, 2, new QTableWidgetItem(""));

                table->setItem(3, 0, new QTableWidgetItem("直径"));
                table->setItem(3, 1, new QTableWidgetItem(QString::number(circle->getDiameter(),'f',6)));
                table->setItem(3, 2, new QTableWidgetItem(""));

                table->setItem(4, 0, new QTableWidgetItem("法向量_x"));
                table->setItem(4, 1, new QTableWidgetItem(QString::number(circle->getNormal().x(),'f',6)));
                table->setItem(4, 2, new QTableWidgetItem(""));

                table->setItem(5, 0, new QTableWidgetItem("法向量_y"));
                table->setItem(5, 1, new QTableWidgetItem(QString::number(circle->getNormal().y(),'f',6)));
                table->setItem(5, 2, new QTableWidgetItem(""));

                table->setItem(6, 0, new QTableWidgetItem("法向量_z"));
                table->setItem(6, 1, new QTableWidgetItem(QString::number(circle->getNormal().z(),'f',6)));
                table->setItem(6, 2, new QTableWidgetItem(""));
            }
        }
        if(obj->GetUniqueType()==enPlane){
            CPlane* plane = dynamic_cast<CPlane*>(obj);
            CPosition position;
            //转换为全局坐标
            QVector4D vec=plane->m_pRefCoord->m_mat*QVector4D(plane->getCenter().x,plane->getCenter().y,plane->getCenter().z,1);
            CPosition gloPosition(vec.x(),vec.y(),vec.z());
            if(m_pMainWin->m_nRelyOnWhichCs==csRef){
                box->setEnabled(true);
                box->setCurrentIndex(box->findText((m_pMainWin->m_pcsListMgr->FindNode(plane->GetRefCoord()))->GetObjectCName()));
                //即为点存储的坐标（出生时参考的坐标）

                plane->SetExtCoord(plane->GetRefCoord());

                //当切换扩展坐标系时检测到变化
                connect(box, &QComboBox::currentIndexChanged, this, [=](){
                    table->blockSignals(true); // 暂停信号
                    // QString currentText =box->currentText();
                    // qDebug() << "当前选中的文本是: " << currentText;
                    if(box->currentText()=="机械坐标系"){
                        plane->SetExtCoord(m_pMainWin->m_pcsListMgr->GetBaseCoordSystem());
                    }else{
                        for(CObject *obj:m_pMainWin->m_ObjectListMgr->getObjectList()){
                            if(box->currentText()==obj->GetObjectCName()){
                                CPcsNode* node=dynamic_cast<CPcsNode*>(obj);
                                plane->SetExtCoord(node->getPcs());
                                // qDebug() << "Current Text: " << box->currentText();
                                // qDebug() << "Object Name: " << obj->GetObjectCName();
                                break;
                            }
                        }
                    }

                    CPosition temposition=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(gloPosition,plane->m_pExtCoord);

                    table->setItem(0, 1, new QTableWidgetItem(QString::number(temposition.x,'f',6)));
                    table->setItem(1, 1, new QTableWidgetItem(QString::number(temposition.y,'f',6)));
                    table->setItem(2, 1, new QTableWidgetItem(QString::number(temposition.z,'f',6)));

                    table->viewport()->update();
                });

            }else{
                box->setCurrentIndex(box->findText((m_pMainWin->m_pcsListMgr->FindNode(plane->GetCurCoord()))->GetObjectCName()));
                box->setEnabled(false);

                plane->SetExtCoord(plane->GetCurCoord());
            }

             position=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(gloPosition,plane->m_pExtCoord);

            if (plane != nullptr){
                table->setItem(0, 0, new QTableWidgetItem("中心_x"));
                table->setItem(0, 1, new QTableWidgetItem(QString::number(position.x,'f',6)));
                table->setItem(0, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(1, 0, new QTableWidgetItem("中心_y"));
                table->setItem(1, 1, new QTableWidgetItem(QString::number(position.y,'f',6)));
                table->setItem(1, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(2, 0, new QTableWidgetItem("中心_z"));
                table->setItem(2, 1, new QTableWidgetItem(QString::number(position.z,'f',6)));
                table->setItem(2, 2, new QTableWidgetItem(""));

                table->setItem(3, 0, new QTableWidgetItem("长度"));
                table->setItem(3, 1, new QTableWidgetItem(QString::number(plane->getLength(),'f',6)));
                table->setItem(3, 2, new QTableWidgetItem(""));

                table->setItem(4, 0, new QTableWidgetItem("宽度"));
                table->setItem(4, 1, new QTableWidgetItem(QString::number(plane->getWidth(),'f',6)));
                table->setItem(4, 2, new QTableWidgetItem(""));

                table->setItem(5, 0, new QTableWidgetItem("法向量_x"));
                table->setItem(5, 1, new QTableWidgetItem(QString::number(plane->getNormal().x(),'f',6)));
                table->setItem(5, 2, new QTableWidgetItem(""));

                table->setItem(6, 0, new QTableWidgetItem("法向量_y"));
                table->setItem(6, 1, new QTableWidgetItem(QString::number(plane->getNormal().y(),'f',6)));
                table->setItem(6, 2, new QTableWidgetItem(""));

                table->setItem(7, 0, new QTableWidgetItem("法向量_z"));
                table->setItem(7, 1, new QTableWidgetItem(QString::number(plane->getNormal().z(),'f',6)));
                table->setItem(7, 2, new QTableWidgetItem(""));
            }
        }
        if(obj->GetUniqueType()==enSphere){
            CSphere* sphere = dynamic_cast<CSphere*>(obj);
            CPosition position;
            //转换为全局坐标
            QVector4D vec=sphere->m_pRefCoord->m_mat*QVector4D(sphere->getCenter().x,sphere->getCenter().y,sphere->getCenter().z,1);
            CPosition gloPosition(vec.x(),vec.y(),vec.z());
            if(m_pMainWin->m_nRelyOnWhichCs==csRef){
                box->setEnabled(true);
                box->setCurrentIndex(box->findText((m_pMainWin->m_pcsListMgr->FindNode(sphere->GetRefCoord()))->GetObjectCName()));
                //即为点存储的坐标（出生时参考的坐标）

                sphere->SetExtCoord(sphere->GetRefCoord());

                //当切换扩展坐标系时检测到变化
                connect(box, &QComboBox::currentIndexChanged, this, [=](){
                    table->blockSignals(true); // 暂停信号
                    // QString currentText =box->currentText();
                    // qDebug() << "当前选中的文本是: " << currentText;
                    if(box->currentText()=="机械坐标系"){
                        sphere->SetExtCoord(m_pMainWin->m_pcsListMgr->GetBaseCoordSystem());
                    }else{
                        for(CObject *obj:m_pMainWin->m_ObjectListMgr->getObjectList()){
                            if(box->currentText()==obj->GetObjectCName()){
                                CPcsNode* node=dynamic_cast<CPcsNode*>(obj);
                                sphere->SetExtCoord(node->getPcs());
                                // qDebug() << "Current Text: " << box->currentText();
                                // qDebug() << "Object Name: " << obj->GetObjectCName();
                                break;
                            }
                        }
                    }

                    CPosition temposition=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(gloPosition,sphere->m_pExtCoord);

                    table->setItem(0, 1, new QTableWidgetItem(QString::number(temposition.x,'f',6)));
                    table->setItem(1, 1, new QTableWidgetItem(QString::number(temposition.y,'f',6)));
                    table->setItem(2, 1, new QTableWidgetItem(QString::number(temposition.z,'f',6)));

                    table->viewport()->update();
                });

            }else{
                box->setCurrentIndex(box->findText((m_pMainWin->m_pcsListMgr->FindNode(sphere->GetCurCoord()))->GetObjectCName()));
                box->setEnabled(false);

                sphere->SetExtCoord(sphere->GetCurCoord());
            }

            position=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(gloPosition,sphere->m_pExtCoord);

            if (sphere != nullptr){
                table->setItem(0, 0, new QTableWidgetItem("_x"));
                table->setItem(0, 1, new QTableWidgetItem(QString::number(position.x,'f',6)));
                table->setItem(0, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(1, 0, new QTableWidgetItem("_y"));
                table->setItem(1, 1, new QTableWidgetItem(QString::number(position.y,'f',6)));
                table->setItem(1, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(2, 0, new QTableWidgetItem("_z"));
                table->setItem(2, 1, new QTableWidgetItem(QString::number(position.z,'f',6)));
                table->setItem(2, 2, new QTableWidgetItem(""));

                table->setItem(3, 0, new QTableWidgetItem("直径"));
                table->setItem(3, 1, new QTableWidgetItem(QString::number(sphere->getDiameter(),'f',6)));
                table->setItem(3, 2, new QTableWidgetItem(""));
            }
        }
        if(obj->GetUniqueType()==enCylinder){
            CCylinder* cylinder = dynamic_cast<CCylinder*>(obj);
            CPosition position;
            //转换为全局坐标
            QVector4D vec=cylinder->m_pRefCoord->m_mat*QVector4D(cylinder->getBtm_center().x,cylinder->getBtm_center().y,cylinder->getBtm_center().z,1);
            CPosition gloPosition(vec.x(),vec.y(),vec.z());
            if(m_pMainWin->m_nRelyOnWhichCs==csRef){
                box->setEnabled(true);
                box->setCurrentIndex(box->findText((m_pMainWin->m_pcsListMgr->FindNode(cylinder->GetRefCoord()))->GetObjectCName()));
                //即为点存储的坐标（出生时参考的坐标）

                cylinder->SetExtCoord(cylinder->GetRefCoord());

                //当切换扩展坐标系时检测到变化
                connect(box, &QComboBox::currentIndexChanged, this, [=](){
                    table->blockSignals(true); // 暂停信号
                    // QString currentText =box->currentText();
                    // qDebug() << "当前选中的文本是: " << currentText;
                    if(box->currentText()=="机械坐标系"){
                        cylinder->SetExtCoord(m_pMainWin->m_pcsListMgr->GetBaseCoordSystem());
                    }else{
                        for(CObject *obj:m_pMainWin->m_ObjectListMgr->getObjectList()){
                            if(box->currentText()==obj->GetObjectCName()){
                                CPcsNode* node=dynamic_cast<CPcsNode*>(obj);
                                cylinder->SetExtCoord(node->getPcs());
                                // qDebug() << "Current Text: " << box->currentText();
                                // qDebug() << "Object Name: " << obj->GetObjectCName();
                                break;
                            }
                        }
                    }

                    CPosition temposition=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(gloPosition,cylinder->m_pExtCoord);

                    table->setItem(0, 1, new QTableWidgetItem(QString::number(temposition.x,'f',6)));
                    table->setItem(1, 1, new QTableWidgetItem(QString::number(temposition.y,'f',6)));
                    table->setItem(2, 1, new QTableWidgetItem(QString::number(temposition.z,'f',6)));

                    table->viewport()->update();
                });

            }else{
                box->setCurrentIndex(box->findText((m_pMainWin->m_pcsListMgr->FindNode(cylinder->GetCurCoord()))->GetObjectCName()));
                box->setEnabled(false);

                cylinder->SetExtCoord(cylinder->GetCurCoord());
            }

            position=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(gloPosition,cylinder->m_pExtCoord);

            if (cylinder != nullptr){
                table->setItem(0, 0, new QTableWidgetItem("_x"));
                table->setItem(0, 1, new QTableWidgetItem(QString::number(position.x,'f',6)));
                table->setItem(0, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(1, 0, new QTableWidgetItem("_y"));
                table->setItem(1, 1, new QTableWidgetItem(QString::number(position.y,'f',6)));
                table->setItem(1, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(2, 0, new QTableWidgetItem("_z"));
                table->setItem(2, 1, new QTableWidgetItem(QString::number(position.z,'f',6)));
                table->setItem(2, 2, new QTableWidgetItem(""));

                table->setItem(3, 0, new QTableWidgetItem("直径"));
                table->setItem(3, 1, new QTableWidgetItem(QString::number(cylinder->getDiameter(),'f',6)));
                table->setItem(3, 2, new QTableWidgetItem(""));

                table->setItem(4, 0, new QTableWidgetItem("高度"));
                table->setItem(4, 1, new QTableWidgetItem(QString::number(cylinder->getHeight(),'f',6)));
                table->setItem(4, 2, new QTableWidgetItem(""));
            }
        }
        if(obj->GetUniqueType()==enCone){
            CCone* cone = dynamic_cast<CCone*>(obj);
            CPosition position;
            //转换为全局坐标
            QVector4D vec=cone->m_pRefCoord->m_mat*QVector4D(cone->GetObjectCenterLocalPoint().x,cone->GetObjectCenterLocalPoint().y,cone->GetObjectCenterLocalPoint().z,1);
            CPosition gloPosition(vec.x(),vec.y(),vec.z());
            if(m_pMainWin->m_nRelyOnWhichCs==csRef){
                box->setEnabled(true);
                box->setCurrentIndex(box->findText((m_pMainWin->m_pcsListMgr->FindNode(cone->GetRefCoord()))->GetObjectCName()));
                //即为点存储的坐标（出生时参考的坐标）

                cone->SetExtCoord(cone->GetRefCoord());

                //当切换扩展坐标系时检测到变化
                connect(box, &QComboBox::currentIndexChanged, this, [=](){
                    table->blockSignals(true); // 暂停信号
                    // QString currentText =box->currentText();
                    // qDebug() << "当前选中的文本是: " << currentText;
                    if(box->currentText()=="机械坐标系"){
                        cone->SetExtCoord(m_pMainWin->m_pcsListMgr->GetBaseCoordSystem());
                    }else{
                        for(CObject *obj:m_pMainWin->m_ObjectListMgr->getObjectList()){
                            if(box->currentText()==obj->GetObjectCName()){
                                CPcsNode* node=dynamic_cast<CPcsNode*>(obj);
                                cone->SetExtCoord(node->getPcs());
                                // qDebug() << "Current Text: " << box->currentText();
                                // qDebug() << "Object Name: " << obj->GetObjectCName();
                                break;
                            }
                        }
                    }

                    CPosition temposition=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(gloPosition,cone->m_pExtCoord);

                    table->setItem(0, 1, new QTableWidgetItem(QString::number(temposition.x,'f',6)));
                    table->setItem(1, 1, new QTableWidgetItem(QString::number(temposition.y,'f',6)));
                    table->setItem(2, 1, new QTableWidgetItem(QString::number(temposition.z,'f',6)));

                    table->viewport()->update();
                });
            }else{
                box->setCurrentIndex(box->findText((m_pMainWin->m_pcsListMgr->FindNode(cone->GetCurCoord()))->GetObjectCName()));
                box->setEnabled(false);

                cone->SetExtCoord(cone->GetCurCoord());
            }

            position=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(gloPosition,cone->m_pExtCoord);

            if (cone != nullptr){
                table->setItem(0, 0, new QTableWidgetItem("_x"));
                table->setItem(0, 1, new QTableWidgetItem(QString::number(position.x,'f',6)));
                table->setItem(0, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(1, 0, new QTableWidgetItem("_y"));
                table->setItem(1, 1, new QTableWidgetItem(QString::number(position.y,'f',6)));
                table->setItem(1, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(2, 0, new QTableWidgetItem("_z"));
                table->setItem(2, 1, new QTableWidgetItem(QString::number(position.z,'f',6)));
                table->setItem(2, 2, new QTableWidgetItem(""));

                table->setItem(3, 0, new QTableWidgetItem("高度"));
                table->setItem(3, 1, new QTableWidgetItem(QString::number(cone->getHeight(),'f',6)));
                table->setItem(3, 2, new QTableWidgetItem(""));
            }
        }
        if(obj->GetUniqueType()==enCuboid){
            CCuboid* cuboid= dynamic_cast<CCuboid*>(obj);
            CPosition position;
            //转换为全局坐标
            QVector4D vec=cuboid->m_pRefCoord->m_mat*QVector4D(cuboid->GetObjectCenterLocalPoint().x,cuboid->GetObjectCenterLocalPoint().y,cuboid->GetObjectCenterLocalPoint().z,1);
            CPosition gloPosition(vec.x(),vec.y(),vec.z());
            if(m_pMainWin->m_nRelyOnWhichCs==csRef){
                box->setEnabled(true);
                box->setCurrentIndex(box->findText((m_pMainWin->m_pcsListMgr->FindNode(cuboid->GetRefCoord()))->GetObjectCName()));
                //即为点存储的坐标（出生时参考的坐标）

                cuboid->SetExtCoord(cuboid->GetRefCoord());

                //当切换扩展坐标系时检测到变化
                connect(box, &QComboBox::currentIndexChanged, this, [=](){
                    table->blockSignals(true); // 暂停信号
                    // QString currentText =box->currentText();
                    // qDebug() << "当前选中的文本是: " << currentText;
                    if(box->currentText()=="机械坐标系"){
                        cuboid->SetExtCoord(m_pMainWin->m_pcsListMgr->GetBaseCoordSystem());
                    }else{
                        for(CObject *obj:m_pMainWin->m_ObjectListMgr->getObjectList()){
                            if(box->currentText()==obj->GetObjectCName()){
                                CPcsNode* node=dynamic_cast<CPcsNode*>(obj);
                                cuboid->SetExtCoord(node->getPcs());
                                // qDebug() << "Current Text: " << box->currentText();
                                // qDebug() << "Object Name: " << obj->GetObjectCName();
                                break;
                            }
                        }
                    }

                    CPosition temposition=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(gloPosition,cuboid->m_pExtCoord);

                    table->setItem(0, 1, new QTableWidgetItem(QString::number(temposition.x,'f',6)));
                    table->setItem(1, 1, new QTableWidgetItem(QString::number(temposition.y,'f',6)));
                    table->setItem(2, 1, new QTableWidgetItem(QString::number(temposition.z,'f',6)));

                    table->viewport()->update();
                });
            }else{
                box->setCurrentIndex(box->findText((m_pMainWin->m_pcsListMgr->FindNode(cuboid->GetCurCoord()))->GetObjectCName()));
                box->setEnabled(false);

                cuboid->SetExtCoord(cuboid->GetCurCoord());
            }

            position=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(gloPosition,cuboid->m_pExtCoord);

            if (cuboid != nullptr){
                table->setItem(0, 0, new QTableWidgetItem("_x"));
                table->setItem(0, 1, new QTableWidgetItem(QString::number(position.x,'f',6)));
                table->setItem(0, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(1, 0, new QTableWidgetItem("_y"));
                table->setItem(1, 1, new QTableWidgetItem(QString::number(position.y,'f',6)));
                table->setItem(1, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(2, 0, new QTableWidgetItem("_z"));
                table->setItem(2, 1, new QTableWidgetItem(QString::number(position.z,'f',6)));
                table->setItem(2, 2, new QTableWidgetItem(""));

                table->setItem(3, 0, new QTableWidgetItem("长度"));
                table->setItem(3, 1, new QTableWidgetItem(QString::number(cuboid->getLength(),'f',6)));
                table->setItem(3, 2, new QTableWidgetItem(""));

                table->setItem(4, 0, new QTableWidgetItem("宽度"));
                table->setItem(4, 1, new QTableWidgetItem(QString::number(cuboid->getWidth(),'f',6)));
                table->setItem(4, 2, new QTableWidgetItem(""));

                table->setItem(5, 0, new QTableWidgetItem("高度"));
                table->setItem(5, 1, new QTableWidgetItem(QString::number(cuboid->getHeight(),'f',6)));
                table->setItem(5, 2, new QTableWidgetItem(""));

                table->setItem(6, 0, new QTableWidgetItem("法向量_x"));
                table->setItem(6, 1, new QTableWidgetItem(QString::number(cuboid->getNormal().x(),'f',6)));
                table->setItem(6, 2, new QTableWidgetItem(""));

                table->setItem(7, 0, new QTableWidgetItem("法向量_y"));
                table->setItem(7, 1, new QTableWidgetItem(QString::number(cuboid->getNormal().y(),'f',6)));
                table->setItem(7, 2, new QTableWidgetItem(""));

                table->setItem(8, 0, new QTableWidgetItem("法向量_z"));
                table->setItem(8, 1, new QTableWidgetItem(QString::number(cuboid->getNormal().z(),'f',6)));
                table->setItem(8, 2, new QTableWidgetItem(""));
            }
        }
        if(obj->m_strCName.left(5)=="工件坐标系" || obj->m_strCName.left(5)=="临时坐标系"){
            CPcsNode* pcsnode=dynamic_cast<CPcsNode*>(obj);
            CPosition position;
            position.x=pcsnode->getPcs()->m_poso.x;
            position.y=pcsnode->getPcs()->m_poso.y;
            position.z=pcsnode->getPcs()->m_poso.z;
            if (pcsnode != nullptr){
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

        if(obj->m_strCName.left(2)=="距离"){
            CDistance* distance=dynamic_cast<CDistance*>(obj);
            qDebug()<<"data里面的距离"<<distance->getdistance();
            CPosition position;
            position.x= std::fabs(distance->getUptolerance());
            position.y= std::fabs(distance->getUndertolerance());
            position.z = std::fabs(distance->getdistance());
            qDebug()<<"data里面的距离"<<position.z;
            if (distance != nullptr){
                table->setItem(0, 0, new QTableWidgetItem("上公差"));
                table->setItem(0, 1, new QTableWidgetItem(QString::number(position.x,'f',6)));
                table->setItem(0, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(1, 0, new QTableWidgetItem("下公差"));
                table->setItem(1, 1, new QTableWidgetItem(QString::number(position.y,'f',6)));
                table->setItem(1, 2, new QTableWidgetItem("")); // 可选择设置状态

                table->setItem(2, 0, new QTableWidgetItem("距离"));
                table->setItem(2, 1, new QTableWidgetItem(QString::number(position.z,'f',6)));
                table->setItem(2, 2, new QTableWidgetItem(""));

                table->setItem(3, 0, new QTableWidgetItem("是否合格"));
                if(distance->judge()==true){
                    table->setItem(3, 1, new QTableWidgetItem(QString("合格")));
                }else{
                    table->setItem(3, 1, new QTableWidgetItem(QString("不合格")));
                }
            }
        }
        
        if(obj->m_strCName.left(2)=="角度"){
            CAngle* angle=dynamic_cast<CAngle*>(obj);
            qDebug()<<"data里面的角度"<<angle->getAngleValue();
            CPosition position;
            position.x=angle->getUptolerance();
            position.y=angle->getUndertolerance();
            position.z=angle->getAngleValue();
            qDebug()<<"data里面的角度"<<position.z;
            if (angle != nullptr){
            table->setItem(0, 0, new QTableWidgetItem("上公差"));
            table->setItem(0, 1, new QTableWidgetItem(QString::number(position.x,'f',6)));
            table->setItem(0, 2, new QTableWidgetItem("")); // 可选择设置状态

            table->setItem(1, 0, new QTableWidgetItem("下公差"));
            table->setItem(1, 1, new QTableWidgetItem(QString::number(position.y,'f',6)));
            table->setItem(1, 2, new QTableWidgetItem("")); // 可选择设置状态

            table->setItem(2, 0, new QTableWidgetItem("角度"));
            table->setItem(2, 1, new QTableWidgetItem(QString::number(position.z,'f',6)));
            table->setItem(2, 2, new QTableWidgetItem(""));

            table->setItem(3, 0, new QTableWidgetItem("是否合格"));
            if(angle->judge()==true){
                table->setItem(3, 1, new QTableWidgetItem(QString("合格")));
            }else{
                table->setItem(3, 1, new QTableWidgetItem(QString("不合格")));
            }
            }
        }

        dataModify();

}

// 创建坐标系修改数据，切换extend坐标系后无法修改（blockSignals）
void DataWidget::dataModify(){
    disconnect(table, &QTableWidget::cellChanged, nullptr, nullptr); //断开之前的信号槽连接

    table->blockSignals(false); // 恢复信号
    CObject* obj=m_pMainWin->getObjectListMgr()->getObjectList()[index];

    connect(table, &QTableWidget::cellChanged, this, [=](int row, int column) {
        // 检查是否是坐标列被修改
        if (column == 1) {
            bool ok;
            double newValue = table->item(row, column)->text().toDouble(&ok); // 获取用户输入的值
            if (!ok) {
                // 如果输入无效，恢复旧值并返回
                QMessageBox::warning(nullptr, "Invalid Input", "Please enter a valid number.");
                return;
            }

            // 根据 obj 类型更新对应的坐标值
            if (obj->GetUniqueType() == enPoint) {
                CPoint* point = dynamic_cast<CPoint*>(obj);
                // 更新点坐标
                switch (row) {
                case 0:{
                    CPosition pTemp={newValue,point->m_pt.y,point->m_pt.z};

                    if(!(box->currentText()=="机械坐标系")){
                        for(CObject *obj:m_pMainWin->m_ObjectListMgr->getObjectList()){
                            if(box->currentText()==obj->GetObjectCName()){
                                CPcsNode* node=dynamic_cast<CPcsNode*>(obj);
                                //转换为全局坐标
                                QVector4D vec=node->getPcs()->m_mat*QVector4D(newValue,point->m_pt.y,point->m_pt.z,1);
                                CPosition glo(vec.x(),vec.y(),vec.z());
                                pTemp=glo;
                                break;
                            }
                        }
                    }
                    CPosition result=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(pTemp,point->m_pRefCoord);
                    point->m_pt=result;
                    break;
                }
                case 1:{
                    CPosition pTemp={point->m_pt.x,newValue,point->m_pt.z};

                    if(!(box->currentText()=="机械坐标系")){
                        for(CObject *obj:m_pMainWin->m_ObjectListMgr->getObjectList()){
                            if(box->currentText()==obj->GetObjectCName()){
                                CPcsNode* node=dynamic_cast<CPcsNode*>(obj);
                                //转换为全局坐标
                                QVector4D vec=node->getPcs()->m_mat*QVector4D(point->m_pt.x,newValue,point->m_pt.z,1);
                                CPosition glo(vec.x(),vec.y(),vec.z());
                                pTemp=glo;
                                break;
                            }
                        }
                    }
                    CPosition result=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(pTemp,point->m_pRefCoord);
                    point->m_pt=result;
                    break;
                }
                case 2:{
                    CPosition pTemp={point->m_pt.x,point->m_pt.y,newValue};

                    if(!(box->currentText()=="机械坐标系")){
                        for(CObject *obj:m_pMainWin->m_ObjectListMgr->getObjectList()){
                            if(box->currentText()==obj->GetObjectCName()){
                                CPcsNode* node=dynamic_cast<CPcsNode*>(obj);
                                //转换为全局坐标
                                QVector4D vec=node->getPcs()->m_mat*QVector4D(point->m_pt.x,point->m_pt.y,newValue,1);
                                CPosition glo(vec.x(),vec.y(),vec.z());
                                pTemp=glo;
                                break;
                            }
                        }
                    }
                    CPosition result=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(pTemp,point->m_pRefCoord);
                    point->m_pt=result;
                    break;
                }
                }
            }

            else if (obj->GetUniqueType() == enLine) {
                CLine* line = dynamic_cast<CLine*>(obj);
                // 更新线的坐标
                switch (row) {
                case 0:{
                    CPosition pTemp={newValue,line->getPosition1().y,line->getPosition1().z};

                    if(!(box->currentText()=="机械坐标系")){
                        for(CObject *obj:m_pMainWin->m_ObjectListMgr->getObjectList()){
                            if(box->currentText()==obj->GetObjectCName()){
                                CPcsNode* node=dynamic_cast<CPcsNode*>(obj);
                                //转换为全局坐标
                                QVector4D vec=node->getPcs()->m_mat*QVector4D(newValue,line->getPosition1().y,line->getPosition1().z,1);
                                CPosition glo(vec.x(),vec.y(),vec.z());
                                pTemp=glo;
                                break;
                            }
                        }
                    }
                    CPosition result=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(pTemp,line->m_pRefCoord);
                    line->begin=result;
                    break;
                }
                case 1:{
                    CPosition pTemp={line->getPosition1().x,newValue,line->getPosition1().z};

                    if(!(box->currentText()=="机械坐标系")){
                        for(CObject *obj:m_pMainWin->m_ObjectListMgr->getObjectList()){
                            if(box->currentText()==obj->GetObjectCName()){
                                CPcsNode* node=dynamic_cast<CPcsNode*>(obj);
                                //转换为全局坐标
                                QVector4D vec=node->getPcs()->m_mat*QVector4D(line->getPosition1().x,newValue,line->getPosition1().z,1);
                                CPosition glo(vec.x(),vec.y(),vec.z());
                                pTemp=glo;
                                break;
                            }
                        }
                    }
                    CPosition result=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(pTemp,line->m_pRefCoord);
                    line->begin=result;
                    break;
                }
                case 2:{
                    CPosition pTemp={line->getPosition1().x,line->getPosition1().y,newValue};

                    if(!(box->currentText()=="机械坐标系")){
                        for(CObject *obj:m_pMainWin->m_ObjectListMgr->getObjectList()){
                            if(box->currentText()==obj->GetObjectCName()){
                                CPcsNode* node=dynamic_cast<CPcsNode*>(obj);
                                //转换为全局坐标
                                QVector4D vec=node->getPcs()->m_mat*QVector4D(line->getPosition1().x,line->getPosition1().y,newValue,1);
                                CPosition glo(vec.x(),vec.y(),vec.z());
                                pTemp=glo;
                                break;
                            }
                        }
                    }
                    CPosition result=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(pTemp,line->m_pRefCoord);
                    line->begin=result;
                    break;
                }
                case 3:{
                    CPosition pTemp={newValue,line->getPosition2().y,line->getPosition2().z};

                    if(!(box->currentText()=="机械坐标系")){
                        for(CObject *obj:m_pMainWin->m_ObjectListMgr->getObjectList()){
                            if(box->currentText()==obj->GetObjectCName()){
                                CPcsNode* node=dynamic_cast<CPcsNode*>(obj);
                                //转换为全局坐标
                                QVector4D vec=node->getPcs()->m_mat*QVector4D(newValue,line->getPosition2().y,line->getPosition2().z,1);
                                CPosition glo(vec.x(),vec.y(),vec.z());
                                pTemp=glo;
                                break;
                            }
                        }
                    }
                    CPosition result=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(pTemp,line->m_pRefCoord);
                    line->end=result;
                    break;
                }
                case 4:{
                    CPosition pTemp={line->getPosition2().x,newValue,line->getPosition2().z};

                    if(!(box->currentText()=="机械坐标系")){
                        for(CObject *obj:m_pMainWin->m_ObjectListMgr->getObjectList()){
                            if(box->currentText()==obj->GetObjectCName()){
                                CPcsNode* node=dynamic_cast<CPcsNode*>(obj);
                                //转换为全局坐标
                                QVector4D vec=node->getPcs()->m_mat*QVector4D(line->getPosition2().x,newValue,line->getPosition2().z,1);
                                CPosition glo(vec.x(),vec.y(),vec.z());
                                pTemp=glo;
                                break;
                            }
                        }
                    }
                    CPosition result=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(pTemp,line->m_pRefCoord);
                    line->end=result;
                    break;
                }
                case 5:{
                    CPosition pTemp={line->getPosition2().x,line->getPosition2().y,newValue};

                    if(!(box->currentText()=="机械坐标系")){
                        for(CObject *obj:m_pMainWin->m_ObjectListMgr->getObjectList()){
                            if(box->currentText()==obj->GetObjectCName()){
                                CPcsNode* node=dynamic_cast<CPcsNode*>(obj);
                                //转换为全局坐标
                                QVector4D vec=node->getPcs()->m_mat*QVector4D(line->getPosition2().x,line->getPosition2().y,newValue,1);
                                CPosition glo(vec.x(),vec.y(),vec.z());
                                pTemp=glo;
                                break;
                            }
                        }
                    }
                    CPosition result=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(pTemp,line->m_pRefCoord);
                    line->end=result;
                    break;
                }
                }
            }

            else if (obj->GetUniqueType() == enCircle) {
                CCircle* circle = dynamic_cast<CCircle*>(obj);
                // 更新圆的坐标或直径
                switch (row) {
                case 0:{
                    CPosition pTemp={newValue,circle->m_pt.y,circle->m_pt.z};

                    if(!(box->currentText()=="机械坐标系")){
                        for(CObject *obj:m_pMainWin->m_ObjectListMgr->getObjectList()){
                            if(box->currentText()==obj->GetObjectCName()){
                                CPcsNode* node=dynamic_cast<CPcsNode*>(obj);
                                //转换为全局坐标
                                QVector4D vec=node->getPcs()->m_mat*QVector4D(newValue,circle->m_pt.y,circle->m_pt.z,1);
                                CPosition glo(vec.x(),vec.y(),vec.z());
                                pTemp=glo;
                                break;
                            }
                        }
                    }
                    CPosition result=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(pTemp,circle->m_pRefCoord);
                    circle->m_pt=result;
                    break;
                }
                case 1:{
                    CPosition pTemp={circle->m_pt.x,newValue,circle->m_pt.z};

                    if(!(box->currentText()=="机械坐标系")){
                        for(CObject *obj:m_pMainWin->m_ObjectListMgr->getObjectList()){
                            if(box->currentText()==obj->GetObjectCName()){
                                CPcsNode* node=dynamic_cast<CPcsNode*>(obj);
                                //转换为全局坐标
                                QVector4D vec=node->getPcs()->m_mat*QVector4D(circle->m_pt.x,newValue,circle->m_pt.z,1);
                                CPosition glo(vec.x(),vec.y(),vec.z());
                                pTemp=glo;
                                break;
                            }
                        }
                    }
                    CPosition result=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(pTemp,circle->m_pRefCoord);
                    circle->m_pt=result;
                    break;
                }
                case 2:{
                    CPosition pTemp={circle->m_pt.x,circle->m_pt.y,newValue};

                    if(!(box->currentText()=="机械坐标系")){
                        for(CObject *obj:m_pMainWin->m_ObjectListMgr->getObjectList()){
                            if(box->currentText()==obj->GetObjectCName()){
                                CPcsNode* node=dynamic_cast<CPcsNode*>(obj);
                                //转换为全局坐标
                                QVector4D vec=node->getPcs()->m_mat*QVector4D(circle->m_pt.x,circle->m_pt.y,newValue,1);
                                CPosition glo(vec.x(),vec.y(),vec.z());
                                pTemp=glo;
                                break;
                            }
                        }
                    }
                    CPosition result=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(pTemp,circle->m_pRefCoord);
                    circle->m_pt=result;
                    break;
                }
                case 3:{
                    circle->m_d=newValue;
                    break;
                }
                }
            }

            else if(obj->GetUniqueType() == enPlane){
                CPlane* plane = dynamic_cast<CPlane*>(obj);
                switch(row){
                case 0:{
                    CPosition pTemp={newValue,plane->getCenter().y,plane->getCenter().z};
                    if(!(box->currentText()=="机械坐标系")){
                        for(CObject *obj:m_pMainWin->m_ObjectListMgr->getObjectList()){
                            if(box->currentText()==obj->GetObjectCName()){
                                CPcsNode* node=dynamic_cast<CPcsNode*>(obj);
                                //转换为全局坐标
                                QVector4D vec=node->getPcs()->m_mat*QVector4D(newValue,plane->getCenter().y,plane->getCenter().z,1);
                                CPosition glo(vec.x(),vec.y(),vec.z());
                                pTemp=glo;
                                break;
                            }
                        }
                    }
                    CPosition result=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(pTemp,plane->m_pRefCoord);
                    plane->center=result;
                    break;
                }
                case 1:{
                    CPosition pTemp={plane->getCenter().x,newValue,plane->getCenter().z};
                    if(!(box->currentText()=="机械坐标系")){
                        for(CObject *obj:m_pMainWin->m_ObjectListMgr->getObjectList()){
                            if(box->currentText()==obj->GetObjectCName()){
                                CPcsNode* node=dynamic_cast<CPcsNode*>(obj);
                                //转换为全局坐标
                                QVector4D vec=node->getPcs()->m_mat*QVector4D(plane->getCenter().x,newValue,plane->getCenter().z,1);
                                CPosition glo(vec.x(),vec.y(),vec.z());
                                pTemp=glo;
                                break;
                            }
                        }
                    }
                    CPosition result=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(pTemp,plane->m_pRefCoord);
                    plane->center=result;
                    break;
                }
                case 2:{
                    CPosition pTemp={plane->getCenter().x,plane->getCenter().y,newValue};
                    if(!(box->currentText()=="机械坐标系")){
                        for(CObject *obj:m_pMainWin->m_ObjectListMgr->getObjectList()){
                            if(box->currentText()==obj->GetObjectCName()){
                                CPcsNode* node=dynamic_cast<CPcsNode*>(obj);
                                //转换为全局坐标
                                QVector4D vec=node->getPcs()->m_mat*QVector4D(plane->getCenter().x,plane->getCenter().y,newValue,1);
                                CPosition glo(vec.x(),vec.y(),vec.z());
                                pTemp=glo;
                                break;
                            }
                        }
                    }
                    CPosition result=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(pTemp,plane->m_pRefCoord);
                    plane->center=result;
                    break;
                }
                case 3:{
                    plane->length=newValue;
                    break;
                }
                case 4:{
                    plane->width=newValue;
                    break;
                }
                case 5:{
                    plane->normal.setX(newValue);
                    break;
                }
                case 6:{
                    plane->normal.setY(newValue);
                    break;
                }
                case 7:{
                    plane->normal.setZ(newValue);
                    break;
                }
                }
            }

            else if(obj->GetUniqueType()==enSphere){
                CSphere* sphere = dynamic_cast<CSphere*>(obj);
                switch(row){
                case 0:{
                    CPosition pTemp={newValue,sphere->getCenter().y,sphere->getCenter().z};
                    if(!(box->currentText()=="机械坐标系")){
                        for(CObject *obj:m_pMainWin->m_ObjectListMgr->getObjectList()){
                            if(box->currentText()==obj->GetObjectCName()){
                                CPcsNode* node=dynamic_cast<CPcsNode*>(obj);
                                //转换为全局坐标
                                QVector4D vec=node->getPcs()->m_mat*QVector4D(newValue,sphere->getCenter().y,sphere->getCenter().z,1);
                                CPosition glo(vec.x(),vec.y(),vec.z());
                                pTemp=glo;
                                break;
                            }
                        }
                    }
                    CPosition result=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(pTemp,sphere->m_pRefCoord);
                    sphere->center=result;
                    break;
                }
                case 1:{
                    CPosition pTemp={sphere->getCenter().x,newValue,sphere->getCenter().z};
                    if(!(box->currentText()=="机械坐标系")){
                        for(CObject *obj:m_pMainWin->m_ObjectListMgr->getObjectList()){
                            if(box->currentText()==obj->GetObjectCName()){
                                CPcsNode* node=dynamic_cast<CPcsNode*>(obj);
                                //转换为全局坐标
                                QVector4D vec=node->getPcs()->m_mat*QVector4D(sphere->getCenter().x,newValue,sphere->getCenter().z,1);
                                CPosition glo(vec.x(),vec.y(),vec.z());
                                pTemp=glo;
                                break;
                            }
                        }
                    }
                    CPosition result=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(pTemp,sphere->m_pRefCoord);
                    sphere->center=result;
                    break;
                }
                case 2:{
                    CPosition pTemp={sphere->getCenter().x,sphere->getCenter().y,newValue};
                    if(!(box->currentText()=="机械坐标系")){
                        for(CObject *obj:m_pMainWin->m_ObjectListMgr->getObjectList()){
                            if(box->currentText()==obj->GetObjectCName()){
                                CPcsNode* node=dynamic_cast<CPcsNode*>(obj);
                                //转换为全局坐标
                                QVector4D vec=node->getPcs()->m_mat*QVector4D(sphere->getCenter().x,sphere->getCenter().y,newValue,1);
                                CPosition glo(vec.x(),vec.y(),vec.z());
                                pTemp=glo;
                                break;
                            }
                        }
                    }
                    CPosition result=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(pTemp,sphere->m_pRefCoord);
                    sphere->center=result;
                    break;
                }
                case 3:{
                    sphere->diameter=newValue;
                    break;
                }
                }
            }

            else if(obj->GetUniqueType()==enCylinder){
                CCylinder* cylinder = dynamic_cast<CCylinder*>(obj);
                switch(row){
                case 0:{
                    CPosition pTemp={newValue,cylinder->getBtm_center().y,cylinder->getBtm_center().z};
                    if(!(box->currentText()=="机械坐标系")){
                        for(CObject *obj:m_pMainWin->m_ObjectListMgr->getObjectList()){
                            if(box->currentText()==obj->GetObjectCName()){
                                CPcsNode* node=dynamic_cast<CPcsNode*>(obj);
                                //转换为全局坐标
                                QVector4D vec=node->getPcs()->m_mat*QVector4D(newValue,cylinder->getBtm_center().y,cylinder->getBtm_center().z,1);
                                CPosition glo(vec.x(),vec.y(),vec.z());
                                pTemp=glo;
                                break;
                            }
                        }
                    }
                    CPosition result=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(pTemp,cylinder->m_pRefCoord);
                    cylinder->btm_center=result;
                    break;
                }
                case 1:{
                    CPosition pTemp={cylinder->getBtm_center().x,newValue,cylinder->getBtm_center().z};
                    if(!(box->currentText()=="机械坐标系")){
                        for(CObject *obj:m_pMainWin->m_ObjectListMgr->getObjectList()){
                            if(box->currentText()==obj->GetObjectCName()){
                                CPcsNode* node=dynamic_cast<CPcsNode*>(obj);
                                //转换为全局坐标
                                QVector4D vec=node->getPcs()->m_mat*QVector4D(cylinder->getBtm_center().x,newValue,cylinder->getBtm_center().z,1);
                                CPosition glo(vec.x(),vec.y(),vec.z());
                                pTemp=glo;
                                break;
                            }
                        }
                    }
                    CPosition result=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(pTemp,cylinder->m_pRefCoord);
                    cylinder->btm_center=result;
                    break;
                }
                case 2:{
                    CPosition pTemp={cylinder->getBtm_center().x,cylinder->getBtm_center().y,newValue};
                    if(!(box->currentText()=="机械坐标系")){
                        for(CObject *obj:m_pMainWin->m_ObjectListMgr->getObjectList()){
                            if(box->currentText()==obj->GetObjectCName()){
                                CPcsNode* node=dynamic_cast<CPcsNode*>(obj);
                                //转换为全局坐标
                                QVector4D vec=node->getPcs()->m_mat*QVector4D(cylinder->getBtm_center().x,cylinder->getBtm_center().y,newValue,1);
                                CPosition glo(vec.x(),vec.y(),vec.z());
                                pTemp=glo;
                                break;
                            }
                        }
                    }
                    CPosition result=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(pTemp,cylinder->m_pRefCoord);
                    cylinder->btm_center=result;
                    break;
                }
                case 3:{
                    cylinder->diameter=newValue;
                    break;
                }
                case 4:{
                    cylinder->height=newValue;
                }
                }
            }

            else if(obj->GetUniqueType()==enCone){
                CCone* cone = dynamic_cast<CCone*>(obj);
                switch(row){
                case 0:{
                    CPosition pTemp={newValue,cone->GetObjectCenterLocalPoint().y,cone->GetObjectCenterLocalPoint().z};
                    if(!(box->currentText()=="机械坐标系")){
                        for(CObject *obj:m_pMainWin->m_ObjectListMgr->getObjectList()){
                            if(box->currentText()==obj->GetObjectCName()){
                                CPcsNode* node=dynamic_cast<CPcsNode*>(obj);
                                //转换为全局坐标
                                QVector4D vec=node->getPcs()->m_mat*QVector4D(newValue,cone->GetObjectCenterLocalPoint().y,cone->GetObjectCenterLocalPoint().z,1);
                                CPosition glo(vec.x(),vec.y(),vec.z());
                                pTemp=glo;
                                break;
                            }
                        }
                    }
                    CPosition result=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(pTemp,cone->m_pRefCoord);
                    cone->vertex=result;
                    break;
                }
                case 1:{
                    CPosition pTemp={cone->GetObjectCenterLocalPoint().x,newValue,cone->GetObjectCenterLocalPoint().z};
                    if(!(box->currentText()=="机械坐标系")){
                        for(CObject *obj:m_pMainWin->m_ObjectListMgr->getObjectList()){
                            if(box->currentText()==obj->GetObjectCName()){
                                CPcsNode* node=dynamic_cast<CPcsNode*>(obj);
                                //转换为全局坐标
                                QVector4D vec=node->getPcs()->m_mat*QVector4D(cone->GetObjectCenterLocalPoint().x,newValue,cone->GetObjectCenterLocalPoint().z,1);
                                CPosition glo(vec.x(),vec.y(),vec.z());
                                pTemp=glo;
                                break;
                            }
                        }
                    }
                    CPosition result=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(pTemp,cone->m_pRefCoord);
                    cone->vertex=result;
                    break;
                }
                case 2:{
                    CPosition pTemp={cone->GetObjectCenterLocalPoint().x,cone->GetObjectCenterLocalPoint().y,newValue};
                    if(!(box->currentText()=="机械坐标系")){
                        for(CObject *obj:m_pMainWin->m_ObjectListMgr->getObjectList()){
                            if(box->currentText()==obj->GetObjectCName()){
                                CPcsNode* node=dynamic_cast<CPcsNode*>(obj);
                                //转换为全局坐标
                                QVector4D vec=node->getPcs()->m_mat*QVector4D(cone->GetObjectCenterLocalPoint().x,cone->GetObjectCenterLocalPoint().y,newValue,1);
                                CPosition glo(vec.x(),vec.y(),vec.z());
                                pTemp=glo;
                                break;
                            }
                        }
                    }
                    CPosition result=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(pTemp,cone->m_pRefCoord);
                    cone->vertex=result;
                    break;
                }
                case 3:{
                    cone->height=newValue;
                    break;
                }
                }
            }

            else if(obj->GetUniqueType()==enCuboid){
                CCuboid* cuboid= dynamic_cast<CCuboid*>(obj);
                switch(row){
                case 0:{
                    CPosition pTemp={newValue,cuboid->GetObjectCenterLocalPoint().y,cuboid->GetObjectCenterLocalPoint().z};
                    if(!(box->currentText()=="机械坐标系")){
                        for(CObject *obj:m_pMainWin->m_ObjectListMgr->getObjectList()){
                            if(box->currentText()==obj->GetObjectCName()){
                                CPcsNode* node=dynamic_cast<CPcsNode*>(obj);
                                //转换为全局坐标
                                QVector4D vec=node->getPcs()->m_mat*QVector4D(newValue,cuboid->GetObjectCenterLocalPoint().y,cuboid->GetObjectCenterLocalPoint().z,1);
                                CPosition glo(vec.x(),vec.y(),vec.z());
                                pTemp=glo;
                                break;
                            }
                        }
                    }
                    CPosition result=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(pTemp,cuboid->m_pRefCoord);
                    cuboid->center=result;
                    break;
                }
                case 1:{
                    CPosition pTemp={cuboid->GetObjectCenterLocalPoint().x,newValue,cuboid->GetObjectCenterLocalPoint().z};
                    if(!(box->currentText()=="机械坐标系")){
                        for(CObject *obj:m_pMainWin->m_ObjectListMgr->getObjectList()){
                            if(box->currentText()==obj->GetObjectCName()){
                                CPcsNode* node=dynamic_cast<CPcsNode*>(obj);
                                //转换为全局坐标
                                QVector4D vec=node->getPcs()->m_mat*QVector4D(cuboid->GetObjectCenterLocalPoint().x,newValue,cuboid->GetObjectCenterLocalPoint().z,1);
                                CPosition glo(vec.x(),vec.y(),vec.z());
                                pTemp=glo;
                                break;
                            }
                        }
                    }
                    CPosition result=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(pTemp,cuboid->m_pRefCoord);
                    cuboid->center=result;
                    break;
                }
                case 2:{
                    CPosition pTemp={cuboid->GetObjectCenterLocalPoint().x,cuboid->GetObjectCenterLocalPoint().y,newValue};
                    if(!(box->currentText()=="机械坐标系")){
                        for(CObject *obj:m_pMainWin->m_ObjectListMgr->getObjectList()){
                            if(box->currentText()==obj->GetObjectCName()){
                                CPcsNode* node=dynamic_cast<CPcsNode*>(obj);
                                //转换为全局坐标
                                QVector4D vec=node->getPcs()->m_mat*QVector4D(cuboid->GetObjectCenterLocalPoint().x,cuboid->GetObjectCenterLocalPoint().y,newValue,1);
                                CPosition glo(vec.x(),vec.y(),vec.z());
                                pTemp=glo;
                                break;
                            }
                        }
                    }
                    CPosition result=m_pMainWin->m_pcsListMgr->GetLocalPosOfCertainPcs(pTemp,cuboid->m_pRefCoord);
                    cuboid->center=result;
                    break;
                }
                case 3:{
                    cuboid->length=newValue;
                    break;
                }
                case 4:{
                    cuboid->width=newValue;
                    break;
                }
                case 5:{
                    cuboid->height=newValue;
                    break;
                }
                case 6:{
                    cuboid->normal.setX(newValue);
                    break;
                }
                case 7:{
                    cuboid->normal.setY(newValue);
                    break;
                }
                case 8:{
                    cuboid->normal.setZ(newValue);
                    break;
                }
                }
            }

            // else if(obj->m_strCName.left(5)=="工件坐标系"){
            // }

            // 重新绘制图形
            m_pMainWin->NotifySubscribe();
        }
    });

}
