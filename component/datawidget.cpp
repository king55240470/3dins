#include "datawidget.h"
#include <QHeaderView>
#include <QVector4D>
#include <QTimer>
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

void DataWidget::getobjindex(int objindex)
{
    index=objindex;
}

void DataWidget::updateinfo()
{
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
            CPosition position;
            //转换为全局坐标
            QVector4D vec=point->m_pRefCoord->m_mat*QVector4D(point->m_pt.x,point->m_pt.y,point->m_pt.z,1);
            CPosition gloPosition(vec.x(),vec.y(),vec.z());

            if(m_pMainWin->m_nRelyOnWhichCs==csRef){
                box->setEnabled(true);
                box->setCurrentIndex(box->findText((m_pMainWin->m_pcsListMgr->FindNode(point->GetRefCoord()))->GetObjectCName())); // 设置box当前数据显示参考的坐标系
                //即为点存储的坐标（出生时参考的坐标）

                point->SetExtCoord(point->GetRefCoord());

                //当切换扩展坐标系时检测到变化
                connect(box, &QComboBox::currentIndexChanged, this, [=](){
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
        if(obj->m_strCName.left(5)=="工件坐标系"){
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
}
