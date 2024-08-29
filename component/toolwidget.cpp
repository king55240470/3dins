#include"toolwidget.h"
#include"toolaction.h"
#include <QDir>
#include <QStringList>
#include <QFileInfo>
#include<QGridLayout>
ToolWidget::ToolWidget(QWidget *parent)
    : QWidget(parent) {
    // 设置布局
    QVBoxLayout *layout = new QVBoxLayout(this);
    setLayout(layout);
    // 创建工具栏
    createToolBars();
}
bool getImagePaths(const QString& directory,QStringList &iconPaths,QStringList & iconNames ) {
    // 指定图片所在目录
    QString imagePath = directory; // 替换成你的图片目录路径

    // 创建 QDir 对象来操作目录
    QDir dir(imagePath);
    if (!dir.exists()) {
        qWarning() << "Directory does not exist:" << imagePath;
        return false;
    }

    // 获取目录中的所有 PNG 文件
    QStringList filters;
    filters << "*.png";
    QStringList fileNames = dir.entryList(filters, QDir::Files);

    foreach (const QString &fileName, fileNames) {
        // 去掉文件后缀
        QString baseName = QFileInfo(fileName).baseName();
        iconNames << baseName;
    }
    // qDebug()<<fileNames;


    QStringList filters2;
    filters << "*.png";
    dir.setNameFilters(filters2);
    QFileInfoList fileInfoList = dir.entryInfoList(filters2);
    for (const QFileInfo& fileInfo : fileInfoList) {
        iconPaths << fileInfo.absoluteFilePath();
    }
    qDebug()<<iconPaths;
   qDebug()<<iconNames;
    return true;
}
void ToolWidget::createToolBars() {
    ToolActionKind actionKinds[4]={FirstToolBarAction,SecondToolBarAction,
    ThirdToolBarAction,FourthToolBarAction};
    QString ToolBarTitle[4]={"第一栏","第二栏","第三栏","第四栏"};
    QString iconPaths_First[FirstToolBarActions_Num]={":/icon/all.png", ":/icon/angle.png", ":/icon/arcLength.png", ":/icon/arclong.png", ":/icon/arclong1.png", ":/icon/axes.png", ":/icon/chongzhi.png", ":/icon/diameter.png", ":/icon/dista.png", ":/icon/distance.png", ":/icon/download.png", ":/icon/end.png", ":/icon/file.png", ":/icon/front.png", ":/icon/full.png", ":/icon/guide.png", ":/icon/half.png", ":/icon/icon.zip", ":/icon/isometric.png", ":/icon/jia.png", ":/icon/jian.png", ":/icon/jiaodu.png", ":/icon/label.png", ":/icon/move.png", ":/icon/noxuanzhuan.png", ":/icon/pause.png", ":/icon/piliang.png", ":/icon/point.png", ":/icon/point3.png", ":/icon/position.png", ":/icon/qinkong.png", ":/icon/radius.png", ":/icon/rectangle.png", ":/icon/right.png", ":/icon/settings.png", ":/icon/setup.png", ":/icon/shang.png", ":/icon/start.png", ":/icon/tole.png", ":/icon/tolerance.png", ":/icon/up.png", ":/icon/x.png", ":/icon/x1.png", ":/icon/x2.png", ":/icon/xia.png", ":/icon/xian.png", ":/icon/xmove.png", ":/icon/xuanzhuan.png", ":/icon/y.png", ":/icon/y1.png", ":/icon/Y2.png", ":/icon/you.png", ":/icon/yuan.png", ":/icon/z1.png", ":/icon/Z2.png", ":/icon/zhuan.png", ":/icon/zoomin.png", ":/icon/zoomout.png", ":/icon/zuo.png"};;
    QString iconPaths_Second[SecondToolBarActions_Num]={":/iconhxt/1.1.png", ":/iconhxt/1.10.png", ":/iconhxt/1.11.png", ":/iconhxt/1.12.png", ":/iconhxt/1.13.png", ":/iconhxt/1.14.png", ":/iconhxt/1.15.png", ":/iconhxt/1.16.png", ":/iconhxt/1.17.png", ":/iconhxt/1.18.png", ":/iconhxt/1.19.png", ":/iconhxt/1.2.png", ":/iconhxt/1.20.png", ":/iconhxt/1.21.png", ":/iconhxt/1.22.png", ":/iconhxt/1.23.png", ":/iconhxt/1.3.png", ":/iconhxt/1.4.png", ":/iconhxt/1.5.png", ":/iconhxt/1.6.png", ":/iconhxt/1.7.png", ":/iconhxt/1.8.png", ":/iconhxt/1.9.png", ":/iconhxt/2,16.png", ":/iconhxt/2.1.png", ":/iconhxt/2.10.png", ":/iconhxt/2.11.png", ":/iconhxt/2.12.png", ":/iconhxt/2.13.png", ":/iconhxt/2.14.png", ":/iconhxt/2.15.png", ":/iconhxt/2.16.png", ":/iconhxt/2.17.png", ":/iconhxt/2.18.png", ":/iconhxt/2.19.png", ":/iconhxt/2.2.png", ":/iconhxt/2.20.png", ":/iconhxt/2.21.png", ":/iconhxt/2.22.png", ":/iconhxt/2.23.png", ":/iconhxt/2.3.png", ":/iconhxt/2.4.png", ":/iconhxt/2.5.png", ":/iconhxt/2.6.png", ":/iconhxt/2.7.png", ":/iconhxt/2.8.png", ":/iconhxt/2.9.png", ":/iconhxt/3.1.png", ":/iconhxt/3.10.png", ":/iconhxt/3.11.png", ":/iconhxt/3.12.png", ":/iconhxt/3.13.png", ":/iconhxt/3.14.png", ":/iconhxt/3.15.png", ":/iconhxt/3.16.png", ":/iconhxt/3.17.png", ":/iconhxt/3.18.png", ":/iconhxt/3.19.png", ":/iconhxt/3.2.png", ":/iconhxt/3.20.png", ":/iconhxt/3.3.png", ":/iconhxt/3.4.png", ":/iconhxt/3.5.png", ":/iconhxt/3.6.png", ":/iconhxt/3.7.png", ":/iconhxt/3.8.png", ":/iconhxt/3.9.png", ":/iconhxt/4.1.png", ":/iconhxt/4.2.png", ":/iconhxt/4.3.png", ":/iconhxt/4.4.png", ":/iconhxt/4.5.png", ":/iconhxt/4.6.png", ":/iconhxt/5.1.png", ":/iconhxt/6.1.png", ":/iconhxt/continue.png", ":/iconhxt/false.png", ":/iconhxt/miss.png", ":/iconhxt/pause.png", ":/iconhxt/plusMinus.png", ":/iconhxt/position.png", ":/iconhxt/recycle.png", ":/iconhxt/right.png", ":/iconhxt/round.png", ":/iconhxt/start.png", ":/iconhxt/stop.png", ":/iconhxt/zhankai.png", ":/iconhxt/zhedie.png"};
    QString iconPaths_Third[ThirdToolBarActions_Num]={":/iconJW/icon1.jpg", ":/iconJW/icon2.jpg", ":/iconJW/icon3.jpg", ":/iconJW/icon4.jpg", ":/iconJW/lefthand.jpeg", ":/iconJW/righthand.jpeg"};
    QString IconNames_First[FirstToolBarActions_Num]={"all", "angle", "arcLength", "arclong", "arclong1", "axes", "chongzhi", "diameter", "dista", "distance", "download", "end", "file", "front", "full", "guide", "half", "isometric", "jia", "jian", "jiaodu", "label", "move", "noxuanzhuan", "pause", "piliang", "point", "point3", "position", "qinkong", "radius", "rectangle", "right", "settings", "setup", "shang", "start", "tole", "tolerance", "up", "x", "x1", "x2", "xia", "xian", "xmove", "xuanzhuan", "y", "y1", "Y2", "you", "yuan", "z1", "Z2", "zhuan", "zoomin", "zoomout", "zuo"};
    QString IconNames_Second[SecondToolBarActions_Num]={"1.1", "1.10", "1.11", "1.12", "1.13", "1.14", "1.15",
     "1.16", "1.17", "1.18", "1.19", "1.2", "1.20", "1.21",
     "1.22", "1.23", "1.3", "1.4", "1.5", "1.6", "1.7", "1.8", "1.9",
    "2,16", "2.1", "2.10", "2.11", "2.12", "2.13", "2.14", "2.15", "2.16", "2.17",
    "2.18", "2.19", "2.2","2.20", "2.21", "2.22","2.23", "2.3", "2.4", "2.5", "2.6", "2.7", "2.8", "2.9",
     "3.1", "3.10", "3.11", "3.12", "3.13", "3.14", "3.15", "3.16",
     "3.17", "3.18", "3.19", "3.20", "3.3", "3.4", "3.5", "3.6",
     "3.7","3.8","3.9", "4.1", "4.2", "4.3", "4.4", "4.5", "4.6", "5.1", "6.1",
     "continue", "false", "miss", "pause", "plusMinus",
     "position", "recycle", "right", "round", "start",
    "stop", "zhankai", "zhedie"};
    QString IconNames_Third[ThirdToolBarActions_Num]={"icon1","icon2","icon3","icon4","lefthamd","righthand"};
    for(int i=0;i<FirstToolBarActions_Num;i++)iconNames_First[i]=IconNames_First[i];
    for(int i=0;i<SecondToolBarActions_Num;i++)iconNames_Second[i]=IconNames_Second[i];
    for(int i=0;i<ThirdToolBarActions_Num;i++)iconNames_Third[i]=iconNames_Third[i];
    QString dirPaths[3]={":/icon",":/iconhxt",":/iconJW"};

    //初始化toolBars
    int allActionNum=FirstToolBarActions_Num+SecondToolBarActions_Num+ThirdToolBarActions_Num;
    toolBars=new QToolBar*[allActionNum/SingalToolBarActionNum+3];
    int toolBarCount=0;
    int toolBarIndex=-1;
    for(int j=0;j<3;j++){
        // QWidget *toolBarWidget = new QWidget(toolBars[j]);
        // QGridLayout* gridLayout=new QGridLayout(toolBarWidget);
        // gridLayout->setColumnStretch(12,0);
        // toolBarWidget->setLayout(gridLayout);

        QString* iconPaths,*iconNames;
        int Num=0;
        if(j==0){
            iconPaths=iconPaths_First;
            iconNames=iconNames_First;
            Num=FirstToolBarActions_Num;
        }else if(j==1){
            iconPaths=iconPaths_Second;
            iconNames=iconNames_Second;
            Num=SecondToolBarActions_Num;
        }
        else if(j==2){
            iconPaths=iconPaths_Third;
            iconNames=iconNames_Third;
            Num=ThirdToolBarActions_Num;
        }
        if(j==0)FirstToolBarActions=new ToolAction*[Num+1];
        else  if(j==1)SecondToolBarActions=new ToolAction*[Num+1];
        else  if(j==2)ThirdToolBarActions=new ToolAction*[Num+1];
        int actionCount=0;
        for(int i=0;i<Num;i++){
            if(actionCount%SingalToolBarActionNum==0){
                toolBarCount++;
                toolBarIndex++;
                toolBars[toolBarIndex] = new QToolBar(ToolBarTitle[j], this);
                toolBars[toolBarIndex]->setMovable(true);
                toolBars[toolBarIndex]->setFloatable(true);
                toolBars[toolBarIndex]->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
                toolBars[toolBarIndex]->setIconSize(QSize( ActionSize_Length, ActionSize_Width));
                layout()->addWidget(toolBars[toolBarIndex]);
            }
            ToolAction * action=new ToolAction(actionKinds[j],this,iconPaths[i],toolBarIndex,iconNames[i]);
            action->setToolTip(iconNames[i]);

            // toolBars[j]->addAction(action);
            if(j==0){
                FirstToolBarActions[i]=action;
            }
            else if(j==1){
                SecondToolBarActions[i]=action;
            }
            else if(j==2){
                ThirdToolBarActions[i]=action;
            }
        //更新actionCount
            actionCount++;
         toolBars[toolBarIndex]->addAction(action);
        }
    }
    // QString qs[4]={"Cube","Cuboid","Sphere","Cone"};
    // QString iconPath[16]={":/icon/3.png",":/icon/1.png",":/icon/2.png",":/icon/4.png"
    //     , ":/icon/5.png",":/icon/all.png",":/icon/angle.png",":/icon/arcLength.png",":/icon/arclong.png"
    //      ,":/icon/arclong1.png",":/icon/axes.png",":/icon/chongzhi.png",":/icon/diameter.png",":/icon/dista.png"
    //     ,":/icon/distance.png",":/icon/end.png"};
    // QString firstIconPath[10]={":/icon/all.png"};
    // for (int i = 0; i < 4; ++i) {
    //     toolBars[i] = new QToolBar(qs[i], this);
    //     layout()->addWidget(toolBars[i]);

    //     // 添加 QAction 到工具栏
    //     for (int j = 0;  j < 4; ++j) {
    //         ToolAction *action=new ToolAction(actionKinds[i],this,iconPath[i*4+j],i,qs[i]+tr("%1").arg(j+1));
    //         if(i==0)cubeActions[j]=action;
    //         else if(i==1)cuboidActions[j]=action;
    //         else if(i==2)sphereActions[j]=action;
    //         else coneActions[j]=action;
    //         action->setText(qs[i]+tr("%1").arg(j+1));
    //         action->setToolTip(qs[i]+tr("%1").arg(j+1));
    //         /*QAction *action = new QAction(tr("Action %1-%2").arg(i + 1).arg(j + 1), this);*/
    //         toolBars[i]->addAction(action);
    //     }
    // }
}
void ToolWidget::clear(){
    deleteToolActions(FirstToolBarAction);
    deleteToolActions(SecondToolBarAction);
    deleteToolActions(ThirdToolBarAction);
}
void ToolWidget::recreateToolBars(ToolActionKind* ToolActionKinds,int Num){
    clear();
    for(int i=0;i<Num;i++){
        addToolActions(ToolActionKinds[i]);
    }
}
void ToolWidget::addToolActions(ToolActionKind actionKind){
    if(actionKind==FirstToolBarAction){
        for(int i=0;i<FirstToolBarActions_Num;i++){
            int addLine=FirstToolBarActions[i]->addLine;
            toolBars[addLine]->addAction(FirstToolBarActions[i]);
        }
    }else if(actionKind==SecondToolBarAction){
        for(int i=0;i<SecondToolBarActions_Num;i++){
            int addLine=SecondToolBarActions[i]->addLine;
            toolBars[addLine]->addAction(SecondToolBarActions[i]);
        }
    }
    else if(actionKind==ThirdToolBarAction){
        for(int i=0;i<ThirdToolBarActions_Num;i++){
            int addLine=ThirdToolBarActions[i]->addLine;
            toolBars[addLine]->addAction(ThirdToolBarActions[i]);
        }
    }/*else if(actionKind==FourthToolBarAction){
        for(int i=0;i<eachToolActionNum;i++){
            int addLine=FourthToolBarActions[i]->addLine;
            toolBars[addLine]->addAction(FourthToolBarActions[i]);
        }
    }*/
}
void ToolWidget::deleteToolActions(ToolActionKind actionKind){
    if(actionKind==FirstToolBarAction){
        for(int i=0;i<FirstToolBarActions_Num;i++){
            int addLine=FirstToolBarActions[i]->addLine;
            toolBars[addLine]->removeAction(FirstToolBarActions[i]);
        }
    }else if(actionKind==SecondToolBarAction){
        for(int i=0;i<SecondToolBarActions_Num;i++){
            int addLine=SecondToolBarActions[i]->addLine;
            toolBars[addLine]->removeAction(SecondToolBarActions[i]);
        }
    }
    else if(actionKind==ThirdToolBarAction){
        for(int i=0;i<ThirdToolBarActions_Num;i++){
            int addLine=ThirdToolBarActions[i]->addLine;
            toolBars[addLine]->removeAction(ThirdToolBarActions[i]);
        }
     }//else if(actionKind==FourthToolBarAction){
    //     for(int i=0;i<eachToolActionNum;i++){
    //         int addLine=FourthToolBarActions[i]->addLine;
    //         toolBars[addLine]->removeAction(FourthToolBarActions[i]);
    //     }
    // }
}
void ToolWidget::addToolAction(ToolActionKind actionKind,QString name){
    if(actionKind==FirstToolBarAction){
        for(int i=0;i<FirstToolBarActions_Num;i++){
            if(FirstToolBarActions[i]->name==name){
                int addLine=FirstToolBarActions[i]->addLine;
                toolBars[addLine]->addAction(FirstToolBarActions[i]);
                break;
            }
        }
    }
    else if(actionKind==SecondToolBarAction){
        for(int i=0;i<SecondToolBarActions_Num;i++){
            if(SecondToolBarActions[i]->name==name){
                int addLine=SecondToolBarActions[i]->addLine;
                toolBars[addLine]->addAction(SecondToolBarActions[i]);
                break;
            }
        }
    }
    else if(actionKind==ThirdToolBarAction){
        for(int i=0;i<ThirdToolBarActions_Num;i++){
            if(ThirdToolBarActions[i]->name==name){
                int addLine=ThirdToolBarActions[i]->addLine;
                toolBars[addLine]->addAction(ThirdToolBarActions[i]);
                break;
            }
        }
    }
    // else if(actionKind==FourthToolBarAction){

    //     for(int i=0;i<eachToolActionNum;i++){
    //         if(FourthToolBarActions[i]->name==name){
    //             int addLine=coneActions[i]->addLine;
    //             toolBars[addLine]->addAction(FourthToolBarActions[i]);
    //             break;
    //         }
    //     }
    // }
}
void ToolWidget::deleteToolAction(ToolActionKind actionKind,QString name){
    if(actionKind==FirstToolBarAction){
        for(int i=0;i<FirstToolBarActions_Num;i++){
            if(FirstToolBarActions[i]->name==name){
                int addLine=FirstToolBarActions[i]->addLine;
                toolBars[addLine]->removeAction(FirstToolBarActions[i]);
                break;
            }
        }
    }
    else if(actionKind==SecondToolBarAction){
        for(int i=0;i<SecondToolBarActions_Num;i++){
            if(SecondToolBarActions[i]->name==name){
                int addLine=SecondToolBarActions[i]->addLine;
                toolBars[addLine]->removeAction(SecondToolBarActions[i]);
                break;
            }
        }
    }
    else if(actionKind==ThirdToolBarAction){
        for(int i=0;i<ThirdToolBarActions_Num;i++){
            if(ThirdToolBarActions[i]->name==name){
                int addLine=ThirdToolBarActions[i]->addLine;
                toolBars[addLine]->removeAction(ThirdToolBarActions[i]);
                break;
            }
        }
    }
    // else if(actionKind==FourthToolBarAction){

    //     for(int i=0;i<eachToolActionNum;i++){
    //         if(FourthToolBarActions[i]->name==name){
    //             int addLine=coneActions[i]->addLine;
    //             toolBars[addLine]->removeAction(FourthToolBarActions[i]);
    //             break;
    //         }
    //     }
    // }
}
ToolWidget::~ToolWidget(){
    if(FirstToolBarActions)delete FirstToolBarActions;
    if(SecondToolBarActions)delete SecondToolBarActions;
    if(ThirdToolBarActions)delete ThirdToolBarActions;
    if(FourthToolBarActions)delete FourthToolBarActions;
}
