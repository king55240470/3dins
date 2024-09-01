#include"toolwidget.h"
#include"toolaction.h"
#include <QDir>
#include <QStringList>
#include <QFileInfo>
#include<QGridLayout>
#include <QFile>
#include <QTextStream>
#include <QStringList>
#include <QDebug>
#include<QResource>
ToolWidget::ToolWidget(QWidget *parent)
    : QWidget(parent) {
    // 设置布局
    QVBoxLayout *layout = new QVBoxLayout(this);
    setLayout(layout);
    m_nToolbarNum = 3;
    int allActionNum=FirstToolBarActions_Num+SecondToolBarActions_Num+ThirdToolBarActions_Num;
     toolBars=new QToolBar*[(allActionNum/SingalToolBarActionNum)+3];
     FirstToolBarActions=new ToolAction*[FirstToolBarActions_Num];
     SecondToolBarActions=new ToolAction*[SecondToolBarActions_Num];
     ThirdToolBarActions=new ToolAction*[ThirdToolBarActions_Num];
     iconNames_First=new QString[FirstToolBarActions_Num];
     iconNames_Second=new QString[SecondToolBarActions_Num];
     iconNames_Third=new QString[ThirdToolBarActions_Num];
    createToolBars();//调用createToolBar函数进一步构造

}
QString * ToolWidget::geticonNames_First()const{
    return iconNames_First;
}
QString * ToolWidget::geticonNames_Second()const{
    return iconNames_Second;
}
QString * ToolWidget::geticonNames_Third()const{
    return iconNames_Third;
}

//创建工具栏函数，是内部函数，不对外开放，在调用构造函数时使用
void ToolWidget::createToolBars() {
    ToolActionKind actionKinds[4]={FirstToolBarAction,SecondToolBarAction,
    ThirdToolBarAction,FourthToolBarAction};
    QString ToolBarTitle[4]={"第一栏","第二栏","第三栏","第四栏"};
    //动态只能读取图标的路径，有些图标的名字无法读取，只能存储在静态中
    //暂时无法保存到本地文件中,只能先放在这里了
    QString iconPaths_First[FirstToolBarActions_Num]={":/component/icon/1.png", ":/component/icon/2.png", ":/component/icon/3.png", ":/component/icon/4.png", ":/component/icon/5.png", ":/component/icon/all.png", ":/component/icon/angle.png", ":/component/icon/arcLength.png", ":/component/icon/arclong.png", ":/component/icon/arclong1.png", ":/component/icon/axes.png", ":/component/icon/chongzhi.png", ":/component/icon/diameter.png", ":/component/icon/dista.png", ":/component/icon/distance.png", ":/component/icon/end.png", ":/component/icon/front.png", ":/component/icon/full.png", ":/component/icon/guide.png", ":/component/icon/half.png", ":/component/icon/icon.zip", ":/component/icon/isometric.png", ":/component/icon/jia.png", ":/component/icon/jian.png", ":/component/icon/jiaodu.png", ":/component/icon/label.png", ":/component/icon/noxuanzhuan.png", ":/component/icon/pause.png", ":/component/icon/piliang.png", ":/component/icon/point.png", ":/component/icon/point3.png", ":/component/icon/qinkong.png", ":/component/icon/radius.png", ":/component/icon/rectangle.png", ":/component/icon/right.png", ":/component/icon/setup.png", ":/component/icon/shang.png", ":/component/icon/start.png", ":/component/icon/tole.png", ":/component/icon/tolerance.png", ":/component/icon/up.png", ":/component/icon/x.png", ":/component/icon/x1.png", ":/component/icon/x2.png", ":/component/icon/xia.png", ":/component/icon/xian.png", ":/component/icon/xmove.png", ":/component/icon/xuanzhuan.png", ":/component/icon/y.png", ":/component/icon/y1.png", ":/component/icon/Y2.png", ":/component/icon/you.png", ":/component/icon/yuan.png", ":/component/icon/z1.png", ":/component/icon/Z2.png", ":/component/icon/zhuan.png", ":/component/icon/zoomin.png", ":/component/icon/zoomout.png", ":/component/icon/zuo.png"};
    QString iconPaths_Second[SecondToolBarActions_Num]={":/component/iconhxt/1.1.png", ":/component/iconhxt/1.10.png", ":/component/iconhxt/1.11.png", ":/component/iconhxt/1.12.png", ":/component/iconhxt/1.13.png", ":/component/iconhxt/1.14.png", ":/component/iconhxt/1.15.png", ":/component/iconhxt/1.16.png", ":/component/iconhxt/1.17.png", ":/component/iconhxt/1.18.png", ":/component/iconhxt/1.19.png", ":/component/iconhxt/1.2.png", ":/component/iconhxt/1.20.png", ":/component/iconhxt/1.21.png", ":/component/iconhxt/1.22.png", ":/component/iconhxt/1.23.png", ":/component/iconhxt/1.3.png", ":/component/iconhxt/1.4.png", ":/component/iconhxt/1.5.png", ":/component/iconhxt/1.6.png", ":/component/iconhxt/1.7.png", ":/component/iconhxt/1.8.png", ":/component/iconhxt/1.9.png", ":/component/iconhxt/2,16.png", ":/component/iconhxt/2.1.png", ":/component/iconhxt/2.10.png", ":/component/iconhxt/2.11.png", ":/component/iconhxt/2.12.png", ":/component/iconhxt/2.13.png", ":/component/iconhxt/2.14.png", ":/component/iconhxt/2.15.png", ":/component/iconhxt/2.16.png", ":/component/iconhxt/2.17.png", ":/component/iconhxt/2.18.png", ":/component/iconhxt/2.19.png", ":/component/iconhxt/2.2.png", ":/component/iconhxt/2.20.png", ":/component/iconhxt/2.21.png", ":/component/iconhxt/2.22.png", ":/component/iconhxt/2.23.png", ":/component/iconhxt/2.3.png", ":/component/iconhxt/2.4.png", ":/component/iconhxt/2.5.png", ":/component/iconhxt/2.6.png", ":/component/iconhxt/2.7.png", ":/component/iconhxt/2.8.png", ":/component/iconhxt/2.9.png", ":/component/iconhxt/3.1.png", ":/component/iconhxt/3.10.png", ":/component/iconhxt/3.11.png", ":/component/iconhxt/3.12.png", ":/component/iconhxt/3.13.png", ":/component/iconhxt/3.14.png", ":/component/iconhxt/3.15.png", ":/component/iconhxt/3.16.png", ":/component/iconhxt/3.17.png", ":/component/iconhxt/3.18.png", ":/component/iconhxt/3.19.png", ":/component/iconhxt/3.2.png", ":/component/iconhxt/3.20.png", ":/component/iconhxt/3.3.png", ":/component/iconhxt/3.4.png", ":/component/iconhxt/3.5.png", ":/component/iconhxt/3.6.png", ":/component/iconhxt/3.7.png", ":/component/iconhxt/3.8.png", ":/component/iconhxt/3.9.png", ":/component/iconhxt/4.1.png", ":/component/iconhxt/4.2.png", ":/component/iconhxt/4.3.png", ":/component/iconhxt/4.4.png", ":/component/iconhxt/4.5.png", ":/component/iconhxt/4.6.png", ":/component/iconhxt/5.1.png", ":/component/iconhxt/6.1.png", ":/component/iconhxt/continue.png", ":/component/iconhxt/false.png", ":/component/iconhxt/miss.png", ":/component/iconhxt/pause.png", ":/component/iconhxt/plusMinus.png", ":/component/iconhxt/position.png", ":/component/iconhxt/recycle.png", ":/component/iconhxt/right.png", ":/component/iconhxt/round.png", ":/component/iconhxt/start.png", ":/component/iconhxt/stop.png", ":/component/iconhxt/zhankai.png", ":/component/iconhxt/zhedie.png"};
    QString iconPaths_Third[ThirdToolBarActions_Num]={":/component/iconJW/icon1.jpg", ":/component/iconJW/icon2.jpg", ":/component/iconJW/icon3.jpg", ":/component/iconJW/icon4.jpg", ":/component/iconJW/lefthand.jpeg", ":/component/iconJW/righthand.jpeg"};
    QString IconNames_First[FirstToolBarActions_Num]={"all", "angle", "arcLength", "arclong", "arclong1", "axes", "chongzhi", "diameter", "dista", "distance", "download", "end", "file", "front", "full", "guide", "half", "isometric", "jia", "jian", "jiaodu", "label", "move", "noxuanzhuan", "pause", "piliang", "point", "point3", "position", "qinkong", "radius", "rectangle", "right", "settings", "setup", "shang", "start", "tole", "tolerance", "up", "x", "x1", "x2", "xia", "xian", "xmove", "xuanzhuan", "y", "y1", "Y2", "you", "yuan", "z1", "Z2", "zhuan", "zoomin", "zoomout", "zuo"};
    QString IconNames_Second[SecondToolBarActions_Num]={"1.1", "1.10", "1.11", "1.12", "1.13", "1.14", "1.15","1.16", "1.17", "1.18", "1.19", "1.2", "1.20", "1.21", "1.22", "1.23", "1.3", "1.4", "1.5", "1.6", "1.7", "1.8", "1.9", "2,16", "2.1", "2.10", "2.11", "2.12", "2.13", "2.14", "2.15", "2.16", "2.17","2.18", "2.19", "2.2","2.20", "2.21", "2.22","2.23", "2.3", "2.4", "2.5", "2.6", "2.7", "2.8", "2.9","3.1", "3.10", "3.11", "3.12", "3.13", "3.14", "3.15", "3.16", "3.17", "3.18", "3.19", "3.20", "3.3", "3.4", "3.5", "3.6", "3.7","3.8","3.9", "4.1", "4.2", "4.3", "4.4", "4.5", "4.6", "5.1", "6.1", "continue", "false", "miss", "pause", "plusMinus", "position", "recycle", "right", "round", "start","stop", "zhankai", "zhedie"};
    QString IconNames_Third[ThirdToolBarActions_Num]={"icon1","icon2","icon3","icon4","lefthamd","righthand"};

    QString* iconPaths,*iconNames;//记录当前的图片路径和图片名，用于循环
    int Num=0;//Qaction的图片总数，每栏不同
    int toolBarIndex=-1;//记录toolBar索引
    int actionCount=0;//记录QAaction数目，便于分行
    ToolAction * action;//暂时存储新申请的QAction

    for(int i=0;i<FirstToolBarActions_Num;i++)iconNames_First[i]=IconNames_First[i];
    for(int i=0;i<SecondToolBarActions_Num;i++)iconNames_Second[i]=IconNames_Second[i];
    for(int i=0;i<ThirdToolBarActions_Num;i++)iconNames_Third[i]=IconNames_Third[i];

    for(int j=0;j< m_nToolbarNum;j++){
        if(j==FirstToolBarIndex){//处理第一栏的toolBarAcrion
            iconPaths=iconPaths_First;
            iconNames=iconNames_First;
            Num=FirstToolBarActions_Num;
        }else if(j==SecondToolBarIndex){//处理第二栏的
            iconPaths=iconPaths_Second;
            iconNames=iconNames_Second;
            Num=SecondToolBarActions_Num;
        }
        else if(j==ThirdToolBarIndex){//处理第三栏的
            iconPaths=iconPaths_Third;
            iconNames=iconNames_Third;
            Num=ThirdToolBarActions_Num;
        }
        actionCount=0;
        for(int i=0;i<Num;i++){
            if(actionCount%SingalToolBarActionNum==0){//每行固定图标数
                toolBarIndex++;
                toolBars[toolBarIndex] = new QToolBar(ToolBarTitle[j], this);//动态分配空间
                toolBars[toolBarIndex]->setIconSize(QSize( ActionSize_Length, ActionSize_Width));
                layout()->addWidget(toolBars[toolBarIndex]);
            }
            action=new ToolAction(actionKinds[j],this,iconPaths[i],toolBarIndex,iconNames[i]);
            action->setToolTip(iconNames[i]);
            if(j==FirstToolBarIndex){//把QActin存入数组中，表示总共的QAction（与显示的QAction相对）
                FirstToolBarActions[i]=action;
            }
            else if(j==SecondToolBarIndex){
                SecondToolBarActions[i]=action;
            }
            else if(j==ThirdToolBarIndex){
                ThirdToolBarActions[i]=action;
            }
        //更新actionCount
         actionCount++;
        //把QAction加入工具栏
         toolBars[toolBarIndex]->addAction(action);
        }
    }
}
void ToolWidget::clear(){
    deleteToolActions(FirstToolBarAction);
    deleteToolActions(SecondToolBarAction);
    deleteToolActions(ThirdToolBarAction);
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
     }
}

void ToolWidget::deleteToolAction(ToolActionKind actionKind,QString name){
    int addLine;//QAction将要加入的ToolBar行数
    if(actionKind==FirstToolBarAction){
        for(int i=0;i<FirstToolBarActions_Num;i++){
            if(FirstToolBarActions[i]->name==name){
                addLine=FirstToolBarActions[i]->addLine;
                toolBars[addLine]->removeAction(FirstToolBarActions[i]);
                break;
            }
        }
    }
    else if(actionKind==SecondToolBarAction){
        for(int i=0;i<SecondToolBarActions_Num;i++){
            if(SecondToolBarActions[i]->name==name){
                addLine=SecondToolBarActions[i]->addLine;
                toolBars[addLine]->removeAction(SecondToolBarActions[i]);
                break;
            }
        }
    }
    else if(actionKind==ThirdToolBarAction){
        for(int i=0;i<ThirdToolBarActions_Num;i++){
            if(ThirdToolBarActions[i]->name==name){
                 addLine=ThirdToolBarActions[i]->addLine;
                toolBars[addLine]->removeAction(ThirdToolBarActions[i]);
                break;
            }
        }
    }
}
ToolWidget::~ToolWidget(){
    if(FirstToolBarActions!=nullptr)delete FirstToolBarActions;
    if(SecondToolBarActions!=nullptr)delete SecondToolBarActions;
    if(ThirdToolBarActions!=nullptr)delete ThirdToolBarActions;
}
int  ToolWidget::addToolActions(ToolActionKind actionKind ,QString * ToolActionNames,int actionNum,int lastToolBarIndex){
    int actionCount=0;
    int ToolBarIndex=lastToolBarIndex;
    QString name;//当前检测的QAction的name
    for(int j=0;j<actionNum;j++){
        name=ToolActionNames[j];
        if(actionCount%SingalToolBarActionNum==0)
            ToolBarIndex++;
        actionCount++;
        if(actionKind==FirstToolBarAction){
            for(int i=0;i<FirstToolBarActions_Num;i++){
                if(FirstToolBarActions[i]->name==name){
                    toolBars[ToolBarIndex]->addAction(FirstToolBarActions[i]);
                    FirstToolBarActions[i]->addLine=ToolBarIndex;
                    break;
                }
            }
        }
        else if(actionKind==SecondToolBarAction){
            for(int i=0;i<SecondToolBarActions_Num;i++){
                if(SecondToolBarActions[i]->name==name){
                    toolBars[ToolBarIndex]->addAction(SecondToolBarActions[i]);
                   SecondToolBarActions[i]->addLine=ToolBarIndex;
                    break;
                }
            }
        }
        else if(actionKind==ThirdToolBarAction){
            for(int i=0;i<ThirdToolBarActions_Num;i++){
                if(ThirdToolBarActions[i]->name==name){
                    toolBars[ToolBarIndex]->addAction(ThirdToolBarActions[i]);
                    ThirdToolBarActions[i]->addLine=ToolBarIndex;
                    break;
                }
            }
        }

    }
    return ToolBarIndex;
}
