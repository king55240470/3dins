#include "toolaction.h"
#include<QVBoxLayout>
#include<QLabel>
ToolAction::ToolAction(QWidget* parent):QAction(parent){
}
void ToolAction::setToolActionKind(ToolActionKind action_kind){
    action_kind_=action_kind;
}
void ToolAction::setName(QString name) {
    name_=name;
}

UniqueToolBar::UniqueToolBar(){
    m_name=0;
    m_parent=nullptr;
}
UniqueToolBar::~UniqueToolBar(){
    clear();
}
void UniqueToolBar::clear(){
    //清除路径和名称列表，释放QAction
    m_nameList.clear();
    m_pathList.clear();
    for(int i=0;i<m_actionList.size();i++){
        if(m_actionList[i]!=nullptr){
            delete m_actionList[i];
        }
    }
}
void UniqueToolBar::setChosenNameList(QStringList chosenNameList){
    //先清空
    m_chosenActionList.clear();
    m_chosenNameList=chosenNameList;
    //一样比对 ，name与action下标相同 可以直接使用
    for(int i=0;i<chosenNameList.size();i++){
        for(int j=0;j<m_nameList.size();j++){
            if(chosenNameList[i]==m_nameList[j]){
                m_chosenActionList.push_back(m_actionList[j]);
                break;
            }
        }
    }
}
bool UniqueToolBar::loadAction(QWidget* parent){
    //从路径下读入QAction 并设置初始值
    //如果路径和名称数目不同，则报错
    if(m_nameList.size()!=m_pathList.size()){
        return false;
    }
    m_num=m_nameList.size();
    for(int i=0;i<m_num;i++){
        ToolAction* action=new ToolAction(parent);
        action->setToolActionKind(m_actionKind);
        action->setName(m_nameList[i]);
        action->setIcon(QIcon(m_pathList[i]));
        action->setToolTip(m_nameList[i]);
        m_actionList.push_back(action);
    }
    m_chosenActionList=m_actionList;
    return true;
}


ToolBarGathter::ToolBarGathter(){
    m_allActionNum=0;
    m_nToolbarNum=0;
    m_singalToolBarActionNum=9;
}
ToolBarGathter::~ToolBarGathter(){
    clear();
}
void ToolBarGathter::addToWidget(QWidget* toolWidget){
    //把所有存储的UniqueToolBar加入新布局layout中，并设置为工具栏窗口布局
    QVBoxLayout *layout = new QVBoxLayout(toolWidget);
    toolWidget->setLayout(layout);
    //清空
    for(int i=0;i<toolBars.size();i++){
        if(toolBars[i]!=nullptr){
            delete toolBars[i];
        }
    }
     //计算需要的工具栏数目
    for(int i=0;i<m_UniqueToolBarList.size();i++){
        m_allActionNum=m_UniqueToolBarList[i]->getNum();
    }
    m_nToolbarNum =(m_allActionNum/ m_singalToolBarActionNum)+5;
    //初始化工具栏
    for(int i=0;i<m_nToolbarNum;i++){
        QToolBar * toolbar=new QToolBar(toolWidget);
        toolbar->setIconSize(QSize(28,28));
        toolbar->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        toolbar->setStyleSheet(
            "QToolButton {"
            "    border: 2px solid lightgray;" // 浅灰色边框
            "    padding: 5px;" // 内部填充
            "}"
            "QToolButton:pressed {"
            "    border: 2px solid gray;" // 按下状态下的灰色边框
            "}"
            "QToolButton:hover {"
            "    border: 2px solid darkgray;" // 悬浮状态下的深灰色边框
            "}"
            "QToolBar::item {"
            "    margin: 10px;" // 工具栏项之间的间隔
            "}"
            "QToolBar::separator {"
            "    width: 3px;" // 设置分隔符的宽度
            "    background: transparent;" // 分隔符背景颜色
            "}"
            );
        toolBars.append(toolbar);
    }
    int index=0;//记录使用的toolbar索引
    int actionCount=0;
    for(int i=0;i<m_UniqueToolBarList.size();i++){
        UniqueToolBar* toolbar=m_UniqueToolBarList[i];
        actionCount=0;
        layout->addWidget(new QLabel(toolbar->getName()));
        for(int j=0;j<toolbar->m_chosenActionList.size();j++){
            toolBars[index]->addAction(toolbar->m_chosenActionList[j]);
            toolBars[index]->addSeparator();
            actionCount++;
            if(actionCount==m_singalToolBarActionNum){
                actionCount=0;
                layout->addWidget(toolBars[index]);
                index++;
            }  
        }
        layout->addWidget(toolBars[index]);
        if(actionCount!=0&&i!=m_UniqueToolBarList.size()-1)index++;
    }
}

void ToolBarGathter::clear(){
    for(int i=0;i<m_UniqueToolBarList.size();i++){
        m_UniqueToolBarList[i]->clear();
    }
    for(int i=0;i<toolBars.size();i++){
        if(toolBars[i]!=nullptr){
            delete toolBars[i];
        }
    }
}
