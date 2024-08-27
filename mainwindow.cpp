#include "mainwindow.h"
#include "component/datawidget.h"
#include "component/elementlistwidget.h"
#include "component/filemanagerwidget.h"
#include "component/toolwidget.h"
#include "component/unknownwidget.h"
#include "component/vtkwindowreportwidget.h"

#include <QSettings>
#include <QLabel>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    LoadWidgets();
    setupUi();
    RestoreWidgets();
}

void MainWindow::setupUi(){
    //菜单栏
    bar=menuBar();
    setMenuBar(bar);
    QMenu *fileMenu=bar->addMenu("文件");

    //状态栏
    stbar=statusBar();
    setStatusBar(stbar);
    QLabel *label1=new QLabel("左侧状态栏",this);
    stbar->addWidget(label1);
    QLabel *label2=new QLabel("右侧状态栏",this);
    stbar->addPermanentWidget(label2);

    spMainWindow=new QSplitter(Qt::Horizontal,this);
    spLeft=new QSplitter(Qt::Vertical,spMainWindow);
    spMiddleup=new QSplitter(Qt::Vertical,spMainWindow);
    spRightup=new QSplitter(Qt::Vertical,spMainWindow);

    spLeft->addWidget(pWinElementListWidget);
    spMiddleup->addWidget(pWinVtkWindowReportWidget);
    spRightup->addWidget(pWinFileManagerWidget);

    spMiddledown=new QSplitter(Qt::Horizontal,spMiddleup);
    spMiddleup->addWidget(spMiddledown);
    spMiddledown->addWidget(pWinDataWidget);
    spMiddledown->addWidget(pWinToolWidget);

    spRightdown=new QSplitter(Qt::Horizontal,spRightup);
    spRightup->addWidget(spRightdown);
    spRightdown->addWidget(pWinUnknownWidget);

    spMainWindow->addWidget(spLeft);
    spMainWindow->addWidget(spMiddleup);
    spMainWindow->addWidget(spRightup);

    spMainWindow->setStyleSheet("QSplitter::handle{background-color:#cccccc}");
    spMainWindow->setHandleWidth(5);

    setCentralWidget(spMainWindow);
}

void MainWindow::LoadWidgets(){
    pWinDataWidget=new DataWidget(this);
    pWinElementListWidget=new ElementListWidget(this);
    pWinFileManagerWidget=new FileManagerWidget(this);
    pWinToolWidget=new ToolWidget(this);
    pWinUnknownWidget=new UnknownWidget(this);
    pWinVtkWindowReportWidget=new VtkWindowReportWidget(this);
}

void MainWindow::RestoreWidgets() {
    QSettings settings("3D", "3D");
    QByteArray mainSplitterState = settings.value("spMainWindow/state").toByteArray();
    QByteArray leftSplitterState = settings.value("spLeft/state").toByteArray();
    QByteArray middleupSplitterState = settings.value("spMiddleup/state").toByteArray();
    QByteArray rightupSplitterState = settings.value("spRightup/state").toByteArray();
    QByteArray middledownSplitterState = settings.value("spMiddledown/state").toByteArray();
    QByteArray rightdownSplitterState = settings.value("spRightdown/state").toByteArray();

    spMainWindow->restoreState(mainSplitterState);
    spLeft->restoreState(leftSplitterState);
    spMiddleup->restoreState(middleupSplitterState);
    spRightup->restoreState(rightupSplitterState);
    spMiddledown->restoreState(middledownSplitterState);
    spRightdown->restoreState(rightdownSplitterState);
}

MainWindow::~MainWindow() {
    QSettings settings("3D", "3D");
    settings.setValue("spMainWindow/state", spMainWindow->saveState());
    settings.setValue("spLeft/state", spLeft->saveState());
    settings.setValue("spMiddleup/state", spMiddleup->saveState());
    settings.setValue("spRightup/state", spRightup->saveState());
    settings.setValue("spMiddledown/state", spMiddledown->saveState());
    settings.setValue("spRightdown/state", spRightdown->saveState());
}

