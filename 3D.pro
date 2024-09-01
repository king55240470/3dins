QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17
# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0
SOURCES += \
    component/contralwidget.cpp \
    component/datawidget.cpp \
    component/elementlistwidget.cpp \
    component/filemanagerwidget.cpp \
    component/logwidget.cpp \
    component/reportwidget.cpp \
    component/toolaction.cpp \
    component/toolwidget.cpp \
    component/unknownwidget.cpp \
    component/vtkwidget.cpp \
    geometry/cobject.cpp \
    geometry/cshape.cpp \
    main.cpp \
    mainwindow.cpp \
    manager/cobjectmgr.cpp

HEADERS += \
    component/contralwidget.h \
    component/datawidget.h \
    component/elementlistwidget.h \
    component/filemanagerwidget.h \
    component/logwidget.h \
    component/reportwidget.h \
    component/toolaction.h \
    component/toolwidget.h \
    component/unknownwidget.h \
    component/vtkwidget.h \
    geometry/cobject.h \
    geometry/cshape.h \
    mainwindow.h \
    manager/cobjectmgr.h

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

RESOURCES += \
    toolwidget.qrc
