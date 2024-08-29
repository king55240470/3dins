#include "toolaction.h"

ToolAction::ToolAction(ToolActionKind ActionKind,QWidget* parent,QString iconPath,
    int AddLine,QString Name)
    :QAction(parent) {
    actionKind=ActionKind;
    addLine=AddLine;
    setIcon(QIcon(iconPath));
    setText(Name);
    name=Name;
}
