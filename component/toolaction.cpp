#include "toolaction.h"
ToolAction::ToolAction(QWidget* parent):QAction(parent){
}
void ToolAction::setToolActionKind(ToolActionKind action_kind){
    action_kind_=action_kind;
}
void ToolAction::setName(QString name) {
    name_=name;
}
