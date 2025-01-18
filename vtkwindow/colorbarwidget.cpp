#include "ColorBarWidget.h" // 假设ColorBarWidget.h是上述代码的头文件

// 在VtkWidget类中
class VtkWidget : public QWidget {
    Q_OBJECT

public:
    VtkWidget(QWidget *parent = nullptr) : QWidget(parent) {
        // 初始化其他成员...

        // 创建颜色条
        colorBar = new ColorBarWidget(this);
        colorBar->setFixedSize(20, 200); // 设置颜色条的大小
        colorBar->move(10, 10); // 设置颜色条的位置

        // 其他初始化...
    }



private:
    ColorBarWidget *colorBar;
    // 其他成员变量...
};
