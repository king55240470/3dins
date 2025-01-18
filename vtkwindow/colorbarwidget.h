#include <QWidget>
#include <QPainter>

class ColorBarWidget : public QWidget {
    Q_OBJECT

public:
    ColorBarWidget(QWidget *parent = nullptr) : QWidget(parent), minDistance(0), maxDistance(1) {}

    void setRange(float min, float max) {
        minDistance = min;
        maxDistance = max;
        update(); // 重新绘制
    }

protected:
    void paintEvent(QPaintEvent *event) override {
        QPainter painter(this);
        int width = this->width();
        int height = this->height();

        // 绘制颜色条
        for (int x = 0; x < width; ++x) {
            float normalizedPosition = static_cast<float>(x) / (width - 1);
            float value = minDistance + normalizedPosition * (maxDistance - minDistance);
            int r = static_cast<int>(255 * (value - minDistance) / (maxDistance - minDistance));
            int b = 255 - r;
            QColor color(r, 0, b);
            painter.setPen(color);
            painter.drawLine(x, 0, x, height);
        }
    }

private:
    float minDistance;
    float maxDistance;
};
