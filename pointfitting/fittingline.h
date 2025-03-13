#ifndef FITTINGLINE_H
#define FITTINGLINE_H

#include <pcl/point_cloud.h>     // PCL 的点云类
#include <pcl/visualization/pcl_visualizer.h> // PCL 的可视化工具
#include <QVector>
#include <mutex>

class FittingLine
{
public:
    FittingLine();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr RANSAC(pcl::PointXYZRGB,pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
    double euclideanDistance(const pcl::PointXYZRGB&, const pcl::PointXYZRGB&);
    double calculateCurvature(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&, int, double);
    bool isPointInLine(const pcl::PointXYZRGB&);
    void setRadius(double);
    void setDistance(double);
    double getDistance();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getLineCloud();
    Eigen::Vector3f getBegin();
    Eigen::Vector3f getEnd();
private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_subset;//存储searchPoint邻域内的点
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud;//存出searchPoint周围并且曲率大于点云曲率的点
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr lineCloud;//存储拟合平面上的点
    pcl::ModelCoefficients::Ptr coefficients;//存储拟合平面方程的4个系数
    pcl::PointIndices::Ptr inliers;//存储内点
    //double radius;//邻域
    double distance;//距离阈值

    Eigen::Vector3f linePoint;//存储searchPoint

    Eigen::Vector3f normal;//存储直线的法向量
    Eigen::Vector3f onePoint;//存储直线上一点
    Eigen::Vector3f beginPoint;
    Eigen::Vector3f endPoint;

    QVector<double> curvatures;
    QVector<int> indexs;
    double averageCurvature=0.0;//点云平均曲率

    std::mutex mtx;  // 添加 mutex 成员变量
};


#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <atomic>

// 线程池类
class ThreadPool {
public:
    ThreadPool(size_t numThreads) : stop(false) {
        for (size_t i = 0; i < numThreads; ++i) {
            workers.emplace_back([this] {
                while (true) {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock(this->queueMutex);
                        this->condition.wait(lock, [this] { return this->stop ||!this->tasks.empty(); });
                        if (this->stop && this->tasks.empty())
                            return;
                        task = std::move(this->tasks.front());
                        this->tasks.pop();
                    }
                    task();
                }
            });
        }
    }

    template<class F>
    void enqueue(F&& task) {
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            if (stop)
                throw std::runtime_error("enqueue on stopped ThreadPool");
            tasks.emplace(std::forward<F>(task));
        }
        condition.notify_one();
    }

    ~ThreadPool() {
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            stop = true;
        }
        condition.notify_all();
        for (std::thread& worker : workers) {
            worker.join();
        }
    }

private:
    std::vector<std::thread> workers;
    std::queue<std::function<void()>> tasks;
    std::mutex queueMutex;
    std::condition_variable condition;
    std::atomic<bool> stop;
};

#endif // FITTINGLINE_H
