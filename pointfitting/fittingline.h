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
    double calculateCurvature(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&, pcl::PointXYZRGB, double);
    bool isPointInLine(const pcl::PointXYZRGB&);
    void setDistance(double);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getLineCloud();
    Eigen::Vector3f getBegin();
    Eigen::Vector3f getEnd();
private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_subset;//存储searchPoint邻域内的点
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud;//存出searchPoint周围并且曲率大于点云曲率的点
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr lineCloud;//存储拟合平面上的点
    pcl::ModelCoefficients::Ptr coefficients;//存储拟合平面方程的4个系数
    pcl::PointIndices::Ptr inliers;//存储内点
    double distance;//距离阈值

    Eigen::Vector3f linePoint;//存储searchPoint

    Eigen::Vector3f normal;//存储直线的法向量
    Eigen::Vector3f onePoint;//存储直线上一点
    Eigen::Vector3f beginPoint;
    Eigen::Vector3f endPoint;

    double curvature;//保存searchPoint的曲率
};


#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <future>

class ThreadPool {
public:
    ThreadPool(size_t numThreads) : stop(false) {
        for (size_t i = 0; i < numThreads; ++i) {
            workers.emplace_back([this] {
                while (true) {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock(this->queueMutex);
                        this->condition.wait(lock, [this] { return this->stop || !this->tasks.empty(); });
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

    ~ThreadPool() {
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            stop = true;
        }
        condition.notify_all();
        for (std::thread &worker : workers) {
            worker.join();
        }
    }

    template<class F, class... Args>
    auto enqueue(F&& f, Args&&... args)
        -> std::future<typename std::result_of<F(Args...)>::type> {
        using return_type = typename std::result_of<F(Args...)>::type;

        auto task = std::make_shared< std::packaged_task<return_type()> >(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...)
            );

        std::future<return_type> res = task->get_future();
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            if (stop)
                throw std::runtime_error("enqueue on stopped ThreadPool");
            tasks.emplace([task]() { (*task)(); });
        }
        condition.notify_one();
        return res;
    }

private:
    std::vector<std::thread> workers;
    std::queue<std::function<void()>> tasks;
    std::mutex queueMutex;
    std::condition_variable condition;
    bool stop;
};

#endif // FITTINGLINE_H
