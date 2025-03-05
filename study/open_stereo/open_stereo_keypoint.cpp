#include <opencv2/opencv.hpp>  // 引入 OpenCV 库，用于图像处理
#include <opencv2/features2d.hpp>  // 引入 OpenCV 特征点检测库
#include <xv-sdk.h>  // 引入 XV SDK，用于与摄像头设备交互
#include <iostream>  // 引入标准输入输出库
#include <mutex>  // 引入互斥量库，用于线程同步
#include <thread>  // 引入线程库
#include <chrono>  // 引入时间处理库

// 全局变量
std::shared_ptr<const xv::FisheyeImages> s_stereo = nullptr;  // 用于保存获取的鱼眼图像
std::mutex s_mtx_stereo;  // 用于保护全局变量 s_stereo 的互斥量
bool s_stop = false;  // 控制显示循环是否停止的标志

// 特征点检测函数
std::vector<cv::KeyPoint> detectFeatures(const cv::Mat &img)
{
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();  // 创建一个 ORB 特征点检测器
    std::vector<cv::KeyPoint> keypoints;  // 存储检测到的特征点
    detector->detect(img, keypoints);  // 检测图像中的特征点
    return keypoints;  // 返回检测到的特征点
}

// 显示图像和特征点
void display()
{
    // 创建两个窗口来显示左图和右图
    cv::namedWindow("Left", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Right", cv::WINDOW_AUTOSIZE);

    while (!s_stop)  // 循环直到收到停止信号
    {
        std::shared_ptr<const xv::FisheyeImages> stereo;
        {
            std::lock_guard<std::mutex> l(s_mtx_stereo);  // 锁住互斥量，保护全局变量 s_stereo
            stereo = s_stereo;
        }

        if (stereo)  // 如果获取到了立体图像
        {
            // 初始化左图和右图，大小与鱼眼图像相同
            cv::Mat left = cv::Mat::zeros(stereo->images[0].height, stereo->images[0].width, CV_8UC1);
            cv::Mat right = cv::Mat::zeros(stereo->images[1].height, stereo->images[1].width, CV_8UC1);

            // 将原始鱼眼图像数据拷贝到左图和右图中
            if (stereo->images[0].data != nullptr)
            {
                std::memcpy(left.data, stereo->images[0].data.get(), static_cast<size_t>(left.rows * left.cols));
            }
            if (stereo->images[1].data != nullptr)
            {
                std::memcpy(right.data, stereo->images[1].data.get(), static_cast<size_t>(right.rows * right.cols));
            }

            // 将灰度图转换为彩色图
            cv::cvtColor(left, left, cv::COLOR_GRAY2BGR);
            cv::cvtColor(right, right, cv::COLOR_GRAY2BGR);

            // 检测左图和右图的特征点
            auto leftKeypoints = detectFeatures(left);
            auto rightKeypoints = detectFeatures(right);

            // 调整特征点的大小
            for (auto &kp : leftKeypoints)
            {
                kp.size = 2.0;  // 设置左图特征点的大小
            }
            for (auto &kp : rightKeypoints)
            {
                kp.size = 2.0;  // 设置右图特征点的大小
            }

            // 在图像上绘制特征点
            cv::drawKeypoints(left, leftKeypoints, left, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            cv::drawKeypoints(right, rightKeypoints, right, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

            // 显示带有特征点的图像
            cv::imshow("Left", left);
            cv::imshow("Right", right);
        }

        // 按下 ESC 键时退出
        if (cv::waitKey(1) == 27)
        { 
            s_stop = true;
        }
    }
}

int main(int argc, char *argv[])
try
{
    // 输出 XV SDK 的版本信息
    std::cout << "xvsdk version: " << xv::version() << std::endl;

    // 设置日志级别为调试
    xv::setLogLevel(xv::LogLevel::debug);

    // 获取设备列表
    auto devices = xv::getDevices(10., "");
    if (devices.empty())  // 如果没有找到设备
    {
        std::cout << "Timeout: no device found\n";
        return EXIT_FAILURE;
    }

    // 选择第一个设备
    auto device = devices.begin()->second;

    // 检查设备是否支持鱼眼摄像头
    if (!device->fisheyeCameras())
    {
        std::cout << "No stereo camera found.\n";
        return EXIT_FAILURE;
    }

    // 注册回调函数，当获取到鱼眼图像时触发
    device->fisheyeCameras()->registerCallback([](const xv::FisheyeImages &stereo)
                                               {
        std::lock_guard<std::mutex> l(s_mtx_stereo);  // 锁住互斥量，防止多线程竞争
        s_stereo = std::make_shared<xv::FisheyeImages>(stereo);  // 更新全局变量 s_stereo
    });

    // 启动鱼眼摄像头捕获
    device->fisheyeCameras()->start();

    // 创建一个线程来显示图像和特征点
    std::thread t(display);

    std::cout << "Press ESC to stop" << std::endl;

    // 等待显示线程完成
    t.join();

    // 停止鱼眼摄像头捕获
    device->fisheyeCameras()->stop();

    return EXIT_SUCCESS;
}
catch (const std::exception &e)
{
    // 捕获并输出异常
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
