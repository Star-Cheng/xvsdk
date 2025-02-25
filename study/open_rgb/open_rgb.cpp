#include <iostream>
#include <opencv2/opencv.hpp>
#include <xv-sdk.h>
#include "colors.h" // 假设colors.h中定义了颜色映射表`colors`
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>

// 定义RGB图像转换为OpenCV格式的函数
cv::Mat raw_to_opencv(std::shared_ptr<const xv::ColorImage> rgb)
{
    cv::Mat img;
    switch (rgb->codec)
    {
        case xv::ColorImage::Codec::YUV420p:
        {
            img = cv::Mat::zeros(rgb->height, rgb->width, CV_8UC3);
            auto raw = rgb->data.get();
            auto rawImg = cv::Mat(rgb->height * 3 / 2, rgb->width, CV_8UC1, const_cast<unsigned char *>(raw));
            cv::cvtColor(rawImg, img, cv::COLOR_YUV2BGR_I420);
            break;
        }
        case xv::ColorImage::Codec::YUYV:
        {
            img = cv::Mat::zeros(rgb->height, rgb->width, CV_8UC3);
            auto raw = rgb->data.get();
            auto rawImg = cv::Mat(rgb->height, rgb->width, CV_8UC2, const_cast<unsigned char *>(raw));
            cv::cvtColor(rawImg, img, cv::COLOR_YUV2BGR_YUYV);
            break;
        }
        case xv::ColorImage::Codec::JPEG:
        {
            cv::Mat raw(1, rgb->width * rgb->height, CV_8UC1, const_cast<unsigned char *>(rgb->data.get()));
            img = cv::imdecode(raw, cv::IMREAD_COLOR);
            break;
        }
        default:
            std::cerr << "Unsupported codec for RGB image" << std::endl;
            return cv::Mat();
    }
    return img;
}

// 全局变量
std::shared_ptr<const xv::ColorImage> s_rgb = nullptr;
std::mutex s_mtx_rgb;
std::condition_variable cvFrameUpdated;
bool running = true;

// 用于计算帧率的变量
std::chrono::steady_clock::time_point lastFrameTime;
int frameCount = 0;
double fps = 0.0;

// 显示线程
void displayLoop()
{
    cv::namedWindow("RGB Camera", cv::WINDOW_AUTOSIZE);
    cv::Mat displayFrame;

    while (running)
    {
        {
            std::unique_lock<std::mutex> lock(s_mtx_rgb);
            cvFrameUpdated.wait(lock, [] { return s_rgb != nullptr; });
            displayFrame = raw_to_opencv(s_rgb);
        }

        if (!displayFrame.empty())
        {
            cv::imshow("RGB Camera", displayFrame);
        }

        // 控制帧率
        if (cv::waitKey(60) == 'q')
        {
            running = false;
            break;
        }
    }

    cv::destroyAllWindows();
}

int main(int argc, char *argv[])
{
    try
    {
        // 设置日志级别
        xv::setLogLevel(xv::LogLevel::debug);

        // 获取设备
        auto devices = xv::getDevices(10.); // 超时时间为10秒
        if (devices.empty())
        {
            std::cerr << "No device found!" << std::endl;
            return -1;
        }

        auto device = devices.begin()->second;

        // 检查RGB相机是否存在
        if (!device->colorCamera())
        {
            std::cerr << "No RGB camera found on the device!" << std::endl;
            return -1;
        }

        // 注册RGB图像回调函数
        device->colorCamera()->registerCallback([&](const xv::ColorImage &rgb)
        {
            std::lock_guard<std::mutex> lock(s_mtx_rgb);
            s_rgb = std::make_shared<xv::ColorImage>(rgb);
            cvFrameUpdated.notify_one();  // 通知显示线程更新图像

            // 计算帧率
            auto currentTime = std::chrono::steady_clock::now();
            if (frameCount == 0)
            {
                lastFrameTime = currentTime;
            }
            frameCount++;
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastFrameTime).count();
            if (duration >= 1000) // 从毫秒到秒的转换
            {
                fps = frameCount / (duration / 1000.0);
                std::cout << "FPS: " << fps << ", Resolution: " << rgb.width << "x" << rgb.height << std::endl;
                frameCount = 0;
                lastFrameTime = currentTime;
            }
        });

        // 启动RGB相机
        device->colorCamera()->start();

        // 启动显示线程
        std::thread displayThread(displayLoop);

        std::cout << "Press 'q' to exit." << std::endl;

        // 主循环
        while (running)
        {
            // 这里可以添加其他逻辑
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 控制主循环频率
        }

        // 停止RGB相机
        device->colorCamera()->stop();
        running = false;
        displayThread.join(); // 等待显示线程结束
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}