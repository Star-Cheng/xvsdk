#include <iostream>
#include <opencv2/opencv.hpp>
#include <xv-sdk.h>
#include "colors.h"  // 假设colors.h中定义了颜色映射表`colors`

// 定义ToF深度图像转换为OpenCV格式的函数
cv::Mat raw_to_opencv(std::shared_ptr<const xv::DepthImage> tof)
{
    cv::Mat out;
    if (tof->height > 0 && tof->width > 0) {
        out = cv::Mat::zeros(tof->height, tof->width, CV_8UC3);
        if (tof->type == xv::DepthImage::Type::Depth_32) {
            float dmax = 7.5;  // 最大深度范围（单位：米）
            const auto tmp_d = reinterpret_cast<float const*>(tof->data.get());
            for (unsigned int i = 0; i < tof->height * tof->width; i++) {
                const auto &d = tmp_d[i];
                if (d < 0.01 || d > 9.9) {
                    out.at<cv::Vec3b>(i / tof->width, i % tof->width) = 0;  // 无效深度值
                } else {
                    unsigned int u = static_cast<unsigned int>(std::max(0.0f, std::min(255.0f, d * 255.0f / dmax)));
                    const auto &cc = colors.at(u);  // 使用颜色映射表
                    out.at<cv::Vec3b>(i / tof->width, i % tof->width) = cv::Vec3b(cc.at(2), cc.at(1), cc.at(0));
                }
            }
        } else if (tof->type == xv::DepthImage::Type::Depth_16) {
            float dmax = 2494.0;  // 最大深度范围（单位：毫米）
            const auto tmp_d = reinterpret_cast<int16_t const*>(tof->data.get());
            for (unsigned int i = 0; i < tof->height * tof->width; i++) {
                const auto &d = tmp_d[i];
                unsigned int u = static_cast<unsigned int>(std::max(0.0f, std::min(255.0f, d * 255.0f / dmax)));
                const auto &cc = colors.at(u);  // 使用颜色映射表
                out.at<cv::Vec3b>(i / tof->width, i % tof->width) = cv::Vec3b(cc.at(2), cc.at(1), cc.at(0));
            }
        }
    }
    return out;
}

int main(int argc, char* argv[])
{
    try {
        // 设置日志级别
        xv::setLogLevel(xv::LogLevel::debug);

        // 获取设备
        auto devices = xv::getDevices(3.);  // 超时时间为10秒
        if (devices.empty()) {
            std::cerr << "No device found!" << std::endl;
            return -1;
        }

        auto device = devices.begin()->second;

        // 检查ToF相机是否存在
        if (!device->tofCamera()) {
            std::cerr << "No ToF camera found on the device!" << std::endl;
            return -1;
        }

        // 创建全局变量用于存储ToF图像
        std::shared_ptr<const xv::DepthImage> s_tof = nullptr;
        std::mutex s_mtx_tof;
        
        // 用于计算帧率的变量
        std::chrono::steady_clock::time_point lastFrameTime = std::chrono::steady_clock::now();
        int frameCount = 0;
        double fps = 0.0;
        // 注册ToF图像回调函数
        device->tofCamera()->registerCallback([&](const xv::DepthImage& tof) {
            std::lock_guard<std::mutex> lock(s_mtx_tof);
            s_tof = std::make_shared<xv::DepthImage>(tof);
                        // 打印分辨率（仅在第一次打印）
            static bool resolutionPrinted = false;
            if (!resolutionPrinted) {
                std::cout << "Resolution: " << tof.width << "x" << tof.height << std::endl;
                resolutionPrinted = true;
            }

            // 计算帧率
            auto currentTime = std::chrono::steady_clock::now();
            ++frameCount;
            auto duration = std::chrono::duration_cast<std::chrono::seconds>(currentTime - lastFrameTime).count();
            if (duration >= 1) {
                fps = static_cast<double>(frameCount) / duration;
                std::cout << "FPS: " << fps << std::endl;
                frameCount = 0;
                lastFrameTime = currentTime;
            }
        });

        // 启动ToF相机
        device->tofCamera()->start();

        // 创建OpenCV窗口
        cv::namedWindow("ToF Depth Camera", cv::WINDOW_AUTOSIZE);

        std::cout << "Press 'q' to exit." << std::endl;

        // 主循环：显示ToF深度图像
        while (true) {
            std::shared_ptr<const xv::DepthImage> tof;
            {
                std::lock_guard<std::mutex> lock(s_mtx_tof);
                tof = s_tof;
            }

            if (tof) {
                cv::Mat img = raw_to_opencv(tof);
                cv::imshow("ToF Depth Camera", img);
            }

            if (cv::waitKey(1) == 'q') {
                break;
            }
        }

        // 停止ToF相机
        device->tofCamera()->stop();

        cv::destroyAllWindows();
    }
    catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}