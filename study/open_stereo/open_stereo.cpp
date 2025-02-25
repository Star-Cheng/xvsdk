#include <iostream>
#include <xv-sdk.h> // Stereo PRO SDK 头文件
#include "fps_count.hpp"
#include <opencv2/opencv.hpp> // OpenCV 头文件，用于图像处理和显示

// 回调函数，用于处理双目相机的左右图像数据
void stereoCallback(const xv::FisheyeImages &stereo)
{
    static int frameCount = 0;
    static FpsCount fc;
    fc.tic();

    // 打印调试信息
    std::cout << "Stereo Frame: " << stereo.images[0].width << "x" << stereo.images[0].height << " @ " << std::round(fc.fps()) << "fps" << std::endl;

    // 将左右图像数据转换为 OpenCV 的 Mat 格式
    cv::Mat left(stereo.images[0].height, stereo.images[0].width, CV_8UC1, const_cast<uint8_t *>(stereo.images[0].data.get()));
    cv::Mat right(stereo.images[1].height, stereo.images[1].width, CV_8UC1, const_cast<uint8_t *>(stereo.images[1].data.get()));

    // 转换为彩色图像以便显示
    cv::cvtColor(left, left, cv::COLOR_GRAY2BGR);
    cv::cvtColor(right, right, cv::COLOR_GRAY2BGR);

    // 显示左右图像
    cv::imshow("Left Camera", left);
    cv::imshow("Right Camera", right);

    cv::waitKey(1); // 等待 1ms，用于刷新图像显示
}

int main(int argc, char *argv[])
{
    try
    {
        // 初始化 Stereo PRO SDK
        xv::setLogLevel(xv::LogLevel::info); // 设置日志级别
        std::cout << "Stereo PRO SDK version: " << xv::version() << std::endl;

        // 获取设备列表
        auto devices = xv::getDevices(10.); // 超时时间 10 秒
        if (devices.empty())
        {
            std::cerr << "No device found!" << std::endl;
            return EXIT_FAILURE;
        }

        auto device = devices.begin()->second;
        std::cout << "Device found." << std::endl;

        if (!device->fisheyeCameras())
        {
            std::cerr << "No Fisheye camera found on the device!" << std::endl;
            return EXIT_FAILURE;
        }

        // 注册双目相机回调函数
        device->fisheyeCameras()->registerCallback(stereoCallback);

        // 启动双目相机
        device->fisheyeCameras()->start();

        std::cout << "Fisheye cameras started. Press 'q' to quit." << std::endl;

        // 创建窗口并指定位置
        cv::namedWindow("Left Camera", cv::WINDOW_NORMAL);
        cv::moveWindow("Left Camera", 100, 100);
        cv::namedWindow("Right Camera", cv::WINDOW_NORMAL);
        cv::moveWindow("Right Camera", 800, 100);

        // 主循环，等待用户退出
        while (true)
        {
            if (cv::waitKey(1) == 'q')
            { // 按下 'q' 键退出
                break;
            }
        }

        // 停止双目相机
        device->fisheyeCameras()->stop();
        std::cout << "Fisheye cameras stopped." << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception occurred: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}