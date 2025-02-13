#include <iostream>
#include <xv-sdk.h> // Stereo PRO SDK 头文件
#include "fps_count.hpp"
#include <opencv2/opencv.hpp> // OpenCV 头文件，用于图像处理和显示

// 回调函数，用于处理 RGB 图像数据
void rgbCallback(const xv::ColorImage &rgb)
{
    static int frameCount = 0;
    static FpsCount fc;
    fc.tic();
    // 打印调试信息
    std::cout << "RGB Frame: " << rgb.width << "x" << rgb.height << " @ " << std::round(fc.fps()) << "fps" << std::endl;

    // 将 RGB 图像数据转换为 OpenCV 的 Mat 格式
    cv::Mat image(rgb.height, rgb.width, CV_8UC3, const_cast<uint8_t *>(rgb.data.get()));
    cv::imshow("RGB Camera", image);

    cv::waitKey(3); // 等待 1ms，用于刷新图像显示
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

        if (!device->colorCamera())
        {
            std::cerr << "No RGB camera found on the device!" << std::endl;
            return EXIT_FAILURE;
        }

        // 设置 RGB 相机的分辨率（可选）
        device->colorCamera()->setResolution(xv::ColorCamera::Resolution::RGB_1920x1080);

        // 注册 RGB 图像回调函数
        device->colorCamera()->registerCallback(rgbCallback);

        // 启动 RGB 相机
        device->colorCamera()->start();

        std::cout << "RGB camera started. Press 'q' to quit." << std::endl;

        // 创建窗口并指定位置
        cv::namedWindow("RGB Camera", cv::WINDOW_NORMAL);
        cv::moveWindow("RGB Camera", 100, 100);

        // 主循环，等待用户退出
        while (true)
        {
            if (cv::waitKey(1) == 'q')
            { // 按下 'q' 键退出
                break;
            }
        }

        // 停止 RGB 相机
        device->colorCamera()->stop();
        std::cout << "RGB camera stopped." << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception occurred: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}