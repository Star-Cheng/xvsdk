#include <iostream>
#include <xv-sdk.h> // Stereo PRO SDK 头文件
#include "fps_count.hpp"
#include <opencv2/opencv.hpp> // OpenCV 头文件，用于图像处理和显示
#include <fstream> // 用于保存点云数据

// 回调函数，用于处理 TOF 深度图像数据
void tofCallback(const xv::DepthImage& tof) {
    static int frameCount = 0;
    static FpsCount fc;
    fc.tic();

    // 打印调试信息
    std::cout << "TOF Frame: " << tof.width << "x" << tof.height << " @ " << std::round(fc.fps()) << "fps" << std::endl;

    // 将深度图像数据转换为 OpenCV 的 Mat 格式
    if (tof.type == xv::DepthImage::Type::Depth_16) {
        cv::Mat depthImage(tof.height, tof.width, CV_16UC1, const_cast<uint16_t*>(reinterpret_cast<const uint16_t*>(tof.data.get())));
        cv::Mat depthImage8U;
        cv::normalize(depthImage, depthImage8U, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        cv::applyColorMap(depthImage8U, depthImage8U, cv::COLORMAP_JET);
        cv::imshow("TOF Depth Camera", depthImage8U);

        // 保存深度图像为 PNG 文件
        std::string depthImageFilename = "depth_frame_" + std::to_string(frameCount) + ".png";
        cv::imwrite(depthImageFilename, depthImage8U);
        std::cout << "Saved depth image to " << depthImageFilename << std::endl;

        // 生成点云数据并保存为 PLY 文件
        std::string pointCloudFilename = "pointcloud_" + std::to_string(frameCount) + ".ply";
        savePointCloud(depthImage, pointCloudFilename);
        std::cout << "Saved point cloud to " << pointCloudFilename << std::endl;
    } else {
        std::cerr << "Unsupported TOF image type!" << std::endl;
    }

    cv::waitKey(1); // 等待 1ms，用于刷新图像显示
    frameCount++;
}

// 保存点云数据为 PLY 文件
void savePointCloud(const cv::Mat& depthImage, const std::string& filename) {
    std::ofstream file(filename, std::ios::binary);
    file << "ply\n";
    file << "format ascii 1.0\n";
    file << "element vertex " << depthImage.total() << "\n";
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";
    file << "end_header\n";

    // 假设相机内参（需要根据实际情况调整）
    float fx = 500.0f; // 焦距
    float fy = 500.0f;
    float cx = depthImage.cols / 2.0f; // 光心
    float cy = depthImage.rows / 2.0f;

    for (int y = 0; y < depthImage.rows; y++) {
        for (int x = 0; x < depthImage.cols; x++) {
            uint16_t depth = depthImage.at<uint16_t>(y, x);
            if (depth != 0) { // 排除无效深度值
                float z = depth / 1000.0f; // 深度单位转换为米
                float x_world = (x - cx) * z / fx;
                float y_world = (y - cy) * z / fy;
                file << x_world << " " << y_world << " " << z << "\n";
            }
        }
    }
    file.close();
}

int main(int argc, char* argv[]) {
    try {
        // 初始化 Stereo PRO SDK
        xv::setLogLevel(xv::LogLevel::info); // 设置日志级别
        std::cout << "Stereo PRO SDK version: " << xv::version() << std::endl;

        // 获取设备列表
        auto devices = xv::getDevices(10.); // 超时时间 10 秒
        if (devices.empty()) {
            std::cerr << "No device found!" << std::endl;
            return EXIT_FAILURE;
        }

        auto device = devices.begin()->second;
        std::cout << "Device found." << std::endl;

        if (!device->tofCamera()) {
            std::cerr << "No TOF camera found on the device!" << std::endl;
            return EXIT_FAILURE;
        }

        // 设置 TOF 相机的模式（可选）
        device->tofCamera()->setLibWorkMode(xv::TofCamera::SonyTofLibMode::LABELIZE_DF);

        // 注册 TOF 深度图像回调函数
        device->tofCamera()->registerCallback(tofCallback);

        // 启动 TOF 相机
        device->tofCamera()->start();

        std::cout << "TOF camera started. Press 'q' to quit." << std::endl;

        // 创建窗口并指定位置
        cv::namedWindow("TOF Depth Camera", cv::WINDOW_NORMAL);
        cv::moveWindow("TOF Depth Camera", 100, 100);

        // 主循环，等待用户退出
        while (true) {
            if (cv::waitKey(1) == 'q') { // 按下 'q' 键退出
                break;
            }
        }

        // 停止 TOF 相机
        device->tofCamera()->stop();
        std::cout << "TOF camera stopped." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Exception occurred: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}