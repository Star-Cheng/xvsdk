#include <iostream>
#include <xv-sdk.h> // Stereo PRO SDK 头文件
#include "fps_count.hpp"
#include <opencv2/opencv.hpp> // OpenCV 头文件，用于图像处理和显示
#include <fstream>
#include <vector>

// 保存点云数据到文件，格式为 XYZ
void savePointCloud(const std::vector<std::array<float, 3>>& points, const std::string& filename) {
    std::ofstream file(filename);
    for (const auto& point : points) {
        file << point[0] << " " << point[1] << " " << point[2] << std::endl;
    }
    file.close();
}

// 将 TOF 深度图像转换为点云数据
std::vector<std::array<float, 3>> tofToPointCloud(const xv::DepthImage& tof) {
    std::vector<std::array<float, 3>> points;

    for (int y = 0; y < tof.height; ++y) {
        for (int x = 0; x < tof.width; ++x) {
            uint16_t depth = reinterpret_cast<const uint16_t*>(tof.data.get())[y * tof.width + x];
            
            if (depth > 0) {
                // 使用相机的内参将深度值转换为 3D 点
                float z = depth * 0.001f;  // 将深度从毫米转换为米
                float x_3d = (x - tof.width / 2) * z / 500.0f;  // 假设 500px 焦距
                float y_3d = (y - tof.height / 2) * z / 500.0f;

                points.push_back({x_3d, y_3d, z});
            }
        }
    }

    return points;
}

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

        // 归一化深度图像以便于显示
        cv::Mat depthImage8U;
        cv::normalize(depthImage, depthImage8U, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        cv::applyColorMap(depthImage8U, depthImage8U, cv::COLORMAP_JET);

        // 显示深度图像
        cv::imshow("TOF Depth Camera", depthImage8U);
        
        // 保存深度图像
        std::string imageFilename = "depth_image_" + std::to_string(frameCount) + ".png";
        cv::imwrite(imageFilename, depthImage8U);
        std::cout << "保存深度图像: " << imageFilename << std::endl;

        // 每10帧保存一次点云数据
        if (frameCount % 10 == 0) {
            auto points = tofToPointCloud(tof);
            std::string pointCloudFilename = "point_cloud_" + std::to_string(frameCount) + ".xyz";
            savePointCloud(points, pointCloudFilename);
            std::cout << "保存点云数据: " << pointCloudFilename << std::endl;
        }

        frameCount++;
    } else {
        std::cerr << "不支持的 TOF 图像类型!" << std::endl;
    }

    cv::waitKey(1); // 等待 1ms，用于刷新图像显示
}

int main(int argc, char* argv[]) {
    try {
        // 初始化 Stereo PRO SDK
        xv::setLogLevel(xv::LogLevel::info); // 设置日志级别
        std::cout << "Stereo PRO SDK 版本: " << xv::version() << std::endl;

        // 获取设备列表
        auto devices = xv::getDevices(10.); // 超时时间 10 秒
        if (devices.empty()) {
            std::cerr << "未找到设备!" << std::endl;
            return EXIT_FAILURE;
        }

        auto device = devices.begin()->second;
        std::cout << "找到设备." << std::endl;

        if (!device->tofCamera()) {
            std::cerr << "设备上未找到 TOF 相机!" << std::endl;
            return EXIT_FAILURE;
        }

        // 设置 TOF 相机的模式（可选）
        device->tofCamera()->setLibWorkMode(xv::TofCamera::SonyTofLibMode::LABELIZE_DF);

        // 注册 TOF 深度图像回调函数
        device->tofCamera()->registerCallback(tofCallback);

        // 启动 TOF 相机
        device->tofCamera()->start();

        std::cout << "TOF 相机已启动，按 'q' 键退出。" << std::endl;

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
        std::cout << "TOF 相机已停止。" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "发生异常: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
