#include <iostream>
#include <fstream>
#include <chrono>
#include <xv-sdk.h>
#include "fps_count.hpp"
#include <opencv2/opencv.hpp>

// 全局变量用于帧计数和文件存储路径
static int frameCount = 0;
const std::string SAVE_DIR = "./tof_data/"; // 存储路径

void savePointCloud(const std::vector<xv::Vector3f>& points, const std::string& filename) {
    std::ofstream plyFile(filename);
    if (!plyFile.is_open()) {
        std::cerr << "Failed to create PLY file: " << filename << std::endl;
        return;
    }

    plyFile << "ply\n"
            << "format ascii 1.0\n"
            << "element vertex " << points.size() << "\n"
            << "property float x\n"
            << "property float y\n"
            << "property float z\n"
            << "end_header\n";

    for (const auto& point : points) {
        plyFile << point[0] << " " << point[1] << " " << point[2] << "\n";
    }

    plyFile.close();
}

int main(int argc, char* argv[]) {
    try {
        // 创建存储目录
        system(("mkdir -p " + SAVE_DIR).c_str());

        xv::setLogLevel(xv::LogLevel::info);
        std::cout << "Stereo PRO SDK version: " << xv::version() << std::endl;

        auto devices = xv::getDevices(10.);
        if (devices.empty()) {
            std::cerr << "No device found!" << std::endl;
            return EXIT_FAILURE;
        }

        auto device = devices.begin()->second;
        std::cout << "Device found." << std::endl;

        if (!device->tofCamera()) {
            std::cerr << "No TOF camera found!" << std::endl;
            return EXIT_FAILURE;
        }

        // 注册TOF回调（使用lambda捕获device）
        device->tofCamera()->registerCallback([&device](const xv::DepthImage& tof) {
            static FpsCount fc;
            fc.tic();

            // 保存深度图像
            if (tof.type == xv::DepthImage::Type::Depth_16) {
                cv::Mat depthImage16U(tof.height, tof.width, CV_16UC1, 
                    const_cast<uint16_t*>(reinterpret_cast<const uint16_t*>(tof.data.get())));
                
                // 生成唯一文件名
                std::string imgPath = SAVE_DIR + "depth_" + std::to_string(frameCount) + ".png";
                cv::imwrite(imgPath, depthImage16U);

                // 保存点云
                auto points = device->tofCamera()->depthImageToPointCloud(tof)->points;
                std::string plyPath = SAVE_DIR + "pointcloud_" + std::to_string(frameCount) + ".ply";
                savePointCloud(points, plyPath);

                frameCount++;
            }
        });

        // 启动TOF相机
        device->tofCamera()->start();
        std::cout << "TOF camera started. Press 'q' to quit." << std::endl;

        cv::namedWindow("TOF Depth", cv::WINDOW_NORMAL);
        cv::moveWindow("TOF Depth", 100, 100);

        // 主循环
        while (true) {
            if (cv::waitKey(1) == 'q') break;
        }

        device->tofCamera()->stop();
        std::cout << "TOF camera stopped. Data saved to: " << SAVE_DIR << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}