#include <iostream>
#include <opencv2/opencv.hpp>
#include <xv-sdk.h>
#include "fps_count.hpp"

// 回调函数，用于处理 SLAM 的 6DOF 姿态信息
void slamCallback(const xv::Pose& pose) {
    static FpsCount fc; // 假设 FpsCount 是一个用于计算帧率的工具类
    fc.tic();

    // 提取姿态信息
    auto translation = pose.translation();
    auto rotation = pose.rotation();
    auto pitchYawRoll = xv::rotationToPitchYawRoll(rotation);

    // 打印姿态信息
    std::cout << "SLAM Pose: "
              << "Translation: (" << translation[0] << ", " << translation[1] << ", " << translation[2] << "), "
              << "Rotation (Pitch, Yaw, Roll): (" << pitchYawRoll[0] * 180.0 / M_PI << ", "
              << pitchYawRoll[1] * 180.0 / M_PI << ", " << pitchYawRoll[2] * 180.0 / M_PI << "), "
              << "Confidence: " << pose.confidence() << ", "
              << "FPS: " << std::round(fc.fps()) << std::endl;

    // 可视化部分（简单示例）
    cv::Mat img = cv::Mat::zeros(480, 640, CV_8UC3);
    cv::putText(img, "SLAM Pose", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
    cv::putText(img, "Translation: (" + std::to_string(translation[0]) + ", " + std::to_string(translation[1]) + ", " + std::to_string(translation[2]) + ")",
                cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    cv::putText(img, "Rotation (Pitch, Yaw, Roll): (" + std::to_string(pitchYawRoll[0] * 180.0 / M_PI) + ", " + std::to_string(pitchYawRoll[1] * 180.0 / M_PI) + ", " + std::to_string(pitchYawRoll[2] * 180.0 / M_PI) + ")",
                cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    cv::putText(img, "Confidence: " + std::to_string(pose.confidence()), cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    cv::putText(img, "FPS: " + std::to_string(std::round(fc.fps())), cv::Point(10, 150), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

    cv::imshow("SLAM Visualization", img);
    cv::waitKey(1);
}

int main(int argc, char* argv[]) {
    try {
        // 初始化设备
        auto devices = xv::getDevices(10.);
        if (devices.empty()) {
            std::cerr << "No device found." << std::endl;
            return -1;
        }

        auto device = devices.begin()->second;

        // 检查 SLAM 功能是否可用
        if (!device->slam()) {
            std::cerr << "SLAM is not available on this device." << std::endl;
            return -1;
        }

        // 启动 SLAM 功能
        device->slam()->start(xv::Slam::Mode::Mixed); // 使用混合模式

        // 注册 SLAM 回调函数
        device->slam()->registerCallback(slamCallback);

        std::cout << "SLAM is running. Press 'q' to exit." << std::endl;

        // 主循环，等待用户按下 'q' 键退出
        while (true) {
            if (cv::waitKey(1) == 'q') {
                break;
            }
        }

        // 停止 SLAM 功能
        device->slam()->stop();

        std::cout << "SLAM stopped." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Exception occurred: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}