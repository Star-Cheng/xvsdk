#define _USE_MATH_DEFINES // for C++

#include <xv-sdk.h>
#include <iostream>
#include <fstream>
#include <thread>
#include <atomic>
#include <cmath>
#include "frequency_counter.hpp"
#include <iomanip>

// 文件名用于保存SLAM数据
const std::string SLAM_DATA_FILE = "slam_6dof_data.txt";

// SLAM回调函数，用于获取6自由度信息并保存到文件
void onPose(const xv::Pose& pose) {
    static FrequencyCounter fps;
    fps.tic();

    // 将欧拉角从弧度转换为度
    auto pitchYawRoll = xv::rotationToPitchYawRoll(pose.rotation());
    double pitch = pitchYawRoll[0] * 180.0 / M_PI;
    double yaw = pitchYawRoll[1] * 180.0 / M_PI;
    double roll = pitchYawRoll[2] * 180.0 / M_PI;

    // 打开文件并写入数据
    std::ofstream outFile(SLAM_DATA_FILE, std::ios::app);
    if (outFile.is_open()) {
        outFile << std::fixed << std::setprecision(6)
                << "Timestamp: " << pose.hostTimestamp() << " "
                << "x: " << pose.x() << " "
                << "y: " << pose.y() << " "
                << "z: " << pose.z() << " "
                << "pitch: " << pitch << "° "
                << "yaw: " << yaw << "° "
                << "roll: " << roll << "° "
                << "fps: " << fps.fps() << std::endl;
        outFile.close();
    }

    // 每500次回调打印一次信息到控制台
    if (fps.count() % 500 == 1) {
        std::cout << "SLAM pose callback : " << fps.fps() << " Hz [timestamp=" << pose.hostTimestamp()
                  << " x=" << pose.x() << " y=" << pose.y() << " z=" << pose.z()
                  << " pitch=" << pitch << "° "
                  << " yaw=" << yaw << "° "
                  << " roll=" << roll << "° "
                  << std::endl;
    }
}

int main(int /*argc*/, char* /*argv*/[]) {
    // 设置日志级别
    xv::setLogLevel(xv::LogLevel::debug);

    // 检测设备，等待最多5秒
    auto devices = xv::getDevices(5.0);
    if (devices.empty()) {
        std::cerr << "Timeout for device detection." << std::endl;
        return EXIT_FAILURE;
    }

    // 获取第一个设备
    auto device = devices.begin()->second;

    // 检查设备是否支持SLAM
    if (!device->slam()) {
        std::cerr << "Host SLAM algorithm not supported." << std::endl;
        return EXIT_FAILURE;
    }

    // 注册SLAM回调函数
    device->slam()->registerCallback(onPose);

    // 启动SLAM
    device->slam()->start();

    std::cout << "SLAM started. Press Enter to stop..." << std::endl;

    // 等待用户输入以停止SLAM
    std::cin.get();

    // 停止SLAM
    device->slam()->stop();

    std::cout << "SLAM stopped. Data saved to " << SLAM_DATA_FILE << std::endl;

    return EXIT_SUCCESS;
}