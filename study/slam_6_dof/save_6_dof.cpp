#define _USE_MATH_DEFINES // for C++

#include <xv-sdk.h>
#include <iostream>
#include <thread>
#include <atomic>
#include <cmath>
#include <fstream> // 用于文件操作
#include "frequency_counter.hpp"
#include <iomanip>

std::ofstream outFile("slam_data.csv"); // 打开文件写入数据

// 回调函数，用于处理SLAM的位姿数据
void savePoseToCSV(const xv::Pose &pose)
{
    // 获取姿态的旋转数据
    auto pitchYawRoll = xv::rotationToPitchYawRoll(pose.rotation());

    // 打印获取的位姿数据
    std::cout << "Saving pose to CSV: "
              << "timestamp=" << pose.hostTimestamp() << ", "
              << "x=" << pose.x() << ", "
              << "y=" << pose.y() << ", "
              << "z=" << pose.z() << ", "
              << "pitch=" << pitchYawRoll[0] * 180.0 / M_PI << "°, "
              << "yaw=" << pitchYawRoll[1] * 180.0 / M_PI << "°, "
              << "roll=" << pitchYawRoll[2] * 180.0 / M_PI << "°, "
              << "confidence=" << pose.confidence()
              << std::endl;

    // 将数据写入CSV文件
    outFile << pose.hostTimestamp() << ","
            << pose.x() << ","
            << pose.y() << ","
            << pose.z() << ","
            << pitchYawRoll[0] * 180.0 / M_PI << "," // pitch
            << pitchYawRoll[1] * 180.0 / M_PI << "," // yaw
            << pitchYawRoll[2] * 180.0 / M_PI << "," // roll
            << pose.confidence() << std::endl;       // confidence
}

// 回调函数，用于处理SLAM的位姿数据
void onPose(xv::Pose const &pose)
{
    // 保存姿态数据到CSV并打印调试信息
    savePoseToCSV(pose);

    // 获取姿态的旋转数据
    auto pitchYawRoll = xv::rotationToPitchYawRoll(pose.rotation());
    static FrequencyCounter fps;
    fps.tic();
    if (fps.count() % 500 == 1)
    {
        std::cout << "SLAM pose callback : " << fps.fps() << " Hz [timestamp=" << pose.hostTimestamp()
                  << " x=" << pose.x() << " y=" << pose.y() << " z=" << pose.z()
                  << " pitch=" << pitchYawRoll[0] * 180. / M_PI << "°"
                  << " yaw=" << pitchYawRoll[1] * 180. / M_PI << "°"
                  << " roll=" << pitchYawRoll[2] * 180. / M_PI << "°"
                  << std::endl;
    }
}

int main(int /*argc*/, char * /*argv*/[])
{
    // 打开CSV文件并写入标题
    outFile << "timestamp,x,y,z,pitch,yaw,roll,confidence" << std::endl;

    // 设置日志级别
    xv::setLogLevel(xv::LogLevel::debug);

    // 获取设备列表
    auto devices = xv::getDevices(5.);
    if (devices.empty())
    {
        std::cerr << "Timeout for device detection." << std::endl;
        return EXIT_FAILURE;
    }

    // 选择第一个设备
    auto device = devices.begin()->second;

    if (!device->slam())
    {
        std::cerr << "Host SLAM algorithm not supported." << std::endl;
        return EXIT_FAILURE;
    }

    // 注册方向（3dof）跟踪
    // device->orientationStream()->registerCallback(on3dof);

    // 启动3dof跟踪
    device->orientationStream()->start();

    // 显示设置
    auto display = device->display();
    if (display && display->calibration().size() == 2)
    {
        std::cout << "left eye display: " << display->calibration()[0].pdcm[0].w << "x" << display->calibration()[0].pdcm[0].h << std::endl;
        std::cout << "right eye display: " << display->calibration()[1].pdcm[0].w << "x" << display->calibration()[1].pdcm[0].h << std::endl;
    }

    // 注册回调函数，获取SLAM的位姿
    device->slam()->registerCallback(onPose);

    // 模拟60Hz循环来获取SLAM位姿
    std::atomic<bool> stop(false);
    std::thread threadLoop60Hz([&stop, &device]
                               {
        while (!stop) {
            auto now = std::chrono::steady_clock::now();
            xv::Pose pose;
            // 获取当前位姿（没有延迟，因为内部补偿了最后一次IMU数据接收到的预测）
            if (device->slam()->getPose(pose)) {
                auto pitchYawRoll = xv::rotationToPitchYawRoll(pose.rotation());
                static FrequencyCounter fps;
                fps.tic();
                if (fps.count() % 120 == 1) {
                    std::cout << "Current SLAM : " << fps.fps() << " Hz [timestamp=" << pose.hostTimestamp() << " x=" << pose.x() << " y=" << pose.y() << " z=" << pose.z()
                              << " pitch="  << pitchYawRoll[0]*180./M_PI << "°" << " yaw="  << pitchYawRoll[1]*180./M_PI << "°" 
                              << " roll="  << pitchYawRoll[2]*180./M_PI << "°" << std::endl;
                }
            }
            std::this_thread::sleep_until(now + std::chrono::microseconds(long(1. / 60. * 1e6)));
        } });

    std::cout << "Press enter to start SLAM ..." << std::endl;
    std::cin.get();

    // 启动SLAM
    device->slam()->start();

    std::cout << "Press enter to stop SLAM ..." << std::endl;
    std::cin.get();

    // 停止SLAM
    device->slam()->stop();

    // 停止线程
    stop = true;

    if (threadLoop60Hz.joinable())
    {
        threadLoop60Hz.join();
    }

    // 关闭文件
    outFile.close();

    return EXIT_SUCCESS;
}
