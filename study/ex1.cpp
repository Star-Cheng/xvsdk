#include <iostream>
#include <xv-sdk.h>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <fstream>

#define M_PI 3.14159265358979323846

// 全局变量
std::shared_ptr<xv::Device> device = nullptr;
std::atomic<bool> stop(false);
std::mutex file_mutex; // 用于保护文件写入
std::ofstream csv_file; // CSV 文件流

// SLAM数据回调函数
void slamCallback(const xv::Pose &pose) {
    static std::mutex mtx;
    std::lock_guard<std::mutex> lock(mtx);

    // 获取时间戳
    double timestamp = pose.hostTimestamp();

    // 获取位置
    double x = pose.x();
    double y = pose.y();
    double z = pose.z();

    // 获取姿态（pitch, yaw, roll）
    auto pitchYawRoll = xv::rotationToPitchYawRoll(pose.rotation());
    double pitch = pitchYawRoll[0] * 180.0 / M_PI; // 转换为度
    double yaw = pitchYawRoll[1] * 180.0 / M_PI;   // 转换为度
    double roll = pitchYawRoll[2] * 180.0 / M_PI;  // 转换为度

    // 获取置信度
    double confidence = pose.confidence();

    // 打印数据
    std::cout << "Timestamp: " << timestamp << "s"
              << ", Position: (" << x << ", " << y << ", " << z << ")"
              << ", Orientation: (pitch=" << pitch << "°, yaw=" << yaw << "°, roll=" << roll << "°)"
              << ", Confidence: " << confidence
              << std::endl;

    // 将数据写入 CSV 文件
    {
        std::lock_guard<std::mutex> file_lock(file_mutex);
        if (csv_file.is_open()) {
            csv_file << timestamp << ","
                     << x << "," << y << "," << z << ","
                     << pitch << "," << yaw << "," << roll << ","
                     << confidence << "\n";
        }
    }
}

int main(int argc, char *argv[])
try {
    std::cout << "xvsdk version: " << xv::version() << std::endl;

    // 获取设备
    auto devices = xv::getDevices(10., "");
    if (devices.empty()) {
        std::cerr << "Timeout: no device found\n";
        return EXIT_FAILURE;
    }

    device = devices.begin()->second;

    // 检查SLAM模块是否存在
    if (!device->slam()) {
        std::cerr << "No SLAM module found.\n";
        return EXIT_FAILURE;
    }

    // 打开 CSV 文件
    csv_file.open("slam_data.csv");
    if (!csv_file.is_open()) {
        std::cerr << "Failed to open CSV file.\n";
        return EXIT_FAILURE;
    }

    // 写入 CSV 文件头
    csv_file << "Timestamp,X,Y,Z,Pitch,Yaw,Roll,Confidence\n";

    // 启动SLAM模块
    device->slam()->start();

    // 注册SLAM回调函数
    int slamCallbackId = device->slam()->registerCallback(slamCallback);

    std::cout << "SLAM started. Press ENTER to stop.\n";
    std::cin.get(); // 等待用户按下回车键

    // 停止SLAM模块
    device->slam()->unregisterCallback(slamCallbackId);
    device->slam()->stop();
    std::cout << "SLAM stopped.\n";

    // 关闭 CSV 文件
    csv_file.close();
    std::cout << "SLAM data saved to slam_data.csv.\n";

    return EXIT_SUCCESS;
}
catch (const std::exception &e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
}