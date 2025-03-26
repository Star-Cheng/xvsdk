#include <xv-sdk.h>
#include <iostream>
#include <memory>
#include <functional>

class IMUDataInterface {
public:
    // 回调函数类型定义
    using IMUCallback = std::function<void(const xv::Imu&)>;

    // 构造函数
    IMUDataInterface(std::shared_ptr<xv::Device> device) 
        : m_device(device), m_callbackId(-1), m_isRunning(false) {}

    // 注册IMU数据回调
    bool registerIMUCallback(IMUCallback callback) {
        if (!m_device || !m_device->imuSensor()) {
            std::cerr << "Device or IMU sensor not available" << std::endl;
            return false;
        }

        m_callback = callback;
        
        // 注册回调函数
        m_callbackId = m_device->imuSensor()->registerCallback(
            [this](xv::Imu const& imu) {
                if (m_callback) {
                    m_callback(imu);
                }
            });
            
        if (m_callbackId == -1) {
            std::cerr << "Failed to register IMU callback" << std::endl;
            return false;
        }

        return true;
    }

    // 启动IMU数据流
    bool start() {
        if (m_isRunning) {
            return true;
        }

        if (!m_device || !m_device->imuSensor()) {
            std::cerr << "Device or IMU sensor not available" << std::endl;
            return false;
        }

        if (m_callbackId == -1 && !registerIMUCallback(nullptr)) {
            return false;
        }

        m_device->imuSensor()->start();
        m_isRunning = true;
        
        std::cout << "IMU data stream started" << std::endl;
        return true;
    }

    // 停止IMU数据流
    bool stop() {
        if (!m_isRunning) {
            return true;
        }

        if (!m_device || !m_device->imuSensor()) {
            std::cerr << "Device or IMU sensor not available" << std::endl;
            return false;
        }

        if (m_callbackId != -1) {
            m_device->imuSensor()->unregisterCallback(m_callbackId);
            m_callbackId = -1;
        }

        m_device->imuSensor()->stop();
        m_isRunning = false;
        
        std::cout << "IMU data stream stopped" << std::endl;
        return true;
    }

    // 获取单次IMU数据 - 修改后的版本
    bool getIMUData(xv::Imu& imuData) {
        if (!m_device || !m_device->imuSensor()) {
            std::cerr << "Device or IMU sensor not available" << std::endl;
            return false;
        }

        // 根据文档2，IMU数据主要通过回调获取
        // 这里我们只能返回最后接收到的IMU数据
        // 或者你需要实现一个缓存机制来存储最新的IMU数据
        std::cerr << "Warning: getIMUData() may not work as expected. Consider using callback instead." << std::endl;
        return false;
    }

    // 检查是否正在运行
    bool isRunning() const {
        return m_isRunning;
    }

    // 析构函数
    ~IMUDataInterface() {
        stop();
    }

private:
    std::shared_ptr<xv::Device> m_device;
    IMUCallback m_callback;
    int m_callbackId;
    bool m_isRunning;
};

// 使用示例
int main() {
    // 初始化设备
    auto devices = xv::getDevices(10.);
    if (devices.empty()) {
        std::cerr << "No device found" << std::endl;
        return -1;
    }
    auto device = devices.begin()->second;

    // 创建IMU接口实例
    IMUDataInterface imuInterface(device);

    // 定义IMU数据回调函数
    auto imuCallback = [](const xv::Imu& imu) {
        static int count = 0;
        if (count++ % 10 == 0) {  // 每100条数据打印一次
            std::cout << "IMU Data - "
                      << "Gyro: (" << imu.gyro[0] << ", " << imu.gyro[1] << ", " << imu.gyro[2] << "), "
                      << "Accel: (" << imu.accel[0] << ", " << imu.accel[1] << ", " << imu.accel[2] << "), "
                      << "Temp: " << imu.temperature
                      << std::endl;
        }
    };

    // 注册回调并启动IMU数据流
    if (imuInterface.registerIMUCallback(imuCallback) && imuInterface.start()) {
        std::cout << "IMU data collection started. Press Enter to stop..." << std::endl;
        std::cin.get();  // 等待用户输入停止
        
        // 停止IMU数据流
        imuInterface.stop();
    }

    return 0;
}