#include <opencv2/opencv.hpp>
#include <xv-sdk.h>
#include <thread>
#include <mutex>
#include <memory>

// 图像数据共享指针和互斥锁
std::shared_ptr<const xv::ColorImage> s_rgb = nullptr;
std::mutex s_mtx_rgb;
// 用于计算帧率的变量
std::chrono::steady_clock::time_point lastFrameTime;
int frameCount = 0;
double fps = 0.0;
// 图像转换函数（来自raw2opencv.cpp）
cv::Mat raw_to_opencv(std::shared_ptr<const xv::ColorImage> rgb) {
    cv::Mat img;
    switch (rgb->codec) {
    case xv::ColorImage::Codec::YUV420p: {
        img = cv::Mat(rgb->height, rgb->width, CV_8UC3);
        auto rawImg = cv::Mat(rgb->height * 3 / 2, rgb->width, CV_8UC1, 
                            const_cast<unsigned char*>(rgb->data.get()));
        cv::cvtColor(rawImg, img, cv::COLOR_YUV2BGR_I420);
        break;
    }
    case xv::ColorImage::Codec::YUYV: {
        img = cv::Mat(rgb->height, rgb->width, CV_8UC3);
        auto rawImg = cv::Mat(rgb->height, rgb->width, CV_8UC2, 
                            const_cast<unsigned char*>(rgb->data.get()));
        cv::cvtColor(rawImg, img, cv::COLOR_YUV2BGR_YUYV);
        break;
    }
    case xv::ColorImage::Codec::JPEG: {
        cv::Mat raw(1, rgb->width*rgb->height, CV_8UC1, 
                   const_cast<unsigned char*>(rgb->data.get()));
        img = cv::imdecode(raw, cv::IMREAD_COLOR);
        break;
    }
    }
    return img;
}

// 显示线程函数
void display() {
    cv::namedWindow("RGB Camera", cv::WINDOW_AUTOSIZE);
    while (true) {
        std::shared_ptr<const xv::ColorImage> rgb;
        {
            std::lock_guard<std::mutex> lock(s_mtx_rgb);
            rgb = s_rgb;
        }
        if (rgb && rgb->width > 0) {
            cv::Mat img = raw_to_opencv(rgb);
            cv::imshow("RGB Camera", img);
        }
        if (cv::waitKey(1) == 27) break; // ESC键退出
    }
}

int main() try {
    // 初始化设备
    auto devices = xv::getDevices(10.0);
    if (devices.empty()) {
        std::cerr << "未找到设备" << std::endl;
        return EXIT_FAILURE;
    }
    auto device = devices.begin()->second;

    // 检查RGB相机
    if (!device->colorCamera()) {
        std::cerr << "未找到RGB相机" << std::endl;
        return EXIT_FAILURE;
    }

    // 设置RGB相机回调
    device->colorCamera()->registerCallback([](const xv::ColorImage& im) {
        std::lock_guard<std::mutex> lock(s_mtx_rgb);
        s_rgb = std::make_shared<xv::ColorImage>(im);
        // 计算帧率
        auto currentTime = std::chrono::steady_clock::now();
        if (frameCount == 0)
        {
            lastFrameTime = currentTime;
        }
        frameCount++;
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastFrameTime).count();
        if (duration >= 1000) // 从毫秒到秒的转换
        {
            fps = frameCount / (duration / 1000.0);
            std::cout << "FPS: " << fps << ", Resolution: " << im.width << "x" << im.height << std::endl;
            frameCount = 0;
            lastFrameTime = currentTime;
        }
    });

    // 启动相机
    device->colorCamera()->setResolution(xv::ColorCamera::Resolution::RGB_1920x1080);
    device->colorCamera()->start();

    // 启动显示线程
    std::thread display_thread(display);
    
    // 等待退出
    std::cout << "按ESC键退出..." << std::endl;
    display_thread.join();

    // 停止相机
    device->colorCamera()->stop();
    cv::destroyAllWindows();
    return EXIT_SUCCESS;
}
catch (const std::exception& e) {
    std::cerr << "错误: " << e.what() << std::endl;
    return EXIT_FAILURE;
}