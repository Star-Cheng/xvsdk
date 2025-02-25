#include <opencv2/opencv.hpp>
#include <xv-sdk.h>
#include <iostream>
#include <mutex>
#include <thread>
#include "colors.h"  // 假设colors.h中定义了颜色映射表`colors`>

// 全局变量
std::shared_ptr<const xv::DepthImage> s_tof = nullptr;
std::shared_ptr<const xv::GrayScaleImage> s_ir = nullptr;
std::mutex s_mtx_tof;
std::mutex s_mtx_ir;
bool s_stop = false;

// TOF相机回调函数
void tofCallback(const xv::DepthImage& tof) {
    if (tof.type == xv::DepthImage::Type::Depth_16 || tof.type == xv::DepthImage::Type::Depth_32) {
        std::lock_guard<std::mutex> l(s_mtx_tof);
        s_tof = std::make_shared<xv::DepthImage>(tof);
    } else if (tof.type == xv::DepthImage::Type::IR) {
        auto ir = std::make_shared<xv::GrayScaleImage>();
        ir->width = tof.width;
        ir->height = tof.height;
        ir->data = tof.data;
        std::lock_guard<std::mutex> l(s_mtx_ir);
        s_ir = ir;
    }
}

// 将TOF深度图像转换为OpenCV格式
cv::Mat raw_to_opencv(std::shared_ptr<const xv::DepthImage> tof) {
    cv::Mat out;
    if (tof->height > 0 && tof->width > 0) {
        out = cv::Mat::zeros(tof->height, tof->width, CV_8UC3);
        if (tof->type == xv::DepthImage::Type::Depth_32) {
            float dmax = 7.5;
            const auto tmp_d = reinterpret_cast<float const*>(tof->data.get());
            for (unsigned int i = 0; i < tof->height * tof->width; i++) {
                const auto& d = tmp_d[i];
                if (d < 0.01 || d > 9.9) {
                    out.at<cv::Vec3b>(i / tof->width, i % tof->width) = 0;
                } else {
                    unsigned int u = static_cast<unsigned int>(std::max(0.0f, std::min(255.0f, d * 255.0f / dmax)));
                    const auto& cc = colors.at(u);
                    out.at<cv::Vec3b>(i / tof->width, i % tof->width) = cv::Vec3b(cc.at(2), cc.at(1), cc.at(0));
                }
            }
        } else if (tof->type == xv::DepthImage::Type::Depth_16) {
            float dmax = 2494.0; // maybe 7494,2494,1498,1249 see mode_manage.h in sony toflib
            const auto tmp_d = reinterpret_cast<int16_t const*>(tof->data.get());
            for (unsigned int i = 0; i < tof->height * tof->width; i++) {
                const auto& d = tmp_d[i];
                unsigned int u = static_cast<unsigned int>(std::max(0.0f, std::min(255.0f, d * 255.0f / dmax)));
                const auto& cc = colors.at(u);
                out.at<cv::Vec3b>(i / tof->width, i % tof->width) = cv::Vec3b(cc.at(2), cc.at(1), cc.at(0));
            }
        }
    }
    return out;
}

// 将TOF IR图像转换为OpenCV格式
cv::Mat raw_to_opencv_tof_ir(const xv::GrayScaleImage& tof_ir) {
    cv::Mat out;
    out = cv::Mat::zeros(tof_ir.height, tof_ir.width, CV_8UC3);
    float dmax = 2191 / 4;
    auto tmp_d = reinterpret_cast<short const*>(tof_ir.data.get());
    for (unsigned int i = 0; i < tof_ir.height * tof_ir.width; i++) {
        short d = tmp_d[i];
        const auto& cc = colors.at(d);
        out.at<cv::Vec3b>(i / tof_ir.width, i % tof_ir.width) = cv::Vec3b(cc.at(2), cc.at(1), cc.at(0));
    }
    return out;
}

// 显示TOF图像
void display() {
    cv::namedWindow("TOF");
    cv::moveWindow("TOF", 500, 462);
    cv::namedWindow("IR");
    cv::moveWindow("IR", 500 + 640, 462);

    while (!s_stop) {
        std::shared_ptr<const xv::DepthImage> tof = nullptr;
        std::shared_ptr<const xv::GrayScaleImage> ir = nullptr;

        s_mtx_tof.lock();
        tof = s_tof;
        s_mtx_tof.unlock();

        s_mtx_ir.lock();
        ir = s_ir;
        s_mtx_ir.unlock();

        if (tof) {
            cv::Mat img = raw_to_opencv(tof);
            if (img.rows > 0 && img.cols > 0)
                cv::imshow("TOF", img);
        }

        if (ir) {
            cv::Mat img = raw_to_opencv_tof_ir(*ir);
            if (img.rows > 0 && img.cols > 0)
                cv::imshow("IR", img);
        }

        cv::waitKey(1);
    }
}

int main(int argc, char* argv[]) try {
    auto devices = xv::getDevices(10., "");
    if (devices.empty()) {
        std::cout << "Timeout: no device found\n";
        return EXIT_FAILURE;
    }

    auto device = devices.begin()->second;

    if (device->tofCamera()) {
        device->tofCamera()->registerCallback(tofCallback);
        device->tofCamera()->start();
    } else {
        std::cout << "No TOF camera.\n";
        return EXIT_FAILURE;
    }

    s_stop = false;
    std::thread t(display);

    std::cout << "Press ENTER to stop" << std::endl;
    std::cin.get();

    s_stop = true;
    if (t.joinable()) {
        t.join();
    }

    if (device->tofCamera()) {
        device->tofCamera()->stop();
    }

    return EXIT_SUCCESS;
} catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}