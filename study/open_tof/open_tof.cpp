#include <opencv2/opencv.hpp>
#include <xv-sdk.h>
#include <iostream>
#include <mutex>
#include <thread>
#include <chrono>
#include "colors.h"

// 全局变量
std::shared_ptr<const xv::DepthImage> s_tof = nullptr;
std::mutex s_mtx_tof;
bool s_stop = false;
cv::Mat raw_to_opencv(std::shared_ptr<const xv::DepthImage> tof)
{
    cv::Mat out;
    if (tof->height > 0 && tof->width > 0)
    {
        out = cv::Mat::zeros(tof->height, tof->width, CV_8UC3);
        if (tof->type == xv::DepthImage::Type::Depth_32)
        {
            float dmax = 7.5;
            const auto tmp_d = reinterpret_cast<float const *>(tof->data.get());
            for (unsigned int i = 0; i < tof->height * tof->width; i++)
            {
                const auto &d = tmp_d[i];
                if (d < 0.01 || d > 9.9)
                {
                    out.at<cv::Vec3b>(i / tof->width, i % tof->width) = 0;
                }
                else
                {
                    unsigned int u = static_cast<unsigned int>(std::max(0.0f, std::min(255.0f, d * 255.0f / dmax)));
                    const auto &cc = colors.at(u);
                    out.at<cv::Vec3b>(i / tof->width, i % tof->width) = cv::Vec3b(cc.at(2), cc.at(1), cc.at(0));
                }
            }
        }
        else if (tof->type == xv::DepthImage::Type::Depth_16)
        {
            float dmax = 2494.0; // maybe 7494,2494,1498,1249 see mode_manage.h in sony toflib
            const auto tmp_d = reinterpret_cast<int16_t const *>(tof->data.get());
            for (unsigned int i = 0; i < tof->height * tof->width; i++)
            {
                const auto &d = tmp_d[i];
                unsigned int u = static_cast<unsigned int>(std::max(0.0f, std::min(255.0f, d * 255.0f / dmax)));
                const auto &cc = colors.at(u);
                out.at<cv::Vec3b>(i / tof->width, i % tof->width) = cv::Vec3b(cc.at(2), cc.at(1), cc.at(0));
            }
        }
    }
    return out;
}
void tofCallback(const xv::DepthImage &tof)
{
    if (tof.type == xv::DepthImage::Type::Depth_16 || tof.type == xv::DepthImage::Type::Depth_32)
    {
        std::lock_guard<std::mutex> l(s_mtx_tof);
        s_tof = std::make_shared<xv::DepthImage>(tof);
    }
}
void display()
{
    cv::namedWindow("TOF");
    cv::moveWindow("TOF", 500, 462);
    while (!s_stop)
    {
        std::shared_ptr<const xv::DepthImage> tof = nullptr;

        s_mtx_tof.lock();
        tof = s_tof;
        s_mtx_tof.unlock();

        if (tof)
        {
            cv::Mat img = raw_to_opencv(tof);
            if (img.rows > 0 && img.cols > 0)
                cv::imshow("TOF", img);
        }

        cv::waitKey(1);
    }
}
int main(int argc, char **argv)
{
    auto devices = xv::getDevices(10.0);
    if (devices.empty())
    {
        std::cout << "Timeout: no device found\n";
    }
    auto device = devices.begin()->second;
    std::mutex mutex_str;
    std::string localize_str;

    std::mutex mutex_images;
    xv::FisheyeImages images;

    if (device->tofCamera())
    {
        device->tofCamera()->registerCallback(tofCallback);
        device->tofCamera()->start();
    }
    else
    {
        std::cout << "No TOF camera.\n";
        return EXIT_FAILURE;
    }

    std::thread t(display);
    if (t.joinable()) {
        t.join();
    }

    if (device->tofCamera()) {
        device->tofCamera()->stop();
    }

    return EXIT_SUCCESS;
}