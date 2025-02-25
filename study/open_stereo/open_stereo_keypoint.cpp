#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <xv-sdk.h>
#include <iostream>
#include <mutex>
#include <thread>
#include <chrono>

// 全局变量
std::shared_ptr<const xv::FisheyeImages> s_stereo = nullptr;
std::mutex s_mtx_stereo;
bool s_stop = false;

// 特征点检测函数
std::vector<cv::KeyPoint> detectFeatures(const cv::Mat &img)
{
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(img, keypoints);
    return keypoints;
}

// 显示图像和特征点
void display()
{
    cv::namedWindow("Left", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Right", cv::WINDOW_AUTOSIZE);

    while (!s_stop)
    {
        std::shared_ptr<const xv::FisheyeImages> stereo;
        {
            std::lock_guard<std::mutex> l(s_mtx_stereo);
            stereo = s_stereo;
        }

        if (stereo)
        {
            cv::Mat left = cv::Mat::zeros(stereo->images[0].height, stereo->images[0].width, CV_8UC1);
            cv::Mat right = cv::Mat::zeros(stereo->images[1].height, stereo->images[1].width, CV_8UC1);

            if (stereo->images[0].data != nullptr)
            {
                std::memcpy(left.data, stereo->images[0].data.get(), static_cast<size_t>(left.rows * left.cols));
            }
            if (stereo->images[1].data != nullptr)
            {
                std::memcpy(right.data, stereo->images[1].data.get(), static_cast<size_t>(right.rows * right.cols));
            }

            cv::cvtColor(left, left, cv::COLOR_GRAY2BGR);
            cv::cvtColor(right, right, cv::COLOR_GRAY2BGR);

            // 检测特征点
            auto leftKeypoints = detectFeatures(left);
            auto rightKeypoints = detectFeatures(right);

            // 缩小特征点大小
            for (auto &kp : leftKeypoints)
            {
                kp.size = 2.0;
            }
            for (auto &kp : rightKeypoints)
            {
                kp.size = 2.0;
            }

            // 绘制特征点
            cv::drawKeypoints(left, leftKeypoints, left, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            cv::drawKeypoints(right, rightKeypoints, right, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

            // 显示图像
            cv::imshow("Left", left);
            cv::imshow("Right", right);
        }

        if (cv::waitKey(1) == 27)
        { // 按下 ESC 键退出
            s_stop = true;
        }
    }
}

int main(int argc, char *argv[])
try
{
    std::cout << "xvsdk version: " << xv::version() << std::endl;

    xv::setLogLevel(xv::LogLevel::debug);

    auto devices = xv::getDevices(10., "");
    if (devices.empty())
    {
        std::cout << "Timeout: no device found\n";
        return EXIT_FAILURE;
    }

    auto device = devices.begin()->second;

    if (!device->fisheyeCameras())
    {
        std::cout << "No stereo camera found.\n";
        return EXIT_FAILURE;
    }

    device->fisheyeCameras()->registerCallback([](const xv::FisheyeImages &stereo)
                                               {
        std::lock_guard<std::mutex> l(s_mtx_stereo);
        s_stereo = std::make_shared<xv::FisheyeImages>(stereo); });

    device->fisheyeCameras()->start();

    std::thread t(display);

    std::cout << "Press ESC to stop" << std::endl;

    t.join();

    device->fisheyeCameras()->stop();

    return EXIT_SUCCESS;
}
catch (const std::exception &e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}