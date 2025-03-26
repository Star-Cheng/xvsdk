#include <opencv2/opencv.hpp>  
#include <opencv2/features2d.hpp>  
#include <iostream>  

void orb_feature_detection(const std::string& image_path) {  
    // 读取图像  
    cv::Mat img = cv::imread(image_path, cv::IMREAD_GRAYSCALE);  
    if (img.empty()) {  
        std::cerr << "Could not read the image: " << image_path << std::endl;  
        return;  
    }  

    // 创建ORB特征检测器  
    std::shared_ptr<cv::ORB> orb = cv::ORB::create();  

    // 检测关键点和计算描述符  
    std::vector<cv::KeyPoint> keypoints;  
    cv::Mat descriptors;  
    orb->detectAndCompute(img, cv::noArray(), keypoints, descriptors);  

    // 调整特征点的大小
    for (auto &kp : keypoints)
    {
        kp.size = 1.0;  // 设置左图特征点的大小
    }
    // 绘制关键点  
    cv::Mat img_keypoints;  
    cv::drawKeypoints(img, keypoints, img_keypoints, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);  

    // 显示结果  
    cv::imshow("ORB Keypoints", img_keypoints);  
    cv::waitKey(0);  
    cv::destroyAllWindows();  
}  

int main() {  
    orb_feature_detection("/home/gym/data/Test_data/test/IMG_4675img.png");  
    return 0;  
}  