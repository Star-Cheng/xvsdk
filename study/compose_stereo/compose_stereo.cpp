#include <iostream>  
#include <opencv2/opencv.hpp>  
#include <Eigen/Eigen>  
#include <pcl/point_cloud.h>  
#include <pcl/visualization/pcl_visualizer.h>  

void showPointCloudPCL(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointcloud)  
{  
    pcl::visualization::PCLVisualizer visualizer("showcloud");  
    visualizer.addPointCloud(pointcloud);  
    visualizer.spin();  
}  

void showPointCloudCV(const std::vector<cv::Vec3f>& cloudpos, const std::vector<cv::Vec3b>& cloudcol)  
{  
    cv::viz::Viz3d window("showcloud");  
    cv::viz::WCloud cloud_widget(cloudpos, cloudcol);  
    window.showWidget("pointcloud", cloud_widget);  
    window.spin();  
}  

int main()  
{  
    double f = 718.856, cx = 607.1928, cy = 185.2157;  
    double b = 0.573;  

    cv::Mat left_img = cv::imread("/home/gym/code/edge/xvsdk/left_image_0.png");  
    cv::Mat right_img = cv::imread("/home/gym/code/edge/xvsdk/right_image_0.png");  

    // StereoSGBM parameters  
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);  
    cv::Mat disparity_sgbm, disparity;  
    sgbm->compute(left_img, right_img, disparity_sgbm);  
    disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0);  

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);  
    for (int v = 0; v < left_img.rows; v++)  
    {  
        for (int u = 0; u < left_img.cols; u++) // 注意这里为 left_img.cols  
        {  
            float disp = disparity.at<float>(v, u);  
            if (disp <= 10 || disp >= 96) continue;  

            pcl::PointXYZRGB point;  
            double x = (u - cx) / f;  
            double y = (v - cy) / f;  
            double depth = f * b / disp;  
            point.x = x * depth;  
            point.y = y * depth;  
            point.z = depth;  
            point.b = left_img.at<cv::Vec3b>(v, u)[0];  
            point.g = left_img.at<cv::Vec3b>(v, u)[1];  
            point.r = left_img.at<cv::Vec3b>(v, u)[2];  
            pointcloud->push_back(point);  
        }  
    }  
    // showPointCloudPCL(pointcloud);

    cv::imshow("disparity", disparity / 96);  
    cv::waitKey(0);  

    return 0;  
}