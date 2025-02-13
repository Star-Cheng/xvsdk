#include <opencv2/opencv.hpp>
#include <xv-types.h>


void gray8_to_bgr24(unsigned char* buffer,unsigned char* bgrbuffer, int width, int height)
{
    int i, j;
    for (j = 0; j < height; j += 1)
    {
        for (i = 0; i < width; i += 1)
        {
            *bgrbuffer++ = *buffer;
            *bgrbuffer++ = *buffer;
            *bgrbuffer++ = *buffer;
            buffer++;
        }
    }
}

cv::Mat raw_to_opencv(xv::EyetrackingImage const & eyetracking)
{
    cv::Mat img = cv::Mat::zeros(eyetracking.images[0].height * 2, eyetracking.images[0].width, CV_8UC3);
    unsigned char* raw;
    raw = const_cast<unsigned char *>(eyetracking.images[0].data.get());
    gray8_to_bgr24(raw, img.data, eyetracking.images[0].width, eyetracking.images[0].height );
    raw = const_cast<unsigned char *>(eyetracking.images[1].data.get());
    gray8_to_bgr24(raw, img.data + eyetracking.images[0].width * eyetracking.images[0].height * 3, eyetracking.images[1].width, eyetracking.images[1].height );
    return img;
}
