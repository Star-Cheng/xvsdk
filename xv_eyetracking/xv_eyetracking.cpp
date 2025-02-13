#include <xv-sdk.h>

#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <thread>

#ifdef USE_OPENCV
#include <opencv2/opencv.hpp>
cv::Mat raw_to_opencv(xv::EyetrackingImage const & eyetracking);
#endif

#include "fps_count.hpp"

#include <signal.h>

static bool stop = false;

void signal_handler(int /*sig*/)
{
    stop = true;
}

static std::shared_ptr<xv::Device> device;

static int et_ae_manul_control(void)
{
    int r1,r2,r3;

	uint32_t aecmode = 0;
	uint32_t aecgain;
	uint32_t aec_time;

	const char menubuf[] = "enter et aec mode [0:auto, 1:manul]: ";
	printf("%s\n", menubuf);
    r1 = scanf("%d", &aecmode);

	const char menubuf1[] = "enter et gain[0-255]: ";
	printf("%s\n", menubuf1);
	r2 = scanf("%d", &aecgain);

	const char menubuf2[] = "enter exposure [0-330] ";
	printf("%s\n", menubuf2);
	r3 = scanf("%d", &aec_time);

    if (r1<=0 || r2<=0|| r3<=0)
        return -1;

    //xvusb_data_t cmd;
	//cmd.devID = 0x0c;
	//cmd.cmd = 1;
	//cmd.args.val[0] = aecmode;
	//cmd.args.val[1] = aecgain;
	//*(uint32_t *)&cmd.args.val[2] = aec_time;
	//g_pvsc->camControl(&cmd);

    device->eyetracking()->setExposure(aecgain, aec_time/10.0, aecgain, aec_time/10.0);

	return 0;
}

static int et_led_control(void)
{
    int r1,r2,r3;

	uint32_t selector;
	uint32_t channel;
	uint32_t brightness;

	const char menubuf[] = "enter et led selector id [0, 1]: ";
	printf("%s\n", menubuf);
    r1 = scanf("%d", &selector);

	const char menubuf1[] = "enter et led channel[0-7]: ";
	printf("%s\n", menubuf1);
    r2 =scanf("%d", &channel);

	const char menubuf2[] = "enter et led brightness [0-255]: ";
	printf("%s\n", menubuf2);
    r3 = scanf("%d", &brightness);

    if (r1<=0 || r2<=0|| r3<=0)
        return -1;

	printf("selector,channel,brightness: %d,%d,%d\n", selector,channel,brightness);

    //xvusb_data_t cmd;
	//cmd.devID = 0x0c;
	//cmd.cmd = 3;
	//cmd.args.val[0] = selector;
	//cmd.args.val[1] = channel;
	//cmd.args.val[2] = brightness;
	//g_pvsc->camControl(&cmd);

    device->eyetracking()->setLedBrighness(selector, channel, brightness);

	return 0;
}

static int cmd_handle(int cmd)
{
	int ret = 1;
	switch (cmd) {
	case 1:
		printf("eyetracking capture start\n");
        device->eyetracking()->start();
		break;

	case 2:
		printf("eyetracking capture stop\n");
        device->eyetracking()->stop();
		break;

	case 3:
		printf("set ae exposure time\n");
		et_ae_manul_control();
		break;

	case 4:
		printf("set led brightness\n");
		et_led_control();
		break;

	case 0:
	default:
		printf("program will exit\n");
		ret = 0;
		break;
	}

	return ret;
}

int main( int /*argc*/, char* /*argv*/[] )
{
    std::cout << "XVSDK version: " << xv::version() << std::endl;

    signal(SIGINT, signal_handler);

    auto devices = xv::getDevices(10.);
    if (devices.empty()) {
        std::cout << "Timeout: no device found\n";
        return EXIT_FAILURE;
    }

    device = devices.begin()->second;

    device->eyetracking()->registerCallback( [](xv::EyetrackingImage const & eyetracking){
        static FpsCount fc;
        fc.tic();
        static int k = 0;
        if(k++%50==0){
            std::cout << eyetracking.images[0].width << "x" << eyetracking.images[0].height << "@" << std::round(fc.fps()) << "fps" << std::endl;
        }
    });

#ifdef USE_OPENCV
    device->eyetracking()->registerCallback( [](xv::EyetrackingImage const & eyetracking){
        cv::Mat img = raw_to_opencv(eyetracking);
        cv::imshow("STEREO", img);
        cv::waitKey(1);
    });
#endif

    static const char meun_main[]={
        "\n\nET control interface Demo\n"
        "------------------------------\n"
        "1 : Start ET stream\n"
        "2 : Stop ET stream\n"
        "3 : Set ET exposure time\n"
        "4 : Set led brightness\n"
        "0: exit program\n"
        "------------------------------\n"
        "enter select:"
    };

    while (1) {
        printf("%s", meun_main);

        int a;
        if(scanf("%d", &a) > 0) {
            if(a == 0) break;
            cmd_handle(a);
        }
    }

    //std::this_thread::sleep_for( std::chrono::seconds(30) );


    return EXIT_SUCCESS;
}
