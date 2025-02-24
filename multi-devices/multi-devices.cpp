#define _USE_MATH_DEFINES
#include <xv-sdk.h>
#include <iostream>
#include <thread>
#include <mutex>
#include <cmath>

#include "fps_count.hpp"
#define USE_GET_DEVICES_UNTIL_TIMEOUT 1

class CameraDevice {
public:
	CameraDevice(std::shared_ptr<xv::Device> device, std::string name) : m_device(device), m_deviceName(name) {
		start();
	}

	CameraDevice(const CameraDevice& cam) {
		this->m_device = cam.m_device;
		this->m_deviceName = cam.m_deviceName;
		this->imu_fc = cam.imu_fc;
		this->fe_fc = cam.fe_fc;
		this->slam_fc = cam.slam_fc;
	}

	void start()
	{
		std::cout << "start device: " << m_device << std::endl;
		m_device->orientationStream()->start();
		m_device->orientationStream()->registerCallback([&](xv::Orientation const& o) {
			m_imumtx.lock();
			imu_fc.tic();
			if (imu_k++ % 100 == 0) {
				auto& q = o.quaternion();
				std::cout << "device: " << m_deviceName << "  orientation" << "@" << std::round(imu_fc.fps()) << "fps"
					<< " 3dof=(" << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << "),"
					<< std::endl;
			}
			m_imumtx.unlock();
			});
		m_device->fisheyeCameras()->registerCallback([&](xv::FisheyeImages const& fisheye) {
			m_femtx.lock();
			fe_fc.tic();
			if (fisheye_k++ % 50 == 0 && fisheye.images.size() >= 1) {
				std::cout << "device: " << m_deviceName << "  "<<"fisheye " << fisheye.images.at(0).width << "x" << fisheye.images.at(0).height << "@" << std::round(fe_fc.fps()) << "fps" << std::endl;
			}
			m_femtx.unlock();

			});
		m_device->fisheyeCameras()->start();

        m_device->colorCamera()->registerCallback([&](xv::ColorImage const & image){
            rgb_fc.tic();
            if(rgb_k++ % 100 == 0)
            {
                std::cout << "device: " << m_deviceName << "  "<<"RGB " << image.width << "x" << image.height << "@" << std::round(rgb_fc.fps()) << "fps" << std::endl;
            }
        });
        m_device->colorCamera()->start();
        m_device->colorCamera()->setResolution(xv::ColorCamera::Resolution::RGB_1920x1080);

		m_device->slam()->registerCallback([&](const xv::Pose& pose) {
			m_slammtx.lock();
			slam_fc.tic();
			if (slam_k++ % 500 == 0) {
				auto pitchYawRoll = xv::rotationToPitchYawRoll(pose.rotation());
				std::cout << "device: " << m_deviceName << "  " << "slam-pose" << timeShowStr(pose.edgeTimestampUs(), pose.hostTimestamp()) << "@" << std::round(slam_fc.fps()) << "fps" << " (" << pose.x() << "," << pose.y() << "," << pose.z() << "," << pitchYawRoll[0] * 180 / M_PI << "," << pitchYawRoll[1] * 180 / M_PI << "," << pitchYawRoll[2] * 180 / M_PI << ")" << pose.confidence() << std::endl;
			}
			m_slammtx.unlock();
			});
		m_device->slam()->start();
	}

	std::string getDeviceName() const {
		return m_deviceName;
	}
private:
	std::shared_ptr<xv::Device> m_device = {};
	std::string m_deviceName = "";
	FpsCount imu_fc;
	FpsCount fe_fc;
	FpsCount slam_fc;
    FpsCount rgb_fc;
	std::mutex m_imumtx;
	std::mutex m_femtx;
	std::mutex m_slammtx;
	int count = 0;
    int slam_k = 0;
    int fisheye_k = 0;
    int imu_k = 0;
    int rgb_k = 0;

	std::string timeShowStr(std::int64_t edgeTimestampUs, double hostTimestamp) {
		char s[1024];
		double now = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() * 1e-6;
		std::sprintf(s, " (device=%lld host=%.4f now=%.4f delay=%.4f) ", (long long)edgeTimestampUs, hostTimestamp, now, now - hostTimestamp);
		return std::string(s);
	}

};



int main()
{
	// before use this demo, please ensure all devices have been connected.
	std::map<std::string, std::shared_ptr<xv::Device>> devices;
	std::mutex mtx;
	std::vector<CameraDevice *> cameras;
	// use a thread to get devices all the time, ensure xvsdk can get all the devices

#if USE_GET_DEVICES_UNTIL_TIMEOUT

    devices = xv::getDevicesUntilTimeout(10.0);
    std::cout<<"******************************************"<<std::endl;
    //wait all devices have been connect then control them.
    if(devices.size() > 0) {
        for (auto item : devices) {
                std::string uuid = item.first;
                auto dev = item.second;
                bool find = false;
                for (int i = 0; i < cameras.size(); i++) {
                    if (cameras[i]->getDeviceName() == uuid) {
                        find = true;
                        break;
                    }
                }
                if (!find) {
                    std::cout << "New device: " << uuid << std::endl;
                    cameras.push_back(new CameraDevice(dev, uuid));
                }
        }
    }
    while(true){
		std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
#else

    bool isFindAll = false;
	std::thread monity([&]() {
		while (!isFindAll)
		{
			mtx.lock();	
			devices = xv::getDevices(10.0);
			mtx.unlock();
			std::this_thread::sleep_for(std::chrono::seconds(1));
		}
	});

	bool isprint = false;
	while (true)
	{
		mtx.lock();

		//wait all devices have been connect then control them.
		for (auto item : devices) {
			std::string uuid = item.first;
			auto dev = item.second;
			bool find = false;
			for (int i = 0; i < cameras.size(); i++) {
				if (cameras[i]->getDeviceName() == uuid) {
					find = true;
					break;
				}
			}
			if (!find) {
				std::cout << "New device: " << uuid << std::endl;
				cameras.push_back(new CameraDevice(dev, uuid));
			}
		}
		mtx.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	monity.join();

#endif
}
