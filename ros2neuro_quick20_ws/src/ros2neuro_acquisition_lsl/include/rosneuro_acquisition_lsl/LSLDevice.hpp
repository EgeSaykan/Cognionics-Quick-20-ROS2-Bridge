#ifndef ROSNEURO_ACQUISITION_PLUGIN_LSLDEVICE_HPP
#define ROSNEURO_ACQUISITION_PLUGIN_LSLDEVICE_HPP

#include <errno.h>
#include <string.h>
#include <iostream>
#include <cstring>
#include <regex>
#include <limits>
#include <lsl_cpp.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <rosneuro_data/NeuroData.hpp>
#include <rosneuro_acquisition/Device.hpp>
#include <gtest/gtest_prod.h>

namespace rosneuro {
    class LSLDevice : public Device {
        public:
            LSLDevice(void);
            LSLDevice(NeuroFrame* frame);
            virtual ~LSLDevice(void);
            bool Configure(NeuroFrame* frame, unsigned int framerate);

            bool Setup(void);
            bool Open(void);
            bool Close(void);
            bool Start(void);
            bool Stop(void);
            size_t Get(void);
            size_t GetAvailable(void);

        private:
            void destroy_lsl_structures(void);

            lsl::stream_inlet* stream_;
            lsl::stream_info*  info_;
            std::string		stream_name_;
            std::string		stream_type_;
            std::string		lsl_type_;
            std::string		lsl_name_;
            unsigned int	samplerate_;
            unsigned int	framerate_;

            FRIEND_TEST(LSLDeviceTestSuite, Constructor);
            FRIEND_TEST(LSLDeviceTestSuite, Destructor);
    };
    PLUGINLIB_EXPORT_CLASS(rosneuro::LSLDevice, rosneuro::Device)
}

#endif
