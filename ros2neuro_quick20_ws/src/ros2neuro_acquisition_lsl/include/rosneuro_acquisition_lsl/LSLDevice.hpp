#ifndef ROSNEURO_ACQUISITION_PLUGIN_LSLDEVICE_HPP
#define ROSNEURO_ACQUISITION_PLUGIN_LSLDEVICE_HPP

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <lsl_cpp.h>
#include <iostream>
#include <limits>
#include <regex>
#include <string>
#include <cstring>

#include <ros2neuro_data/NeuroData.hpp>
#include <ros2neuro_acquisition/Device.hpp>

namespace rosneuro {

class LSLDevice : public ros2neuro::Device {
public:
    LSLDevice(void);
    LSLDevice(NeuroFrame* frame);
    virtual ~LSLDevice(void);

    bool Configure(NeuroFrame* frame, unsigned int framerate) override;
    bool Setup(void) override;
    bool Open(void) override;
    bool Close(void) override;
    bool Start(void) override;
    bool Stop(void) override;
    size_t Get(void) override;
    size_t GetAvailable(void) override;

private:
    void destroy_lsl_structures(void);

private:
    lsl::stream_inlet* stream_;
    lsl::stream_info* info_;
    std::string stream_name_;
    std::string stream_type_;
    std::string lsl_type_;
    std::string lsl_name_;
    unsigned int samplerate_;
    unsigned int framerate_;
};

} // namespace rosneuro

PLUGINLIB_EXPORT_CLASS(rosneuro::LSLDevice, ros2neuro::Device)

#endif  // ROSNEURO_ACQUISITION_PLUGIN_LSLDEVICE_HPP
