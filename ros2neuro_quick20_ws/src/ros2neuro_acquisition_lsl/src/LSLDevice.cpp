#include "LSLDevice.hpp"

namespace rosneuro {

LSLDevice::LSLDevice(void) : ros2neuro::Device() {
    this->stream_     = nullptr;
    this->info_       = nullptr;
    this->framerate_  = 0;
    this->samplerate_ = 0;
}

LSLDevice::LSLDevice(NeuroFrame* frame) : ros2neuro::Device(frame) {
    this->stream_     = nullptr;
    this->info_       = nullptr;
    this->framerate_  = 0;
    this->samplerate_ = 0;
}

LSLDevice::~LSLDevice(void) {
    this->destroy_lsl_structures();
}

void LSLDevice::destroy_lsl_structures(void) {
    if (this->stream_ != nullptr) {
        delete this->stream_;
        this->stream_ = nullptr;
    }

    if (this->info_ != nullptr) {
        delete this->info_;
        this->info_ = nullptr;
    }
}

bool LSLDevice::Configure(NeuroFrame* frame, unsigned int framerate) {
    RCLCPP_INFO(rclcpp::get_logger("LSLDevice"), "Configuring LSLDevice");

    this->stream_name_ = this->get_parameter("lsl_name", "Quick20");
    this->stream_type_ = this->get_parameter("lsl_type", "EEG");

    this->framerate_   = framerate;

    RCLCPP_INFO(rclcpp::get_logger("LSLDevice"),
                "LSL stream configuration: name = %s, type = %s",
                this->stream_name_.c_str(), this->stream_type_.c_str());

    this->data_  = frame;
    this->nch_   = this->data_->layout.channels;
    this->nsamp_ = this->data_->layout.samples;

    return true;
}

bool LSLDevice::Setup(void) {
    RCLCPP_INFO(rclcpp::get_logger("LSLDevice"), "Setting up LSLDevice");
    return true;
}

bool LSLDevice::Open(void) {
    RCLCPP_INFO(rclcpp::get_logger("LSLDevice"), "Opening LSLDevice");

    try {
        RCLCPP_INFO(rclcpp::get_logger("LSLDevice"), "Resolving LSL stream...");
        std::vector<lsl::stream_info> results = lsl::resolve_stream("name", this->stream_name_, 1, 5.0);

        if (results.empty()) {
            RCLCPP_ERROR(rclcpp::get_logger("LSLDevice"), "No LSL stream found with name: %s", this->stream_name_.c_str());
            return false;
        }

        this->info_   = new lsl::stream_info(results[0]);
        this->stream_ = new lsl::stream_inlet(*this->info_);

        this->samplerate_ = static_cast<unsigned int>(this->info_->nominal_srate());

        RCLCPP_INFO(rclcpp::get_logger("LSLDevice"), "Connected to LSL stream: %s", this->stream_name_.c_str());
        RCLCPP_INFO(rclcpp::get_logger("LSLDevice"), "LSL stream sample rate: %u", this->samplerate_);

        return true;
    } catch (std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("LSLDevice"), "Exception while opening LSL stream: %s", e.what());
        return false;
    }
}

bool LSLDevice::Close(void) {
    RCLCPP_INFO(rclcpp::get_logger("LSLDevice"), "Closing LSLDevice");
    this->destroy_lsl_structures();
    return true;
}

bool LSLDevice::Start(void) {
    RCLCPP_INFO(rclcpp::get_logger("LSLDevice"), "Starting LSLDevice");
    return true;
}

bool LSLDevice::Stop(void) {
    RCLCPP_INFO(rclcpp::get_logger("LSLDevice"), "Stopping LSLDevice");
    return true;
}

size_t LSLDevice::GetAvailable(void) {
    return 0;
}

size_t LSLDevice::Get(void) {
    if (this->stream_ == nullptr || this->data_ == nullptr)
        return 0;

    std::vector<std::vector<float>> raw;
    double ts = this->stream_->pull_chunk(raw);

    if (raw.empty())
        return 0;

    unsigned int nsamples = raw[0].size();
    unsigned int nchan    = raw.size();

    if (nchan != this->nch_) {
        RCLCPP_ERROR(rclcpp::get_logger("LSLDevice"),
                     "Channel mismatch: expected %u, got %u", this->nch_, nchan);
        return 0;
    }

    for (unsigned int s = 0; s < nsamples; ++s) {
        for (unsigned int c = 0; c < nchan; ++c) {
            this->data_->eeg.data[c][s] = raw[c][s];
        }
    }

    this->data_->header.stamp = rclcpp::Clock().now();
    this->data_->header.seq++;
    this->data_->header.nsamples = nsamples;

    return nsamples;
}

} // namespace rosneuro
