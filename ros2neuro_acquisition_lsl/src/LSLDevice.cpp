#ifndef ROSNEURO_ACQUISITION_PLUGIN_LSLDEVICE_CPP
#define ROSNEURO_ACQUISITION_PLUGIN_LSLDEVICE_CPP

#include "rosneuro_acquisition_lsl/LSLDevice.hpp"

namespace rosneuro {
    LSLDevice::LSLDevice(void) : Device() {
        this->name_ = "lsldev";
        this->stream_ = nullptr;
        this->info_   = nullptr;

        this->stream_name_ = "unknown";
        this->stream_type_ = "unknown";
    }

    LSLDevice::LSLDevice(NeuroFrame* frame) : Device(frame) {

        this->name_ = "lsldev";
        this->stream_ = nullptr;
        this->info_   = nullptr;

        this->stream_name_ = "unknown";
        this->stream_type_ = "unknown";
    }

    LSLDevice::~LSLDevice(void) {
        this->destroy_lsl_structures();
    }

    bool LSLDevice::Configure(NeuroFrame* frame, unsigned int framerate) {
        this->frame_	 = frame;
        this->framerate_ = framerate;

        if(!ros::param::get("~stream_type", this->stream_type_)) {
            ROS_ERROR("Missing mandatory 'stream_type' parameter in the server.");
            return false;
        }

        if(!ros::param::get("~stream_name", this->stream_name_)) {
            ROS_ERROR("Missing mandatory 'stream_name' parameter in the server.");
            return false;
        }

        return true;
    }

    bool LSLDevice::Open(void) {
        std::vector<lsl::stream_info> results;
        results = lsl::resolve_stream("name", this->stream_name_, 1, 1);

        if (results.empty()) {
            ROS_ERROR("Stream with name '%s' not found in the current streams", this->stream_name_.c_str());
            return false;
        }

        if(this->stream_type_.compare(results[0].type()) != 0) {
            ROS_ERROR("Stream with type '%s' not found in the current streams", this->stream_type_.c_str());
            return false;
        }

        this->stream_ = new lsl::stream_inlet(results[0]);
        this->info_   = new lsl::stream_info(this->stream_->info());

        return true;
    }

    bool LSLDevice::Setup(void) {
        size_t ns;
        size_t neeg, nexg, ntri;
        unsigned int sampling_rate;
        NeuroDataInfo *ieeg, *iexg, *itri;

        this->samplerate_ = (unsigned int)this->info_->nominal_srate();
        this->frame_->sr = this->samplerate_;

        if(this->info_->nominal_srate() == lsl::IRREGULAR_RATE) {
            ROS_ERROR("LSL plugin does not support irregular rate data stream");
            return false;
        }

        ns = (size_t)(this->samplerate_/this->framerate_);
        neeg = this->info_->channel_count();
        nexg = 0;
        ntri = 0;

        this->frame_->eeg.reserve(ns, neeg);
        this->frame_->exg.reserve(ns, nexg);
        this->frame_->tri.reserve(ns, ntri);

        this->frame_->eeg.info()->unit = "uV";
        this->frame_->eeg.info()->transducter = "n/a";
        this->frame_->eeg.info()->prefiltering = "n/a";
        this->frame_->eeg.info()->minmax[0] = std::numeric_limits<float>::lowest();
        this->frame_->eeg.info()->minmax[1] = std::numeric_limits<float>::max();
        this->frame_->eeg.info()->isint = 0;

        this->frame_->eeg.info()->labels.clear();
        for (auto i = 0; i<neeg; i++)
            this->frame_->eeg.info()->labels.push_back(std::string("eeg:"+std::to_string(i)));

        ROS_INFO("'%s' device correctly configured with samplerate=%d Hz", this->GetName().c_str(), this->samplerate_);

        return true;
    }

    bool LSLDevice::Start(void) {
        try {
            this->stream_->open_stream(1);
        } catch (const std::runtime_error& e) {
            ROS_ERROR("Cannot start the device '%s': %s", this->GetName().c_str(), e.what());
            return false;
        }
        return true;
    }

    bool LSLDevice::Stop(void) {
        std::cerr<<"[Warning] - Stop function undefined for '"
                 << this->GetName() <<"' device. The stream will not be closed."<<std::endl;
        return true;
    }

    bool LSLDevice::Close(void) {
        this->stream_->close_stream();
        return true;
    }

    size_t LSLDevice::Get(void) {
        size_t ns = this->frame_->eeg.nsamples() * this->frame_->eeg.nchannels();
        size_t size = this->stream_->pull_chunk_multiplexed(this->frame_->eeg.data(), nullptr, ns, 0, 1);

        if (size != ns) {
            ROS_ERROR("Corrupted data reading: unexpected chunk size: %zu", size);
        }
        return size;
    }

    size_t LSLDevice::GetAvailable(void) {
        return this->stream_->samples_available();
    }

    void LSLDevice::destroy_lsl_structures(void) {
        if(this->stream_ != nullptr) {
            delete this->stream_;
        }

        this->stream_ = nullptr;

        if(this->info_ != nullptr) {
            delete this->info_;
        }

        this->info_ = nullptr;
    }
}

#endif
