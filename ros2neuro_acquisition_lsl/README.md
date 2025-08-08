# ROSNeuro Acquisition LSL Plugin

The **LSL** plugin allows to acquire neurodata from LabStreaming Layer (LSL) streams. The plugin relies on [libsls 1.13.0](https://github.com/sccn/labstreaminglayer) that needs to be installed. Notice that the LSL plugin for rosneuro_acquisition allows to read one stream at the time with one data type. Furthermore, it is not possible to set the samplerate of stream from the plugin (LSL must be configured with the desired sampling rate). Finally, the timestamp of the LSL stream is not forwarded and it is replaced by the ROS timestamp.

**LSL plugin is still in beta version. Furthermore, since Ubuntu does not list liblsl package, this plugin is not included in ros-noetic-rosneuro-full**

## Usage
To launch the rosneuro_acquisition with the LSL plugin, first it is required to set the `plugin` parameter to `rosneuro::LSLDevice`, then to provide the desired `framerate`. Notice that these are the two mandatory parameters for every plugin of rosneuro_acquisition.

### Plugin-specific parameters:
~<name>/`stream_type` (`std::string`) [**mandatory**]
	
  The data type provided by LSL stream (e.g., "EEG")

~<name>/`stream_name` (`std::string`) [**mandatory**]
  
  The name of the LSL stream to connect (e.g., "obci_eeg1")
  
## Installing *liblsl*
The LSL library can be installed following the official [instructions](https://github.com/sccn/labstreaminglayer). Please, notice that the required version is 1.13.0. 

**Trick.** More recent versions of LSL may work. However, starting from version 1.14.0, the liblsl library changed name (from `liblsl64.so` to `libslsl.so`). To make it work with the rosneuro LSL plugin, create a symbolic link to the new library, for example:
```bash
sudo ln -s /usr/lib/liblsl.so /usr/lib/liblsl64.so
```
