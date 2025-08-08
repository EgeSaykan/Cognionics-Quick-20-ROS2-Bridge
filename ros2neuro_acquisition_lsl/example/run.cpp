#include <thread>
#include <iostream>
#include <lsl_cpp.h>
#include <string>
#include <math.h>

void printChunk(const std::vector<float>& chunk, std::vector<double>& ts, std::size_t n_channels) {
	unsigned int ns = ts.size();
	
	unsigned int i = 0;
	for(auto u = 0; u < ns; u++) {
		printf("[%2d][%f] ", u, ts[u]);
		for(i = n_channels*u; i < n_channels*(u+1); i++) {
			std::cout << "(" <<i<<") " <<chunk[i] << ' ';
		}
		std::cout << '\n';
	}
}

void printChunk(const std::vector<float>& chunk, std::size_t n_channels) {
	for(std::size_t i=0; i < chunk.size(); ++i) {
		std::cout << chunk[i] << ' ';
		if (i % n_channels == n_channels - 1)
			std::cout << '\n';
	}
}

int main(int argc, char* argv[]) {
	std::string field, value;
	const int max_samples = argc > 3 ? std::stoi(argv[3]) : 10;
	if (argc < 3) {
		std::cout << "This connects to a stream which has a particular value for a "
		             "given field and receives data.\nPlease enter a field name and the desired "
		             "value (e.g. \"type EEG\" (without the quotes)):"
		          << std::endl;
		std::cin >> field >> value;
	} else {
		field = argv[1];
		value = argv[2];
	}

	std::cout << "Now resolving streams..." << std::endl;
	std::vector<lsl::stream_info> results = lsl::resolve_stream(field, value);
	std::cout << "Here is what was resolved: " << std::endl;
	std::cout << results[0].as_xml() << std::endl;
	std::cout << "Now creating the inlet..." << std::endl;
	lsl::stream_inlet inlet(results[0]);
	std::cout << "Now pulling samples..." << std::endl;

	lsl::stream_info inf = inlet.info();
	double sr = inf.nominal_srate();
	double fr = 10;
	double ns = sr/fr;
	int sleep = (int)1000/ns;
	unsigned int nchans = inlet.get_channel_count();
	
	std::cout<< "Nominal rate: " << sr << " Hz" << std::endl;
	std::cout<< "Frame rate: "   << fr << " Hz" << std::endl;
	std::cout<< "Number of samples: "   << ns <<std::endl;
	std::cout<< "Sleep time: "   << sleep << " ms" << std::endl;

	std::vector<float> vsample(ns*nchans);
	std::vector<double> timestamp(ns);

	for(int i=0; i < max_samples; ++i) {
		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		size_t size = ns*nchans;
		size_t recs;

		recs = inlet.pull_chunk_multiplexed(vsample.data(), timestamp.data(), size, ns, LSL_FOREVER );

		printChunk(vsample, timestamp, inlet.get_channel_count());
	}

	if(argc == 1) {
		std::cout << "Press any key to exit. " << std::endl;
		std::cin.get();
	}
	return 0;
}
