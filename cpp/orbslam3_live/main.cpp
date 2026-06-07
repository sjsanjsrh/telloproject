#include <chrono>
#include <cctype>
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>

#include <opencv2/opencv.hpp>

#include "slam/System.h"

namespace {

std::map<std::string, std::string> parse_args(int argc, char** argv) {
	std::map<std::string, std::string> args;
	for (int i = 1; i < argc; ++i) {
		std::string key = argv[i];
		if (key.rfind("--", 0) != 0) {
			continue;
		}
		if (i + 1 >= argc) {
			throw std::runtime_error("missing value for " + key);
		}
		args[key.substr(2)] = argv[++i];
	}
	return args;
}

std::string require_arg(const std::map<std::string, std::string>& args, const std::string& key) {
	const auto it = args.find(key);
	if (it == args.end() || it->second.empty()) {
		throw std::runtime_error("missing --" + key);
	}
	return it->second;
}

double now_seconds() {
	using clock = std::chrono::steady_clock;
	static const auto start = clock::now();
	const auto elapsed = clock::now() - start;
	return std::chrono::duration<double>(elapsed).count();
}

cv::VideoCapture open_source(const std::string& source) {
	if (source.size() == 1 && std::isdigit(static_cast<unsigned char>(source[0]))) {
		return cv::VideoCapture(std::stoi(source));
	}
	return cv::VideoCapture(source);
}

void write_pose_jsonl(std::ofstream& out, const cv::Mat& tcw, const std::string& state) {
	if (tcw.empty()) {
		out << "{\"timestamp\":" << now_seconds()
			<< ",\"tracking_state\":\"" << state << "\"}" << std::endl;
		return;
	}

	cv::Mat pose64;
	tcw.convertTo(pose64, CV_64F);
	cv::Mat twc = pose64.inv();

	out << "{\"timestamp\":" << now_seconds()
		<< ",\"tracking_state\":\"" << state << "\""
		<< ",\"position\":[" << twc.at<double>(0, 3) << "," << twc.at<double>(1, 3) << "," << twc.at<double>(2, 3) << "]"
		<< ",\"rotation_matrix\":[";
	for (int r = 0; r < 3; ++r) {
		if (r > 0) {
			out << ",";
		}
		out << "[";
		for (int c = 0; c < 3; ++c) {
			if (c > 0) {
				out << ",";
			}
			out << twc.at<double>(r, c);
		}
		out << "]";
	}
	out << "]}" << std::endl;
}

}  // namespace

int main(int argc, char** argv) {
	try {
		const auto args = parse_args(argc, argv);
		const std::string mode = args.count("mode") ? args.at("mode") : "mono";
		const std::string vocab = require_arg(args, "vocab");
		const std::string settings = require_arg(args, "settings");
		const std::string source = require_arg(args, "source");
		const std::string pose_out = require_arg(args, "pose-out");

		if (mode != "mono") {
			throw std::runtime_error("this live wrapper currently supports --mode mono");
		}

		cv::VideoCapture capture = open_source(source);
		if (!capture.isOpened()) {
			throw std::runtime_error("failed to open source: " + source);
		}

		std::ofstream pose_file(pose_out, std::ios::out | std::ios::app);
		if (!pose_file.is_open()) {
			throw std::runtime_error("failed to open pose output: " + pose_out);
		}

		ORB_SLAM3::System slam(vocab, settings, ORB_SLAM3::System::MONOCULAR, false);

		cv::Mat frame;
		while (capture.read(frame)) {
			if (frame.empty()) {
				std::this_thread::sleep_for(std::chrono::milliseconds(2));
				continue;
			}
			const double timestamp = now_seconds();
			const cv::Mat tcw = slam.TrackMonocular(frame, timestamp);
			const std::string state = tcw.empty() ? "LOST" : "OK";
			write_pose_jsonl(pose_file, tcw, state);
		}

		slam.Shutdown();
		return 0;
	} catch (const std::exception& exc) {
		std::cerr << "orbslam3_live error: " << exc.what() << std::endl;
		return 1;
	}
}
