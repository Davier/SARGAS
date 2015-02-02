#pragma once

#include <fstream>
#include <string>
#include <exception>

namespace sysfs {

	std::string resolveFilePath(const std::string &filepath);

	template<typename T>
	T readFile(const std::string &filepath) {
		std::ifstream file(filepath.c_str());
		if(file.fail()) {
			throw  std::ios::failure("The file " + filepath + " could not be opened.");
		}
		T ret;
		file >> ret;
		return ret;
	}

	template<typename T>
	void writeFile(const std::string &filepath, T data) {
		std::ofstream file(filepath.c_str(), std::ios::trunc);
		if(file.fail()) {
			throw  std::ios::failure("The file " + filepath + " could not be opened.");
		}
		file << data;
	}

}; // namespace sysfs
