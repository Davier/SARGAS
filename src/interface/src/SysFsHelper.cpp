#include "SysFsHelper.hpp"

#include <exception>
#include <string>
#include <cstdio>
#include <cstring>

using namespace std;

namespace sysfs {

	string resolveFilePath(const string &filepath) {
		// Execute the command ls to find the correct file path
		const string command = "/bin/ls " + filepath;
		FILE *fp = popen(command.c_str(), "r");
		if (!fp) {
			throw ios::failure("Could not execute /bin/ls");
		}
		// Read only the first line
		const size_t buffer_size = 1024;
		char read_buffer[buffer_size];
		if (!fgets(read_buffer, buffer_size, fp)) {
			pclose(fp);
			throw ios::failure("The file " + filepath + " does not exists.");
		}
		// Remove the trailing line feed
		const size_t path_size = strlen(read_buffer);
		if (path_size > 0 && read_buffer[path_size - 1] == '\n') {
			read_buffer[path_size - 1] = '\0';
		}
		pclose(fp);
		return std::string(read_buffer);
	}

	// For bool, we want to write and read "1" or "0", not "true" or "false"
	template<>
	bool readFile<bool>(const std::string &filepath) {
		return readFile<int>(filepath);
	}
	template<>
	void writeFile<bool>(const std::string &filepath, bool data) {
		if (data) {
			writeFile(filepath, "1");
		}
		else {
			writeFile(filepath, "0");
		}
	}

}; // namespace sysfs