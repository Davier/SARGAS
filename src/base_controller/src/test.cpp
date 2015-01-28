#include <exception>
#include <iostream>
#include <string>

#include "SysFsHelper.hpp"

using namespace std;
using namespace sysfs;

int main() {
	try {
		// Do "cd ~;echo 1337 > test" to make it work !
		string file = resolveFilePath("/home/*/test");
		// Print the found filepath
		cout << "Path : " << file << endl;
		// Read and print an int value
		cout << "Old value (int) : " << readFile<int>(file) << endl; // It can be converted to anything : int, float, string, ...
																	 // Write a float
		writeFile(file, 4.2f);
		// Read and print a float
		cout << "New value (float) : " << readFile<float>(file) << endl;
	}
	catch (exception &e) {
		cerr << "Exception : " << e.what() << endl;
	}
	return 0;
}