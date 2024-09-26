#ifndef _STY_SYSTEM_OPS_H
#define _STY_SYSTEM_OPS_H

#include <string>

#if defined(__WIN32__)
#include <direct.h> // ::_mkdir ::_rmdir
#include <io.h>  // ::_access
#elif defined(__LINUX__)
#include <sys/stat.h>
#include <sys/types.h>
#endif

#include <cstdio>
#include <iostream>  
#include <fstream>

namespace components {
class ISSystemOps {
public:
#if defined(__WIN32__)
	static void create_dir(const std::string& dir) {
		::_mkdir(dir.c_str());
	}
	static void delete_dir(const std::string& dir) {
		::_rmdir(dir.c_str());
	}
	static bool file_exist(const std::string& path) {
		return (::_access(path.c_str(), 0) != -1);
	}
	static bool create_file(const std::string& path) {
		std::fstream fs(path.c_str(), std::ios::out); // a more graceful way?
		return true;
	}
	static bool remove_file(const std::string& path) { // true for success to remove
		return ::remove(path.c_str()) == 0;
	}
#elif defined(__LINUX__)
	static void create_dir(const std::string& dir) {
		mkdir(dir.c_str());
	}
	static void delete_dir(const std::string& dir) {
	}
	static bool file_exist(const std::string& path) {
		return false;
	}
	static bool remove_file(const std::string& path) { // true for success to remove
		return false;
	}
#endif
	static bool create_file(const std::string& path) {
		std::fstream fs(path.c_str(), std::ios::out); // a more graceful way?
		return true;
	}
};
} // components 

#endif // _STY_SYSTEM_OPS_H
