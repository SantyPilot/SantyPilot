/**
 * @file: StyIO.h
 * @brief io implementation, cross-platform support
 * @author zhangxin
 * @date 2023-3-28
 */
#include <fstream>
#include <string.h> // memcpy
#include "StyIO.h"
#include "StyString.h"
#include "StyLog.h"
#include "StySystemOps.h"

namespace components {
FileFormat::~FileFormat() {
	if (!_fs.is_open()) {
		return;
	}
	_fs.close();
}

size_t FileFormat::fileSize() {
	if (!_fs.is_open()) {
		return 0;
	}
	return _fs.tellg(); // must init as std::ios::ate
}

void FileFormat::reset() {
	if (!_fs.is_open()) {
		return;
	}
	_fs.seekg(0); // rewind 
}

void FileFormat::close() {
	_fs.close();
}

// -- text file
TextFileFormat::~TextFileFormat() {}

bool TextFileFormat::init(const IOContext& ctx) {
	_ctx = ctx;
	_fs.open(ctx.filename, std::ios::in | std::ios::out | 
		std::ios::binary | std::ios::app);
	// create file if not exists
	return true;
}

void TextFileFormat::read(char* buffer, size_t& len) {
	std::vector<std::string> lines;
	lines.resize(0);
	std::string raw;
	while (std::getline(_fs, raw)) {
		lines.push_back(raw);
	}
	_fs.seekg(0); // rewind 
	auto ret = components::join_string(lines, '\n');
	len = ret.size();
	memcpy(buffer, &ret[0], len);
}

void TextFileFormat::write(const char*, const size_t&) {
	return;
}

// TODO:
const std::string& TextFileFormat::readLine(const std::string& context) {
	std::string ret;
	return ret;
}

const std::string& TextFileFormat::readBytes(char* buffer, const size_t& len) {
	// has status operations
	// refer to https://cplusplus.com/reference/istream/istream/seekg/
    _fs.read(buffer, len);
	std::string ret(buffer, len);
	return ret;
}

// -- binary file
BinFileFormat::~BinFileFormat() {};

bool BinFileFormat::init(const IOContext& ctx) {
	_ctx = ctx;
	return true;
}

void BinFileFormat::read(char* buffer/* user must malloc enough space */, size_t& len) {
	const size_t LEN = BLOCK_SIZE;
	char head[LEN] = "";
	len = 0;
	_fs.open(_ctx.filename, std::ios::in | std::ios::binary);
	if (!_fs.is_open()) {
		LOG(ERROR) << "Open File " << _ctx.filename << " failed!";
		return;
	}
	while (_fs.read(head, LEN)) { // make it to read
		auto bytes_read = _fs.gcount();
		len += bytes_read;
		memcpy(buffer, head, bytes_read);
		if (bytes_read != LEN) {
			break;
		}
	}
	auto bytes_read = _fs.gcount();
	len += bytes_read;
	memcpy(buffer, head, bytes_read);
	_fs.close(); // stupid
	return;
}

void BinFileFormat::write(const char* buffer, const size_t& len) {
	_fs.open(_ctx.filename, std::ios::out | std::ios::binary);
	if (!_fs.is_open()) {
		return;
	}
	_fs.write(buffer, len);
	_fs.close();
	return;
}

} // components
