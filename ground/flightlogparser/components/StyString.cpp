/**
 * @file STYString.cpp
 * string utils
 * @author: santypilot 
 * @date: 2024-1-29
 */
#include <vector>
#include "StyLog.h"
#include "StyString.h"

namespace components {
	const std::string join_string(const std::vector<std::string>& list, char split) {
		std::string ret = "";
		for (auto i = 0; i < list.size(); i++) {
			ret += list[i];
			if (i != list.size() - 1) {
				ret += split;
			}
		}
		return ret;
	}

	const std::vector<std::string> split_string(const std::string& raw, char split) {
		std::vector<std::string> ret;
		if (raw.empty()) {
			return ret;
		}
		size_t l = 0, r = 0;
		while (true) {
			r = raw.find_first_of(split, l);
			auto seg = raw.substr(l, r - l);
			ret.emplace_back(std::move(seg));
			if (r == std::string::npos) {
				return ret;
			}
			l = r + 1;
			if (l >= raw.size()) {
				return ret;
			}
		}
		return ret;
	}

	bool string_2_wchar(const std::string& str, wchar_t* dst, const size_t& dst_len) {
		// size_t cvt_len = 0;
		// mbstowcs_s(&cvt_len, dst, dst_len, str.c_str(), str.size());
		mbstowcs(dst, str.c_str(), str.size());
		//if (cvt_len != cvt_len) {
		//	LOG(ERROR) << "error occurs when string convert to wchar_t!\n";
		//	return false;
		//}
		return true;
	}

	const std::string trim_spaces(const std::string& raw) {
		std::string ret = "";
		auto st = raw.find_first_not_of(' ');
		if (st == std::string::npos) {
			return ret;
		}
		auto ed = raw.find_last_not_of(' ');
		ret = raw.substr(st, ed - st + 1);
		return ret;
	}
} // components
