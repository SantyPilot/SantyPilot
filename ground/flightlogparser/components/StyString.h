/**
 * @file StyString.cpp
 * string utils
 * @author: zhangxin
 * @date: 2024-1-29
 */
#ifndef _STY_STRING_H
#define _STY_STRING_H
#include <string>
#include <cwchar> // w_char
#include <cstdlib> // mbstowcs_s
#include <vector>

namespace components {
	const std::string join_string(const std::vector<std::string>& list, char split = ' ');
	bool string_2_wchar(const std::string& str, wchar_t* dst, const size_t& dst_len);
	const std::vector<std::string> split_string(const std::string& raw, char split);
	const std::string trim_spaces(const std::string& raw);
}

#endif // _STY_STRING_H
