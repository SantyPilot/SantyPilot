/**
 * @file: LogAnalyzer.h
 * @brief: base class for LogEntry Analyze
 * @author: zhangxin@santypilot
 * @date: 2024-3-17
 */
#ifndef _LOG_ANALYZER_H
#define _LOG_ANALYZER_H

#include "ExtendedDebugLogEntry.h"
#include "uavobjectparser.h" // ObjectInfo
#include <QList>
#include <vector>
#include <string> // std::stod

namespace santypilot_gcs {
namespace flightlogparser {

class LogAnalyzer {
public:
	using DataType = 
		std::map<std::string, std::vector<std::pair<size_t, float>>>;
	bool init(const std::set<std::string>& obj_fields) {
		_obj_fields.clear();
		for (auto& obj_field: obj_fields) {
		    const auto &kv = split(obj_field, ':');
			if (_obj_fields.count(lowercase(kv.front())) <= 0) {
			    _obj_fields[lowercase(kv.front())] = { kv.back() };
			} else {
			    _obj_fields[lowercase(kv.front())].emplace_back(kv.back());
			}
		}
		return true;
	}

	void process(const QList<ObjectInfo *>& info,
		std::vector<ExtendedDebugLogEntry*>& logs) {
		std::map<std::string, std::string> name_unit;
		for (auto i = 0; i < info.count(); i++) {
			if (info.at(i) == nullptr) {
			    continue;
			}
			const auto& name = info.at(i)->namelc.toStdString();
			if (_obj_fields.count(name) <= 0) {
			    continue;
			}
			const auto& fields = info.at(i)->fields;
			// map name to unit string
			for (auto j = 0; j < fields.count(); j++) {
				auto* field_info = fields.at(j);
				if (field_info == nullptr) {
				    continue;
				}
				name_unit[field_info->name.toStdString()]
					= field_info->units.toStdString();
			}
		}
		/*
		for (auto& kv: name_unit) {
			std::cout << "name-unit kv" << kv.first << " " 
				<< kv.second << std::endl;
		}
		for (auto& kv: _obj_fields) {
		    std::cout << "interested fields: " << kv.first << " "
				<< kv.second << std::endl;
		}
		*/
		for (auto* entry: logs) {
		    auto data = entry->getLogData();
			double time = entry->getFlightTime(); // double(1e6); us->s
			if (data.empty()) {
			    continue;
			}
			auto tokens = split(data, ' ');
			auto key = lowercase(trim_spaces(tokens.front()));
			if (_obj_fields.count(key) <= 0) {
			    continue; // not interested
			}

			/*
			std::cout << tokens.size() << "----" 
				<< tokens.front() << std::endl;
			for (auto i = 0; i < tokens.size(); i++) {
			    std::cout << tokens[i] << " ";
			}
			std::cout << std::endl;
			*/

			size_t idx = 0;
			for (; idx < tokens.size(); idx++) {
			    if (tokens[idx] == "Data:") {
					idx++;
				    break;
				}
			}
			if (idx == tokens.size()) {
			    continue; // invalid
			}
			auto &fields = _obj_fields[key]; // fetch fields list
			while (idx < tokens.size()) {
				try {
					// 1. find field
					auto field = trim_spaces(tokens[idx]);
					if (field.empty()) {
					    idx++;
						continue;
					}
					field = field.substr(0, field.size() - 1); // remove :
					size_t table_idx = 0;
					for (; table_idx < fields.size(); table_idx++) {
					    if (fields[table_idx] == field) {
						    break;
						}
					}
					if (table_idx == fields.size()) { // never find
					    idx++;
						continue;
					}
					// 2. find val, check space?
					// idx + 1; // [
					double val = std::stod(tokens[idx + 2]); // opt this type, skip [
					std::string xaxis = key + ":" + field;
					if (_data.count(xaxis) > 0) {
					    _data[xaxis].emplace_back(std::make_pair(time, val));
					} else {
					    _data[xaxis] = { {time, val} };
					}
					// idx + 3; // ]
					// idx + split(name_unit, ' '); // unit
					// finish this field
					auto unit_str = split(name_unit[field], ' ');
					idx += 3 + unit_str.size();
				} catch (std::invalid_argument& e) {
				    std::cout << tokens[idx + 2] << 
						" can not be converted to double\n";
					exit(1);
				}
			} // tokens
		} // logs
	}

	void plot() {
	    if (_data.size() == 0) {
		    return;
		}
	}

	virtual void analyze() = 0;

	const DataType& getData() {
	    return _data; // us->data
	}
protected:
	std::vector<std::string> split(const std::string& raw, const char& c) {
		std::vector<std::string> ret;
		if (raw.empty()) {
		    return ret;
		}
		size_t l = 0, r = 0;
		while (true) {
			r = raw.find_first_of(c, l);
			ret.push_back(raw.substr(l, r - l));
			if (r == std::string::npos) {
				return ret;
			}
			l = r + 1;
		}
	}

	std::string lowercase(const std::string& raw) {
	    std::string str = raw;
		std::transform(str.begin(), str.end(), str.begin(),
				[](unsigned char c){ return std::tolower(c); });
		return str;
	}

	std::string trim_spaces(const std::string& raw) {
	    std::string str = raw;
		auto st = str.find_first_not_of(' ');
		if (st == std::string::npos) {
		    return "";
		}
		auto ed = str.find_last_not_of(' ');
		return str.substr(st, ed - st + 1);
	}
protected:
	std::map<std::string, std::vector<std::string>> _obj_fields;
	DataType _data;
}; // LogAnalyzer 
} // fliglogparser
} // santypilot_gcs

#endif // _LOG_ANALYZER_H
