/**
 ******************************************************************************
 *
 * @file       main.cpp
 * @author     The SantyPilot Project, http://www.librepilot.org Copyright (C) 2015-2017.
 * @brief      offline flightlog parser main.
 *
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 */
#include <QtCore/QCoreApplication>
#include <QDir>
#include <QFile>
#include <QString>
#include <QStringList>
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <vector>
#include <string>
#include <cstring>
#include <unistd.h>
#include <chrono>
#include <ctime>
#include <set>
#include <iomanip>
#include "debuglogentry.h"
#include "components/ExtendedDebugLogEntry.h"
#include "components/EKFLogAnalyzer.h"
#include "components/StyConfiguration.h"
#include "uavobjectmanager.h"
#include "uavobjectsinit.h"
#include "uavobjectparser.h"

#define RETURN_ERR_USAGE 1
#define RETURN_ERR_XML   2
#define RETURN_OK        0

#define DEBUGLOGENTRY_OBJID DebugLogEntry::OBJID
#define LOG_GET_FLIGHT_OBJID(x) ((DEBUGLOGENTRY_OBJID & ~0xFF) | (x & 0xFF))

using santypilot_gcs::flightlogparser::ExtendedDebugLogEntry;
using santypilot_gcs::flightlogparser::EKFLogAnalyzer;

UAVObjectManager g_mgr;
std::map<std::string, std::string> g_opts;

void objectFilename(uint32_t obj_id, uint16_t obj_inst_id, uint8_t *filename) {
    uint32_t prefix = obj_id + (obj_inst_id / 256) * 16; 
    uint8_t suffix  = obj_inst_id & 0xff;
	// 1 obj_inst_id around 200B
	// uint32_t prefix = obj_id;
	// uint8_t suffix = (obj_inst_id / 256) * 16;
    snprintf((char *)filename, 13, "%08X.o%02X", prefix, suffix);
}

/* read parse and output functions */
int32_t load_obj(const std::string& dir,
		        uint32_t obj_id, 
				uint16_t obj_inst_id, 
				char *obj_data, 
				uint16_t obj_size) {
    uint8_t filename[14];

    // Get filename
    objectFilename(obj_id, obj_inst_id, filename);
	// std::cout << "loading output object from file " << filename << std::endl;

    // Open file
	const std::string fullpath = dir + "/" + 
		std::string((char*)filename);
	FILE *fptr = fopen(fullpath.c_str(), "rb");
	if (fptr == nullptr) {
		std::cout << "open file " << filename << " failed\n";
	    return -1;
	}
    // Load object
	size_t b = fread(obj_data, sizeof(char), obj_size, fptr);
	if (b != obj_size) {
		std::cout << "read file " << filename << 
			" failed, should get " << obj_size << " but get " << b << std::endl;
	    return -2;
	}
	fclose(fptr);
	// fseek
    return 0;
}

enum FILTER_TYPE {
    TOP_K = 0,
	DOWN_SAMPLE,
	UNKNOWN_TYPE
};

struct CtrlInfo {
    FILTER_TYPE type;
	size_t amount;
};


int32_t read_parse_uavo_log(std::vector<ExtendedDebugLogEntry*>& logs,
		const CtrlInfo& info) {
	const std::string dir = g_opts["InputDirectory:LogDir"];
	uint16_t flightnum = 0;
	// auto files = file_list(dir);
	uint16_t lognum = 0; // mock
	bool has_file = true;
	bool has_log = true;
	size_t idx = 0;

	DebugLogEntry::DataFields entry;
	entry.Flight = flightnum;
	entry.Entry = lognum;
	entry.Type = DebugLogEntry::TypeOptions::TYPE_EMPTY;
	uint16_t obj_size = sizeof(DebugLogEntry::DataFields);

	// init output csv file
	bool en_output = g_opts["Output:Enable"] == "true";
	auto now = std::chrono::system_clock::now();
	auto now_time_t = std::chrono::system_clock::to_time_t(now);
	std::tm* local_time = std::localtime(&now_time_t);
	std::ostringstream time_stream;
	time_stream << std::put_time(local_time, "%Y_%m_%d_%H_%M_%S_stylog.csv");
	std::string current_time = time_stream.str();
    QFile csvFile(QString::fromStdString(current_time));

	// std::cout << "init output file " << current_time << std::endl;
	if (en_output && !csvFile.open(QFile::WriteOnly | QFile::Truncate)) {
		std::cout << "failed to open output file\n";
		return -1;
	}
	QTextStream csvStream(&csvFile);
	quint32 baseTime = 0;
	// quint32 currentFlight = 0;
	csvStream << "Flight" << '\t' << "Flight Time"
		<< '\t' << "Entry" << '\t' << "Data" << '\n';

	while (has_file) {
		// 1. set flight num and flightobj instance
		uint32_t obj_id = LOG_GET_FLIGHT_OBJID(flightnum);
	    char buffer[obj_size];

		// 2. cycle flight instance id
		while (has_log) {
		    uint16_t obj_inst_id = lognum;
			int32_t res = load_obj(dir, obj_id, obj_inst_id, buffer, obj_size);
			if (res < 0) {
				if (lognum == 0) {
				    has_file = false; // no log in this flight index
				}
				break;
			}
			// see UAVObjectField for
			memcpy((char*)&entry, buffer, obj_size);

			/*
			std::cout << "entry flight time: " << entry.FlightTime << std::endl;
			std::cout << "entry object id: " << entry.ObjectID << std::endl;
			std::cout << "entry flight: " << entry.Flight << std::endl;
			std::cout << "entry entry: " << entry.Entry << std::endl;
			std::cout << "entry InstanceID: " << entry.InstanceID << std::endl;
			std::cout << "entry Size: " << entry.Size << std::endl;
			// std::cout << "entry Type: " << entry.Type << std::endl;
			std::cout << (entry.Type == DebugLogEntry::TYPE_EMPTY) << "-empty "
					  << (entry.Type == DebugLogEntry::TYPE_TEXT) << "-text "
					  << (entry.Type == DebugLogEntry::TYPE_UAVOBJECT) << "-uavo "
					  << (entry.Type == DebugLogEntry::TYPE_MULTIPLEUAVOBJECTS) << "-mul\n";
					  */
			// next cycle		  
			lognum++;

			// log output entry
			ExtendedDebugLogEntry* ex_entry = new ExtendedDebugLogEntry;
			ex_entry->setData(entry, &g_mgr);
			if (en_output) {
				ex_entry->toCSV(&csvStream, baseTime);
			}
			
			if (DebugLogEntry::TYPE_MULTIPLEUAVOBJECTS == ex_entry->getType()) {
				const quint32 total_len  = sizeof(DebugLogEntry::DataFields);
				const quint32 data_len   = sizeof(((DebugLogEntry::DataFields *)0)->Data);
				const quint32 header_len = total_len - data_len;

				DebugLogEntry::DataFields fields;
				quint32 start = ex_entry->getData().Size;

				// cycle until there is space for another object
				while (start + header_len + 1 < data_len) {
					memset(&fields, 0xFF, total_len);
					auto tmp = ex_entry->getData().Data[start];
					memcpy(&fields, &tmp, header_len);
					// check wether a packed object is found
					// note that empty data blocks are set as 0xFF in 
					// flight side to minimize flash wearing
					// thus as soon as this read outside of used area,
					// the test will fail as lenght would be 0xFFFF
					quint32 toread = header_len + fields.Size;
					if (!(toread + start > data_len)) {
						memcpy(&fields, &tmp, toread);
						ExtendedDebugLogEntry* subEntry = new ExtendedDebugLogEntry;
						subEntry->setData(fields, &g_mgr);
						if (en_output) {
							ex_entry->toCSV(&csvStream, baseTime);
						}
					    // logs.emplace_back(subEntry);
						delete subEntry;
					}
					start += toread;
				}
			} // multi objs
			switch (info.type) {
				case FILTER_TYPE::DOWN_SAMPLE: 
					if (idx % info.amount == 0) { // managed outside
						logs.emplace_back(ex_entry);
					} else {
						delete ex_entry;
					}
					break;
				case FILTER_TYPE::TOP_K:
					if (idx < info.amount) {
						logs.emplace_back(ex_entry);
					} else {
					    delete ex_entry;
						has_log = false;
						has_file = false;
					}
					break;
				default:
					break;
			}
			idx++;
			// delete ex_entry;
		} // has log 
		if (!has_file) {
			std::cout << "file read end!\n";
			break;
		}
		lognum = 0; // reset
		flightnum++;
	} // has_file
	csvStream.flush();
	csvFile.flush();
	csvFile.close();
	return 0;
}

void freeExtendedDebugLogEntry(std::vector<ExtendedDebugLogEntry*>& logs) {
    for (auto& log: logs) {
	    if (log != nullptr) {
		    delete log;
			log = nullptr;
		}
	}
	return;
}

std::string read_content(const std::string& filename) {
	FILE *fptr = fopen(filename.c_str(), "rb");
	if (fptr == nullptr) {
		// std::cout << "open file " << filename << " failed\n";
	    return "";
	}
	const size_t BUFFER_SIZE = 1024 * 5;
	char buffer[BUFFER_SIZE]; // 5KB
	size_t b = fread(buffer, sizeof(char), BUFFER_SIZE, fptr);
    fclose(fptr);
	std::string str(buffer, b);
    return str;
}

int main(int argc, char** argv) {
	components::StyConfiguration config;
	std::string fn = 
		"/home/santy/source/SantyPilot/ground/flightlogparser/configuration.xml";
	auto content = read_content(fn);
	g_opts = config.parseXML(content);
	/*
	for (auto& kv: g_opts) {
	    std::cout << kv.first << " " << kv.second << std::endl;
	}
	*/
	// 1. collect file list
    QStringList filters     = QStringList("*.*");
	std::string log_dir = g_opts["InputDirectory:LogDir"];
    QDir path = QDir(QString::fromStdString(log_dir));
    path.setNameFilters(filters);
    QFileInfoList files = path.entryInfoList(); // sort
	// std::cout << "get log file list: " << files.size() << std::endl;

	size_t input_num = std::stoul(g_opts["Analyze:InputNum"]);
	size_t down_sample_rate = files.size() / input_num;
	down_sample_rate = (down_sample_rate != 0) ? down_sample_rate : 1;
	CtrlInfo info;
	// TODO: use macro
	info.type = g_opts["Analyze:InputType"] == "TOPK" ? TOP_K: DOWN_SAMPLE;
	info.amount = input_num;
	// 2. parse and build reference trees, used to parse logs
    filters     = QStringList("*.xml");
	auto xml_dir = g_opts["Analyze:XMLDir"];
    path = QDir(QString::fromStdString(xml_dir));
    path.setNameFilters(filters);
    auto xmls = path.entryInfoList();
    UAVObjectParser *parser = new UAVObjectParser();
	for (auto& xml: xmls) {
		std::string fn = xml_dir + "/" + xml.fileName().toStdString();
		auto xmlstr = read_content(fn);
		auto status = parser->parseXML(xmlstr, fn);
	}
	auto sz = parser->getNumObjects();
	// std::cout << "num of objects: " << sz << std::endl;
	UAVObjectsInitialize(&g_mgr);
	// 3. parse and collect logs
	std::vector<ExtendedDebugLogEntry*> logs;
    read_parse_uavo_log(logs, info);
	// 4. analyze logs 
	bool en_analyze = g_opts["Analyze:Enable"] == "true";
	if (!en_analyze) {
	    return 0;
	}
	EKFLogAnalyzer analyzer;
	std::set<std::string> table {};
	std::string nsn = "Analyze:InputObject:Name";
	std::string nsf = "Analyze:InputObject:Field";
	size_t idx = 0;
	while (true) {
		auto key = nsf + std::to_string(idx);
	    if (g_opts.count(key) <= 0) {
		    break;
		}
		table.insert(g_opts[nsn] + ":" + g_opts[key]);
		idx++;
	}
	analyzer.init(table);
	analyzer.process(parser->getObjectInfo(), logs);
	analyzer.analyze();
	/*
	const auto& data = analyzer.getData();
	for (auto &kv: data) {
		std::cout << kv.first << " data len: " << kv.second.size() << std::endl;
		size_t bidx = 0;
		for (auto& d: kv.second) {
		    std::cout << d.first << ":" << d.second << " ";
			bidx++;
			if (bidx % 3 == 0) {
				std::cout << std::endl;
			}
		}
	}
	*/
	// 5. free mem
	freeExtendedDebugLogEntry(logs);
	return 0;
}
