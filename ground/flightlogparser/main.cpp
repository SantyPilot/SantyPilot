/**
 ******************************************************************************
 *
 * @file       main.cpp
 * @author     The LibrePilot Project, http://www.librepilot.org Copyright (C) 2015-2017.
 *             The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      UAVObjectGenerator main.
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
#include <QtCore/QCoreApplication>
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
#include <iomanip>
#include "debuglogentry.h"
#include "ExtendedDebugLogEntry.h"
#include "uavobjectmanager.h"
#include "uavobjectsinit.h"

#define RETURN_ERR_USAGE 1
#define RETURN_ERR_XML   2
#define RETURN_OK        0

#define DEBUGLOGENTRY_OBJID DebugLogEntry::OBJID
#define LOG_GET_FLIGHT_OBJID(x) ((DEBUGLOGENTRY_OBJID & ~0xFF) | (x & 0xFF))

using santypilot_gcs::flightlogparser::ExtendedDebugLogEntry;
/**
 * entrance
 */
const std::string LOGS_DIR = "/home/santy/source/SantyPilot/logs/";
UAVObjectManager g_mgr;

void objectFilename(uint32_t obj_id, uint16_t obj_inst_id, uint8_t *filename) {
    uint32_t prefix = obj_id + (obj_inst_id / 256) * 16; 
    uint8_t suffix  = obj_inst_id & 0xff;
	// 1 obj_inst_id around 200B
	// uint32_t prefix = obj_id;
	// uint8_t suffix = (obj_inst_id / 256) * 16;
    snprintf((char *)filename, 13, "%08X.o%02X", prefix, suffix);
}

std::vector<std::string> split(const std::string& s, char c) {
	std::vector<std::string> ret;
	int32_t idx = 0;
	while (true) {
		size_t lidx = idx;
		idx = s.find_first_of(c, idx);
		if (idx == -1) {
			break;
		}
		auto sub = s.substr(lidx, idx - lidx);
		ret.emplace_back(std::move(sub));
		idx++;
	}
	return ret;
}

std::vector<std::string> file_list(const std::string& dir) {
	std::string command = "ls " + dir;
	std::vector<std::string> ret;
	int pipefd[2];
	if (pipe(pipefd) == -1) {
		std::cout << "Failed to create pipe." << std::endl;
		return ret;
	}

	pid_t pid = fork();
	if (pid == -1) {
	    std::cout << "Fork failed." << std::endl;
		close(pipefd[0]);
		close(pipefd[1]);
		return ret;
	} else if (pid > 0) { // parent
	    close(pipefd[1]); // close write side
		char buffer[4096];
		size_t bytes;
		while ((bytes = read(pipefd[0],
				   			 buffer,
							 sizeof(buffer))) != 0) {
			// std::cout.write(buffer, bytes);
			std::string str(buffer);
			auto list = split(str, '\n');
			//for (auto& file: list) {
			//    std::cout << "filename len: " << 
			//	file.size() << " " << file << std::endl;
			//}
			return list;
		}
		close(pipefd[0]);
	} else { // child
		dup2(pipefd[1], STDOUT_FILENO);
		close(pipefd[0]);
		close(pipefd[1]);
		auto rcode = system(command.c_str());
		exit(rcode);
	}
	return ret;
}

#define TEST_LOG_FN "test.bin"
struct Data {
    size_t idx;
	// std::vector<std::string> list; // take care of mem management
	double hight;
};

void check_data_file() {
	Data data;
	data.idx = 243;
	data.hight = 12.76;
	// data.list = {"xyz", "ab"};
	Data data1 = data;
	std::cout << "data size: " << sizeof(data) << " bytes\n"; // same as file!
	size_t len = sizeof(data);
	char buffer[len];
	memcpy(buffer, (char*)(&data), len);
	std::cout << "opening...\n";
	FILE *fptr = fopen(TEST_LOG_FN, "wb+");
    std::cout << "reading...\n";
	size_t bytes = fwrite(buffer, sizeof(char), len, fptr);
	if (bytes != len) {
		std::cout << "actually write " << bytes << " expect " << len << std::endl;
	}

	// set cursor position!
	fseek(fptr, 0, SEEK_SET);

	size_t b = fread(buffer, sizeof(char), len, fptr);
	std::cout << "checking...\n";
	if (b != len) {
	    std::cout << "write and should get " << len << 
			" but get " << b << " bytes\n";
		return;
	} else {
	    std::cout << "read write size equals: " << b << std::endl;
	}
	memcpy((char*)(&data1), buffer, len);

	std::cout << "comparing...\n";
	if (data1.idx == data.idx &&
			data1.hight == data.hight) {
		std::cout << "check pass\n";
	} else {
	    std::cout << "check failed\n";
	}
	std::cout << "closing...\n";
	fclose(fptr);
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
	std::cout << "loading output object from file " << filename << std::endl;

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

int32_t read_parse_uavo_log(const std::string& dir) {
	uint16_t flightnum = 0;
	// auto files = file_list(dir);
	uint16_t lognum = 0; // mock
	bool has_file = true;

	DebugLogEntry::DataFields entry;
	entry.Flight = flightnum;
	entry.Entry = lognum;
	entry.Type = DebugLogEntry::TypeOptions::TYPE_EMPTY;
	uint16_t obj_size = sizeof(DebugLogEntry::DataFields);

	ExtendedDebugLogEntry *log_entry = new ExtendedDebugLogEntry();
	// init output csv file
	auto now = std::chrono::system_clock::now();
	auto now_time_t = std::chrono::system_clock::to_time_t(now);
	std::tm* local_time = std::localtime(&now_time_t);
	std::ostringstream time_stream;
	time_stream << std::put_time(local_time, "%Y_%m_%d_%H_%M_%S_stylog.csv");
	std::string current_time = time_stream.str();
    QFile csvFile(QString::fromStdString(current_time));

	std::cout << "init output file " << current_time << std::endl;
	if (!csvFile.open(QFile::WriteOnly | QFile::Truncate)) {
		std::cout << "failed to open output file\n";
		return -1;
	}
	std::cout << "succeed to open file " << current_time << std::endl;
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
		while (true) {
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
			// next cycle		  
			lognum++;

			// log output entry
			ExtendedDebugLogEntry *ex_entry = new ExtendedDebugLogEntry();
			ex_entry->setData(entry, &g_mgr);
			ex_entry->toCSV(&csvStream, baseTime);
			delete ex_entry;
			
			if (DebugLogEntry::TYPE_MULTIPLEUAVOBJECTS == log_entry->getType()) {
				const quint32 total_len  = sizeof(DebugLogEntry::DataFields);
				const quint32 data_len   = sizeof(((DebugLogEntry::DataFields *)0)->Data);
				const quint32 header_len = total_len - data_len;

				DebugLogEntry::DataFields fields;
				quint32 start = log_entry->getData().Size;

				// cycle until there is space for another object
				while (start + header_len + 1 < data_len) {
					memset(&fields, 0xFF, total_len);
					auto tmp = log_entry->getData().Data[start];
					memcpy(&fields, &tmp, header_len);
					// check wether a packed object is found
					// note that empty data blocks are set as 0xFF in 
					// flight side to minimize flash wearing
					// thus as soon as this read outside of used area,
					// the test will fail as lenght would be 0xFFFF
					quint32 toread = header_len + fields.Size;
					if (!(toread + start > data_len)) {
						memcpy(&fields, &tmp, toread);
						ExtendedDebugLogEntry *subEntry = new ExtendedDebugLogEntry();
						subEntry->setData(fields, &g_mgr);
						subEntry->toCSV(&csvStream, baseTime);
						delete subEntry;
					}
					start += toread;
				}
			} // multi objs
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

	delete log_entry;
	return 0;
}

int main(int argc, char** argv) {
	// 1. file list
	auto files = file_list(LOGS_DIR);
	std::cout << "parsed file nums:" << 
		files.size() << std::endl;
	// 2. write read check equal
	// check_data_file();
	// 3. parse args
	// cycle read parse -> fill logEntry -> export csv
	UAVObjectsInitialize(&g_mgr);;
    read_parse_uavo_log(LOGS_DIR);
	return 0;
}
