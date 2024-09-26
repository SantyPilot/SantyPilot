/*
 * @brief: configuration parser
 *         simple implementation all string
 * @author: santypilot 
 * @date: 2024-4-12
 */
#ifndef _STY_CONFIGURATION_H
#define _STY_CONFIGURATION_H

#include <map>
#include <string>

namespace components {
class StyConfiguration {
public:
	void processNode(const QDomNode &node, 
		std::map<std::string, std::string>& ret,
		const std::string&);

	bool isLeafNode(const QDomNode &node);

	std::string getKV(const QDomNode& attr, 
		const std::string& key);

    std::map<std::string, std::string> 
		parseXML(const std::string& xml);
};
} // components

#endif // _STY_CONFIGURATION_H
