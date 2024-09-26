/*
 * @brief: configuration parser
 *         simple implementation all string
 *         implementation
 * @author: santypilot 
 * @date: 2024-4-12
 */
#include <map>
#include <string>
#include <iostream>
#include <QDomDocument>
#include <QDomElement>
#include <QDebug>
#include <QString>
#include <QStringList>
#include <QList>
#include <QDomNode>
#include <QByteArray>
#include "StyConfiguration.h"
#include "StyLog.h"

namespace components {
// TODO: hardcode here, maybe we should tranversal
void StyConfiguration::processNode(const QDomNode &node, 
		std::map<std::string, std::string>& ret, 
		const std::string& pref) {
    if (node.isNull()) {
		return;
	}
	std::string key = "";
	std::string val = "";
    if (node.isElement() && node.childNodes().count() == 1) {
	    QDomElement element = node.toElement();
		key = pref;// + ":" + element.tagName().toStdString();
		val = element.text().toStdString();
		ret[key] = val;
	} else if (node.isText()) {
		QDomText textNode = node.toText();
		std::cout << "---" << textNode.data().toStdString() << std::endl;
	}
	QDomNodeList children = node.childNodes();
	for (int i = 0; i < children.count(); ++i) {
		if (!children.at(i).isElement()) {
			continue;
		}
		auto prefix = pref + ":" + 
			children.at(i).toElement().tagName().toStdString();
	    processNode(children.at(i), ret, prefix);
	}
	return;
}

bool StyConfiguration::isLeafNode(const QDomNode &node) {
    if (node.isNull()) {
	    return false;
	}
	return node.childNodes().count() == 0;
}

std::string StyConfiguration::getKV(const QDomNode& attr, 
		const std::string& key) {
	std::string val;
    if (attr.isNull()) {
	    LOG(ERROR) << key << " node is null!";
		return val;
	}
	val = attr.nodeValue().toStdString();
	return val;
}

std::map<std::string, std::string> 
	StyConfiguration::parseXML(const std::string& xml) {
	std::map<std::string, std::string> ret;
    // Create DOM document and parse it
    QDomDocument doc("Configuration");
	// check valid
    bool parsed = doc.setContent(QString::fromStdString(xml));
    if (!parsed) {
		std::cout << "invalid xml " << xml << std::endl;
        return ret;
    }
	// root
    QDomElement doc_element = doc.documentElement();
	QDomNodeList childNodes = doc_element.childNodes();
	for (int i = 0; i < childNodes.count(); ++i) {
	    const auto& node = childNodes.at(i);
		if (!node.isElement()) {
			continue;
		}
		auto prefix = node.toElement().tagName().toStdString();
		processNode(node, ret, prefix);
	}
	return ret;
}
} // components
