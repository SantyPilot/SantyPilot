/**
 * @file: ExtendedDebugLogEntry.h
 * @brief: extended func for debuglogentry impl
 * @author: zhangxin@santypilot
 * @date: 2024-2-28
 */
#include "ExtendedDebugLogEntry.h"
#include "uavobjectmanager.h"
#include <iostream>

namespace santypilot_gcs {
namespace flightlogparser {

ExtendedDebugLogEntry::ExtendedDebugLogEntry() : DebugLogEntry(),
    m_object(0)
{}

ExtendedDebugLogEntry::~ExtendedDebugLogEntry()
{
    if (m_object) {
        delete m_object;
        m_object = 0;
    }
}

QString ExtendedDebugLogEntry::getLogString()
{
    if (getType() == DebugLogEntry::TYPE_TEXT) {
        return QString((const char *)getData().Data);
    } else if (getType() == DebugLogEntry::TYPE_UAVOBJECT || getType() == DebugLogEntry::TYPE_MULTIPLEUAVOBJECTS) {
        return m_object->toString().replace("\n", " ").replace("\t", " ");
    } else {
        return "";
    }
}

void ExtendedDebugLogEntry::toXML(QXmlStreamWriter *xmlWriter, quint32 baseTime)
{
    xmlWriter->writeStartElement("entry");
    xmlWriter->writeAttribute("flight", QString::number(getFlight() + 1));
    xmlWriter->writeAttribute("flighttime", QString::number(getFlightTime() - baseTime));
    xmlWriter->writeAttribute("entry", QString::number(getEntry()));
    if (getType() == DebugLogEntry::TYPE_TEXT) {
        xmlWriter->writeAttribute("type", "text");
        xmlWriter->writeTextElement("message", QString((const char *)getData().Data));
    } else if (getType() == DebugLogEntry::TYPE_UAVOBJECT || getType() == DebugLogEntry::TYPE_MULTIPLEUAVOBJECTS) {
        xmlWriter->writeAttribute("type", "uavobject");
        m_object->toXML(xmlWriter);
    }
    xmlWriter->writeEndElement(); // entry
}

void ExtendedDebugLogEntry::toCSV(QTextStream *csvStream, quint32 baseTime)
{
    QString data;

    if (getType() == DebugLogEntry::TYPE_TEXT) {
        data = QString((const char *)getData().Data);
    } else if (getType() == DebugLogEntry::TYPE_UAVOBJECT || getType() == DebugLogEntry::TYPE_MULTIPLEUAVOBJECTS) {
        data = m_object->toString().replace("\n", " ").replace("\t", " ");
    }
	std::cout << "writing: " << data.toStdString() << std::endl;
    *csvStream << QString::number(getFlight() + 1) << '\t' << QString::number(getFlightTime() - baseTime) << '\t' << QString::number(getEntry()) << '\t' << data << '\n';
}

void ExtendedDebugLogEntry::setData(const DebugLogEntry::DataFields &data, UAVObjectManager *objectManager)
{
    DebugLogEntry::setData(data);

    if (getType() == DebugLogEntry::TYPE_UAVOBJECT || getType() == DebugLogEntry::TYPE_MULTIPLEUAVOBJECTS) {
        UAVDataObject *object = (UAVDataObject *)objectManager->getObject(getObjectID(), getInstanceID());
        Q_ASSERT(object);
        m_object = object->clone(getInstanceID());
        m_object->unpack(getData().Data);
    }
}

} // flightlogparser
} // santypilot_gcs
