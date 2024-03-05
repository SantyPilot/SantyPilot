/**
 * @file: ExtendedDebugLogEntry.h
 * @brief: extended func for debuglogentry
 * @author: zhangxin@santypilot
 * @date: 2024-2-28
 */
#ifndef _EXTENDED_DEBUGLOG_ENTRY_H
#define _EXTENDED_DEBUGLOG_ENTRY_H

#include "debuglogentry.h"
#include <QObject>
#include <QXmlStreamWriter>
#include <QTextStream>

namespace santypilot_gcs {
namespace flightlogparser {

class ExtendedDebugLogEntry : public DebugLogEntry {
    Q_OBJECT Q_PROPERTY(QString LogString READ getLogString WRITE setLogString NOTIFY LogStringUpdated)

public:
    explicit ExtendedDebugLogEntry();
    ~ExtendedDebugLogEntry();

    QString getLogString();
    void toXML(QXmlStreamWriter *xmlWriter, quint32 baseTime);
    void toCSV(QTextStream *csvStream, quint32 baseTime);
    UAVDataObject *uavObject()
    {
        return m_object;
    }

    void setData(const DataFields & data, UAVObjectManager *objectManager);

public slots:
    void setLogString(QString arg)
    {
        Q_UNUSED(arg);
    }

signals:
    void LogStringUpdated(QString arg);

private:
    UAVDataObject *m_object;
};

} // flightlogparser
} // santypilot_gcs

#endif // _EXTENDED_DEBUGLOG_ENTRY_H
