/**
 ******************************************************************************
 *
 * @file       logfile.h
 * @author     The LibrePilot Project, http://www.librepilot.org Copyright (C) 2017.
 *             The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
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
#ifndef LOGFILE_H
#define LOGFILE_H

#include "utils_global.h"

#include <QIODevice>
#include <QTime>
#include <QTimer>
#include <QMutexLocker>
#include <QDebug>
#include <QBuffer>
#include <QFile>

class QTCREATOR_UTILS_EXPORT LogFile : public QIODevice {
    Q_OBJECT
public:
    explicit LogFile(QObject *parent = 0);

    QString fileName() const
    {
        return m_file.fileName();
    };
    void setFileName(QString name)
    {
        m_file.setFileName(name);
    };

    bool isSequential() const;

    bool isPlaying() const;

    bool open(OpenMode mode);
    void close();

    qint64 bytesAvailable() const;
    qint64 bytesToWrite() const
    {
        return m_file.bytesToWrite();
    };

    qint64 writeData(const char *data, qint64 dataSize);
    qint64 readData(char *data, qint64 maxlen);

    void useProvidedTimeStamp(bool useProvidedTimeStamp)
    {
        m_useProvidedTimeStamp = useProvidedTimeStamp;
    }

    void setNextTimeStamp(quint32 providedTimestamp)
    {
        m_providedTimeStamp = providedTimestamp;
    }

public slots:
    void setReplaySpeed(double val)
    {
        m_playbackSpeed = val;
        qDebug() << "Playback speed is now" << m_playbackSpeed;
    };
    bool startReplay();
    bool stopReplay();
    bool pauseReplay();
    bool resumeReplay();

protected slots:
    void timerFired();

signals:
    void readReady();
    void replayStarted();
    void replayFinished();

protected:
    QByteArray m_dataBuffer;
    QTimer m_timer;
    QTime m_myTime;
    QFile m_file;
    qint32 m_previousTimeStamp;
    qint32 m_nextTimeStamp;
    double m_lastPlayed;
    // QMutex wants to be mutable
    // http://stackoverflow.com/questions/25521570/can-mutex-locking-function-be-marked-as-const
    mutable QMutex m_mutex;

    int m_timeOffset;
    double m_playbackSpeed;
    bool paused;

private:
    bool m_useProvidedTimeStamp;
    qint32 m_providedTimeStamp;
};

#endif // LOGFILE_H
