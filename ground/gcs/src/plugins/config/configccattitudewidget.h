/**
 ******************************************************************************
 *
 * @file       configccattitudewidget.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @addtogroup GCSPlugins GCS Plugins
 * @{
 * @addtogroup ConfigPlugin Config Plugin
 * @{
 * @brief Configure the properties of the attitude module in CopterControl
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
#ifndef CCATTITUDEWIDGET_H
#define CCATTITUDEWIDGET_H

#include "../uavobjectwidgetutils/configtaskwidget.h"

#include "uavobject.h"

#include <QWidget>
#include <QTimer>

class Ui_ccattitude;

class ConfigCCAttitudeWidget : public ConfigTaskWidget {
    Q_OBJECT

public:
    explicit ConfigCCAttitudeWidget(QWidget *parent = 0);
    ~ConfigCCAttitudeWidget();

protected:
    virtual void updateObjectsFromWidgetsImpl();

private slots:
    void sensorsUpdated(UAVObject *obj);
    void timeout();
    void startAccelCalibration();
    void setAccelFiltering(bool active);

private:
    Ui_ccattitude *ui;
    QTimer timer;
    UAVObject::Metadata initialAccelStateMdata;
    UAVObject::Metadata initialGyroStateMdata;

    int accelUpdates;
    int gyroUpdates;

    QList<double> x_accum, y_accum, z_accum;
    QList<double> x_gyro_accum, y_gyro_accum, z_gyro_accum;

    static const int NUM_SENSOR_UPDATES = 300;

protected:
    virtual void enableControls(bool enable);
};

#endif // CCATTITUDEWIDGET_H
