/**
 ******************************************************************************
 *
 * @file       sequences.h
 * @author     The LibrePilot Project, http://www.librepilot.org Copyright (C) 2016.
 *             The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Notify module, sequences configuration.
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
#ifndef SEQUENCES_H_
#define SEQUENCES_H_
#include <optypes.h>
#include <pios_notify.h>
#include <flightstatus.h>
#include <systemalarms.h>
#include <pios_helpers.h>

// This represent the list of basic light sequences, defined later
typedef enum {
    NOTIFY_SEQUENCE_ARMED_FM_MANUAL = 0,
    NOTIFY_SEQUENCE_ARMED_FM_STABILIZED1,
    NOTIFY_SEQUENCE_ARMED_FM_STABILIZED2,
    NOTIFY_SEQUENCE_ARMED_FM_STABILIZED3,
    NOTIFY_SEQUENCE_ARMED_FM_STABILIZED4,
    NOTIFY_SEQUENCE_ARMED_FM_STABILIZED5,
    NOTIFY_SEQUENCE_ARMED_FM_STABILIZED6,
    NOTIFY_SEQUENCE_ARMED_FM_GPS,
    NOTIFY_SEQUENCE_ARMED_FM_RTH,
    NOTIFY_SEQUENCE_ARMED_FM_LAND,
    NOTIFY_SEQUENCE_ARMED_FM_AUTO,
    NOTIFY_SEQUENCE_ALM_WARN_GPS,
    NOTIFY_SEQUENCE_ALM_ERROR_GPS,
    NOTIFY_SEQUENCE_ALM_WARN_BATTERY,
    NOTIFY_SEQUENCE_ALM_ERROR_BATTERY,
    NOTIFY_SEQUENCE_ALM_WARN_MAG,
    NOTIFY_SEQUENCE_ALM_ERROR_MAG,
    NOTIFY_SEQUENCE_ALM_CONFIG,
    NOTIFY_SEQUENCE_ALM_RECEIVER,
    NOTIFY_SEQUENCE_DISARMED,
    NOTIFY_SEQUENCE_ALM_ATTITUDE,
    NOTIFY_SEQUENCE_NULL = 255, // skips any signalling for this condition
} NotifySequences;

// This structure determine sequences attached to an alarm
typedef struct {
    uint32_t timeBetweenNotifications; // time in milliseconds to wait between each notification iteration
    uint8_t  alarmIndex; // Index of the alarm, use one of the SYSTEMALARMS_ALARM_XXXXX defines
    uint8_t  warnNotification; // index of the sequence to be used when alarm is in warning status(pick one from NotifySequences enum)
    uint8_t  criticalNotification; // index of the sequence to be used when alarm is in critical status(pick one from NotifySequences enum)
    uint8_t  errorNotification; // index of the sequence to be used when alarm is in error status(pick one from NotifySequences enum)
} AlarmDefinition_t;

#define STANDARD_ERROR_SEQUENCE(alarm_color, alarm_repeats) \
    { .repeats = alarm_repeats, .steps = { \
          { .time_off = 200, .time_on = 10,  .color = COLOR_BLACK,   .repeats = 1, }, \
          { .time_off = 100, .time_on = 300, .color = COLOR_DARKRED, .repeats = 1, }, \
          { .time_off = 100, .time_on = 300, .color = alarm_color,   .repeats = 2, }, \
          { .time_off = 100, .time_on = 10,  .color = COLOR_BLACK,   .repeats = 1, }, \
      }, }

#define STANDARD_WARN_SEQUENCE(alarm_color, alarm_repeats) \
    { .repeats = alarm_repeats, .steps = { \
          { .time_off = 200, .time_on = 10,  .color = COLOR_BLACK,  .repeats = 1, }, \
          { .time_off = 100, .time_on = 300, .color = COLOR_ORANGE, .repeats = 1, }, \
          { .time_off = 100, .time_on = 300, .color = alarm_color,  .repeats = 1, }, \
          { .time_off = 200, .time_on = 10,  .color = COLOR_BLACK,  .repeats = 1, }, \
      }, }

// This is the list of defined light sequences
/* how each sequence is defined
 * [NOTIFY_SEQUENCE_DISARMED] = {  // Sequence ID
        .repeats = -1,             // Number of repetitions or -1 for infinite
        .steps   = { // List of steps (until NOTIFY_SEQUENCE_MAX_STEPS steps, default to 5)
                    {
                        .time_off = 500,           // Off time for the step
                        .time_on  = 500,           // On time for the step
                        .color    = COLOR_TEAL,    // color
                        .repeats  = 1,             // repetitions for this step
                    },
        },
    },
 *
 * There are two kind of sequences:
 * - "Background" sequences, executed if no higher priority sequence is played;
 * - "Alarm" sequences, that are "modal", they temporarily suspends background sequences and plays.
 * Cannot have "-1" repetitions
 * At the end background sequence are resumed;
 *
 */
const LedSequence_t notifications[] = {
    [NOTIFY_SEQUENCE_DISARMED] =                   { .repeats  = -1,  .steps    = {
                                                   { .time_off = 200, .time_on  = 10,  .color = COLOR_BLACK, .repeats = 1, },
                                                   { .time_off = 100, .time_on  = 100, .color = COLOR_WHITE, .repeats = 1, },
                                                   { .time_off = 200, .time_on  = 10,  .color = COLOR_BLACK, .repeats = 1, },
                                                     }, },
    [NOTIFY_SEQUENCE_ARMED_FM_MANUAL] =            { .repeats  = -1,  .steps    = {
                                                   { .time_off = 900, .time_on  = 100, .color = COLOR_BLUE, .repeats = 1, },
                                                     }, },
    [NOTIFY_SEQUENCE_ARMED_FM_STABILIZED1] =       { .repeats  = -1,  .steps    = {
                                                   { .time_off = 900, .time_on  = 100, .color = COLOR_BLUE, .repeats = 1, },
                                                     }, },
    [NOTIFY_SEQUENCE_ARMED_FM_STABILIZED2] =       { .repeats  = -1,  .steps    = {
                                                   { .time_off = 100, .time_on  = 100, .color = COLOR_BLUE, .repeats = 1, },
                                                   { .time_off = 700, .time_on  = 100, .color = COLOR_BLUE, .repeats = 1, },
                                                     }, },
    [NOTIFY_SEQUENCE_ARMED_FM_STABILIZED3] =       { .repeats  = -1,  .steps    = {
                                                   { .time_off = 100, .time_on  = 100, .color = COLOR_BLUE, .repeats = 2, },
                                                   { .time_off = 500, .time_on  = 100, .color = COLOR_BLUE, .repeats = 1, },
                                                     }, },
    [NOTIFY_SEQUENCE_ARMED_FM_STABILIZED4] =       { .repeats  = -1,  .steps    = {
                                                   { .time_off = 900, .time_on  = 100, .color = COLOR_PURPLE, .repeats = 1, },
                                                     }, },
    [NOTIFY_SEQUENCE_ARMED_FM_STABILIZED5] =       { .repeats  = -1,  .steps    = {
                                                   { .time_off = 100, .time_on  = 100, .color = COLOR_PURPLE, .repeats = 1, },
                                                   { .time_off = 700, .time_on  = 100, .color = COLOR_BLUE,   .repeats = 1, },
                                                     }, },
    [NOTIFY_SEQUENCE_ARMED_FM_STABILIZED6] =       { .repeats  = -1,  .steps    = {
                                                   { .time_off = 100, .time_on  = 100, .color = COLOR_PURPLE, .repeats = 1, },
                                                   { .time_off = 100, .time_on  = 100, .color = COLOR_BLUE,   .repeats = 1, },
                                                   { .time_off = 500, .time_on  = 100, .color = COLOR_BLUE,   .repeats = 1, },
                                                     }, },
    [NOTIFY_SEQUENCE_ARMED_FM_GPS] =               { .repeats  = -1,  .steps    = {
                                                   { .time_off = 800, .time_on  = 200, .color = COLOR_GREEN, .repeats = 1, },
                                                     }, },
    [NOTIFY_SEQUENCE_ARMED_FM_RTH] =               { .repeats  = -1,  .steps    = {
                                                   { .time_off = 100, .time_on  = 100, .color = COLOR_GREEN,  .repeats = 1, },
                                                   { .time_off = 100, .time_on  = 100, .color = COLOR_YELLOW, .repeats = 1, },
                                                     }, },
    [NOTIFY_SEQUENCE_ARMED_FM_LAND] =              { .repeats  = -1,  .steps    = {
                                                   { .time_off = 100, .time_on  = 100, .color = COLOR_GREEN, .repeats = 1, },
                                                     }, },
    [NOTIFY_SEQUENCE_ARMED_FM_AUTO] =              { .repeats  = -1,  .steps    = {
                                                   { .time_off = 100, .time_on  = 200, .color = COLOR_GREEN, .repeats = 2, },
                                                   { .time_off = 500, .time_on  = 200, .color = COLOR_GREEN, .repeats = 1, },
                                                     }, },
    [NOTIFY_SEQUENCE_ALM_WARN_GPS]      = STANDARD_WARN_SEQUENCE(COLOR_GREEN, 1),
    [NOTIFY_SEQUENCE_ALM_ERROR_GPS]     = STANDARD_ERROR_SEQUENCE(COLOR_GREEN, 1),
    [NOTIFY_SEQUENCE_ALM_WARN_BATTERY]  =          { .repeats  = 1,   .steps     = {
                                                   { .time_off = 100, .time_on   = 100, .color = COLOR_ORANGE, .repeats = 5, },
                                                     }, },
    [NOTIFY_SEQUENCE_ALM_ERROR_BATTERY] =          { .repeats  = 1,   .steps     = {
                                                   { .time_off = 100, .time_on   = 100, .color = COLOR_RED, .repeats = 5, },
                                                     }, },
    [NOTIFY_SEQUENCE_ALM_ERROR_MAG]     = STANDARD_ERROR_SEQUENCE(COLOR_PURPLE, 1),
    [NOTIFY_SEQUENCE_ALM_WARN_MAG]      = STANDARD_WARN_SEQUENCE(COLOR_PURPLE, 1),
    [NOTIFY_SEQUENCE_ALM_CONFIG] = STANDARD_ERROR_SEQUENCE(COLOR_RED, 2),
    [NOTIFY_SEQUENCE_ALM_RECEIVER]      = STANDARD_ERROR_SEQUENCE(COLOR_YELLOW, 1),
    [NOTIFY_SEQUENCE_ALM_ATTITUDE]      =          { .repeats  = 10, .steps     = {
                                                   { .time_off = 0, .time_on    = 50, .color = COLOR_RED,  .repeats = 1, },
                                                   { .time_off = 0, .time_on    = 50, .color = COLOR_BLUE, .repeats = 1, },
                                                     }, },
};

// List of background sequences attached to each flight mode
const LedSequence_t *flightModeMap[] = {
    [FLIGHTSTATUS_FLIGHTMODE_MANUAL] = &notifications[NOTIFY_SEQUENCE_ARMED_FM_MANUAL],
    [FLIGHTSTATUS_FLIGHTMODE_STABILIZED1]      = &notifications[NOTIFY_SEQUENCE_ARMED_FM_STABILIZED1],
    [FLIGHTSTATUS_FLIGHTMODE_STABILIZED2]      = &notifications[NOTIFY_SEQUENCE_ARMED_FM_STABILIZED2],
    [FLIGHTSTATUS_FLIGHTMODE_STABILIZED3]      = &notifications[NOTIFY_SEQUENCE_ARMED_FM_STABILIZED3],
    [FLIGHTSTATUS_FLIGHTMODE_STABILIZED4]      = &notifications[NOTIFY_SEQUENCE_ARMED_FM_STABILIZED4],
    [FLIGHTSTATUS_FLIGHTMODE_STABILIZED5]      = &notifications[NOTIFY_SEQUENCE_ARMED_FM_STABILIZED5],
    [FLIGHTSTATUS_FLIGHTMODE_STABILIZED6]      = &notifications[NOTIFY_SEQUENCE_ARMED_FM_STABILIZED6],
    [FLIGHTSTATUS_FLIGHTMODE_PATHPLANNER]      = &notifications[NOTIFY_SEQUENCE_ARMED_FM_AUTO],
    [FLIGHTSTATUS_FLIGHTMODE_POSITIONHOLD]     = &notifications[NOTIFY_SEQUENCE_ARMED_FM_GPS],
    [FLIGHTSTATUS_FLIGHTMODE_COURSELOCK]       = &notifications[NOTIFY_SEQUENCE_ARMED_FM_GPS],
    [FLIGHTSTATUS_FLIGHTMODE_VELOCITYROAM]     = &notifications[NOTIFY_SEQUENCE_ARMED_FM_GPS],
    [FLIGHTSTATUS_FLIGHTMODE_HOMELEASH]        = &notifications[NOTIFY_SEQUENCE_ARMED_FM_GPS],
    [FLIGHTSTATUS_FLIGHTMODE_ABSOLUTEPOSITION] = &notifications[NOTIFY_SEQUENCE_ARMED_FM_GPS],
    [FLIGHTSTATUS_FLIGHTMODE_RETURNTOBASE]     = &notifications[NOTIFY_SEQUENCE_ARMED_FM_RTH],
    [FLIGHTSTATUS_FLIGHTMODE_LAND] = &notifications[NOTIFY_SEQUENCE_ARMED_FM_LAND],
    [FLIGHTSTATUS_FLIGHTMODE_POI] = &notifications[NOTIFY_SEQUENCE_ARMED_FM_GPS],
    [FLIGHTSTATUS_FLIGHTMODE_AUTOCRUISE]       = &notifications[NOTIFY_SEQUENCE_ARMED_FM_GPS],
    [FLIGHTSTATUS_FLIGHTMODE_AUTOTAKEOFF]      = &notifications[NOTIFY_SEQUENCE_ARMED_FM_LAND],
#if !defined(PIOS_EXCLUDE_ADVANCED_FEATURES)
    [FLIGHTSTATUS_FLIGHTMODE_AUTOTUNE] = &notifications[NOTIFY_SEQUENCE_ARMED_FM_MANUAL],
#endif /* !defined(PIOS_EXCLUDE_ADVANCED_FEATURES) */
};

// List of alarms to show with attached sequences for each status
const AlarmDefinition_t alarmsMap[] = {
    {
        .timeBetweenNotifications = 5000,
        .alarmIndex = SYSTEMALARMS_ALARM_GPS,
        .warnNotification = NOTIFY_SEQUENCE_ALM_WARN_GPS,
        .criticalNotification     = NOTIFY_SEQUENCE_ALM_ERROR_GPS,
        .errorNotification = NOTIFY_SEQUENCE_ALM_ERROR_GPS,
    },
    {
        .timeBetweenNotifications = 5000,
        .alarmIndex = SYSTEMALARMS_ALARM_MAGNETOMETER,
        .warnNotification = NOTIFY_SEQUENCE_ALM_WARN_MAG,
        .criticalNotification     = NOTIFY_SEQUENCE_ALM_ERROR_MAG,
        .errorNotification = NOTIFY_SEQUENCE_ALM_ERROR_MAG,
    },
    {
        .timeBetweenNotifications = 15000,
        .alarmIndex = SYSTEMALARMS_ALARM_BATTERY,
        .warnNotification = NOTIFY_SEQUENCE_ALM_WARN_BATTERY,
        .criticalNotification     = NOTIFY_SEQUENCE_ALM_ERROR_BATTERY,
        .errorNotification = NOTIFY_SEQUENCE_ALM_ERROR_BATTERY,
    },
    {
        .timeBetweenNotifications = 5000,
        .alarmIndex = SYSTEMALARMS_ALARM_SYSTEMCONFIGURATION,
        .warnNotification = NOTIFY_SEQUENCE_NULL,
        .criticalNotification     = NOTIFY_SEQUENCE_ALM_CONFIG,
        .errorNotification = NOTIFY_SEQUENCE_ALM_CONFIG,
    },
    {
        .timeBetweenNotifications = 5000,
        .alarmIndex = SYSTEMALARMS_ALARM_RECEIVER,
        .warnNotification = NOTIFY_SEQUENCE_ALM_RECEIVER,
        .criticalNotification     = NOTIFY_SEQUENCE_ALM_RECEIVER,
        .errorNotification = NOTIFY_SEQUENCE_ALM_RECEIVER,
    },
    {
        .timeBetweenNotifications = 1000,
        .alarmIndex = SYSTEMALARMS_ALARM_ATTITUDE,
        .warnNotification = NOTIFY_SEQUENCE_ALM_ATTITUDE,
        .criticalNotification     = NOTIFY_SEQUENCE_NULL,
        .errorNotification = NOTIFY_SEQUENCE_ALM_ATTITUDE,
    },
};

const uint8_t alarmsMapSize = NELEMENTS(alarmsMap);

#endif /* SEQUENCES_H_ */
