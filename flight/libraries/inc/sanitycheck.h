/**
 ******************************************************************************
 * @addtogroup OpenPilotSystem OpenPilot System
 * @{
 * @addtogroup OpenPilotLibraries OpenPilot System Libraries
 * @{
 * @file       sanitycheck.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2012.
 * @brief      Utilities to validate a flight configuration
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


#ifndef SANITYCHECK_H
#define SANITYCHECK_H

#include <systemalarms.h>

#include <stdint.h>
#include <utlist.h>

typedef enum {
    FRAME_TYPE_MULTIROTOR,
    FRAME_TYPE_HELI,
    FRAME_TYPE_FIXED_WING,
    FRAME_TYPE_GROUND,
    FRAME_TYPE_CUSTOM,
} FrameType_t;

typedef SystemAlarmsExtendedAlarmStatusOptions (SANITYCHECK_CustomHook_function)();

#define SANITYCHECK_STATUS_ERROR_NONE         SYSTEMALARMS_EXTENDEDALARMSTATUS_NONE
#define SANITYCHECK_STATUS_ERROR_FLIGHTMODE   SYSTEMALARMS_EXTENDEDALARMSTATUS_FLIGHTMODE

#define BOOTFAULT_STATUS_ERROR_NONE           SYSTEMALARMS_EXTENDEDALARMSTATUS_NONE
#define BOOTFAULT_STATUS_ERROR_REQUIRE_REBOOT SYSTEMALARMS_EXTENDEDALARMSTATUS_REBOOTREQUIRED

#if (SYSTEMALARMS_EXTENDEDALARMSTATUS_NUMELEM != SYSTEMALARMS_EXTENDEDALARMSUBSTATUS_NUMELEM) || \
    (SYSTEMALARMS_EXTENDEDALARMSUBSTATUS_NUMELEM > SYSTEMALARMS_ALARM_NUMELEM)
#error Incongruent SystemAlarms. Please revise the UAVO definition in systemalarms.xml
#endif

extern int32_t configuration_check();

extern FrameType_t GetCurrentFrameType();

/**
 * Attach a custom hook to the sanity check process
 * @param hook a custom hook function
 */
extern void SANITYCHECK_AttachHook(SANITYCHECK_CustomHook_function *hook);

/**
 * Detach a custom hook to the sanity check process
 * @param hook a custom hook function
 */
extern void SANITYCHECK_DetachHook(SANITYCHECK_CustomHook_function *hook);

#endif /* SANITYCHECK_H */
