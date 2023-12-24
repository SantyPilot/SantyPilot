/**
 ******************************************************************************
 * @file       pathfollowerhandler.c
 * @author     The LibrePilot Project, http://www.librepilot.org Copyright (C) 2016.
 *             The OpenPilot Team, http://www.openpilot.org Copyright (C) 2015.
 *
 * @brief Interpretes the control input in ManualControlCommand
 *
 * @see        The GNU Public License (GPL) Version 3
 *
 * @addtogroup LibrePilotModules LibrePilot Modules ManualControl
 ******************************************************************************/
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

#include "inc/manualcontrol.h"
#include <pathdesired.h>
#include <manualcontrolcommand.h>
#include <flightstatus.h>
#include <positionstate.h>
#include <flightmodesettings.h>

#if defined(REVOLUTION)
#include <plans.h>

// Private constants

// Private types

// Private functions

/**
 * @brief Handler to control Guided flightmodes. FlightControl is governed by PathFollower, control via PathDesired
 * @input: NONE: fully automated mode -- TODO recursively call handler for advanced stick commands
 * @output: PathDesired
 */
void pathFollowerHandler(bool newinit)
{
    if (newinit) {
        plan_initialize();
    }

    FlightStatusFlightModeOptions flightMode;
    FlightStatusAssistedControlStateOptions assistedControlFlightMode;
    FlightStatusFlightModeAssistOptions flightModeAssist;
    FlightStatusFlightModeGet(&flightMode);
    FlightStatusFlightModeAssistGet(&flightModeAssist);
    FlightStatusAssistedControlStateGet(&assistedControlFlightMode);

    if (newinit) {
        // After not being in this mode for a while init at current height
        switch (flightMode) {
        case FLIGHTSTATUS_FLIGHTMODE_RETURNTOBASE:
            plan_setup_returnToBase();
            break;
        case FLIGHTSTATUS_FLIGHTMODE_POSITIONHOLD:
            if ((flightModeAssist != FLIGHTSTATUS_FLIGHTMODEASSIST_NONE) &&
                (assistedControlFlightMode == FLIGHTSTATUS_ASSISTEDCONTROLSTATE_PRIMARY)) {
                // Switch from primary (just entered this PH flight mode) into brake
                plan_setup_assistedcontrol();
            } else {
                plan_setup_positionHold();
            }
            break;
        case FLIGHTSTATUS_FLIGHTMODE_COURSELOCK:
            plan_setup_CourseLock();
            break;
        case FLIGHTSTATUS_FLIGHTMODE_VELOCITYROAM:
            plan_setup_VelocityRoam();
            break;
        case FLIGHTSTATUS_FLIGHTMODE_HOMELEASH:
            plan_setup_HomeLeash();
            break;
        case FLIGHTSTATUS_FLIGHTMODE_ABSOLUTEPOSITION:
            plan_setup_AbsolutePosition();
            break;

        case FLIGHTSTATUS_FLIGHTMODE_LAND:
            if (flightModeAssist == FLIGHTSTATUS_FLIGHTMODEASSIST_NONE) {
                plan_setup_land();
            } else {
                plan_setup_VelocityRoam();
            }
            break;
        case FLIGHTSTATUS_FLIGHTMODE_AUTOTAKEOFF:
            plan_setup_AutoTakeoff();
            break;
        case FLIGHTSTATUS_FLIGHTMODE_AUTOCRUISE:
            plan_setup_AutoCruise();
            break;

        case FLIGHTSTATUS_FLIGHTMODE_STABILIZED1:
        case FLIGHTSTATUS_FLIGHTMODE_STABILIZED2:
        case FLIGHTSTATUS_FLIGHTMODE_STABILIZED3:
        case FLIGHTSTATUS_FLIGHTMODE_STABILIZED4:
        case FLIGHTSTATUS_FLIGHTMODE_STABILIZED5:
        case FLIGHTSTATUS_FLIGHTMODE_STABILIZED6:
            if (assistedControlFlightMode == FLIGHTSTATUS_ASSISTEDCONTROLSTATE_BRAKE) {
                // Just initiated braking after returning from stabi control
                plan_setup_assistedcontrol();
            }
            break;

        default:
            plan_setup_positionHold();
            break;
        }
    }

    switch (flightMode) {
    case FLIGHTSTATUS_FLIGHTMODE_COURSELOCK:
        plan_run_CourseLock();
        break;
    case FLIGHTSTATUS_FLIGHTMODE_VELOCITYROAM:
        plan_run_VelocityRoam();
        break;
    case FLIGHTSTATUS_FLIGHTMODE_HOMELEASH:
        plan_run_HomeLeash();
        break;
    case FLIGHTSTATUS_FLIGHTMODE_ABSOLUTEPOSITION:
        plan_run_AbsolutePosition();
        break;
    case FLIGHTSTATUS_FLIGHTMODE_LAND:
        if (flightModeAssist != FLIGHTSTATUS_FLIGHTMODEASSIST_NONE) {
            plan_run_VelocityRoam();
        }
        break;
    case FLIGHTSTATUS_FLIGHTMODE_AUTOCRUISE:
        plan_run_AutoCruise();
        break;
    default:
        break;
    }
}

#else /* if defined(REVOLUTION) */
void pathFollowerHandler(__attribute__((unused)) bool newinit)
{
    AlarmsSet(SYSTEMALARMS_ALARM_MANUALCONTROL, SYSTEMALARMS_ALARM_CRITICAL); // should not be called
}
#endif // REVOLUTION


/**
 * @}
 * @}
 */
