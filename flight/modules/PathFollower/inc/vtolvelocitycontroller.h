/**
 ******************************************************************************
 * @addtogroup OpenPilotModules OpenPilot Modules
 * @{
 * @addtogroup PathFollower Controller
 * @brief Controller for Vtol velocity roam
 * @{
 *
 * @file       vtolvelocitycontroller.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2015.
 * @brief      Executes velocity roam control
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
#ifndef PATHFOLLOWERCONTROLVELOCITYROAM_H
#define PATHFOLLOWERCONTROLVELOCITYROAM_H
#include "pathfollowercontrol.h"
#include "pidcontrolne.h"

class VtolVelocityController : public PathFollowerControl {
private:
    static VtolVelocityController *p_inst;
    VtolVelocityController();


public:
    static VtolVelocityController *instance()
    {
        if (!p_inst) {
            p_inst = new VtolVelocityController();
        }
        return p_inst;
    }

    int32_t Initialize(VtolPathFollowerSettingsData *vtolPathFollowerSettings);


    void Activate(void);
    void Deactivate(void);
    void SettingsUpdated(void);
    void UpdateAutoPilot(void);
    void ObjectiveUpdated(void);
    uint8_t IsActive(void);
    uint8_t Mode(void);

private:
    void UpdateVelocityDesired(void);
    int8_t UpdateStabilizationDesired(bool yaw_attitude, float yaw_direction);

    VtolPathFollowerSettingsData *vtolPathFollowerSettings;
    PIDControlNE controlNE;
    uint8_t mActive;
};

#endif // ifndef PATHFOLLOWERCONTROLVELOCITYROAM_H
