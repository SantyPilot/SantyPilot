/**
 ******************************************************************************
 * @addtogroup OpenPilotModules OpenPilot Modules
 * @{
 * @addtogroup PathFollower CONTROL interface class
 * @brief PID Controler for NE Class definition
 * @{
 *
 * @file       pidcontrolne.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2015.
 * @brief      Executes pid control loop for NE
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
#ifndef PIDCONTROLNE_H
#define PIDCONTROLNE_H
extern "C" {
#include <pid.h>
}
#include "pathfollowerfsm.h"

class PIDControlNE {
public:
    PIDControlNE();
    ~PIDControlNE();
    void Deactivate();
    void Activate();
    void UpdateParameters(float kp, float ki, float kd, __attribute__((unused)) float ilimit, float dT, float velocityMax);
    void UpdatePositionalParameters(float kp);
    void UpdatePositionState(float pvNorth, float pvEast);
    void UpdatePositionSetpoint(float setpointNorth, float setpointEast);
    void UpdateVelocitySetpoint(float setpointNorth, float setpointEast);
    // void RateLimit(float *spDesired, float *spCurrent, float rateLimit);
    void UpdateVelocityState(float pvNorth, float pvEast);
    void UpdateCommandParameters(float minCommand, float maxCommand, float velocityFeedforward);
    void ControlPosition();
    void ControlPositionWithPath(struct path_status *progress);
    void GetNECommand(float *northCommand, float *eastCommand);
    void GetVelocityDesired(float *north, float *east);
    void UpdateBrakeVelocity(float startingVelocity, float dT, float brakeRate, float currentVelocity, float *updatedVelocity);
    void UpdateVelocityStateWithBrake(float pvNorth, float pvEast, float path_time, float brakeRate);

private:
    struct pid2 PIDvel[2]; // North, East
    float mVelocitySetpointTarget[2];
    float mVelocityState[2];
    struct pid PIDposH[2];
    float mPositionSetpointTarget[2];
    float mPositionState[2];


    float deltaTime;
    float mVelocitySetpointCurrent[2];
    float mNECommand;
    float mNeutral;
    float mVelocityMax;
    float mMinCommand;
    float mMaxCommand;
    float mVelocityFeedforward;
    uint8_t mActive;
};

#endif // PIDCONTROLNE_H
