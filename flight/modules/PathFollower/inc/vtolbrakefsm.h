/**
 ******************************************************************************
 * @addtogroup OpenPilotModules OpenPilot Modules
 * @{
 * @addtogroup PathFollower FSM Brake
 * @brief Executes brake seqeuence
 * @{
 *
 * @file       vtolbrakfsm.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2015.
 * @brief      Executes brake sequence fsm
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
#ifndef VTOLBRAKEFSM_H
#define VTOLBRAKEFSM_H

#include "pathfollowerfsm.h"

// Brakeing state machine
typedef enum {
    BRAKE_STATE_INACTIVE = 0, // Inactive state is the initialised state on startup
    BRAKE_STATE_BRAKE, // Initiate altitude hold before starting descent
    BRAKE_STATE_HOLD, // Waiting for attainment of landing descent rate
    BRAKE_STATE_SIZE
} PathFollowerFSM_BrakeState_T;

typedef enum {
    FSMBRAKESTATUS_STATEEXITREASON_NONE = 0
} VtolBrakeFSMStatusStateExitReasonOptions;

class VtolBrakeFSM : public PathFollowerFSM {
private:
    static VtolBrakeFSM *p_inst;
    VtolBrakeFSM();

public:
    static VtolBrakeFSM *instance()
    {
        if (!p_inst) {
            p_inst = new VtolBrakeFSM();
        }
        return p_inst;
    }
    int32_t Initialize(VtolPathFollowerSettingsData *vtolPathFollowerSettings,
                       PathDesiredData *pathDesired,
                       FlightStatusData *flightStatus,
                       PathStatusData *ptr_pathStatus);
    void Inactive(void);
    void Activate(void);
    void Update(void);
    PathFollowerFSMState_T GetCurrentState(void);
    uint8_t PositionHoldState(void);

protected:

    // FSM instance data type
    typedef struct {
        PathFollowerFSM_BrakeState_T currentState;
        uint32_t stateRunCount;
        uint32_t stateTimeoutCount;
        float    sum1;
        float    sum2;
        uint8_t  observationCount;
        uint8_t  observation2Count;
    } VtolBrakeFSMData_T;

    // FSM state structure
    typedef struct {
        void(VtolBrakeFSM::*setup) (void); // Called to initialise the state
        void(VtolBrakeFSM::*run) (uint8_t); // Run the event detection code for a state
    } PathFollowerFSM_BrakeStateHandler_T;

    // Private variables
    VtolBrakeFSMData_T *mBrakeData;
    VtolPathFollowerSettingsData *vtolPathFollowerSettings;
    PathDesiredData *pathDesired;
    PathStatusData *pathStatus;
    FlightStatusData *flightStatus;

    void setup_brake(void);
    void run_brake(uint8_t);
    void initFSM(void);
    void setState(PathFollowerFSM_BrakeState_T newState, VtolBrakeFSMStatusStateExitReasonOptions reason);
    int32_t runState();
    // void updateVtolBrakeFSMStatus();

    void setStateTimeout(int32_t count);

    static PathFollowerFSM_BrakeStateHandler_T sBrakeStateTable[BRAKE_STATE_SIZE];
};

#endif // VTOLBRAKEFSM_H
