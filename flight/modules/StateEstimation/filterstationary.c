/**
 ******************************************************************************
 * @addtogroup OpenPilotModules OpenPilot Modules
 * @{
 * @addtogroup State Estimation
 * @brief Acquires sensor data and computes state estimate
 * @{
 *
 * @file       filterstationary.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2013.
 * @brief      Provides "fake" stationary state data for indoor mode
 *
 * @see        The GNU Public License (GPL) Version 3
 *
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

#include "inc/stateestimation.h"
// Fake pos rate in uS
#define FILTERSTATIONARY_FAKEPOSRATE 100000
// Private constants

#define STACK_REQUIRED               64

// Private types
// Private types
struct data {
    uint32_t lastUpdate;
};
// Private variables

// Private functions

static int32_t init(stateFilter *self);
static filterResult filter(stateFilter *self, stateEstimation *state);


int32_t filterStationaryInitialize(stateFilter *handle)
{
    handle->init      = &init;
    handle->filter    = &filter;
    handle->localdata = pios_malloc(sizeof(struct data));
    return STACK_REQUIRED;
}

static int32_t init(stateFilter *self)
{
    struct data *this = (struct data *)self->localdata;

    this->lastUpdate = PIOS_DELAY_GetRaw();
    return 0;
}

static filterResult filter(stateFilter *self, stateEstimation *state)
{
    struct data *this = (struct data *)self->localdata;

    if (PIOS_DELAY_DiffuS(this->lastUpdate) > FILTERSTATIONARY_FAKEPOSRATE) {
        this->lastUpdate = PIOS_DELAY_GetRaw();
        state->pos[0]    = 0.0f;
        state->pos[1]    = 0.0f;
        state->pos[2]    = 0.0f;
        state->updated  |= SENSORUPDATES_pos;

        state->vel[0]    = 0.0f;
        state->vel[1]    = 0.0f;
        state->vel[2]    = 0.0f;
        state->updated  |= SENSORUPDATES_vel;
    }
    return FILTERRESULT_OK;
}

/**
 * @}
 * @}
 */
