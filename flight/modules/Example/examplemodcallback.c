/**
 ******************************************************************************
 *
 * @file       examplemodcallback.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Example module to be used as a template for actual modules.
 *             Event callback version.
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

/**
 * Input objects: ExampleObject1, ExampleSettings
 * Output object: ExampleObject2
 *
 * This module executes in response to ExampleObject1 updates. When the
 * module is triggered it will update the data of ExampleObject2.
 *
 * No threads are used in this example.
 *
 * UAVObjects are automatically generated by the UAVObjectGenerator from
 * the object definition XML file.
 *
 * Modules have no API, all communication to other modules is done through UAVObjects.
 * However modules may use the API exposed by shared libraries.
 * See the OpenPilot wiki for more details.
 * http://www.openpilot.org/OpenPilot_Application_Architecture
 *
 */

#include "openpilot.h"
#include "callbackinfo.h" // object needed for callback id macro CALLBACKINFO_RUNNING_<MODULENAME>
#include "exampleobject1.h" // object the module will listen for updates (input)
#include "exampleobject2.h" // object the module will update (output)
#include "examplesettings.h" // object holding module settings (input)

// Private constants
#define STACK_SIZE        configMINIMAL_STACK_SIZE
#define CALLBACK_PRIORITY CALLBACK_PRIORITY_LOW
#define CBTASK_PRIORITY   CALLBACK_TASKPRIORITY_AUXILIARY
// Private types

// Private variables
static DelayedCallbackInfo *cbinfo;

// Private functions
static void ObjectUpdatedCb(UAVObjEvent *ev);

static void DelayedCb();
/**
 * Initialise the module, called on startup.
 * \returns 0 on success or -1 if initialisation failed
 */
int32_t ExampleModCallbackInitialize()
{
    // Listen for ExampleObject1 updates, connect a callback function
    ExampleObject1ConnectCallback(&ObjectUpdatedCb);

    cbinfo = PIOS_CALLBACKSCHEDULER_Create(&DelayedCb, CALLBACK_PRIORITY, CBTASK_PRIORITY, CALLBACKINFO_RUNNING_EXAMPLE, STACK_SIZE);

    return 0;
}

/**
 * This function is called each time ExampleObject1 is updated, this could be
 * a local update or a remote update from the GCS. In this example the module
 * does not have its own thread, the callbacks are executed from within the
 * event thread. Because of that the callback execution time must be kept to
 * a minimum.
 */
static void ObjectUpdatedCb(__attribute__((unused)) UAVObjEvent *ev)
{
    PIOS_CALLBACKSCHEDULER_Dispatch(cbinfo);
}

/**
 * This function is called by the PIOS_CALLBACKSCHEDULER_Scheduler when its execution
 * has been requested.  Callbacks scheduled for execution are executed in the
 * same thread in a round robin fashion. The Dispatch function to reschedule
 * execution can be called from within the Callback itself, in which case the
 * re-run will be scheduled after all other callback with equal or higher
 * priority have been executed.  Like event callbacks, delayed callbacks are
 * executed in the same thread context one at a time, therefore blocking IO
 * functions or very long lasting calculations are prohibited. Unlike Event
 * callbacks these callbacks run with a standard (IDLE+1) thread priority and
 * do not block regular threads. They are therefore saver to use.
 */
static void DelayedCb();
ExampleSettingsData settings;
ExampleObject1Data data1;
ExampleObject2Data data2;
int32_t step;

// Update settings with latest value
ExampleSettingsGet(&settings);

// Get the input object
ExampleObject1Get(&data1);

// Determine how to update the output object
if (settings.StepDirection == EXAMPLESETTINGS_STEPDIRECTION_UP) {
    step = settings.StepSize;
} else {
    step = -settings.StepSize;
}

// Update data
data2.Field1    = data1.Field1 + step;
data2.Field2    = data1.Field2 + step;
data2.Field3    = data1.Field3 + step;
data2.Field4[0] = data1.Field4[0] + step;
data2.Field4[1] = data1.Field4[1] + step;

// Update the ExampleObject2, after this function is called
// notifications to any other modules listening to that object
// will be sent and the GCS object will be updated through the
// telemetry link. All operations will take place asynchronously
// and the following call will return immediately.
ExampleObject2Set(&data2);

// call the module again 10 seconds later,
// even if the exampleobject has not been updated
PIOS_CALLBACKSCHEDULER_Schedule(cbinfo, 10 * 1000, CALLBACK_UPDATEMODE_NONE);
}
