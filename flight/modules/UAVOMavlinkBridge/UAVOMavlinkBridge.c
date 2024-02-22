/**
 ******************************************************************************
 * @addtogroup OpenPilotModules OpenPilot Modules
 * @{
 * @addtogroup UAVOMavlinkBridge UAVO to Mavlink Bridge Module
 * @{
 *
 * @file       UAVOMavlinkBridge.c
 * @author     The LibrePilot Project, http://www.librepilot.org Copyright (C) 2016.
 *             dRonin, http://dRonin.org/, Copyright (C) 2015-2016
 *             Tau Labs, http://taulabs.org, Copyright (C) 2013-2014
 * @brief      Bridges selected UAVObjects to Mavlink
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
 *
 * Additional note on redistribution: The copyright and license notices above
 * must be maintained in each individual source file that is a derivative work
 * of this source file; otherwise redistribution is prohibited.
 */

// ****************
#include "openpilot.h"
#include "stabilizationdesired.h"
#include "flightbatterysettings.h"
#include "flightbatterystate.h"

#include "gpspositionsensor.h"
#include "accelsensor.h"
#include "airspeedsensor.h"
#include "barosensor.h"
#include "gyrosensor.h"
#include "gpsvelocitysensor.h"
#include "magsensor.h"

#include "manualcontrolcommand.h"
#include "attitudestate.h"
#include "airspeedstate.h"
#include "actuatordesired.h"
#include "flightstatus.h"
#include "systemstats.h"
#include "homelocation.h"
#include "positionstate.h"
#include "velocitystate.h"
#include "taskinfo.h"
#include "mavlink.h"
#include "hwsettings.h"
#include "oplinkreceiver.h"
#include "receiverstatus.h"
#include "manualcontrolsettings.h"
#include "actuatorcontrol.h"
#include "UAVOMavlinkBridge.h"

#include "custom_types.h"

#include <pios_board_io.h>
#include <limits.h>


#define OPLINK_LOW_RSSI  -110
#define OPLINK_HIGH_RSSI -10

// ****************
// Private functions

static void UAVOMavlinkBridgeTask(void *parameters);

// ****************
// Private constants

#if defined(PIOS_MAVLINK_STACK_SIZE)
#define STACK_SIZE_BYTES PIOS_MAVLINK_STACK_SIZE
#else
#define STACK_SIZE_BYTES 696
#endif

#define TASK_PRIORITY    (tskIDLE_PRIORITY)
#define TASK_RATE_HZ     100

// #define DEBUG_SENSOR_OUTPUT

const float GPS_PERIOD  = 0.1;
const float MAG_PERIOD  = 1.0 / 75.0; // ms
const float BARO_PERIOD = 1.0 / 20.0;

static void mavlink_send_extended_status();
static void mavlink_send_rc_channels();
static void mavlink_send_position();
static void mavlink_send_extra1();
static void mavlink_send_extra2();
static void mavlink_send_hil_actuator_controls();

static void send_mavlink_message(char *buffer, uint16_t len);
static void handle_message(char *buffer, uint16_t bytes_to_process);
static void mavlink_handle_heartbeat(const mavlink_message_t *msg);
static void mavlink_handle_hil_sensor(const mavlink_message_t *msg);
static void mavlink_handle_hil_gps(const mavlink_message_t *msg);

// ! Enumeration to use on the bitmask in HIL_SENSOR
typedef enum SensorSource {
    ACCEL = 0b111,
    GYRO  = 0b111000,
    MAG   = 0b111000000,
    BARO  = 0b1101000000000,
    DIFF_PRESS = 0b10000000000,
};

/*
 * custom part, ref @mavlink:common.h
 */
typedef enum MAV_DATA_STREAM_STY {
    MAV_DATA_STREAM_STY_HIL_ACTUATOR_CONTROLS = 14,
};

static const struct {
    uint8_t rate;
    void    (*handler)();
} mav_rates[] = {
    /*
       [MAV_DATA_STREAM_EXTENDED_STATUS] = {
       .rate    = 2, // Hz
       .handler = mavlink_send_extended_status,
       },
       [MAV_DATA_STREAM_RC_CHANNELS] =     {
       .rate    = 5, // Hz
       .handler = mavlink_send_rc_channels,
       },
       [MAV_DATA_STREAM_POSITION] =        {
       .rate    = 2, // Hz
       .handler = mavlink_send_position,
       },
       [MAV_DATA_STREAM_EXTRA1] =          {
       .rate    = 10, // Hz
       .handler = mavlink_send_extra1,
       },
       [MAV_DATA_STREAM_EXTRA2] =          {
       .rate    = 2, // Hz
       .handler = mavlink_send_extra2,
       },
     */
    [MAV_DATA_STREAM_STY_HIL_ACTUATOR_CONTROLS] = {
        .rate    = 10, // Hz
        .handler = mavlink_send_hil_actuator_controls,
    }
};

#define MAXSTREAMS NELEMENTS(mav_rates)

// ****************
// Private variables

static bool module_enabled = false;

static uint8_t *stream_ticks;

static mavlink_message_t *mav_msg;

static void updateSettings();

/**
 * Initialise the module
 * \return -1 if initialisation failed
 * \return 0 on success
 */
int32_t UAVOMavlinkBridgeStart(void)
{
    if (module_enabled) {
        // Start tasks
        xTaskHandle taskHandle;
        xTaskCreate(UAVOMavlinkBridgeTask, "UAVOMavlinkBridge", STACK_SIZE_BYTES / 4, NULL, TASK_PRIORITY + 1, &taskHandle);
        PIOS_TASK_MONITOR_RegisterTask(TASKINFO_RUNNING_UAVOMAVLINKBRIDGE, taskHandle);
        PIOS_WDG_RegisterFlag(PIOS_WDG_SENSORS);
        return 0;
    }
    return -1;
}
/**
 * Initialise the module
 * \return -1 if initialisation failed
 * \return 0 on success
 */
int32_t UAVOMavlinkBridgeInitialize(void)
{
    if (PIOS_COM_MAVLINK) {
        updateSettings();

        mav_msg = pios_malloc(sizeof(*mav_msg));
        stream_ticks = pios_malloc(MAXSTREAMS);

        if (mav_msg && stream_ticks) {
            for (unsigned x = 0; x < MAXSTREAMS; ++x) {
                if (mav_rates[x].rate == 0) {
                    stream_ticks[x] = INT_MAX;
                } else {
                    stream_ticks[x] = (TASK_RATE_HZ / mav_rates[x].rate);
                }
            }

            module_enabled = true;
        }

        // initialize sensor objs
        AccelSensorInitialize();
        BaroSensorInitialize();
        AirspeedSensorInitialize();
        GyroSensorInitialize();
        GPSPositionSensorInitialize();
        GPSVelocitySensorInitialize();
        MagSensorInitialize();
    }

    return 0;
}
MODULE_INITCALL(UAVOMavlinkBridgeInitialize, UAVOMavlinkBridgeStart);

static void send_message()
{
    uint16_t msg_length = MAVLINK_NUM_NON_PAYLOAD_BYTES +
                          mav_msg->len;

    PIOS_COM_SendBuffer(PIOS_COM_MAVLINK, &mav_msg->magic, msg_length);
}

static void send_mavlink_message(char *buffer, uint16_t len)
{
    PIOS_COM_SendBuffer(PIOS_COM_MAVLINK, buffer, len);
}

static void handle_message(char *buffer, uint16_t bytes_to_process)
{
    mavlink_message_t message;
    mavlink_status_t status;

    for (int i = 0; i < bytes_to_process; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &message, &status) == 1) {
#ifdef DEBUG_SENSOR_OUTPUT
            printf(
                "Received message %d from %d/%d\n",
                message.msgid, message.sysid, message.compid);
#endif
            switch (message.msgid) {
            case MAVLINK_MSG_ID_HEARTBEAT:
                mavlink_handle_heartbeat(&message);
                break;
            case MAVLINK_MSG_ID_HIL_SENSOR:
                mavlink_handle_hil_sensor(&message);
                break;
            case MAVLINK_MSG_ID_HIL_GPS:
                mavlink_handle_hil_gps(&message);
                break;
            }
        }
    }
}

/*
 * below handle mavlink messages
 */
static void mavlink_handle_heartbeat(const mavlink_message_t *msg)
{
    mavlink_heartbeat_t heartbeat;

    mavlink_msg_heartbeat_decode(msg, &heartbeat);

#ifdef DEBUG_SENSOR_OUTPUT
    printf("Got heartbeat from ");
    switch (heartbeat.autopilot) {
    case MAV_AUTOPILOT_GENERIC:
        printf("generic");
        break;
    case MAV_AUTOPILOT_ARDUPILOTMEGA:
        printf("ArduPilot");
        break;
    case MAV_AUTOPILOT_PX4:
        printf("PX4");
        break;
    default:
        printf("other");
        break;
    }
    printf(" autopilot\n");
#endif
}

static void mavlink_handle_hil_sensor(const mavlink_message_t *msg)
{
    mavlink_hil_sensor_t sensor_msg;

    mavlink_msg_hil_sensor_decode(msg, &sensor_msg);

    // 1.gyro acc
    if ((sensor_msg.fields_updated & (uint16_t)ACCEL) &&
        (sensor_msg.fields_updated & (uint16_t)GYRO)) {
        GyroSensorData gyro;
        gyro.x = sensor_msg.xgyro;
        gyro.y = sensor_msg.ygyro;
        gyro.z = sensor_msg.zgyro;
        GyroSensorSet(&gyro);

        AccelSensorData acc;
        acc.x = sensor_msg.xacc;
        acc.y = sensor_msg.yacc;
        acc.z = sensor_msg.zacc;
        AccelSensorSet(&acc);

#ifdef DEBUG_SENSOR_OUTPUT
        printf("Got hil acc gyro msgs\n");
        printf("acc-x:%f,acc-y:%f,acc-z:%f\n",
               acc.x, acc.y, acc.z);
        printf("gyro-x:%f,gyro-y:%f,gyro-z:%f\n",
               gyro.x, gyro.y, gyro.z);
#endif
    }
    // 2.mag, introduce lag
    static uint32_t last_mag_time = 0;
    if ((sensor_msg.fields_updated & (uint16_t)MAG) &&
        PIOS_DELAY_DiffuS(last_mag_time) / 1.0e6 > MAG_PERIOD) {
        MagSensorData mag;
        mag.x = sensor_msg.xmag;
        mag.y = sensor_msg.ymag;
        mag.z = sensor_msg.zmag;
        MagSensorSet(&mag);
        last_mag_time = PIOS_DELAY_GetRaw();

#ifdef DEBUG_SENSOR_OUTPUT
        printf("Got hil mag msgs\n");
        printf("mag-x:%f,mag-y:%f,mag-z:%f\n",
               mag.x, mag.y, mag.z);
#endif
    }
    // 3.baro, any time
    if (sensor_msg.fields_updated & (uint16_t)BARO) {
        BaroSensorData baro;
        baro.Altitude    = sensor_msg.pressure_alt;
        baro.Temperature = sensor_msg.temperature;
        baro.Pressure    = sensor_msg.abs_pressure;

#ifdef DEBUG_SENSOR_OUTPUT
        printf("Got hil baro msgs\n");
        printf("baro-a:%f,baro-t:%f,baro-p:%f\n",
               baro.Altitude,
               baro.Temperature,
               baro.Pressure);
#endif
    }
    // 4. TODO: Airespeed
}

static void mavlink_handle_hil_gps(const mavlink_message_t *msg)
{
    mavlink_hil_gps_t gps_msg;

    mavlink_msg_hil_gps_decode(msg, &gps_msg);
    // gps, introduce lag
    static uint32_t last_gps_time = 1000; // Delay by a millisecond
    if (PIOS_DELAY_DiffuS(last_gps_time) / 1.0e6 > GPS_PERIOD) {
        GPSPositionSensorData gpsPosition;
        GPSPositionSensorGet(&gpsPosition);
        gpsPosition.Latitude    = gps_msg.lat;
        gpsPosition.Longitude   = gps_msg.lon;
        gpsPosition.Altitude    = gps_msg.alt;
        gpsPosition.Groundspeed = gps_msg.vel;
        gpsPosition.Heading     = gps_msg.cog;
        gpsPosition.Satellites  = gps_msg.satellites_visible;
        gpsPosition.PDOP = 1;
        GPSPositionSensorSet(&gpsPosition);

        GPSVelocitySensorData gpsVelocity;
        GPSVelocitySensorGet(&gpsVelocity);
        gpsVelocity.North = gps_msg.vn;
        gpsVelocity.East  = gps_msg.ve;
        gpsVelocity.Down  = gps_msg.vd;
        GPSVelocitySensorSet(&gpsVelocity);

        last_gps_time     = PIOS_DELAY_GetRaw();

#ifdef DEBUG_SENSOR_OUTPUT
        printf("Got hil gps msgs\n");
        printf("lat: %d lon: %d\n",
               gpsPosition.Latitude, gpsPosition.Longitude);
        printf("gzlat:%d,gzlon:%d\n",
               gps_msg.lat, gps_msg.lon);
#endif
    }
}

/*
 * below send mavlink messages
 */
static void mavlink_send_extended_status()
{
#ifndef PIOS_EXCLUDE_ADVANCED_FEATURES
    FlightBatteryStateData batState;
    SystemStatsData systemStats;

    if (FlightBatteryStateHandle() != NULL) {
        FlightBatteryStateGet(&batState);
    }

    SystemStatsGet(&systemStats);

    uint32_t battery_capacity = 0;
    if (FlightBatterySettingsHandle() != NULL) {
        FlightBatterySettingsCapacityGet(&battery_capacity);
    }

    int8_t battery_remaining = 0;
    if (battery_capacity != 0) {
        if (batState.ConsumedEnergy < battery_capacity) {
            battery_remaining = 100 - lroundf(batState.ConsumedEnergy / battery_capacity * 100);
        }
    }

    /*
       mavlink_msg_sys_status_pack(0, 200, mav_msg,
                            // onboard_control_sensors_present Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
                            0,
                            // onboard_control_sensors_enabled Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
                            0,
                            // onboard_control_sensors_health Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
                            0,
                            // load Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
                            (uint16_t)systemStats.CPULoad * 10,
                            // voltage_battery Battery voltage, in millivolts (1 = 1 millivolt)
                            lroundf(batState.Voltage * 1000), // No need to check for validity, Voltage reads 0.0 when measurement is not configured,
                            // current_battery Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
                            lroundf(batState.Current * 100), // Same as for Voltage. 0 means no measurement
                            // battery_remaining Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
                            battery_remaining,
                            // drop_rate_comm Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
                            0,
                            // errors_comm Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
                            0,
                            // errors_count1 Autopilot-specific errors
                            0,
                            // errors_count2 Autopilot-specific errors
                            0,
                            // errors_count3 Autopilot-specific errors
                            0,
                            // errors_count4 Autopilot-specific errors
                            0);

     */
    send_message();
#endif /* PIOS_EXCLUDE_ADVANCED_FEATURES */
}

static void mavlink_send_rc_channels()
{
    ManualControlCommandData manualState;
    FlightStatusData flightStatus;
    SystemStatsData systemStats;

    ManualControlCommandGet(&manualState);
    FlightStatusGet(&flightStatus);
    SystemStatsGet(&systemStats);

    uint8_t mavlinkRssi;

    ManualControlSettingsChannelGroupsData channelGroups;
    ManualControlSettingsChannelGroupsGet(&channelGroups);

#ifdef PIOS_INCLUDE_OPLINKRCVR
    if (channelGroups.Throttle == MANUALCONTROLSETTINGS_CHANNELGROUPS_OPLINK) {
        int8_t rssi;
        OPLinkReceiverRSSIGet(&rssi);

        if (rssi < OPLINK_LOW_RSSI) {
            rssi = OPLINK_LOW_RSSI;
        } else if (rssi > OPLINK_HIGH_RSSI) {
            rssi = OPLINK_HIGH_RSSI;
        }

        mavlinkRssi = ((rssi - OPLINK_LOW_RSSI) * 255) / (OPLINK_HIGH_RSSI - OPLINK_LOW_RSSI);
    } else {
#endif /* PIOS_INCLUDE_OPLINKRCVR */
    uint8_t quality;
#ifndef SIMPOSIX
    ReceiverStatusQualityGet(&quality);
#endif

    // MAVLink RSSI's range is 0-255
    mavlinkRssi = (quality * 255) / 100;
#ifdef PIOS_INCLUDE_OPLINKRCVR
}
#endif

    mavlink_msg_rc_channels_raw_pack(0, 200, mav_msg,
                                     // time_boot_ms Timestamp (milliseconds since system boot)
                                     systemStats.FlightTime,
                                     // port Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.
                                     0,
                                     // chan1_raw RC channel 1 value, in microseconds
                                     manualState.Channel[0],
                                     // chan2_raw RC channel 2 value, in microseconds
                                     manualState.Channel[1],
                                     // chan3_raw RC channel 3 value, in microseconds
                                     manualState.Channel[2],
                                     // chan4_raw RC channel 4 value, in microseconds
                                     manualState.Channel[3],
                                     // chan5_raw RC channel 5 value, in microseconds
                                     manualState.Channel[4],
                                     // chan6_raw RC channel 6 value, in microseconds
                                     manualState.Channel[5],
                                     // chan7_raw RC channel 7 value, in microseconds
                                     manualState.Channel[6],
                                     // chan8_raw RC channel 8 value, in microseconds
                                     manualState.Channel[7],
                                     // rssi Receive signal strength indicator, 0: 0%, 255: 100%
                                     mavlinkRssi);

    send_message();
}

static void mavlink_send_position()
{
    SystemStatsData systemStats;

    SystemStatsGet(&systemStats);

    if (GPSPositionSensorHandle() != NULL) {
        GPSPositionSensorData gpsPosData;
        GPSPositionSensorGet(&gpsPosData);

        uint8_t gps_fix_type = 0;

        switch (gpsPosData.Status) {
        case GPSPOSITIONSENSOR_STATUS_NOGPS:
            gps_fix_type = 0;
            break;
        case GPSPOSITIONSENSOR_STATUS_NOFIX:
            gps_fix_type = 1;
            break;
        case GPSPOSITIONSENSOR_STATUS_FIX2D:
            gps_fix_type = 2;
            break;
        case GPSPOSITIONSENSOR_STATUS_FIX3D:
        case GPSPOSITIONSENSOR_STATUS_FIX3DDGNSS:
            gps_fix_type = 3;
            break;
        }

        /*
           // TODO: update to mavlink v2.0
           mavlink_msg_gps_raw_int_pack(0, 200, mav_msg,
                             // time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
                             (uint64_t)systemStats.FlightTime * 1000,
                             // fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
                             gps_fix_type,
                             // lat Latitude in 1E7 degrees
                             gpsPosData.Latitude,
                             // lon Longitude in 1E7 degrees
                             gpsPosData.Longitude,
                             // alt Altitude in 1E3 meters (millimeters) above MSL
                             gpsPosData.Altitude * 1000,
                             // eph GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
                             gpsPosData.HDOP * 100,
                             // epv GPS VDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
                             gpsPosData.VDOP * 100,
                             // vel GPS ground speed (m/s * 100). If unknown, set to: 65535
                             gpsPosData.Groundspeed * 100,
                             // cog Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
                             gpsPosData.Heading * 100,
                             // satellites_visible Number of satellites visible. If unknown, set to 255
                             gpsPosData.Satellites);
         */

        send_message();
    }

#ifndef PIOS_EXCLUDE_ADVANCED_FEATURES
    if (HomeLocationHandle() != NULL) {
        HomeLocationData homeLocation;
        HomeLocationGet(&homeLocation);

        /*
           mavlink_msg_gps_global_origin_pack(0, 200, mav_msg,
                                   // latitude Latitude (WGS84), expressed as * 1E7
                                   homeLocation.Latitude,
                                   // longitude Longitude (WGS84), expressed as * 1E7
                                   homeLocation.Longitude,
                                   // altitude Altitude(WGS84), expressed as * 1000
                                   homeLocation.Altitude * 1000);
         */

        send_message();
    }
#endif /* PIOS_EXCLUDE_ADVANCED_FEATURES */

    // TODO add waypoint nav stuff
    // wp_target_bearing
    // wp_dist = mavlink_msg_nav_controller_output_get_wp_dist(&msg);
    // alt_error = mavlink_msg_nav_controller_output_get_alt_error(&msg);
    // aspd_error = mavlink_msg_nav_controller_output_get_aspd_error(&msg);
    // xtrack_error = mavlink_msg_nav_controller_output_get_xtrack_error(&msg);
    // mavlink_msg_nav_controller_output_pack
    // wp_number
    // mavlink_msg_mission_current_pack
}

static void mavlink_send_extra1()
{
    AttitudeStateData attState;
    SystemStatsData systemStats;

    AttitudeStateGet(&attState);
    SystemStatsGet(&systemStats);

    mavlink_msg_attitude_pack(0, 200, mav_msg,
                              // time_boot_ms Timestamp (milliseconds since system boot)
                              systemStats.FlightTime,
                              // roll Roll angle (rad)
                              DEG2RAD(attState.Roll),
                              // pitch Pitch angle (rad)
                              DEG2RAD(attState.Pitch),
                              // yaw Yaw angle (rad)
                              DEG2RAD(attState.Yaw),
                              // rollspeed Roll angular speed (rad/s)
                              0,
                              // pitchspeed Pitch angular speed (rad/s)
                              0,
                              // yawspeed Yaw angular speed (rad/s)
                              0);

    send_message();
}

static inline float Sq(float x)
{
    return x * x;
}

#define IS_STAB_MODE(d, m) (((d).Roll == (m)) && ((d).Pitch == (m)))

static void mavlink_send_extra2()
{
    float airspeed    = 0;
    float altitude    = 0;
    float groundspeed = 0;
    float climbrate   = 0;


#ifndef PIOS_EXCLUDE_ADVANCED_FEATURES
    if (AirspeedStateHandle() != NULL) {
        AirspeedStateTrueAirspeedGet(&airspeed);
    }

    if (PositionStateHandle() != NULL) {
        PositionStateDownGet(&altitude);
        altitude *= -1;
    }

    if (VelocityStateHandle() != NULL) {
        VelocityStateData velocityState;
        VelocityStateGet(&velocityState);

        groundspeed = sqrtf(Sq(velocityState.North) + Sq(velocityState.East));
        climbrate   = velocityState.Down * -1;
    }
#endif

    float attitudeYaw = 0;

    AttitudeStateYawGet(&attitudeYaw);
    // round attState.Yaw to nearest int and transfer from (-180 ... 180) to (0 ... 360)
    int16_t heading = lroundf(attitudeYaw);
    if (heading < 0) {
        heading += 360;
    }


    float thrust;
    ActuatorDesiredThrustGet(&thrust);

    mavlink_msg_vfr_hud_pack(0, 200, mav_msg,
                             // airspeed Current airspeed in m/s
                             airspeed,
                             // groundspeed Current ground speed in m/s
                             groundspeed,
                             // heading Current heading in degrees, in compass units (0..360, 0=north)
                             heading,
                             // throttle Current throttle setting in integer percent, 0 to 100
                             thrust * 100,
                             // alt Current altitude (MSL), in meters
                             altitude,
                             // climb Current climb rate in meters/second
                             climbrate);

    send_message();

    FlightStatusData flightStatus;
    FlightStatusGet(&flightStatus);

    StabilizationDesiredStabilizationModeData stabModeData;
    StabilizationDesiredStabilizationModeGet(&stabModeData);

    uint8_t armed_mode = 0;
    if (flightStatus.Armed == FLIGHTSTATUS_ARMED_ARMED) {
        armed_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    uint8_t custom_mode = CUSTOM_MODE_STAB;

    switch (flightStatus.FlightMode) {
    case FLIGHTSTATUS_FLIGHTMODE_POSITIONHOLD:
        custom_mode = CUSTOM_MODE_PHLD;
        break;

    case FLIGHTSTATUS_FLIGHTMODE_RETURNTOBASE:
        custom_mode = CUSTOM_MODE_RTL;
        break;

    case FLIGHTSTATUS_FLIGHTMODE_AUTOCRUISE:
    case FLIGHTSTATUS_FLIGHTMODE_AUTOTAKEOFF:
    case FLIGHTSTATUS_FLIGHTMODE_PATHPLANNER:
        custom_mode = CUSTOM_MODE_AUTO;
        break;

    case FLIGHTSTATUS_FLIGHTMODE_LAND:
        custom_mode = CUSTOM_MODE_LAND;
        break;

    case FLIGHTSTATUS_FLIGHTMODE_MANUAL:
        custom_mode = CUSTOM_MODE_ACRO; // or
        break;
    case FLIGHTSTATUS_FLIGHTMODE_AUTOTUNE:
        custom_mode = CUSTOM_MODE_ATUN;
        break;
    case FLIGHTSTATUS_FLIGHTMODE_STABILIZED1:
    case FLIGHTSTATUS_FLIGHTMODE_STABILIZED2:
    case FLIGHTSTATUS_FLIGHTMODE_STABILIZED3:
    case FLIGHTSTATUS_FLIGHTMODE_STABILIZED4:
    case FLIGHTSTATUS_FLIGHTMODE_STABILIZED5:
    case FLIGHTSTATUS_FLIGHTMODE_STABILIZED6:

        if (IS_STAB_MODE(stabModeData, STABILIZATIONDESIRED_STABILIZATIONMODE_RATE)
            || IS_STAB_MODE(stabModeData, STABILIZATIONDESIRED_STABILIZATIONMODE_RATETRAINER)) {
            custom_mode = CUSTOM_MODE_ACRO;
        } else if (IS_STAB_MODE(stabModeData, STABILIZATIONDESIRED_STABILIZATIONMODE_ACRO)) { // this is Acro+ mode
            custom_mode = CUSTOM_MODE_ACRO;
        } else if (IS_STAB_MODE(stabModeData, STABILIZATIONDESIRED_STABILIZATIONMODE_RATTITUDE)) {
            custom_mode = CUSTOM_MODE_SPORT;
        } else if ((stabModeData.Thrust == STABILIZATIONDESIRED_STABILIZATIONMODE_ALTITUDEHOLD)
                   || (stabModeData.Thrust == STABILIZATIONDESIRED_STABILIZATIONMODE_ALTITUDEVARIO)) {
            custom_mode = CUSTOM_MODE_ALTH;
        } else { // looks like stabilized mode, whichever it is..
            custom_mode = CUSTOM_MODE_STAB;
        }

        break;
    case FLIGHTSTATUS_FLIGHTMODE_COURSELOCK:
    case FLIGHTSTATUS_FLIGHTMODE_VELOCITYROAM:
    case FLIGHTSTATUS_FLIGHTMODE_HOMELEASH:
    case FLIGHTSTATUS_FLIGHTMODE_ABSOLUTEPOSITION:
    case FLIGHTSTATUS_FLIGHTMODE_POI:
        custom_mode = CUSTOM_MODE_STAB; // Don't know any better
        break;
    }

    mavlink_msg_heartbeat_pack(0, 200, mav_msg,
                               // type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
                               MAV_TYPE_GENERIC,
                               // autopilot Autopilot type / class. defined in MAV_AUTOPILOT ENUM
                               MAV_AUTOPILOT_GENERIC, // or MAV_AUTOPILOT_OPENPILOT
                               // base_mode System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
                               armed_mode,
                               // custom_mode A bitfield for use for autopilot-specific flags.
                               custom_mode,
                               // system_status System status flag, see MAV_STATE ENUM
                               0);

    send_message();
}

static void mavlink_send_hil_actuator_controls()
{
    SystemStatsData systemStats;

    SystemStatsGet(&systemStats);

    FlightStatusData flightStatus;
    FlightStatusGet(&flightStatus);

    ActuatorControlData ctrl;
    ActuatorControlGet(&ctrl);

    uint8_t armed_mode = 0;
    if (flightStatus.Armed == FLIGHTSTATUS_ARMED_ARMED) {
        armed_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    mavlink_msg_hil_actuator_controls_pack_chan(
        0 /* system id */,
        200 /* component id */,
        MAVLINK_COMM_0,
        mav_msg,
        (uint64_t)systemStats.FlightTime * 1000,
        ctrl.controls,
        armed_mode,
        // custom_mode A bitfield for use
        // for autopilot-specific flags.
        0);
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const int len = mavlink_msg_to_send_buffer(buffer, mav_msg);
#ifdef DEBUG_SENSOR_OUTPUT
    printf("Sending actuator control messages..\n");
    printf("%f,%f,%f,%f\n%f,%f,%f,%f\n%f,%f,%f,%f\n",
           ctrl.controls[0], ctrl.controls[1],
           ctrl.controls[2], ctrl.controls[3],
           ctrl.controls[4], ctrl.controls[5],
           ctrl.controls[6], ctrl.controls[7],
           ctrl.controls[8], ctrl.controls[9],
           ctrl.controls[10], ctrl.controls[11]);
    printf("arm state: %d\n",
           armed_mode & MAV_MODE_FLAG_SAFETY_ARMED);
#endif
    send_mavlink_message(buffer, len);
}

/**
 * Main task. It does not return.
 */

static void UAVOMavlinkBridgeTask(__attribute__((unused)) void *parameters)
{
    uint32_t lastSysTime;

    // Main task loop
    lastSysTime = xTaskGetTickCount(); // portTICK_RATE_MS;
    AlarmsClear(SYSTEMALARMS_ALARM_SENSORS);

    while (1) {
        vTaskDelayUntil(&lastSysTime, (1000 / TASK_RATE_HZ) / portTICK_RATE_MS);
        PIOS_WDG_UpdateFlag(PIOS_WDG_SENSORS);

        // 1.send simulation mav msgs
        for (unsigned i = 0; i < MAXSTREAMS; ++i) {
            if (!mav_rates[i].rate || !mav_rates[i].handler) {
                continue;
            }

            if (stream_ticks[i] == 0) {
                // trigger now
                uint8_t rate = mav_rates[i].rate;
                if (rate > TASK_RATE_HZ) {
                    rate = TASK_RATE_HZ;
                }
                stream_ticks[i] = TASK_RATE_HZ / rate;

                mav_rates[i].handler();
            }

            --stream_ticks[i];
        }

        // 2. handle received msgs
        char buffer[2048]; // enough for MTU 1500 bytes
        uint16_t bytes_to_process = 0;
        bytes_to_process = PIOS_COM_ReceiveBuffer(PIOS_COM_MAVLINK, buffer,
                                                  sizeof(buffer), 10 /* ms timeout */);
        handle_message(buffer, bytes_to_process);
    }
}

static uint32_t hwsettings_mavlinkspeed_enum_to_baud(uint8_t baud)
{
    switch (baud) {
    case HWSETTINGS_MAVLINKSPEED_2400:
        return 2400;

    case HWSETTINGS_MAVLINKSPEED_4800:
        return 4800;

    case HWSETTINGS_MAVLINKSPEED_9600:
        return 9600;

    case HWSETTINGS_MAVLINKSPEED_19200:
        return 19200;

    case HWSETTINGS_MAVLINKSPEED_38400:
        return 38400;

    case HWSETTINGS_MAVLINKSPEED_57600:
        return 57600;

    default:
    case HWSETTINGS_MAVLINKSPEED_115200:
        return 115200;
    }
}


static void updateSettings()
{
    if (PIOS_COM_MAVLINK) {
        // Retrieve settings
        HwSettingsMAVLinkSpeedOptions mavlinkSpeed;
        HwSettingsMAVLinkSpeedGet(&mavlinkSpeed);

        PIOS_COM_ChangeBaud(PIOS_COM_MAVLINK, hwsettings_mavlinkspeed_enum_to_baud(mavlinkSpeed));
    }
}
/**
 * @}
 * @}
 */
