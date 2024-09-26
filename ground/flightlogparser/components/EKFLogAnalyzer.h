/**
 * @file: EKFLogAnalyzer.h
 * @brief: class decl for EKF LogEntry Analyze
 * @author: zhangxin@santypilot
 * @date: 2024-3-17
 */
#ifndef _EKF_LOG_ANALYZER_H
#define _EKF_LOG_ANALYZER_H

#include "LogAnalyzer.h"
#include <iostream>
#include <cmath>
#include "filterstates.h"
#include <ekfconfiguration.h>
#include <ekfstatevariance.h>
#include <homelocation.h>

extern "C" {
#include "libraries/inc/CoordinateConversions.h"
#include "libraries/math/mathmisc.h"
#include "libraries/inc/insgps.h"
}

namespace santypilot_gcs {
namespace flightlogparser {
using RElem = EKFConfiguration::RElem;
using QElem = EKFConfiguration::QElem;

class EKFLogAnalyzer: public LogAnalyzer {
public:
	// input data type
	using DataType = std::map<std::string, 
		  std::vector<std::pair<size_t, double>>>;

	struct LocalData {
		int32_t init_stage;
		bool  inited;
		bool  navOnly;
		float magLockAlpha;
	};
	struct SensorData {};

	// 1 filter() replay bug 
	// 2 show states
	// 3 evaluate bug occurs: do some warning mechanism 
	// 4 mock by step
	// 5 maybe which matrix init too big or calc not recursive
	virtual void analyze() {
		// 1. filter data
		if (!_inited) {
			// init vars
			_config_data = _config.getData();
			_location_data = _homelocation.getData();
			{
				_location_data.Latitude = 312147396;
				_location_data.Longitude = 1214073671;
				_location_data.Altitude = 0;
				_location_data.Be[0] = 26000;
				_location_data.Be[1] = 400;
				_location_data.Be[2] = 40000;
			}
			float magnorm[3] = { _location_data.Be[0], 
				_location_data.Be[1], _location_data.Be[2] };
			// print_diag(magnorm, 3);

			vector_normalizef(magnorm, 3);
			const float down[3] = { 0, 0, 1 };
			float cross[3];
			CrossProduct(down, magnorm, cross);
			_inited = true;
			// init algo
            INSGPSInit();
			INSSetArmed(false); // debug not armed state
			INSSetMagNorth(_location_data.Be);

			// set variance
            float Be2 = pow(_location_data.Be[0], 2) + 
				pow(_location_data.Be[1], 2) + pow(_location_data.Be[2], 2);

			float pos_r[3] = { _config_data.R[RElem::R_GPSPOSNORTH],
							   _config_data.R[RElem::R_GPSPOSEAST],
							   _config_data.R[RElem::R_GPSPOSDOWN]};
			float vel_r[3] = { _config_data.R[RElem::R_GPSVELNORTH],
							   _config_data.R[RElem::R_GPSVELEAST],
							   _config_data.R[RElem::R_GPSVELDOWN]};
			float mag_r[3] = { _config_data.R[RElem::R_MAGX] /*/ Be2*/,
							  _config_data.R[RElem::R_MAGY] /*/ Be2*/,
							  _config_data.R[RElem::R_MAGZ] /*/ Be2*/ };
			float acc_q[3] = { _config_data.Q[QElem::Q_ACCELX],
							   _config_data.Q[QElem::Q_ACCELY],
							   _config_data.Q[QElem::Q_ACCELZ] };
			float gyro_q[3] = { _config_data.Q[QElem::Q_GYROX],
							    _config_data.Q[QElem::Q_GYROY],
							    _config_data.Q[QElem::Q_GYROZ] };
			float gyro_biasq[3] = { _config_data.Q[QElem::Q_GYRODRIFTX],
								   _config_data.Q[QElem::Q_GYRODRIFTY],
								   _config_data.Q[QElem::Q_GYRODRIFTZ] };
			INSSetPosVelVar(pos_r, vel_r);
            INSSetMagVar(mag_r);
            INSSetBaroVar(_config_data.R[RElem::R_BAROZ]);
            INSSetAccelVar(acc_q);
            INSSetGyroVar(gyro_q);
            INSSetGyroBiasVar(gyro_biasq);
		}
		// never set P use default in insgps13state.c
        print_init_noise_matrix();
		print_noise_matrix();
		// 2. call filter functions 
		// & evaluate states
		size_t data_len = _data[LC_FIELD + "dT"].size();
		for (size_t i = 0; i < data_len; i++) {
			std::cout << "\n\n\n\n\n\n" << i << "st calculation\n";
            // 2.1 state prediction
            // prepare sensors
			// Note: fields below must be parsed!
			// or will core dump
			float gyro[3] = {
				_data[LC_FIELD + "gx"][i].second, 
				_data[LC_FIELD + "gy"][i].second,
			    _data[LC_FIELD + "gz"][i].second};
			float acc[3] = {
			    _data[LC_FIELD + "ax"][i].second,
				_data[LC_FIELD + "ay"][i].second,
				_data[LC_FIELD + "az"][i].second};
			float dT = _data[LC_FIELD + "dT"][i].second;
            float mag[3] = {
                _data[LC_FIELD + "mx"][i].second,
                _data[LC_FIELD + "my"][i].second,
                _data[LC_FIELD + "mz"][i].second
            };
            float pos[3] = {
                _data[LC_FIELD + "GPSNorth"][i].second,
                _data[LC_FIELD + "GPSEast"][i].second,
                _data[LC_FIELD + "GPSDown"][i].second
            };
            float vel[3] = {
                _data[LC_FIELD + "GPSVelNorth"][i].second,
                _data[LC_FIELD + "GPSVelEast"][i].second,
                _data[LC_FIELD + "GPSVelDown"][i].second
            };
            float baro = _data[LC_FIELD + "Altitude"][i].second;

            // evaluate
            // TODO: add title
            std::cout << "sensors:\n";
            print_input_sensors(acc, gyro,
                mag, &baro, pos, vel);
			INSStatePrediction(gyro, acc, dT);
            // evaluate
            std::cout << "states prediction\n";
            print_state_vars(Nav.Pos, Nav.Vel, Nav.q, Nav.gyro_bias);

            // 2.2 covariance prediction
            INSCovariancePrediction(dT);
            std::cout << "noise on prediction\n";
            print_noise_matrix(); // P prediction, Q R const

            // 2.3 correction all
            uint16_t sensors = 0;
            sensors |= MAG_SENSORS | BARO_SENSOR | 
                POS_SENSORS | HORIZ_SENSORS | VERT_SENSORS;
            INSCorrection(mag, pos, vel, baro, sensors);
            // evaluate expected
            float pos_e[3] = {
                _data[LC_FIELD + "North"][i].second,
                _data[LC_FIELD + "East"][i].second,
                _data[LC_FIELD + "Down"][i].second
            };
            float vel_e[3] = {
                _data[LC_FIELD + "NorthVel"][i].second,
                _data[LC_FIELD + "EastVel"][i].second,
                _data[LC_FIELD + "DownVel"][i].second
            };
            float q_e[4] = {
                _data[LC_FIELD + "q1"][i].second,
                _data[LC_FIELD + "q2"][i].second,
                _data[LC_FIELD + "q3"][i].second,
                _data[LC_FIELD + "q4"][i].second,
            };
            float gyro_bias_e[3] = {0.0, 0.0, 0.0};
            std::cout << "expected state from log data:\n";
            print_state_vars(pos_e, vel_e, q_e, gyro_bias_e);
            std::cout << "calculated after correction\n";
            print_state_vars(Nav.Pos, Nav.Vel, Nav.q, Nav.gyro_bias);
            std::cout << "noise after correction\n";
            print_noise_matrix(); // P prediction, Q R const
		}
	    return;
	}
private:
	// check for isnan or close to zero
	static inline bool invalid_var(float data) {
		if (std::isnan(data) || std::isinf(data)) {
			return true;
		}
		if (data < 1e-15f) { 
			// var should not be close to zero. 
			// And not negative either.
			return true;
		}
		return false;
	}
	// x: 13 * 1 p3, v3, q4, gb3
	// u: 6 * 1 g3, a3
	// y: 10 * 1 p3, v3, mag3, baro1
	// x,u -> x
	// P: 13 * 13
	// Q: 9 * 9
	// R: 10 * 10
	// P = FPF^T + GQG^T
	// K = PH^T(HPH^T + R)
	// x = x + K(y - h(x,u))
	// P = (I - KH)P
	// F 13 * 13 
	// G 13 * 9
	// H 10 * 13
	// K 13 * 10

    // helper functions to print intermediate variables
    void print_matrix(float* data, const std::vector<size_t>& dim,
            const std::string& title = "") {
        if (dim.size() != 2) {
            std::cout << "dim size not correct\n";
            return;
        }
        if (!title.empty()) {
            std::cout << "***" << title << "***\n";
        }
        for (size_t i = 0; i < dim[0]; i++) {
            for (size_t j = 0; j < dim[1]; j++) {
                std::cout << data[i * dim[1] + j] << " ";
            }
            std::cout << std::endl;
        }
        return;
    }

	void print_diag(float* data, const size_t dim, const std::string& title = "") {
        if (!title.empty()) {
            std::cout << "***" << title << "***\n";
        }
	    for (size_t i = 0; i < dim; i++) {
			for (size_t j = 0; j < i; j++) {
				std::cout << " ";
			}
			std::cout << data[i] << std::endl;
		}
		std::cout << std::endl;
	}

    void print_init_noise_matrix() {
		print_diag(_location_data.Be, 3, "home location be");
		print_diag(_config_data.P, 13, "init P matrix");
		print_diag(_config_data.Q, 9, "init Q matrix");
		print_diag(_config_data.R, 10, "init R matrix");
    }

    // print_state_vars(Nav.Pos, Nav.Vel, Nav.q, Nav.gyro_bias);
    // for estimated also for expected
    void print_state_vars(float* pos, float* vel, float* q,
            float* gyro_bias) {
        print_matrix(pos, {1, 3}, "position");
        print_matrix(vel, {1, 3}, "velocity");
        print_matrix(q, {1, 4}, "quaternion");
        print_matrix(gyro_bias, {1, 3}, "gyro bias");
    } // x
    void print_input_sensors(float* acc, float* gyro,
            float* mag, float* baro, float* GPS, float* GPSVel) {
        print_matrix(acc, {1, 3}, "accel");
        print_matrix(gyro, {1, 3}, "gyro");
        print_matrix(mag, {1, 3}, "magnetrometer");
        print_matrix(baro, {1, 1}, "barometer");
        print_matrix(GPS, {1, 3}, "gps");
        print_matrix(GPSVel, {1, 3}, "gps velocity");
    } // u y
    /*
        extern struct EKFData {
            // linearized system matrices
            float F[NUMX][NUMX];
            float G[NUMX][NUMW];
            float H[NUMV][NUMX];
            // local magnetic unit vector in NED frame
            float Be[3];
            // covariance matrix and state vector
            float P[NUMX][NUMX];
            float X[NUMX];
            // input noise and measurement noise variances
            float Q[NUMW];
            float R[NUMV];
        } ekf;
     */
    void print_intermediate_vars() {
        print_matrix(ekf.F[0], {13, 13}, "F matrix");
        print_matrix(ekf.G[0], {13, 9}, "G matrix");
        print_matrix(ekf.H[0], {10, 13}, "H matrix");
    } // FGH
    void print_noise_matrix() {
        print_matrix(ekf.P[0], {13, 13}, "P matrix");
        print_matrix(ekf.Q, {1, 9}, "Q matrix");
        print_matrix(ekf.R, {1, 10}, "R matrix");
    } // PQR
    void print_kalman_gain() {}
	// do filter StateEstimation/filterekf.c
	bool _inited = false;
	const std::string LC_FIELD = "filterstates:";
	EKFConfiguration _config;
	EKFConfiguration::DataFields _config_data;
	HomeLocation _homelocation;
	HomeLocation::DataFields _location_data;
}; // EKFLogAnalyzer
} // flightlogparser
} // santypilot_gcs

#endif // _EKF_LOG_ANALYZER_H
