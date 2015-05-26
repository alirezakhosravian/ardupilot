/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *  NavEKF based AHRS (Attitude Heading Reference System) interface for
 *  ArduPilot
 *
 */
#include <AP_HAL.h>
#include <AP_AHRS.h>
#include <AP_Vehicle.h>

#if AP_AHRS_NAVEKF_AVAILABLE

extern const AP_HAL::HAL& hal;

// return the smoothed gyro vector corrected for drift
const Vector3f &AP_AHRS_NavEKF::get_gyro(void) const
{
    switch (using_EKF()) {
    case AHRS_SELECTED_EKF1:
    case AHRS_SELECTED_EKF1_NO_FALLBACK:
        return ekf1._gyro_estimate;
    case AHRS_SELECTED_EKF2:
        return ekf2._gyro_estimate;
    case AHRS_SELECTED_DCM:
    default:
        break;
    }
    return AP_AHRS_DCM::get_gyro();
}

const Matrix3f &AP_AHRS_NavEKF::get_dcm_matrix(void) const
{
    switch (using_EKF()) {
    case AHRS_SELECTED_EKF1:
    case AHRS_SELECTED_EKF1_NO_FALLBACK:
        return ekf1._dcm_matrix;
    case AHRS_SELECTED_EKF2:
        return ekf2._dcm_matrix;
    case AHRS_SELECTED_DCM:
    default:
        break;
    }
    return AP_AHRS_DCM::get_dcm_matrix();
}

const Vector3f &AP_AHRS_NavEKF::get_gyro_drift(void) const
{
    switch (using_EKF()) {
    case AHRS_SELECTED_EKF1:
    case AHRS_SELECTED_EKF1_NO_FALLBACK:
        return ekf1._gyro_bias;
    case AHRS_SELECTED_EKF2:
        return ekf2._gyro_bias;
    case AHRS_SELECTED_DCM:
    default:
        break;
    }
    return AP_AHRS_DCM::get_gyro_drift();
}

// reset the current gyro drift estimate
//  should be called if gyro offsets are recalculated
void AP_AHRS_NavEKF::reset_gyro_drift(void)
{
    // update DCM
    AP_AHRS_DCM::reset_gyro_drift();

    // reset the EKF gyro bias states
    EKF1.resetGyroBias();
    EKF2.resetGyroBias();
}

void AP_AHRS_NavEKF::update(void)
{
    // we need to restore the old DCM attitude values as these are
    // used internally in DCM to calculate error values for gyro drift
    // correction
    roll = _dcm_attitude.x;
    pitch = _dcm_attitude.y;
    yaw = _dcm_attitude.z;
    update_cd_values();

    AP_AHRS_DCM::update();

    // keep DCM attitude available for get_secondary_attitude()
    _dcm_attitude(roll, pitch, yaw);

    _update_ekf1();
    _update_ekf2();
}


void AP_AHRS_NavEKF::_update_ekf1(void)
{
    if (!ekf1.started) {
        // wait 1 second for DCM to output a valid tilt error estimate
        if (ekf1.start_time_ms == 0) {
            ekf1.start_time_ms = hal.scheduler->millis();
        }
        if (hal.scheduler->millis() - ekf1.start_time_ms > startup_delay_ms) {
            ekf1.started = EKF1.InitialiseFilterDynamic();
        }
    }
    if (ekf1.started) {
        EKF1.UpdateFilter();
        EKF1.getRotationBodyToNED(ekf1._dcm_matrix);
        if (using_EKF() == AHRS_SELECTED_EKF1 ||
            using_EKF() == AHRS_SELECTED_EKF1_NO_FALLBACK) {
            ekf1._gyro_estimate += ekf1._gyro_bias;

            float abias1, abias2;
            EKF1.getAccelZBias(abias1, abias2);

            // update _accel_ef_ekf
            for (uint8_t i=0; i<_ins.get_accel_count(); i++) {
                Vector3f accel = _ins.get_accel(i);
                if (i==0) {
                    accel.z -= abias1;
                } else if (i==1) {
                    accel.z -= abias2;
                }
                if (_ins.get_accel_health(i)) {
                    ekf1._accel_ef_ekf[i] = ekf1._dcm_matrix * accel;
                }
            }

            if(_ins.get_accel_health(0) && _ins.get_accel_health(1)) {
                float IMU1_weighting;
                EKF1.getIMU1Weighting(IMU1_weighting);
                ekf1._accel_ef_ekf_blended = ekf1._accel_ef_ekf[0] * IMU1_weighting + ekf1._accel_ef_ekf[1] * (1.0f-IMU1_weighting);
            } else {
                ekf1._accel_ef_ekf_blended = ekf1._accel_ef_ekf[0];
            }


            Vector3f eulers;
            EKF1.getEulerAngles(eulers);
            roll  = eulers.x;
            pitch = eulers.y;
            yaw   = eulers.z;

            update_cd_values();
            update_trig();

            // keep _gyro_bias for get_gyro_drift()
            EKF1.getGyroBias(ekf1._gyro_bias);
            ekf1._gyro_bias = -ekf1._gyro_bias;

            // calculate corrected gryo estimate for get_gyro()
            ekf1._gyro_estimate.zero();
            uint8_t healthy_count = 0;
            for (uint8_t i=0; i<_ins.get_gyro_count(); i++) {
                if (_ins.get_gyro_health(i) && healthy_count < 2) {
                    ekf1._gyro_estimate += _ins.get_gyro(i);
                    healthy_count++;
                }
            }
            if (healthy_count > 1) {
                ekf1._gyro_estimate /= healthy_count;
            }

            ekf1._gyro_estimate += ekf1._gyro_bias;
        }
    }
}


void AP_AHRS_NavEKF::_update_ekf2(void)
{
    if (!ekf2.started) {
        // wait 1 second for DCM to output a valid tilt error estimate
        if (ekf2.start_time_ms == 0) {
            ekf2.start_time_ms = hal.scheduler->millis();
        }
        if (hal.scheduler->millis() - ekf2.start_time_ms > startup_delay_ms) {
            ekf2.started = EKF2.InitialiseFilterDynamic();
        }
    }
    if (ekf2.started) {
        EKF2.UpdateFilter();
        EKF2.getRotationBodyToNED(ekf2._dcm_matrix);
        if (using_EKF() == AHRS_SELECTED_EKF2) {
            ekf2._gyro_estimate += ekf2._gyro_bias;

            float abias1, abias2;
            EKF2.getAccelZBias(abias1, abias2);

            // update _accel_ef_ekf
            for (uint8_t i=0; i<_ins.get_accel_count(); i++) {
                Vector3f accel = _ins.get_accel(i);
                if (i==0) {
                    accel.z -= abias1;
                } else if (i==1) {
                    accel.z -= abias2;
                }
                if (_ins.get_accel_health(i)) {
                    ekf2._accel_ef_ekf[i] = ekf2._dcm_matrix * accel;
                }
            }

            if(_ins.get_accel_health(0) && _ins.get_accel_health(1)) {
                float IMU1_weighting;
                EKF2.getIMU1Weighting(IMU1_weighting);
                ekf2._accel_ef_ekf_blended = ekf2._accel_ef_ekf[0] * IMU1_weighting + ekf2._accel_ef_ekf[1] * (1.0f-IMU1_weighting);
            } else {
                ekf2._accel_ef_ekf_blended = ekf2._accel_ef_ekf[0];
            }


            Vector3f eulers;
            EKF2.getEulerAngles(eulers);
            roll  = eulers.x;
            pitch = eulers.y;
            yaw   = eulers.z;

            update_cd_values();
            update_trig();

            // keep _gyro_bias for get_gyro_drift()
            EKF2.getGyroBias(ekf2._gyro_bias);
            ekf2._gyro_bias = -ekf2._gyro_bias;

            // calculate corrected gryo estimate for get_gyro()
            ekf2._gyro_estimate.zero();
            uint8_t healthy_count = 0;
            for (uint8_t i=0; i<_ins.get_gyro_count(); i++) {
                if (_ins.get_gyro_health(i)) {
                    ekf2._gyro_estimate += _ins.get_gyro(i);
                    healthy_count++;
                }
            }
            if (healthy_count > 1) {
                ekf2._gyro_estimate /= healthy_count;
            }

            ekf2._gyro_estimate += ekf2._gyro_bias;
        }
    }
}


// accelerometer values in the earth frame in m/s/s
const Vector3f &AP_AHRS_NavEKF::get_accel_ef(uint8_t i) const
{
    switch (using_EKF()) {
    case AHRS_SELECTED_EKF1:
    case AHRS_SELECTED_EKF1_NO_FALLBACK:
        return ekf1._accel_ef_ekf[i];
    case AHRS_SELECTED_EKF2:
        return ekf2._accel_ef_ekf[i];
    case AHRS_SELECTED_DCM:
    default:
        return AP_AHRS_DCM::get_accel_ef(i);
    }
}

// blended accelerometer values in the earth frame in m/s/s
const Vector3f &AP_AHRS_NavEKF::get_accel_ef_blended(void) const
{
    switch (using_EKF()) {
    case AHRS_SELECTED_EKF1:
    case AHRS_SELECTED_EKF1_NO_FALLBACK:
        return ekf1._accel_ef_ekf_blended;
    case AHRS_SELECTED_EKF2:
        return ekf2._accel_ef_ekf_blended;
    case AHRS_SELECTED_DCM:
    default:
        return AP_AHRS_DCM::get_accel_ef_blended();
    }
}

void AP_AHRS_NavEKF::reset(bool recover_eulers)
{
    AP_AHRS_DCM::reset(recover_eulers);
    if (ekf1.started) {
        ekf1.started = EKF1.InitialiseFilterBootstrap();
    }
    if (ekf2.started) {
        ekf2.started = EKF2.InitialiseFilterBootstrap();
    }
}

// reset the current attitude, used on new IMU calibration
void AP_AHRS_NavEKF::reset_attitude(const float &_roll, const float &_pitch, const float &_yaw)
{
    AP_AHRS_DCM::reset_attitude(_roll, _pitch, _yaw);
    if (ekf1.started) {
        ekf1.started = EKF1.InitialiseFilterBootstrap();
    }
    if (ekf2.started) {
        ekf2.started = EKF2.InitialiseFilterBootstrap();
    }
}

// dead-reckoning support
bool AP_AHRS_NavEKF::get_position(struct Location &loc) const
{
    Vector3f ned_pos;

    switch (using_EKF()) {
    case AHRS_SELECTED_EKF1:
    case AHRS_SELECTED_EKF1_NO_FALLBACK:
        if (EKF1.getLLH(loc) && EKF1.getPosNED(ned_pos)) {
            // fixup altitude using relative position from AHRS home, not
            // EKF origin
            loc.alt = get_home().alt - ned_pos.z*100;
            return true;
        }
        break;
    case AHRS_SELECTED_EKF2:
        if (EKF2.getLLH(loc) && EKF2.getPosNED(ned_pos)) {
            // fixup altitude using relative position from AHRS home, not
            // EKF origin
            loc.alt = get_home().alt - ned_pos.z*100;
            return true;
        }
        break;
    case AHRS_SELECTED_DCM:
    default:
        break;
    }
    return AP_AHRS_DCM::get_position(loc);
}

// status reporting of estimated errors
float AP_AHRS_NavEKF::get_error_rp(void) const
{
    return AP_AHRS_DCM::get_error_rp();
}

float AP_AHRS_NavEKF::get_error_yaw(void) const
{
    return AP_AHRS_DCM::get_error_yaw();
}

// return a wind estimation vector, in m/s
Vector3f AP_AHRS_NavEKF::wind_estimate(void)
{
    switch (using_EKF()) {
    case AHRS_SELECTED_EKF1:
    case AHRS_SELECTED_EKF1_NO_FALLBACK: {
        Vector3f wind;
        EKF1.getWind(wind);
        return wind;
    }
    case AHRS_SELECTED_EKF2: {
        Vector3f wind;
        EKF2.getWind(wind);
        return wind;
    }
    case AHRS_SELECTED_DCM:
    default:
        return AP_AHRS_DCM::wind_estimate();
    }
}

// return an airspeed estimate if available. return true
// if we have an estimate
bool AP_AHRS_NavEKF::airspeed_estimate(float *airspeed_ret) const
{
    return AP_AHRS_DCM::airspeed_estimate(airspeed_ret);
}

// true if compass is being used
bool AP_AHRS_NavEKF::use_compass(void)
{
    switch (using_EKF()) {
    case AHRS_SELECTED_EKF1:
    case AHRS_SELECTED_EKF1_NO_FALLBACK:
        return EKF1.use_compass();
    case AHRS_SELECTED_EKF2:
        return EKF2.use_compass();
    case AHRS_SELECTED_DCM:
    default:
        return AP_AHRS_DCM::use_compass();
    }
}


// return secondary attitude solution if available, as eulers in radians
bool AP_AHRS_NavEKF::get_secondary_attitude(Vector3f &eulers, uint8_t instance)
{
    if (instance == 1) {
        // always give EKF2
        if (ekf2.started) {
            EKF2.getEulerAngles(eulers);
            return true;
        }
        eulers = _dcm_attitude;
        return true;
    }
    switch (using_EKF()) {
    case AHRS_SELECTED_EKF1:
    case AHRS_SELECTED_EKF1_NO_FALLBACK:
    case AHRS_SELECTED_EKF2:
        // return DCM attitude
        eulers = _dcm_attitude;
        return true;
    case AHRS_SELECTED_DCM:
    default:
        if (ekf1.started && instance == 0) {
            // EKF1 is secondary
            EKF1.getEulerAngles(eulers);
            return true;
        }
        if (ekf2.started && instance == 1) {
            // EKF1 is secondary
            EKF2.getEulerAngles(eulers);
            return true;
        }
    }
    // no secondary available
    return false;
}


// return secondary position solution if available
bool AP_AHRS_NavEKF::get_secondary_position(struct Location &loc, uint8_t instance)
{
    if (instance == 1) {
        // always give EKF2
        if (ekf2.started) {
            EKF2.getLLH(loc);
            return true;
        }
        AP_AHRS_DCM::get_position(loc);
        return true;
    }
    switch (using_EKF()) {
    case AHRS_SELECTED_EKF1:
    case AHRS_SELECTED_EKF1_NO_FALLBACK:
    case AHRS_SELECTED_EKF2:
        // return DCM position
        AP_AHRS_DCM::get_position(loc);
        return true;
    case AHRS_SELECTED_DCM:
    default:
        if (ekf1.started && instance == 0) {
            // EKF1 is secondary
            EKF1.getLLH(loc);
            return true;
        }
        if (ekf2.started && instance == 1) {
            // EKF1 is secondary
            EKF2.getLLH(loc);
            return true;
        }
    }
    // no secondary available
    return false;
}

// EKF has a better ground speed vector estimate
Vector2f AP_AHRS_NavEKF::groundspeed_vector(void)
{
    switch (using_EKF()) {
    case AHRS_SELECTED_EKF1:
    case AHRS_SELECTED_EKF1_NO_FALLBACK: {
        Vector3f vec;
        EKF1.getVelNED(vec);
        return Vector2f(vec.x, vec.y);
    }
    case AHRS_SELECTED_EKF2: {
        Vector3f vec;
        EKF2.getVelNED(vec);
        return Vector2f(vec.x, vec.y);
    }
    case AHRS_SELECTED_DCM:
    default:
        return AP_AHRS_DCM::groundspeed_vector();
    }
}

void AP_AHRS_NavEKF::set_home(const Location &loc)
{
    AP_AHRS_DCM::set_home(loc);
}

// return true if inertial navigation is active
bool AP_AHRS_NavEKF::have_inertial_nav(void) const
{
    return using_EKF() != AHRS_SELECTED_DCM;
}

// return a ground velocity in meters/second, North/East/Down
// order. Must only be called if have_inertial_nav() is true
bool AP_AHRS_NavEKF::get_velocity_NED(Vector3f &vec) const
{
    switch (using_EKF()) {
    case AHRS_SELECTED_EKF1:
    case AHRS_SELECTED_EKF1_NO_FALLBACK:
        EKF1.getVelNED(vec);
        return true;
    case AHRS_SELECTED_EKF2:
        EKF2.getVelNED(vec);
        return true;
    case AHRS_SELECTED_DCM:
    default:
        break;
    }
    return false;
}

// return a relative ground position in meters/second, North/East/Down
// order. Must only be called if have_inertial_nav() is true
bool AP_AHRS_NavEKF::get_relative_position_NED(Vector3f &vec) const
{
    switch (using_EKF()) {
    case AHRS_SELECTED_EKF1:
    case AHRS_SELECTED_EKF1_NO_FALLBACK:
        return EKF1.getPosNED(vec);
    case AHRS_SELECTED_EKF2:
        return EKF2.getPosNED(vec);
    case AHRS_SELECTED_DCM:
    default:
        break;
    }
    return false;
}

AP_AHRS_NavEKF::AHRS_selected AP_AHRS_NavEKF::using_EKF(void) const
{
    AP_AHRS_NavEKF::AHRS_selected selected = (AP_AHRS_NavEKF::AHRS_selected)_ekf_use.get();
    switch (selected) {
    case AHRS_SELECTED_EKF1:
    case AHRS_SELECTED_EKF1_NO_FALLBACK: {
        uint8_t ekf_faults;
        EKF1.getFilterFaults(ekf_faults);
        // If EKF is started we switch away if it reports unhealthy. This could be due to bad
        // sensor data. If EKF reversion is inhibited, we only switch across if the EKF encounters
        // an internal processing error, but not for bad sensor data.
        bool ret = (ekf1.started &&
                    ((selected == AHRS_SELECTED_EKF1 && EKF1.healthy()) ||
                     (selected == AHRS_SELECTED_EKF1_NO_FALLBACK && ekf_faults == 0)));
        if (!ret) {
            selected = AHRS_SELECTED_DCM;
        }
        if (_vehicle_class == AHRS_VEHICLE_FIXED_WING ||
            _vehicle_class == AHRS_VEHICLE_GROUND) {
            nav_filter_status filt_state;
            EKF1.getFilterStatus(filt_state);
            if (hal.util->get_soft_armed() && filt_state.flags.const_pos_mode) {
                selected = AHRS_SELECTED_DCM;
            }
            if (!filt_state.flags.attitude ||
                !filt_state.flags.horiz_vel ||
                !filt_state.flags.vert_vel ||
                !filt_state.flags.horiz_pos_abs ||
                !filt_state.flags.vert_pos) {
                return false;
            }
        }
        return ret;
    }

    case AHRS_SELECTED_EKF2:
        if (!ekf2.started || !EKF2.healthy()) {
            return AHRS_SELECTED_DCM;
        }
        break;

    case AHRS_SELECTED_DCM:
    default:
        break;
    }
    return selected;
}

/*
  check if the AHRS subsystem is healthy
*/
bool AP_AHRS_NavEKF::healthy(void) const
{
    // If EKF is started we switch away if it reports unhealthy. This could be due to bad
    // sensor data. If EKF reversion is inhibited, we only switch across if the EKF encounters
    // an internal processing error, but not for bad sensor data.
    switch (using_EKF()) {
    case AHRS_SELECTED_EKF1:
    case AHRS_SELECTED_EKF1_NO_FALLBACK:
        if ((_vehicle_class == AHRS_VEHICLE_FIXED_WING ||
             _vehicle_class == AHRS_VEHICLE_GROUND) &&
            !using_EKF()) {
            // on fixed wing we want to be using EKF to be considered
            // healthy if EKF is enabled
            return false;
        }
        return ekf1.started && EKF1.healthy();
    case AHRS_SELECTED_EKF2:
        return ekf2.started && EKF2.healthy();
    case AHRS_SELECTED_DCM:
    default:
    break;
    }
    return AP_AHRS_DCM::healthy();
}

void AP_AHRS_NavEKF::set_ekf_use(bool setting)
{
#if !AHRS_EKF_USE_ALWAYS
    _ekf_use.set(setting);
#endif
}

// true if the AHRS has completed initialisation
bool AP_AHRS_NavEKF::initialised(void) const
{
    // initialisation complete 10sec after ekf has started
    return (ekf1.started &&
            (hal.scheduler->millis() - ekf1.start_time_ms > AP_AHRS_NAVEKF_SETTLE_TIME_MS) &&
            ekf2.started &&
            (hal.scheduler->millis() - ekf2.start_time_ms > AP_AHRS_NAVEKF_SETTLE_TIME_MS));
}

// write optical flow data to EKF
void  AP_AHRS_NavEKF::writeOptFlowMeas(uint8_t &rawFlowQuality, Vector2f &rawFlowRates, Vector2f &rawGyroRates, uint32_t &msecFlowMeas)
{
    EKF1.writeOptFlowMeas(rawFlowQuality, rawFlowRates, rawGyroRates, msecFlowMeas);
    // ignore optical flow measurements for EKF2 for now
}

// inhibit GPS useage
uint8_t AP_AHRS_NavEKF::setInhibitGPS(void)
{
    return EKF1.setInhibitGPS();
}

// get speed limit
void AP_AHRS_NavEKF::getEkfControlLimits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler)
{
    EKF1.getEkfControlLimits(ekfGndSpdLimit,ekfNavVelGainScaler);
}

// get compass offset estimates
// true if offsets are valid
bool AP_AHRS_NavEKF::getMagOffsets(Vector3f &magOffsets)
{
    bool status = EKF1.getMagOffsets(magOffsets);
    return status;
}

#endif // AP_AHRS_NAVEKF_AVAILABLE

