/*
By: Alireza Khosravian and Sean O'Brien

This library implements the attitude, velocity, and position predictors
proposed in the following references.
[A] A. Khosravian, J. Trumpf, R. Mahony, and T. Hamel,
    “Velocity aided attitude estimation on SO(3) with sensor delay,”
    in Proc. IEEE Conf. on Decision and Control, December 2014.
[B] A. Khosravian, J. Trumpf, R. Mahony, and T. Hamel,
    "State Estimation for Invariant Systems on Lie Groups with Delayed Output Measurements",
    Automatica (under review since April 2015).

The main use of the predictors is to compensate for time lags (time delays) of sensor measurements.
This has been demostrated by combining the predictor with AP_NavEKF2 library.

There is one attitude predictor, but two velocity predictors and two position predictors.
The first pair of velocity and position predictor is based on the work presented in [A] and
the second pair is based on [B].

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

#ifndef AP_PREDICTORS_H
#define AP_PREDICTORS_H

#include <AP_Math.h>
#include <AP_Param.h>
#include <vectorN.h>

#define BUFFER_SIZE  50   //  buffer size for sensors, this allows buffering of at least BUFFER_SIZE*20 ms=MAX_MSDELAY of data
#define SAVE_PRIOD   5     // Minumum time between two updates (it is used in Save functions)
#define MAX_MSDELAY  BUFFER_SIZE*SAVE_PRIOD  // maximum allowed delay in milliseconds. It should match the maximum delay set in NavEKF2 (which is currently 500ms)

class AP_Predictors
{
public:
    AP_Predictors();
    typedef float ftype;
    void AttitudeModel(Vector3f tilde_q);
    void VelModel(Vector3f corrected_tilde_Vel12);
    void storeDataVector(Vector3f &data, VectorN<Vector3f,BUFFER_SIZE> &buffer, uint32_t &lastStoreTime, uint32_t (&timeStamp)[BUFFER_SIZE], uint16_t &storeIndex);
    void storeDataQuaternion(Quaternion &data, VectorN<Quaternion,BUFFER_SIZE> &buffer, uint32_t &lastStoreTime, uint32_t (&timeStamp)[BUFFER_SIZE], uint16_t &storeIndex);
    void UpdatePredictorStates(Vector3f tilde_q, Vector3f corrected_tilde_Vel12, Vector3f velocity, ftype dtIMU, uint32_t imuSampleTimeP_ms);
    void VectorPredictor(Vector3f &vecPredicted, Vector3f vec, uint32_t currrentTimeStamp_ms, uint16_t _msecTauDelay);
    void BestIndex(uint32_t &closestTime, uint16_t &closestStoreIndex, uint32_t (&timeStamp)[BUFFER_SIZE], uint32_t &currrentTimeStamp_ms, uint16_t &_msecTauDelay);
    void VelPredictor(Vector3f &velocityPredicted, Vector3f velocity, Quaternion quat, uint32_t currrentTimeStamp_ms, uint16_t _msecTauDelay);
    void PosModel(Vector3f velocity, ftype dtIMU);
    void PosNEPredictor(Vector2f &positionNEPredicted, Vector2f positionNE, uint32_t currrentTimeStamp_ms, uint16_t _msecTauDelay);
    void HgtPredictor(float &heightPredicted, float height, uint32_t currrentTimeStamp_ms, uint16_t _msecTauDelay);

private:
    uint32_t imuSampleTimePred_ms;
    Quaternion D_q_k1;
    Vector3f d_p_m;
    Vector3f d_v_m;

    // attitude model variables
    uint16_t storeIndexD;
    uint32_t lastDStoreTime_ms;
    uint32_t DTimeStamp[BUFFER_SIZE];
    VectorN<Quaternion,BUFFER_SIZE> storedD;
    uint32_t bestTimeD;
    uint16_t bestStoreIndexD;

    // position model 2 variables
    uint16_t storeIndexd_p_m;
    uint32_t lastd_p_mStoreTime_ms;
    uint32_t d_p_mTimeStamp[BUFFER_SIZE];
    VectorN<Vector3f,BUFFER_SIZE> storedd_p_m;        //buffer for delta corrsponding to position prediction mixed-invariant
    uint32_t bestTimed_p_m;
    uint16_t bestStoreIndexd_p_m;

    // velocity model 2 variables
    uint16_t storeIndexd_v_m;
    uint32_t lastd_v_mStoreTime_ms;
    uint32_t d_v_mTimeStamp[BUFFER_SIZE];
    VectorN<Vector3f,BUFFER_SIZE> storedd_v_m;        //buffer for delta corrsponding to velocity prediction mixed-invariant
    uint32_t bestTimed_v_m;
    uint16_t bestStoreIndexd_v_m;

    uint32_t ctr_rst;  // reset predictor cntr
};

#endif // AP_PREDICTORS_H
