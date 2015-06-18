
#include "AP_Predictors.h"
#include <AP_HAL.h>

#include <stdio.h>

#include <stdlib.h>

extern const AP_HAL::HAL& hal;

// Gravitational acceleration vector in the NED frame.
const Vector3f gravityNED(0, 0, GRAVITY_MSS);


AP_Predictors::AP_Predictors()     // Constructor
{
  // Initializing quaternion vectors such that they have unit amplitude.
        D_q_k1.q1=1;
        D_q_k1.q2=0;
        D_q_k1.q3=0;
        D_q_k1.q4=0;

        for (uint16_t i=0; i<=(BUFFER_SIZE-1); i++) {
            storedD[i].q1 =1;
            storedD[i].q2 =0;
            storedD[i].q3 =0;
            storedD[i].q4 =0;
        }
}

// The function UpdatePredictorStates uses the corected delta angle (stored in tilde_q)
// and corrected delta velocity (stored in corrected_tilde_Vel12) and updates the
// dynamics of the states of the predictor.
// The functions AttitudeModel, VelModel, and PosModel implement the predictor proposed in [B].
void AP_Predictors::UpdatePredictorStates(Vector3f tilde_q, Vector3f corrected_tilde_Vel12, Vector3f velocity, ftype dtIMU, uint32_t imuSampleTimeP_ms)
{
    // Using the time-stamp of IMU as the time stamp imuSampleTimePred_ms throughout the predictor.
    imuSampleTimePred_ms = imuSampleTimeP_ms;

    // Updating attitude predictor dynamics
    AttitudeModel(tilde_q);

    // Updating the velocity predictor dynamics
    VelModel(corrected_tilde_Vel12);

    // Updating the position predictor dynamics
    PosModel(velocity,dtIMU);
}


// The fuction AttitudeModel uses attitude kinematics and updates the variable D_q_k1 and stores
// the updated one into the buffer storedD.
// The valiable D_q_k1 is an internal state of th predictor and corresponds to the variable \Delta in [A] and [B].
//  The function AttitudeModel implements the dynamics (6) of [A] (which is similar to the dynamics (37) of [B]).
void AP_Predictors::AttitudeModel(Vector3f tilde_q)
{
    // Storing the previous attitude predictor state
    Quaternion D_q;
    D_q = D_q_k1;

    // Computing the amplitude of delta angle
    float n_tilde_q;
    n_tilde_q = sqrt(tilde_q.x*tilde_q.x+tilde_q.y*tilde_q.y+tilde_q.z*tilde_q.z);

    // Computing the quaternion vector associated to tilde_q
    Quaternion delta_q;
    float rotScaler;
    if (n_tilde_q < 1e-12f)
    {
    // If delta angle is small, consider the unit quaternion to avoid numerical errors.
        delta_q.q1 = 1;
        delta_q.q2 = 0;
        delta_q.q3 = 0;
        delta_q.q4 = 0;
    }
    else
    {
    // Computing delta_q=single([cos(0.5*n_tilde_q);(sin(0.5*n_tilde_q)/n_tilde_q).*tilde_q]);
        delta_q.q1 = cosf(0.5f * n_tilde_q);
        rotScaler = (sinf(0.5f * n_tilde_q)) / n_tilde_q;
        delta_q.q2 = tilde_q.x * rotScaler;
        delta_q.q3 = tilde_q.y * rotScaler;
        delta_q.q4 = tilde_q.z * rotScaler;
    }

    // update the quaternions by rotating from the previous attitude through
    // the delta angle rotation quaternion
    D_q_k1.q1 = D_q.q1*delta_q.q1 - D_q.q2*delta_q.q2 - D_q.q3*delta_q.q3 - D_q.q4*delta_q.q4;
    D_q_k1.q2 = D_q.q1*delta_q.q2 + D_q.q2*delta_q.q1 + D_q.q3*delta_q.q4 - D_q.q4*delta_q.q3;
    D_q_k1.q3 = D_q.q1*delta_q.q3 + D_q.q3*delta_q.q1 + D_q.q4*delta_q.q2 - D_q.q2*delta_q.q4;
    D_q_k1.q4 = D_q.q1*delta_q.q4 + D_q.q4*delta_q.q1 + D_q.q2*delta_q.q3 - D_q.q3*delta_q.q2;

    // Normalizing D_q_k1
    D_q_k1.normalize();

    // Buffering the updated D_q_k1
    storeDataQuaternion(D_q_k1, storedD, lastDStoreTime_ms, DTimeStamp, storeIndexD);
}


// The functions VelModel implements the velocity predictor proposed in [B].
// The fuction VelModel uses velocity kinematics and updates the variable d_v_m and stores
// the updated one into the buffer storedd_v_m.
// The valiable d_v_m is an internal state of the predictor and corresponds to the variable \delta in [B].
// This function implements the dynamics (38) of [B]. Due to the nature of the predictor dynamics,
// the internal state d_v_m needs to be reseted to zero priodicaly to keep its trajectory bounded. This resetting
// is performed in this function.
void AP_Predictors::VelModel(Vector3f corrected_tilde_Vel12)
{
    // Teporary rotation matrix
    Matrix3f Tbn_temp;
    D_q_k1.rotation_matrix(Tbn_temp);

    // Velocity predictor dynamics
    d_v_m = d_v_m+Tbn_temp*corrected_tilde_Vel12;

    // buffering d_v_m
    storeDataVector(d_v_m, storedd_v_m, lastd_v_mStoreTime_ms, d_v_mTimeStamp, storeIndexd_v_m);

    // resetting the internal state d_v_m to zero every  20*2*BUFFER_SIZE milliseconds
    // in order to keep its trajectory bounded. This resetting is done according to [B].
    ctr_rst += 1;
    if (ctr_rst == 2*BUFFER_SIZE) {
        ctr_rst = 0;
        for (uint16_t i=0; i<=(BUFFER_SIZE-1); i++)
        {
            storedd_v_m[i]=storedd_v_m[i]-d_v_m;
        }
        d_v_m.x = 0;
        d_v_m.y = 0;
        d_v_m.z = 0;
    }
}

// The function PosModel implements the pose predictor dynamics. This is based on simple forward integration of
// the current estimate of the velocity (given by the variable velocity). The variable d_p_m is the internal state
// of the position predictor. This variable is updated and stored in a buffer.
void AP_Predictors::PosModel(Vector3f velocity, ftype dtIMU)
{
    // position predictor dynamics
    d_p_m+= velocity*dtIMU;

    // buffering d_p_m
    storeDataVector(d_p_m, storedd_p_m, lastd_p_mStoreTime_ms, d_p_mTimeStamp, storeIndexd_p_m);
}


// The function VectorPredictor takes a vector measuremet taken in the past and proides its current prediction.
// In order to use this function, the measurement model should have the form of y = R^\top y_0. Example of such a measurement
// model is magnetometer where y represents the 3-axis magnetic field vector measured by an onboard magnetometer, R represents
// the rotation matrix from the NED frame to the body frame, and y_0 represents the magnetic field vector represented in NED frame
// which is approximately constant. The variable vec is the measured ector which is valid at the time currrentTimeStamp_ms-_msecTauDelay.
// The variable vecPredicted is a prediction of vec which is valid at the time currrentTimeStamp_ms. The function VectorPredictor uses
// the buffer storedD to obtain the predictions.
void AP_Predictors::VectorPredictor(Vector3f &vecPredicted, Vector3f vec, uint32_t currrentTimeStamp_ms, uint16_t _msecTauDelay)
{
    Quaternion q_tmp;
    Quaternion q_tmp2;
    Quaternion D_delay;
    Quaternion D_current;

    uint32_t bestTimeErr;
    uint16_t bestStoreIndex;
    uint16_t zero_16 = 0;

    BestIndex(bestTimeErr, bestStoreIndex, DTimeStamp, currrentTimeStamp_ms, _msecTauDelay);  // check the bestTimeErr!!!!
    D_delay = storedD[bestStoreIndex];
    BestIndex(bestTimeErr, bestStoreIndex, DTimeStamp, currrentTimeStamp_ms, zero_16);
    D_current = storedD[bestStoreIndex];

    // computing the inverse quaternion D_current^{-1}
    q_tmp.q1= D_current.q1;
    q_tmp.q2= -D_current.q2;
    q_tmp.q3= -D_current.q3;
    q_tmp.q4= -D_current.q4;

    // multiplyig D_current^{-1} by D_delay
    q_tmp2.q1 = q_tmp.q1*D_delay.q1 - q_tmp.q2*D_delay.q2 - q_tmp.q3*D_delay.q3 - q_tmp.q4*D_delay.q4;
    q_tmp2.q2 = q_tmp.q1*D_delay.q2 + q_tmp.q2*D_delay.q1 + q_tmp.q3*D_delay.q4 - q_tmp.q4*D_delay.q3;
    q_tmp2.q3 = q_tmp.q1*D_delay.q3 + q_tmp.q3*D_delay.q1 + q_tmp.q4*D_delay.q2 - q_tmp.q2*D_delay.q4;
    q_tmp2.q4 = q_tmp.q1*D_delay.q4 + q_tmp.q4*D_delay.q1 + q_tmp.q2*D_delay.q3 - q_tmp.q3*D_delay.q2;

    // Normalizing the quaternion
    q_tmp2.normalize();

    // Converting to rotation matrix
    Matrix3f T_temp;
    q_tmp2.rotation_matrix(T_temp);

    vecPredicted = T_temp*vec;
}


// The fuction VelPredictor uses the buffer storedd_v_m and the delayed measurement of the velocity (stored in velocity)
// and the delayed estimate of quaternion (stored in quat) and computes the current velocity vector (i.e. the variable velocityPredicted).
// This function is based on the static equasion (40) of [B]. The variables velocity and quat are valid at the time currrentTimeStamp_ms-_msecTauDelay
// and the prediction velocityPredicted is valid at the time currrentTimeStamp_ms.
void AP_Predictors::VelPredictor(Vector3f &velocityPredicted, Vector3f velocity, Quaternion quat, uint32_t currrentTimeStamp_ms, uint16_t _msecTauDelay)
{
    Quaternion q_tmp;
    Quaternion D_delay;

    // picking up the delayed d_v_m
    uint32_t bestTimeErr;
    uint16_t bestStoreIndex;
    BestIndex(bestTimeErr, bestStoreIndex, DTimeStamp, currrentTimeStamp_ms, _msecTauDelay);  // Need to add checks for the bestTimeErr.
    D_delay = storedD[bestStoreIndex];

    // computing the inverse quaternion D_q_delay^{-1}
    q_tmp.q1 = D_delay.q1;
    q_tmp.q2 = -D_delay.q2;
    q_tmp.q3 = -D_delay.q3;
    q_tmp.q4 = -D_delay.q4;

    // computing the quaternion multiplication quat \times D_delay^{-1}
    Quaternion q_tmp_m;
    q_tmp_m.q1 = quat.q1*q_tmp.q1 - quat.q2*q_tmp.q2 - quat.q3*q_tmp.q3 - quat.q4*q_tmp.q4;
    q_tmp_m.q2 = quat.q1*q_tmp.q2 + quat.q2*q_tmp.q1 + quat.q3*q_tmp.q4 - quat.q4*q_tmp.q3;
    q_tmp_m.q3 = quat.q1*q_tmp.q3 + quat.q3*q_tmp.q1 + quat.q4*q_tmp.q2 - quat.q2*q_tmp.q4;
    q_tmp_m.q4 = quat.q1*q_tmp.q4 + quat.q4*q_tmp.q1 + quat.q2*q_tmp.q3 - quat.q3*q_tmp.q2;

    // Converting quaternion to rotation matrix.
    Matrix3f Tbn_temp;
    q_tmp_m.rotation_matrix(Tbn_temp);

    // picking up the bufferred d_v_m at the time currrentTimeStamp_ms-msecTauDelay
    BestIndex(bestTimeErr, bestStoreIndex, d_v_mTimeStamp, currrentTimeStamp_ms, _msecTauDelay);  // Should check the bestTimeErr!
    Vector3f d_v_m_Delay = storedd_v_m[bestStoreIndex];

    // picking up the bufferred d_v_m at the time currrentTimeStamp_ms
    uint16_t zero_16 = 0;    //maybe I can just put in zero without defining new variable
    BestIndex(bestTimeErr, bestStoreIndex, d_v_mTimeStamp, currrentTimeStamp_ms, zero_16);  // Should check the bestTimeErr!
    Vector3f d_v_m_current = storedd_v_m[bestStoreIndex];

    // Computing the velocity prediction according to the equation (40) of [B].
    velocityPredicted = velocity + gravityNED*(0.001f*constrain_int16(_msecTauDelay, 0, MAX_MSDELAY))+Tbn_temp*(d_v_m_current- d_v_m_Delay);
}


// The fuction PosNEPredictor uses the buffer storedd_p_m and the delayed measurement of the North and East position (stored in positionNE)
// and computes the current North and East positions (i.e. the variable positionNEPredicted).
// The variables positionNE is valid at the time currrentTimeStamp_ms-_msecTauDelay
// and the prediction positionNEPredicted is valid at the time currrentTimeStamp_ms.
void AP_Predictors::PosNEPredictor(Vector2f &positionNEPredicted, Vector2f positionNE, uint32_t currrentTimeStamp_ms, uint16_t _msecTauDelay)
{
    uint32_t bestTimeErr;
    uint16_t bestStoreIndex;

    // picking up the bufferred d_p_m at the time currrentTimeStamp_ms-msecTauDelay
    BestIndex(bestTimeErr, bestStoreIndex, d_p_mTimeStamp, currrentTimeStamp_ms, _msecTauDelay); // Should check the bestTimeErr!
    Vector2f d_p_mNE_Delay;
    d_p_mNE_Delay.x = storedd_p_m[bestStoreIndex].x;
    d_p_mNE_Delay.y = storedd_p_m[bestStoreIndex].y;

    // picking up the bufferred d_p_m at the time currrentTimeStamp_ms
    uint16_t zero_16 = 0;    //maybe I can just put in zero without defining new variable
    BestIndex(bestTimeErr, bestStoreIndex, d_p_mTimeStamp, currrentTimeStamp_ms, zero_16);  // Should check the bestTimeErr!
    Vector2f d_p_mNE_current;
    d_p_mNE_current.x = storedd_p_m[bestStoreIndex].x;
    d_p_mNE_current.y = storedd_p_m[bestStoreIndex].y;

    // Computing the prediction of positionNE.
    positionNEPredicted = positionNE + d_p_mNE_current - d_p_mNE_Delay;
}


// The fuction HgtPredictor uses the buffer storedd_p_m and the delayed measurement of the height (stored in height)
// and computes the current height (i.e. the variable heightPredicted).
// The variables height is valid at the time currrentTimeStamp_ms-_msecTauDelay
// and the prediction heightPredicted is valid at the time currrentTimeStamp_ms.
void AP_Predictors::HgtPredictor(float &heightPredicted, float height, uint32_t currrentTimeStamp_ms, uint16_t _msecTauDelay)
{
    uint32_t bestTimeErr;
    uint16_t bestStoreIndex;

    // picking up the bufferred d_p_m at the time currrentTimeStamp_ms-msecTauDelay
    BestIndex(bestTimeErr, bestStoreIndex, d_p_mTimeStamp, currrentTimeStamp_ms, _msecTauDelay);  // Should check the bestTimeErr!
    float d_p_mD_delay = storedd_p_m[bestStoreIndex].z;

    // picking up the bufferred d_p_m at the time currrentTimeStamp_ms
    uint16_t zero_16 = 0;    //maybe I can just put in zero without defining new variable
    BestIndex(bestTimeErr, bestStoreIndex, d_p_mTimeStamp, currrentTimeStamp_ms, zero_16); // Should check the bestTimeErr!
    float d_p_mD_current = storedd_p_m[bestStoreIndex].z;

    // Computing the prediction of height.
    heightPredicted = height + d_p_mD_current - d_p_mD_delay;
}


// The function BestIndex takes the buffer timeStamp and returns the index whoes value is closest to currrentTimeStamp_ms-_msecTauDelay.
// The variable closestStoreIndex contains the closets index ad the variable closestTime contains the error between
// timeStamp[closestStoreIndex] and currrentTimeStamp_ms-_msecTauDelay.
void AP_Predictors::BestIndex(uint32_t &closestTime, uint16_t &closestStoreIndex, uint32_t (&timeStamp)[BUFFER_SIZE], uint32_t &currrentTimeStamp_ms, uint16_t &_msecTauDelay)
{
    uint32_t time_delta;
    closestTime = MAX_MSDELAY;
    closestStoreIndex = 0;

    for (int i=0; i<=(BUFFER_SIZE-1); i++)
    {
        time_delta = abs( (currrentTimeStamp_ms - timeStamp[i]) - constrain_int16(_msecTauDelay, 0, MAX_MSDELAY));
        if (time_delta < closestTime)
        {
            closestStoreIndex = i;
            closestTime = time_delta;
        }
    }
}


// The function storeDataVector is used to store a Vector3f into its corresponding buffer and also to store its
// time stamp into the corresponding time stamp buffer.
void AP_Predictors::storeDataVector(Vector3f &data, VectorN<Vector3f,BUFFER_SIZE> &buffer, uint32_t &lastStoreTime, uint32_t (&timeStamp)[BUFFER_SIZE], uint16_t &storeIndex)
{
    // Save the data only if more than SAVE_PRIOD milliseconds passed since last data saved
    if (imuSampleTimePred_ms - lastStoreTime >= SAVE_PRIOD) {
        lastStoreTime = imuSampleTimePred_ms;
        if (storeIndex > (BUFFER_SIZE-1)) {
            storeIndex = 0;
        }
        buffer[storeIndex]=data;
        timeStamp[storeIndex] = lastStoreTime;
        storeIndex = storeIndex + 1;
   }
}


// The function storeDataQuaternion is used to store a Quaternion into its corresponding buffer and also to store its
// time stamp into the corresponding time stamp buffer.
void AP_Predictors::storeDataQuaternion(Quaternion &data, VectorN<Quaternion,BUFFER_SIZE> &buffer, uint32_t &lastStoreTime, uint32_t (&timeStamp)[BUFFER_SIZE], uint16_t &storeIndex)
{
    // Save the data only if more than SAVE_PRIOD milliseconds passed since last data saved
    if (imuSampleTimePred_ms - lastStoreTime >= SAVE_PRIOD) {
        lastStoreTime = imuSampleTimePred_ms;
        if (storeIndex > (BUFFER_SIZE-1)) {
            storeIndex = 0;
        }
        buffer[storeIndex]=data;
        timeStamp[storeIndex] = lastStoreTime;
        storeIndex = storeIndex + 1;
    }
}

