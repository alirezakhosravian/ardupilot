
#include "AP_Predictors.h"
#include <AP_HAL.h>

#include <stdio.h>

#include <stdlib.h>

extern const AP_HAL::HAL& hal;
const Vector3f gravityNED(0, 0, GRAVITY_MSS);


AP_Predictors::AP_Predictors()
{
    // Constructor
}

// CascadedPredictor is the main function which runs diffrent sub-functions required for predictor.
void AP_Predictors::CascadedPredictor(Vector3f tilde_q, Vector3f corrected_tilde_Vel12, Quaternion quat, ftype dtIMU, uint32_t imuSampleTimeP_ms, AP_Int16 _msecTauDelay, Vector3f velocity, Vector3f position)
{
    // Using the time-stamp of IMU as the time stamp imuSampleTimePred_ms throughout the predictor.
    imuSampleTimePred_ms = imuSampleTimeP_ms;

    // Updating attitude predictor dynamics
    AttitudeModel(tilde_q);

    // Updating the first velocity predictor dynamics
    VelocityModel(corrected_tilde_Vel12, dtIMU);

    // Updating the second velocity predictor dynamics
    VelocityModel2(corrected_tilde_Vel12);

    // Updating the first position predictor dynamics
    PositionModel(dtIMU);

    // Updating the second position predictor dynamics
    PositionModel2(dtIMU);

    // obtaining the closest buffer index corresponding to _msecTauDelay milliseconds ago
    // i.e. the time imuSampleTimePred_ms-_msecTauDelay.
    BestIndex(bestTimeD, bestStoreIndexD, DTimeStamp, _msecTauDelay);
    BestIndex(bestTimed_v, bestStoreIndexd_v, d_vTimeStamp, _msecTauDelay);
    BestIndex(bestTimed_p, bestStoreIndexd_p, d_pTimeStamp, _msecTauDelay);
    BestIndex(bestTimed_v_m, bestStoreIndexd_v_m, d_v_mTimeStamp, _msecTauDelay);
    BestIndex(bestTimed_p_m, bestStoreIndexd_p_m, d_p_mTimeStamp, _msecTauDelay);

    // Obtaining the current prediction of attitude
    AttitudePredictor(quat);

    // Obtaining the current prediction of NEDvelocity (using the first predictor)
    VelocityPredictor(velocity);

    // Obtaining the current prediction of NEDposition (using the first predictor)
    PositionPredictor(position);

    // Obtaining the current prediction of NEDvelocity (using the second predictor)
    VelocityPredictor2(quat, velocity, _msecTauDelay);

    // Obtaining the current prediction of NEDposition (using the second predictor)
    PositionPredictor2(position);


// The following code just for temporary loggin data for SITL for simulation verification and debuging
//    static FILE *mylog;
//    if (mylog==NULL) {
//        mylog = fopen("logtmp.txt", "w");
//    }
//    if (mylog!=NULL) {
//        // fputs ("Ali",  mylog);
//        fprintf (mylog, "{%lu;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f}\n",
//                 (unsigned long)hal.scheduler->millis(),
//                 q_hat[0], q_hat[1],q_hat[2],q_hat[3],v_hat[0],v_hat[1],v_hat[2],d_v[0],d_v[1],d_v[2],d_v_Delay[0],d_v_Delay[1],d_v_Delay[2],p_hat[0],p_hat[1],p_hat[2],v_hat_m[0],v_hat_m[1],v_hat_m[2],p_hat_m[0],p_hat_m[1],p_hat_m[2]);
//    }

}

// The functions AttitudeModel and AttitudePredictor implement the attitude predictor proposed in [A] and [B].

// The fuction AttitudeModel uses attitude kinematics and updates the variable D_q_k1 and stores
// the updated one into the buffer storedD.
// The valiable D_q_k1 is an internal state of th predictor and corresponds to the variable \Delta in [A] and [B].
//  The function AttitudeModel implements the dynamics (6) of [A] (which is similar to the dynamics (37) of [B]).
void AP_Predictors::AttitudeModel(Vector3f tilde_q)
{
  // Initializing quaternion vectors such that they have unit amplitude.
  // We should do this initialization in a proper way later.
  if (init_reset==0) {
        D_q[0]=1;
        D_q[1]=0;
        D_q[2]=0;
        D_q[3]=0;

        D_q_k1[0]=1;
        D_q_k1[1]=0;
        D_q_k1[2]=0;
        D_q_k1[3]=0;

        delta_q[0]=1;
        delta_q[1]=0;
        delta_q[2]=0;
        delta_q[3]=0;

        for (uint16_t i=0; i<=(BUFFER_SIZE-1); i++) {
            storedD[i].q1 =1;
            storedD[i].q2 =0;
            storedD[i].q3 =0;
            storedD[i].q4 =0;
        }
        init_reset=1;
    }

    D_q = D_q_k1;

    n_tilde_q = sqrt(tilde_q[0]*tilde_q[0]+tilde_q[1]*tilde_q[1]+tilde_q[2]*tilde_q[2]);
    if (n_tilde_q < 1e-12f)
    {
        delta_q[0] = 1;
        delta_q[1] = 0;
        delta_q[2] = 0;
        delta_q[3] = 0;
    }
    else
    {
// check: delta_q=single([cos(0.5*n_tilde_q);(sin(0.5*n_tilde_q)/n_tilde_q).*tilde_q]);
        delta_q[0] = cosf(0.5f * n_tilde_q);
        rotScaler = (sinf(0.5f * n_tilde_q)) / n_tilde_q;
        delta_q[1] = tilde_q.x * rotScaler;
        delta_q[2] = tilde_q.y * rotScaler;
        delta_q[3] = tilde_q.z * rotScaler;
    }

// update the quaternions by rotating from the previous attitude through
// the delta angle rotation quaternion
    D_q_k1[0] = D_q[0]*delta_q[0] - D_q[1]*delta_q[1] - D_q[2]*delta_q[2] - D_q[3]*delta_q[3];
    D_q_k1[1] = D_q[0]*delta_q[1] + D_q[1]*delta_q[0] + D_q[2]*delta_q[3] - D_q[3]*delta_q[2];
    D_q_k1[2] = D_q[0]*delta_q[2] + D_q[2]*delta_q[0] + D_q[3]*delta_q[1] - D_q[1]*delta_q[3];
    D_q_k1[3] = D_q[0]*delta_q[3] + D_q[3]*delta_q[0] + D_q[1]*delta_q[2] - D_q[2]*delta_q[1];

    n_D_q_k1 = D_q_k1[0]*D_q_k1[0]+D_q_k1[1]*D_q_k1[1]+D_q_k1[2]*D_q_k1[2]+D_q_k1[3]*D_q_k1[3];
    D_q_k1[0] = D_q_k1[0]/n_D_q_k1;
    D_q_k1[1] = D_q_k1[1]/n_D_q_k1;
    D_q_k1[2] = D_q_k1[2]/n_D_q_k1;
    D_q_k1[3] = D_q_k1[3]/n_D_q_k1;

// Buffering the updated D_q_k1
    storeDataQuaternion(D_q_k1, storedD, lastDStoreTime_ms, DTimeStamp, storeIndexD);
}


// The fuction AttitudePredictor uses the buffer storedD and the delayed estimate of the attitude (stored in quat)
// and computes the current attitude (i.e. the variable q_hat). This function implements the static
// equasions (7) of [A] (which is similar to the equasion (39) of [B]).
void AP_Predictors::AttitudePredictor(Quaternion quat)
{
    Quaternion q_tmp;

    D_Delay = storedD[bestStoreIndexD];

// computing the inverse quaternion D_q_delay^{-1}
    q_tmp[0]= D_Delay[0];
    q_tmp[1]= -D_Delay[1];
    q_tmp[2]= -D_Delay[2];
    q_tmp[3]= -D_Delay[3];

// multiplyig D_q_delay^{-1} by D_q
    q_hat[0] = q_tmp[0]*D_q[0] - q_tmp[1]*D_q[1] - q_tmp[2]*D_q[2] - q_tmp[3]*D_q[3];
    q_hat[1] = q_tmp[0]*D_q[1] + q_tmp[1]*D_q[0] + q_tmp[2]*D_q[3] - q_tmp[3]*D_q[2];
    q_hat[2] = q_tmp[0]*D_q[2] + q_tmp[2]*D_q[0] + q_tmp[3]*D_q[1] - q_tmp[1]*D_q[3];
    q_hat[3] = q_tmp[0]*D_q[3] + q_tmp[3]*D_q[0] + q_tmp[1]*D_q[2] - q_tmp[2]*D_q[1];

// multiplying q_hat_tau by (D_q_delay^{-1} \times D_q)
    q_tmp[0] = quat[0]*q_hat[0] - quat[1]*q_hat[1] - quat[2]*q_hat[2] - quat[3]*q_hat[3];
    q_tmp[1] = quat[0]*q_hat[1] + quat[1]*q_hat[0] + quat[2]*q_hat[3] - quat[3]*q_hat[2];
    q_tmp[2] = quat[0]*q_hat[2] + quat[2]*q_hat[0] + quat[3]*q_hat[1] - quat[1]*q_hat[3];
    q_tmp[3] = quat[0]*q_hat[3] + quat[3]*q_hat[0] + quat[1]*q_hat[2] - quat[2]*q_hat[1];

// Normalizing the quaternion
    q_tmp.normalize();
    q_hat=q_tmp;
}

// The functions VelocityModel and VelocityPredictor implement the first velocity predictor (proposed in [A]).

// The fuction VelocityModel uses velocity kinematics and updates the variable d_v and stores
// the updated one into the buffer storedd_v.
// The valiable d_v is an internal state of th predictor and corresponds to the variable \delta in [A].
// This function implements the dynamics (8) of [A].
void AP_Predictors::VelocityModel(Vector3f corrected_tilde_Vel12, ftype dtIMU)
{
    // Converting q_hat to a rotation matrix
    Matrix3f Tbn_temp;
    q_hat.rotation_matrix(Tbn_temp);

    Matrix3f prevTnb_pred;  // I might need to delete this line
    prevTnb_pred = Tbn_temp.transposed();  // I might need to delete this line

    // Using velocity dynamics to update d_v
    Vector3f tilde_Vel;
    tilde_Vel  = Tbn_temp*corrected_tilde_Vel12 + gravityNED*dtIMU;
    d_v+=tilde_Vel;

    // Storing d_v into the buffer storedd_v
    storeDataVector(d_v, storedd_v, lastd_vStoreTime_ms, d_vTimeStamp, storeIndexd_v);
}

// The fuction VelocityPredictor uses the buffer storedd_v and the delayed estimate of the velocity (stored in velocity)
// and computes the current velocity vector (i.e. the variable v_hat). This function implements the static
// equasions (9) of [A].
void AP_Predictors::VelocityPredictor(Vector3f velocity)
{
// picking up the delayed d_v
    d_v_Delay=storedd_v[bestStoreIndexd_v];

// computing the prediction of velocity v_hat
    v_hat = velocity + d_v- d_v_Delay;
}


// The functions PositionModel and PositionPredictor implement the first position predictor which
// relys on the first velocity predictor to predict the position.

// The fuction PositionModel uses position kinematics and updates the variable d_p and stores
// the updated one into the buffer storedd_p.
// The valiable d_p is an internal state of th predictor. Conseptualy, this function plays
// the same role for position prediction as VelocityModel does for velocity prediction.
void AP_Predictors::PositionModel(ftype dtIMU)
{
// position prediction
    d_p+= v_hat*dtIMU;
// buffering d_p
    storeDataVector(d_p, storedd_p, lastd_pStoreTime_ms, d_pTimeStamp, storeIndexd_p);
}

// The fuction PositionPredictor uses the buffer storedd_p and the delayed estimate of the position (stored in position)
// and computes the current position vector (i.e. the variable p_hat). This function plays the same role for position
// prediction as VelocityPredictor does for velocity prediction.
void AP_Predictors::PositionPredictor(Vector3f position)
{
// picking up the delayed d_p
    d_p_Delay=storedd_p[bestStoreIndexd_p];
// p_hat
    p_hat = position + d_p- d_p_Delay;
}

// The functions VelocityModel2 and VelocityPredictor2 implement the second velocity predictor (proposed in [B]).
// The second velocity predictor is independent of the first velocity predictor and is an alternative to that.
// In our initial simulations, the second velocity predictor performed better than the first predictor, but it has not
// yet fully confirmed.

// The fuction VelocityModel1 uses velocity kinematics and updates the variable d_v_m and stores
// the updated one into the buffer storedd_v_m.
// The valiable d_v_m is an internal state of the predictor and corresponds to the variable \delta in [B].
// This function implements the dynamics (38) of [B]. Due to the nature of the second predictor dynamics,
// the internal state d_v_m needs to be reset to zero every now and the to keep its trajectory bounded. This resetting
// is performed in this function.
void AP_Predictors::VelocityModel2(Vector3f corrected_tilde_Vel12)
{
// Mixed-invariant predictor
    Matrix3f Tbn_temp;
    D_q_k1.rotation_matrix(Tbn_temp);

// second velocity predictor dynamics
    d_v_m= d_v_m+Tbn_temp*corrected_tilde_Vel12 ;

// buffering d_v_m
    storeDataVector(d_v_m, storedd_v_m, lastd_v_mStoreTime_ms, d_v_mTimeStamp, storeIndexd_v_m);

// resetting the internal state d_v_m to zero every  20*2*BUFFER_SIZE milliseconds
// in order to keep its trajectory bounded. This resetting is done according to [B].
    ctr_rst=ctr_rst+1;
    if (ctr_rst== 2*BUFFER_SIZE) {
        ctr_rst=0;
        for (uint16_t i=0; i<=(BUFFER_SIZE-1); i++)
        {
            storedd_v_m[i]=storedd_v_m[i]-d_v_m;
        }
        d_v_m[0]=0;
        d_v_m[1]=0;
        d_v_m[2]=0;
    }
}

// The fuction VelocityPredictor2 uses the buffer storedd_v_m and the delayed estimate of the velocity (stored in velocity)
// and the delayed estimate of quaternion (stored in quat) and computes the current velocity vector (i.e. the variable v_hat_m).
// This function implements the static equasion (40) of [B].
void AP_Predictors::VelocityPredictor2(Quaternion quat, Vector3f velocity, AP_Int16 _msecTauDelay)
{
// picking up the delayed d_v_m
    Vector3f d_v_m_Delay=storedd_v_m[bestStoreIndexd_v_m];

    Quaternion q_tmp;
// computing the inverse quaternion D_q_delay^{-1}
    q_tmp[0]= D_Delay[0];
    q_tmp[1]= -D_Delay[1];
    q_tmp[2]= -D_Delay[2];
    q_tmp[3]= -D_Delay[3];

    Quaternion q_tmp_m;

// computing the quaternion multiplication q_hat_tau \times D_q_delay^{-1}
    q_tmp_m[0] = quat[0]*q_tmp[0] - quat[1]*q_tmp[1] - quat[2]*q_tmp[2] - quat[3]*q_tmp[3];
    q_tmp_m[1] = quat[0]*q_tmp[1] + quat[1]*q_tmp[0] + quat[2]*q_tmp[3] - quat[3]*q_tmp[2];
    q_tmp_m[2] = quat[0]*q_tmp[2] + quat[2]*q_tmp[0] + quat[3]*q_tmp[1] - quat[1]*q_tmp[3];
    q_tmp_m[3] = quat[0]*q_tmp[3] + quat[3]*q_tmp[0] + quat[1]*q_tmp[2] - quat[2]*q_tmp[1];

// Converting quaternion to rotation matrix.
    Matrix3f Tbn_temp;
    q_tmp_m.rotation_matrix(Tbn_temp);

// Computing the second velocity prediction v_hat_m.
    v_hat_m = velocity + gravityNED*(0.001f*constrain_int16(_msecTauDelay, 0, MAX_MSDELAY))+Tbn_temp*(d_v_m- d_v_m_Delay);
}

// The functions PositionModel2 and PositionPredictor2 implement the second position predictor which
// relys on the second velocity predictor to predict the position. The second position predictor is independent of
// the first position predictor and is an alternative to that.

// The fuction PositionModel2 uses position kinematics and updates the variable d_p_m and stores
// the updated one into the buffer storedd_p_m.
// The valiable d_p_m is an internal state of the predictor. Conseptualy, this function plays
// the same role for position prediction as VelocityModel does for velocity prediction.
void AP_Predictors::PositionModel2(ftype dtIMU)
{
// position predictor dynamics
    d_p_m+= v_hat_m*dtIMU;
// buffering d_p_m
    storeDataVector(d_p_m, storedd_p_m, lastd_p_mStoreTime_ms, d_p_mTimeStamp, storeIndexd_p_m);
}

// The fuction PositionPredictor2 uses the buffer storedd_p_m and the delayed estimate of the position (stored in position)
// and computes the current position vector (i.e. the variable p_hat_m). This function plays the same role for position
// prediction as VelocityPredictor does for velocity prediction.
void AP_Predictors::PositionPredictor2(Vector3f position)
{
// Picking up the delayed d_p_m
    d_p_Delay=storedd_p_m[bestStoreIndexd_p_m];
// Computing the prediction of position p_hat_m
    p_hat_m = position + d_p_m- d_p_Delay;
}

// The function storeDataVector is used to store a Vector3f into its corresponding buffer and also to store its
// time stamp into the corresponding time stamp buffer.
void AP_Predictors::storeDataVector(Vector3f &data, VectorN<Vector3f,BUFFER_SIZE> &buffer, uint32_t &lastStoreTime, uint32_t (&timeStamp)[BUFFER_SIZE], uint16_t &storeIndex)
{
    if (imuSampleTimePred_ms - lastStoreTime >= 10) {
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
    if (imuSampleTimePred_ms - lastStoreTime >= 10) {
        lastStoreTime = imuSampleTimePred_ms;
        if (storeIndex > (BUFFER_SIZE-1)) {
            storeIndex = 0;
        }
        buffer[storeIndex]=data;
        timeStamp[storeIndex] = lastStoreTime;
        storeIndex = storeIndex + 1;
    }
}

// The function BestIndex searchs the time stamp buffer and find the index that whose time stamp is closest to
// _msecTauDelay milliseconds before the correcnt time.
// We should later on replace this function with a FIFO buffer to decrease the computation load.
void AP_Predictors::BestIndex(uint32_t &closestTime, uint16_t &closestStoreIndex, uint32_t (&timeStamp)[BUFFER_SIZE], AP_Int16 &_msecTauDelay)
{
    uint32_t time_delta;
    closestTime = MAX_MSDELAY;
    closestStoreIndex = 0;

    for (int i=0; i<=(BUFFER_SIZE-1); i++)
    {
        time_delta = abs( (imuSampleTimePred_ms - timeStamp[i]) - constrain_int16(_msecTauDelay, 0, MAX_MSDELAY));
        if (time_delta < closestTime)
        {
            closestStoreIndex = i;
            closestTime = time_delta;
        }
    }
}

// The function getAttitudePrediction returns the current prediction of attitude.
void AP_Predictors::getAttitudePrediction(Quaternion &att) const
{
    att = q_hat;
}

// The function getVelocityPrediction returns the current prediction of velocity from the first predictor.
void AP_Predictors::getVelocityPrediction(Vector3f &vel) const
{
    vel = v_hat;
}

// The function getPositionPrediction returns the current prediction of position from the first predictor.
void AP_Predictors::getPositionPrediction(Vector3f &pos) const
{
    pos = p_hat;
}

// The function getVelocity2Prediction returns the current prediction of velocity from the second predictor.
void AP_Predictors::getVelocity2Prediction(Vector3f &vel) const
{
    vel = v_hat_m;
}

// The function getPosition2Prediction returns the current prediction of position from the second predictor.
void AP_Predictors::getPosition2Prediction(Vector3f &pos) const
{
    pos = p_hat_m;
}
