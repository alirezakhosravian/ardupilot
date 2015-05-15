#ifndef AP_PREDICTORS_H
#define AP_PREDICTORS_H

#include <AP_Math.h>
#include <AP_Param.h>
#include <vectorN.h>

#define BUFFER_SIZE  50   //  buffer size for sensors, this allows buffering of at least BUFFER_SIZE*20 ms=MAX_MSDELAY of data
#define MAX_MSDELAY  BUFFER_SIZE*20   // maximum allowed delay

class AP_Predictors
{
public:
    AP_Predictors();
    typedef float ftype;
    void AttitudeModel(Vector3f tilde_q);
    void VelocityModel(Vector3f tilde_Vel);
    void VelocityModel2(Vector3f corrected_tilde_Vel12);
    void PositionModel(ftype dtIMU);
    void PositionModel2(ftype dtIMU);
    void AttitudePredictor(Quaternion quat);
    void PositionPredictor(Vector3f position);
    void PositionPredictor2(Vector3f position);
    void VelocityPredictor(Vector3f velocity);
    void VelocityPredictor2(Quaternion quat, Vector3f velocity, AP_Int16 _msecTauDelay);
    void CascadedPredictor(Vector3f tilde_q, Vector3f tilde_Vel, Vector3f corrected_tilde_Vel12, Quaternion quat, ftype dtIMU, AP_Int16 _msecTauDelay, Vector3f velocity, Vector3f position);
    void BestIndex(uint32_t &closestTime, uint16_t &closestStoreIndex, uint32_t (&timeStamp)[BUFFER_SIZE], AP_Int16 &_msecTauDelay);
    void storeDataVector(Vector3f &data, VectorN<Vector3f,BUFFER_SIZE> &buffer, uint32_t &lastStoreTime, uint32_t (&timeStamp)[BUFFER_SIZE], uint16_t &storeIndex);
    void storeDataQuaternion(Quaternion &data, VectorN<Quaternion,BUFFER_SIZE> &buffer, uint32_t &lastStoreTime, uint32_t (&timeStamp)[BUFFER_SIZE], uint16_t &storeIndex);
    void getAttitudePrediction(Quaternion &att);
    void getPositionPrediction(Vector3f &pos);
    void getPosition2Prediction(Vector3f &pos);
    void getVelocityPrediction(Vector3f &vel);
    void getVelocity2Prediction(Vector3f &vel);

private:
    uint32_t imuSampleTime_ms;
    Matrix3f D;
    Matrix3f D_T;
    Quaternion D_q;
    Quaternion D_q_k1;
    float n_D_q_k1;

    Quaternion q_hat;    // prediction of the current quaternion
    Quaternion q_hat_T_k1;
    float n_tilde_q;
    Quaternion delta_q;
    Matrix3f R_hat_T;
    Matrix3f R_hat;
    Vector3f d_v;
    Vector3f v_hat; // prediction of current velocity
    Vector3f d_p;
    Vector3f p_hat; // prediction of current position
    Vector3f v_hat_m; // prediction of current velocity mixed-invariant
    Vector3f d_p_m;
    Vector3f p_hat_m; // prediction of current position mixed-invariant
    Vector3f d_v_m;  // mixed-invariant
    Quaternion D_Delay;

    uint16_t storeIndexIMU;
    uint32_t lastAngRateStoreTime_ms;
    VectorN<Vector3f,BUFFER_SIZE> storedAngRate;
    VectorN<Vector3f,BUFFER_SIZE> storeddVelIMU1;
    VectorN<Vector3f,BUFFER_SIZE>  storeddVelIMU2;
    uint32_t angRateTimeStamp[BUFFER_SIZE];

    uint16_t storeIndexMag;
    uint32_t lastMagStoreTime_ms;
    VectorN<Vector3f,BUFFER_SIZE> storedMag;
    uint32_t MagTimeStamp[BUFFER_SIZE];

    uint16_t storeIndexTas;
    uint32_t lastTasStoreTime_ms;
    float storedTas[BUFFER_SIZE];
    uint32_t TasTimeStamp[BUFFER_SIZE];

    uint16_t storeIndexHgt;
    uint32_t lastHgtStoreTime_ms;
    float storedHgt[BUFFER_SIZE];
    uint32_t HgtTimeStamp[BUFFER_SIZE];
    uint32_t lastHealthyHgtTime_ms;  //time the barometer was last declared healthy

    // attitude model variables
    uint16_t storeIndexD;
    uint32_t lastDStoreTime_ms;
    uint32_t DTimeStamp[BUFFER_SIZE];
    VectorN<Quaternion,BUFFER_SIZE> storedD;
    uint32_t bestTimeD;
    uint16_t bestStoreIndexD;

    // velocity model variables
    uint16_t storeIndexd_v;
    uint32_t lastd_vStoreTime_ms;
    uint32_t d_vTimeStamp[BUFFER_SIZE];
    VectorN<Vector3f,BUFFER_SIZE> storedd_v;        //buffer for delta corrsponding to velocity prediction
    uint32_t bestTimed_v;
    uint16_t bestStoreIndexd_v;

    // position model variables
    uint16_t storeIndexd_p;
    uint32_t lastd_pStoreTime_ms;
    uint32_t d_pTimeStamp[BUFFER_SIZE];
    VectorN<Vector3f,BUFFER_SIZE> storedd_p;        //buffer for delta corrsponding to position prediction
    uint32_t bestTimed_p;
    uint16_t bestStoreIndexd_p;

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

    float init_reset;   // reset initial quaternions

    float rotScaler;
    Vector3f D_q_tmp;
    Quaternion q_tmp;

    uint32_t bestTime;
    uint16_t bestStoreIndex;

    Matrix3f prevTnb_pred;
    Matrix3f Tbn_temp;

    Vector3f d_p_Delay;
    Vector3f d_v_Delay;

};

#endif // AP_PREDICTORS_H
