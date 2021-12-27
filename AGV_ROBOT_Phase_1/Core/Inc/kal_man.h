/*
 * kal_man.h
 *
 *  Created on: Nov 10, 2021
 *      Author: tu
 */

#ifndef INC_KAL_MAN_H_
#define INC_KAL_MAN_H_

class SimpleKalmanFilter
{

public:
  SimpleKalmanFilter(float mea_e, float est_e, float q);
  float updateEstimate(float mea);
  void setMeasurementError(float mea_e);
  void setEstimateError(float est_e);
  void setProcessNoise(float q);
  float getKalmanGain();
  float getEstimateError();

private:
  float _err_measure;
  float _err_estimate;
  float _q;
  float _current_estimate;
  float _last_estimate;
  float _kalman_gain;

};

#endif /* INC_KAL_MAN_H_ */
