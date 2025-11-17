#if ARDUINO
#ifndef SIMPLE_KALMAN_FILTER_WRAPPER_H
#define SIMPLE_KALMAN_FILTER_WRAPPER_H

#include "interfaces/IKalmanFilter.h"
#include <SimpleKalmanFilter.h>

class SimpleKalmanFilterWrapper : public IKalmanFilter {
public:
    SimpleKalmanFilterWrapper(float mea_e, float est_e, float q);
    float updateEstimate(float measurement) override;

private:
    SimpleKalmanFilter _filter;
};

#endif // SIMPLE_KALMAN_FILTER_WRAPPER_H
#endif // ARDUINO
