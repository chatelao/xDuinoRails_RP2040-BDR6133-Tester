#if ARDUINO
#include "SimpleKalmanFilterWrapper.h"

SimpleKalmanFilterWrapper::SimpleKalmanFilterWrapper(float mea_e, float est_e, float q)
    : _filter(mea_e, est_e, q) {}

float SimpleKalmanFilterWrapper::updateEstimate(float measurement) {
    return _filter.updateEstimate(measurement);
}
#endif // ARDUINO
