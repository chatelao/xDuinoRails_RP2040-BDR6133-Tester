#ifndef MOCK_KALMAN_FILTER_H
#define MOCK_KALMAN_FILTER_H

#include "interfaces/IKalmanFilter.h"

class MockKalmanFilter : public IKalmanFilter {
public:
    float updateEstimate(float measurement) override {
        // In the mock, just pass the measurement through directly.
        // This allows us to test the rest of the system without the filter's influence.
        return measurement;
    }
};

#endif // MOCK_KALMAN_FILTER_H
