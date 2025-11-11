#ifndef IKALMAN_FILTER_H
#define IKALMAN_FILTER_H

class IKalmanFilter {
public:
    virtual ~IKalmanFilter() {}
    virtual float updateEstimate(float measurement) = 0;
};

#endif // IKALMAN_FILTER_H
