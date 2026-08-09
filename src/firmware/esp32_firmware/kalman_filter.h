/**
 * @file kalman_filter.h
 * @brief 1D Discrete Kalman Filter implementation for sensor noise reduction
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

class KalmanFilter {
private:
    float _q; // Process noise covariance
    float _r; // Measurement noise covariance
    float _x; // Estimated state
    float _p; // Estimation error covariance
    float _k; // Kalman gain

public:
    /**
     * @brief Construct a new Kalman Filter object
     * @param processNoise Process noise covariance Q (e.g. 0.01)
     * @param measurementNoise Measurement noise covariance R (e.g. 0.1)
     * @param estimationError Initial estimation error P (e.g. 1.0)
     * @param initialValue Initial state value X (e.g. 0.0)
     */
    KalmanFilter(float processNoise = 0.01f, float measurementNoise = 0.1f, float estimationError = 1.0f, float initialValue = 0.0f) {
        _q = processNoise;
        _r = measurementNoise;
        _p = estimationError;
        _x = initialValue;
        _k = 0.0f;
    }

    /**
     * @brief Update the filter with a new measurement
     * @param measurement New raw measurement
     * @return float Filtered estimate
     */
    float update(float measurement) {
        // Prediction update
        _p = _p + _q;

        // Kalman gain calculation
        _k = _p / (_p + _r);

        // State measurement update
        _x = _x + _k * (measurement - _x);

        // Error covariance update
        _p = (1.0f - _k) * _p;

        return _x;
    }

    /**
     * @brief Get the current filtered estimate value
     */
    float getValue() const {
        return _x;
    }

    /**
     * @brief Reset the state estimate
     */
    void reset(float initialValue = 0.0f) {
        _x = initialValue;
        _p = 1.0f;
    }
};

#endif // KALMAN_FILTER_H
