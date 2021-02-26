package org.firstinspires.ftc.teamcode.Odometry;

class OneDimensionlKalmanFilter {
    double processNoise = 3;
    double measurementNoise = 0.1;

    double state = 0.0;
    double covariance = 0.0;

    double KalmanGain = 0.0;

    public void predict(double state, double dt, double omega){
        this.state = state + dt * omega;
        covariance += processNoise;
    }

    public void correct(double z){
        KalmanGain = covariance * Math.pow((covariance + measurementNoise), -1);

        covariance -= KalmanGain * covariance;
        state += KalmanGain * z;
    }
}
