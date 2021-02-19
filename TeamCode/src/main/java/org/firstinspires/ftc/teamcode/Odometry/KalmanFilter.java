package org.firstinspires.ftc.teamcode.Odometry;

import android.graphics.Point;
import android.sax.StartElementListener;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class KalmanFilter {
    private SimpleMatrix H;
    private SimpleMatrix covariance;
    private SimpleMatrix state;
    private SimpleMatrix processNoise;
    private SimpleMatrix measurementNoise;

    private SimpleMatrix KalmanGain;

    private Telemetry telemetry;

    public KalmanFilter(Telemetry telemetry){
        this.telemetry = telemetry;
        H = SimpleMatrix.identity(3);

        state = new SimpleMatrix(3, 1);

        KalmanGain = new SimpleMatrix(3, 3);

        covariance = new SimpleMatrix(3, 3);

        //Odometry Noise
        processNoise = new SimpleMatrix(3, 3, true, new double[]{
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01
        });

        //Camera Noise
        measurementNoise = new SimpleMatrix(3, 3, true, new double[]{
                0.5, 0.0, 0.0,
                0.0, 0.5, 0.0,
                0.0, 0.0, 0.5
        });;
    }

    public void correct(SimpleMatrix z){
        //Computing the kalman gain
        KalmanGain = covariance.mult(H.transpose()).mult((H.mult(covariance).mult(H.transpose()).plus(measurementNoise)).invert());

        //Tweaking the state and covariance matrix using the kalman gain
        state = state.plus(KalmanGain.mult(z.minus(H.mult(state))));
        telemetry.addData("state matrix", state);
        covariance = covariance.minus(KalmanGain.mult(H).mult(covariance));
    }

    public Pose2d getEstimate(){
        return new Pose2d(state.get(0, 0), state.get(1, 0), state.get(2, 0));
    }

    //For the state parameter, just pass in the pose exponential
    public void predict(SimpleMatrix state, Pose2d velo, float dt){
        SimpleMatrix veloMat = new SimpleMatrix(3, 1, true, new double[]{velo.getX() * dt, velo.getY() * dt, velo.getHeading() * dt});
        this.state = state.plus(veloMat);
        covariance = ((SimpleMatrix.identity(3).mult(covariance)).mult(SimpleMatrix.identity(3).transpose())).plus(processNoise);
    }
}
