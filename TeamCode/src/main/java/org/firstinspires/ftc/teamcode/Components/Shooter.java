package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Motor;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;
import org.openftc.revextensions2.RevBulkData;

@Config
public class Shooter {
    public static double kp_shooter = 0;
    public static double ki_shooter = 0;
    public static double kd_shooter = 0;

    private Caching_Servo pushSlide;
    public Caching_Servo stopper;
    public Caching_Servo flicker;

    public Caching_Motor shooter;
    private Telemetry telemetry;

    public final double flickPosDown = 0.304;//.07
    public final double flickPosUp = 0.0;

    public  double stopPosUp = 0.0;
    public  double stopPosDown = 0.346;

    public final double pushIdle = 0.859;
    public final double pushForward = 0.346;

    private boolean rbToggle = false;
    private boolean PROTO_AlignSlides = false;

    private Caching_Motor leftSlide;
    private Caching_Motor rightSlide;

    public ElapsedTime mStateTime;
    private DistanceSensor sensor;
    private RevBulkData data;

    private PIDFController slidesController;
    private boolean flickerToggle = false;

    public enum ShootState{
        PREPARE,
        SHOOT,
        IDLE
    }
    ShootState mRobotState = ShootState.IDLE;

    public Shooter(HardwareMap map, Telemetry telemetry){
        rightSlide = new Caching_Motor(map, "right_slide");
        leftSlide = new Caching_Motor(map, "left_slide");

        pushSlide = new Caching_Servo(map, "push_slide");
        stopper = new Caching_Servo(map, "stopper");
        flicker = new Caching_Servo(map, "flicker");

        shooter = new Caching_Motor(map, "shooter");
        mStateTime = new ElapsedTime();
        mStateTime.startTime();
        flicker.setPosition(flickPosUp);
        stopper.setPosition(stopPosDown);
        pushSlide.setPosition(pushIdle);
        sensor = map.get(DistanceSensor.class, "dist");

        slidesController = new PIDFController(new PIDCoefficients(kp_shooter, ki_shooter, kd_shooter));

        leftSlide.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.telemetry = telemetry;

        shooter.setPower(0.2);
        write();
    }

    public double getShooterAngle(/*RevBulkData data*/){
        return Math.atan2(sensor.getDistance(DistanceUnit.MM), 46.877636208134828577760733236494);
        //return sensor.getDistance(DistanceUnit.MM);
        //return ((data.get...(ma3) * (2 * Math.PI))/(3260.0)) + Math.toRadians(16.25);
    }

    public void write(){
        rightSlide.write();
        leftSlide.write();
        pushSlide.write();
        stopper.write();
        flicker.write();
        shooter.write();
    }

    public double calculateShooterAngle(double dist) {
        double myDist = dist - 7.25;
        telemetry.addData("Dist to Ultimate Goal", myDist);
        /*double y = (1.7212 * Math.pow(10.0, -11.0)) * Math.pow(myDist, 6);
        y -= (1.106 * Math.pow(10.0, -8.0)) * Math.pow(myDist, 5);
        y += 0.00000305118 * Math.pow(myDist, 4);
        y -= 0.000470727 * Math.pow(myDist, 3);
        y += 0.0443061 * Math.pow(myDist, 2);
        y -= 2.49734 * myDist;
        y += 89.3327;*/

        /*double y = (1.4892 * Math.pow(10.0, -11.0)) * Math.pow(myDist, 6);
        //y -= (9.2159 * Math.pow(10.0, -9.0)) * Math.pow(myDist, 5);
        //y += 0.0000024491 * Math.pow(myDist, 4);
        y -= 0.00036536 * Math.pow(myDist, 3);
        y += 0.0336898 * Math.pow(myDist, 2);
        y -= 1.90701 * myDist;
        y += 75.5784;*/

        double y = -0.000146832 * Math.pow(myDist, 3);
        y += 0.0422034 * Math.pow(myDist, 2);
        y -= 4.09676 * myDist;
        y += 153.494;

        return y;
    }

    public void setShooterAngle(double targetAngle, double currentAngle){
        slidesController.setTargetPosition(targetAngle);
        double power = slidesController.update(currentAngle);
        telemetry.addData("PID Power", power);
        telemetry.addData("Target Angle", targetAngle);
        telemetry.addData("Current Angle", currentAngle);
        telemetry.addData("Error", targetAngle - currentAngle);
        //rightSlide.setPower(Range.clip(power, -1, 1));
        //leftSlide.setPower(Range.clip(power, -1, 1));
        slideSetPower(power);
    }

    public void setData(RevBulkData data){
        this.data = data;
    }

    public void slideSetPower(double power){
        rightSlide.setPower(Range.clip(power + 0.05, -1, 1));
        leftSlide.setPower(Range.clip(power + 0.05, -1, 1));
        telemetry.addData("Slide Power", power);
    }

    double desiredAngle = 25;

    public void operate(GamepadEx gamepad1, GamepadEx gamepad2, double distFromGoal){
        double shooterTargetAngle = calculateShooterAngle(distFromGoal);

        telemetry.addData("Shooter Angle Required", shooterTargetAngle);
        telemetry.addData("Shooter Angle Current", Math.toDegrees(getShooterAngle()));

        if(gamepad2.isPress(GamepadEx.Control.left_bumper)){
            mStateTime.reset();
            mRobotState = !rbToggle ? ShootState.PREPARE : ShootState.IDLE;
            rbToggle = !rbToggle;
        }

        if(gamepad2.isPress(GamepadEx.Control.y)){
            mRobotState = ShootState.SHOOT;
        }

        if(gamepad2.isPress(GamepadEx.Control.dpad_up)){
            PROTO_AlignSlides = !PROTO_AlignSlides;
            //desiredAngle += 1;
        }

        /*if(gamepad2.isPress(GamepadEx.Control.dpad_down)){
            desiredAngle -= 1;
        }*/

        telemetry.addData("CHECK THIS", PROTO_AlignSlides);

        if(PROTO_AlignSlides){
            setShooterAngle(Math.toRadians(shooterTargetAngle), getShooterAngle());
        }else{
            //if(Math.abs(getShooterAngle() - Math.toRadians(desiredAngle)) >= Math.toRadians(0.1) && gamepad2.gamepad.atRest()){
            //    setShooterAngle(Math.toRadians(desiredAngle), getShooterAngle());
            //}else{
                slideSetPower(gamepad2.gamepad.left_stick_y * 0.25);
            //}
        }

        telemetry.addData("desired angle value", desiredAngle);

        if(gamepad2.isPress(GamepadEx.Control.b)){
            flickerToggle = !flickerToggle;
        }

        if (flickerToggle) {
            flicker.setPosition(flickPosDown);
        }else{
            flicker.setPosition(flickPosUp);
        }

        telemetry.addData("Motor Velocity: ", shooter.motor.getVelocity(AngleUnit.RADIANS));

        switch (mRobotState){
            case PREPARE:
                if(Math.abs(shooter.motor.getVelocity(AngleUnit.RADIANS)) <= 5.4){
                    shooter.setPower(1);
                    flicker.setPosition(flickPosDown);
                    stopper.setPosition(stopPosUp);
                } else {
                    mStateTime.reset();
                    mRobotState = ShootState.SHOOT;
                }
                break;
            case SHOOT:
                if(mStateTime.time() <= 2){
                    shooter.setPower(1);
                    pushSlide.setPosition(pushForward);
                } else {
                    shooter.setPower(0.2);
                    stopper.setPosition(stopPosDown);
                    flicker.setPosition(flickPosUp);
                    mRobotState = ShootState.IDLE;
                    mStateTime.reset();
                }
                break;
            case IDLE:
                if (mStateTime.time() <= 1){
                    pushSlide.setPosition(pushIdle);
                }
                break;
        }
    }
}