package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.revextensions2.RevBulkData;

@Config
public class Shooter {
    public static double kp_shooter = 6;
    public static double ki_shooter = 0;
    public static double kd_shooter = 0.1;

    private Caching_Servo pushSlide;
    public Caching_Servo stopper;
    public Caching_Servo flicker;

    private Caching_Motor shooter;
    private Telemetry telemetry;

    private final double AUTO_LIFT_HEIGHT = 0.2;
    private final double RESTING_POS = 0.01;

    public final double flickPosDown = 0.1139999;
    public final double flickPosUp = 0.314;

    public final double stopPosUp = 0.6;
    public final double stopPosDown = 1;

    public final double pushIdle = 0.90;
    public final double pushForward = 0.29;

    private double slidePos = 0.12;

    private float prev_time = 0;

    private final double FF = 0.2;

    private boolean previousLB = false;
    private boolean previousA = false;
    public boolean aToggle = false;
    private boolean rbToggle = false;
    private boolean first = true;
    private boolean previousDpadUp = false;
    private boolean PROTO_AlignSlides = false;

    private Caching_Motor leftSlide;
    private Caching_Motor rightSlide;

    public ElapsedTime mStateTime;
    private AnalogInput MA3;
    private DistanceSensor sensor;

    private PIDFController slidesController;
    private RevBulkData data;

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
        flicker.setPosition(flickPosDown);
        stopper.setPosition(stopPosDown);
        pushSlide.setPosition(pushIdle);
        MA3 = map.get(AnalogInput.class, "ma3");
        sensor = map.get(DistanceSensor.class, "dist");

        slidesController = new PIDFController(new PIDCoefficients(kp_shooter, ki_shooter, kd_shooter));

        leftSlide.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.telemetry = telemetry;
    }

    public double getShooterAngle(RevBulkData data){
        return ((data.getAnalogInputValue(MA3) * (2 * Math.PI))/(3260.0)) + Math.toRadians(16.25);
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

        double y = (1.4892 * Math.pow(10.0, -11.0)) * Math.pow(myDist, 6);
        y -= (9.2159 * Math.pow(10.0, -9.0)) * Math.pow(myDist, 5);
        y += 0.0000024491 * Math.pow(myDist, 4);
        y -= 0.00036536 * Math.pow(myDist, 3);
        y += 0.0336898 * Math.pow(myDist, 2);
        y -= 1.90701 * myDist;
        y += 75.5784;

        return y;
    }

    public void setShooterAngle(double targetAngle, double currentAngle){
        slidesController.setTargetPosition(targetAngle);
        double power = slidesController.update(currentAngle);
        telemetry.addData("PID Power", power);
        //rightSlide.setPower(Range.clip(power, -1, 1));
        //leftSlide.setPower(Range.clip(power, -1, 1));
        slideSetPower(power);
    }

    public void setData(RevBulkData data){
        this.data = data;
    }

    public void slideSetPower(double power){
        rightSlide.setPower(Range.clip(power + 0.26, -1, 1));
        leftSlide.setPower(Range.clip(power + 0.26, -1, 1));
    }

    public void setShooterPower(double power){
        shooter.setPower(power);
    }

    public void operate(Gamepad gamepad1, Gamepad gamepad2, double distFromGoal){
        /*setHeightSlidePos(slidePos);
        if(gamepad2.left_trigger > 0.3){
            slidePos += 0.001;
        }*/

        /*if(gamepad1.dpad_up){
            slidePos+=.001;
        }else if(gamepad1.dpad_down){
            slidePos-=.001;
        }*/

        double shooterTargetAngle = calculateShooterAngle(distFromGoal);

        telemetry.addData("Shooter Angle Required", shooterTargetAngle);
        telemetry.addData("Sensor Data: ", sensor.getDistance(DistanceUnit.INCH));

        if(gamepad1.b){
            pushSlide.setPosition(pushForward);
        }else{
            pushSlide.setPosition(pushIdle);
        }

        if(gamepad1.a && !previousA){
            aToggle = !aToggle;
        }

        if(gamepad1.left_bumper && !previousLB){
            mStateTime.reset();
            mRobotState = !rbToggle ? ShootState.PREPARE : ShootState.IDLE;
            rbToggle = !rbToggle;
        }

        /*if(gamepad1.right_bumper || gamepad2.a){
            shooter.setPower(-1.0);
        } else {
            shooter.setPower(0);
        }*/

        if(gamepad1.dpad_up && !previousDpadUp){
            PROTO_AlignSlides = !PROTO_AlignSlides;
        }

        if(PROTO_AlignSlides){
            setShooterAngle(Math.toRadians(shooterTargetAngle), getShooterAngle(data));
        }else{
            slideSetPower(gamepad2.left_stick_y);
        }

        previousA = gamepad1.a;
        previousLB = gamepad1.left_bumper;

        previousDpadUp = gamepad1.dpad_up;


        switch (mRobotState){
            case PREPARE:
                if(Math.abs(shooter.motor.getVelocity(AngleUnit.RADIANS) - 5.2) <= 0.2){
                    shooter.setPower(1);
                    flicker.setPosition(flickPosDown);
                    stopper.setPosition(stopPosUp);
                    first = false;
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
                    stopper.setPosition(stopPosDown);
                    flicker.setPosition(flickPosUp);
                    shooter.setPower(FF);
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

        telemetry.addData("slide pos: ", slidePos);
        telemetry.addData("servo pos: ", pushSlide.getPosition());
        telemetry.addData("Velocity", shooter.motor.getVelocity(AngleUnit.RADIANS));
        telemetry.addData("Encoder Velocity", shooter.motor.getCurrentPosition());
    }
}
