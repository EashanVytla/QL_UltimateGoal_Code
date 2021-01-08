package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
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
import org.firstinspires.ftc.teamcode.OpModes.LinearTeleOp;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Motor;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;
import org.openftc.revextensions2.RevBulkData;

@Config
public class Shooter {
    public static double kp_shooter = 5;
    public static double ki_shooter = 0;
    public static double kd_shooter = 0.1;

    public static double SlidesTunerAngle = 30;

    public Caching_Servo pushSlide;
    public Caching_Servo stopper;
    public Caching_Servo flicker;

    public Caching_Motor shooter;
    private Telemetry telemetry;

    public final double flickPosDown = 0.2599;//.07
    public final double flickPosUp = 0.0;

    public  double stopPosUp = 0.0;
    public  double stopPosDown = 0.16;

    public final double pushIdle = 0.9327;
    public final double pushForward = 0.346;
    double previousLT = 0;

    private boolean rbToggle = false;
    private boolean PROTO_AlignSlides = false;

    private Caching_Motor leftSlide;
    private Caching_Motor rightSlide;

    public ElapsedTime mStateTime;
    private DistanceSensor sensor;
    private RevBulkData data;

    private PIDFController slidesController;
    private boolean flickerToggle = false;
    private boolean shooterToggle = false;

    private double downPos;

    private int powerShotToggle = 0;
    private double shooterff = 0.2;

    private boolean yToggle = false;

    public enum ShootState{
        PREPARE,
        SHOOT,
        IDLE
    }
    public ShootState mRobotState = ShootState.IDLE;

    public Shooter(HardwareMap map, Telemetry telemetry){
        downPos = Math.toRadians(19.6);

        rightSlide = new Caching_Motor(map, "right_slide");
        leftSlide = new Caching_Motor(map, "left_slide");

        pushSlide = new Caching_Servo(map, "push_slide");
        stopper = new Caching_Servo(map, "stopper");
        flicker = new Caching_Servo(map, "flicker");

        shooter = new Caching_Motor(map, "shooter");
        mStateTime = new ElapsedTime();
        mStateTime.startTime();

        sensor = map.get(DistanceSensor.class, "dist");

        slidesController = new PIDFController(new PIDCoefficients(kp_shooter, ki_shooter, kd_shooter));

        leftSlide.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.telemetry = telemetry;
    }

    public void init(){
        flicker.setPosition(flickPosUp);
        stopper.setPosition(stopPosDown);
        pushSlide.setPosition(pushIdle);
        shooter.setPower(shooterff);
        mRobotState = ShootState.IDLE;
        write();
    }

    public double getShooterAngle(/*RevBulkData data*/){
        return Math.atan2(sensor.getDistance(DistanceUnit.INCH), 1.6693915023766982548289189220147);
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

    public void setShooterAngle(double targetAngle, double currentAngle, double maxPower){
        slidesController.setTargetPosition(targetAngle);
        double power = slidesController.update(currentAngle);
        telemetry.addData("PID Power", power);
        telemetry.addData("Target Angle", Math.toDegrees(targetAngle));
        telemetry.addData("Current Angle", Math.toDegrees(currentAngle));
        telemetry.addData("Error", Math.toDegrees(targetAngle - currentAngle));
        //rightSlide.setPower(Range.clip(power, -1, 1));
        //leftSlide.setPower(Range.clip(power, -1, 1));
        slideSetPower(Range.clip(power, -maxPower, maxPower));
    }

    public void setData(RevBulkData data){
        this.data = data;
    }

    public void slideSetPower(double power){
        rightSlide.setPower(Range.clip(power + 0, -1, 1));
        leftSlide.setPower(Range.clip(power + 0, -1, 1));
        telemetry.addData("Slide Power", power);
    }

    double desiredAngle = 25;
    private boolean reset = false;

    public void powerShot(ElapsedTime time){
        if(time.time() >= 3){
            pushSlide.setPosition(pushIdle);
        }else if(time.time() >= 2){
            pushSlide.setPosition(pushForward);
        }else if(time.time() >= 1){
            pushSlide.setPosition(pushIdle - ((2*(pushIdle - pushForward)/3)));
        }else{
            pushSlide.setPosition(pushIdle - ((pushIdle - pushForward)/3));
        }
    }

    public void powerShot(int state){
        if(state == 3){
            pushSlide.setPosition(pushIdle);
        }else if(state == 2){
            pushSlide.setPosition(pushForward);
        }else if(state == 1){
            pushSlide.setPosition(pushIdle - ((2*(pushIdle - pushForward)/3) - 0.025));
        }else if(state == 0){
            pushSlide.setPosition(pushIdle - ((pushIdle - pushForward)/3));
        }else{
            pushSlide.setPosition(pushIdle);
        }
    }

    public void reset(){
        double currentAngle = getShooterAngle();

        setShooterAngle(downPos, currentAngle, 0.5);
        shooter.setPower(shooterff);
        stopper.setPosition(stopPosDown);
        flicker.setPosition(flickPosUp);
        pushSlide.setPosition(pushIdle);
        mRobotState = ShootState.IDLE;
        mStateTime.reset();
    }

    private boolean powerShotAngle = false;

    public void operate(GamepadEx gamepad1, GamepadEx gamepad2, double distFromGoal){
        double shooterTargetAngle = calculateShooterAngle(distFromGoal);

        telemetry.addData("Shooter Angle Required", shooterTargetAngle);
        telemetry.addData("State", mRobotState);


        if(gamepad2.isPress(GamepadEx.Control.left_trigger)){
            mStateTime.reset();
            flicker.setPosition(flickPosDown);
            stopper.setPosition(stopPosUp);
            rbToggle = !rbToggle;
            mRobotState = rbToggle ? ShootState.PREPARE : ShootState.IDLE;
        }

        if(gamepad1.isPress(GamepadEx.Control.a)){
            stopper.setPosition(stopPosUp);
        }

        if(gamepad2.isPress(GamepadEx.Control.y)){

            yToggle = !yToggle;
            if(yToggle) {
                shooter.setPower(1.0);
            }else{
                shooter.setPower(0.0);
            }
        }

        telemetry.addData("Y Toggle", yToggle);

        if(gamepad2.isPress(GamepadEx.Control.dpad_up)){
            PROTO_AlignSlides = !PROTO_AlignSlides;
            //desiredAngle += 1;
        }

        if(gamepad2.isPress(GamepadEx.Control.dpad_right)){
            PROTO_AlignSlides = true;
            powerShotAngle = true;
        }

        telemetry.addData("Dpad Left", gamepad2.gamepad.dpad_left);
        telemetry.addData("Dpad Right", gamepad2.gamepad.dpad_right);

        if(gamepad2.isPress(GamepadEx.Control.dpad_down)){
            PROTO_AlignSlides = false;
            reset = true;
        }

        if(gamepad2.isPress(GamepadEx.Control.start)){
            powerShot(powerShotToggle);
            powerShotToggle += 1;
            powerShotToggle %= 4;
        }

        double currentAngle = 0;



        telemetry.addData("desired angle value", desiredAngle);
        telemetry.addData("Current Angle(Last Read)", Math.toDegrees(currentAngle));
        telemetry.addData("Current Angle", Math.toDegrees(getShooterAngle()));


        if(gamepad2.isPress(GamepadEx.Control.right_trigger)){
            flickerToggle = !flickerToggle;
            if (flickerToggle) {
                flicker.setPosition(flickPosDown);
                shooter.setPower(1.0);
            }else{
                flicker.setPosition(flickPosUp);
                shooter.setPower(shooterff);
            }
        }

        telemetry.addData("RB Toggle", flickerToggle);
        telemetry.addData("reset", reset);

        if(gamepad2.isPress(GamepadEx.Control.b)){
            reset();
        }

        telemetry.addData("push_slide Position: ", pushSlide.getPosition());

        switch (mRobotState){
            case PREPARE:
                double velo = shooter.motor.getVelocity(AngleUnit.RADIANS);
                telemetry.addData("Motor Velocity: ", velo);
                if(Math.abs(velo) <= 5.2){
                    shooter.setPower(1);
                    flicker.setPosition(flickPosDown);
                    stopper.setPosition(stopPosUp);
                } else {
                    mStateTime.reset();
                    mRobotState = ShootState.SHOOT;
                }
                break;
            case SHOOT:
                if(mStateTime.time() <= 1){
                    shooter.setPower(1);
                    pushSlide.setPosition(pushForward);
                } else {
                    rbToggle = false;
                    yToggle = false;
                    flickerToggle = false;
                    shooter.setPower(shooterff);
                    stopper.setPosition(stopPosDown);
                    flicker.setPosition(flickPosUp);
                    mRobotState = ShootState.IDLE;
                    mStateTime.reset();
                }
                break;
            case IDLE:
                if (mStateTime.time() <= 0.5){
                    pushSlide.setPosition(pushIdle);
                }

                if(PROTO_AlignSlides){
                    reset = false;
                    currentAngle = getShooterAngle();
                    if(Math.abs(currentAngle - (powerShotAngle ? Math.toRadians(24.9) : Math.toRadians(SlidesTunerAngle))) < Math.toRadians(0.2) && !gamepad2.gamepad.atRest()){
                        PROTO_AlignSlides = false;
                        powerShotAngle = false;
                    }else{
                        //setShooterAngle(Math.toRadians(shooterTargetAngle), currentAngle, 1.0);
                        setShooterAngle(powerShotAngle ? Math.toRadians(24.9) : Math.toRadians(SlidesTunerAngle), currentAngle, 1.0);
                    }
                }else{
                    //if(Math.abs(getShooterAngle() - Math.toRadians(desiredAngle)) >= Math.toRadians(0.1) && gamepad2.gamepad.atRest()){
                    //    setShooterAngle(Math.toRadians(desiredAngle), getShooterAngle());
                    //}else{

                    //}

                    if(reset){
                        currentAngle = getShooterAngle();
                        setShooterAngle(downPos, currentAngle, 0.5);
                        if(Math.abs(currentAngle - Math.toRadians(downPos)) < Math.toRadians(0.3) || !gamepad2.gamepad.atRest()){
                            powerShotAngle = false;
                            reset = false;
                        }
                    }else{
                        slideSetPower(gamepad2.gamepad.left_stick_y * 0.25);
                    }
                }
                break;
        }

        previousLT = gamepad2.gamepad.left_trigger;
    }
}