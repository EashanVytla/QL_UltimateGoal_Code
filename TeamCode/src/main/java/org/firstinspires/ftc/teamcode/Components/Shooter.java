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
    public static double kp_shooter = 5;
    public static double ki_shooter = 0;
    public static double kd_shooter = 0.1;

    public static double SlidesTunerAngle = 22;

    public Caching_Servo pushSlide;
    public Caching_Servo stopper;
    public Caching_Servo flicker;

    public Caching_Motor shooter;
    private Telemetry telemetry;

    public final double flickPosDown = .248;//.07
    public final double flickPosUp = 0.0;

    public  double stopPosUp = 0.0;
    public  double stopPosDown = 0.3;

    public final double pushIdle = 0.859;
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

    ElapsedTime time = new ElapsedTime();

    private int powerShotToggle = 0;

    public enum ShootState{
        PREPARE,
        SHOOT,
        IDLE
    }
    public ShootState mRobotState = ShootState.IDLE;

    public Shooter(HardwareMap map, Telemetry telemetry){
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



        time.startTime();
    }

    public void init(){
        flicker.setPosition(flickPosUp);
        stopper.setPosition(stopPosDown);
        pushSlide.setPosition(pushIdle);
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
        rightSlide.setPower(Range.clip(power + 0.05, -1, 1));
        leftSlide.setPower(Range.clip(power + 0.05, -1, 1));
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
        setShooterAngle(17.1, currentAngle, 0.5);
        shooter.setPower(0.2);
        stopper.setPosition(stopPosDown);
        flicker.setPosition(flickPosUp);
        mRobotState = ShootState.IDLE;
        mStateTime.reset();
    }

    public void operate(GamepadEx gamepad1, GamepadEx gamepad2, double distFromGoal){
        double shooterTargetAngle = calculateShooterAngle(distFromGoal);
        double LT = gamepad2.gamepad.left_trigger;

        telemetry.addData("Shooter Angle Required", shooterTargetAngle);


        if(gamepad2.isPress(GamepadEx.Control.left_trigger)){
            mStateTime.reset();
            stopper.setPosition(stopPosUp);
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

        if(PROTO_AlignSlides){
            currentAngle = getShooterAngle();
            if(Math.abs(currentAngle - Math.toRadians(shooterTargetAngle)) < Math.toRadians(0.2)){
                PROTO_AlignSlides = false;
            }else{
                //setShooterAngle(Math.toRadians(shooterTargetAngle), currentAngle, 1.0);
                setShooterAngle(Math.toRadians(SlidesTunerAngle), currentAngle, 1.0);
            }
        }else{
            //if(Math.abs(getShooterAngle() - Math.toRadians(desiredAngle)) >= Math.toRadians(0.1) && gamepad2.gamepad.atRest()){
            //    setShooterAngle(Math.toRadians(desiredAngle), getShooterAngle());
            //}else{

            //}

            if(reset){
                currentAngle = getShooterAngle();
                setShooterAngle(Math.toRadians(17.1), currentAngle, 0.5);
                if(Math.abs(currentAngle - Math.toRadians(17.1)) < Math.toRadians(0.3) || !gamepad2.gamepad.atRest()){
                    reset = false;
                }
            }else{
                slideSetPower(gamepad2.gamepad.left_stick_y * 0.25);
            }
        }

        telemetry.addData("desired angle value", desiredAngle);
        telemetry.addData("Current Angle(Last Read)", Math.toDegrees(currentAngle));


        if(gamepad2.isPress(GamepadEx.Control.right_trigger)){
            flickerToggle = !flickerToggle;
            if (flickerToggle) {
                flicker.setPosition(flickPosDown);
                shooter.setPower(1.0);
                stopper.setPosition(stopPosUp);
            }else{
                flicker.setPosition(flickPosUp);
                shooter.setPower(0.2);
                stopper.setPosition(stopPosDown);
            }
        }

        if(gamepad2.isPress(GamepadEx.Control.b)){
            reset();
        }

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
                if(mStateTime.time() <= 1){
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
                if (mStateTime.time() <= 0.5){
                    pushSlide.setPosition(pushIdle);
                }
                break;
        }

        previousLT = gamepad2.gamepad.left_trigger;
    }
}