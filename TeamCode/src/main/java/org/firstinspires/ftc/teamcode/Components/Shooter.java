package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
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
import org.firstinspires.ftc.teamcode.Odometry.S4T_Localizer;
import org.firstinspires.ftc.teamcode.OpModes.LinearTeleOp;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Motor;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

@Config
public class Shooter {
    public static double kp_shooter = 10.25;
    public static double ki_shooter = 0;
    public static double kd_shooter = 0.05;

    public static double SlidesTunerAngle = 28.25;

    public Caching_Servo pushSlide;
    public Caching_Servo stopper;
    public Caching_Servo flicker;

    public Caching_Motor shooter;
    private Telemetry telemetry;

    public final double flickPosDown = 0.235;//0.22;//.07
    public final double flickPosDownInitial = 0.2449;//.07
    public final double flickPosUp = 0.0;

    public  double stopPosUp = 0.0;
    public  double stopPosDown = 0.16;

    public final double pushIdle = 0.9327;
    public final double pushForward = 0.37;//0.346;

    private boolean rbToggle = false;
    public boolean PROTO_AlignSlides = false;

    private Caching_Motor leftSlide;
    private Caching_Motor rightSlide;

    public ElapsedTime mStateTime;
    ///private DistanceSensor sensor;
    ExpansionHubMotor encoder;
    private RevBulkData data;

    private PIDFController slidesController;
    private boolean flickerToggle = false;

    private double downPos;

    public int powerShotToggle = 0;
    private double shooterff = 0.2;

    private boolean yToggle = false;
    public boolean automation = false;

    public enum ShootState{
        PREPARE,
        SHOOT,
        IDLE
    }
    public ShootState mRobotState = ShootState.IDLE;

    public Shooter(HardwareMap map, Telemetry telemetry){
        downPos = Math.toRadians(20.25);

        rightSlide = new Caching_Motor(map, "right_slide");
        leftSlide = new Caching_Motor(map, "left_slide");

        pushSlide = new Caching_Servo(map, "push_slide");
        stopper = new Caching_Servo(map, "stopper");
        flicker = new Caching_Servo(map, "flicker");

        shooter = new Caching_Motor(map, "shooter");
        mStateTime = new ElapsedTime();
        mStateTime.startTime();

        //sensor = map.get(DistanceSensor.class, "dist");
        encoder = (ExpansionHubMotor) map.get(DcMotor.class, "intake");

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slidesController = new PIDFController(new PIDCoefficients(kp_shooter, ki_shooter, kd_shooter));

        leftSlide.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlide.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

    double offset = 0;

    public double getShooterAngle(){
        //return Math.atan2(sensor.getDistance(DistanceUnit.INCH), 1.6693915023766982548289189220147);
        //return sensor.getDistance(DistanceUnit.MM);
        double angle = (data.getMotorCurrentPosition(encoder) * (2 * Math.PI)) / 8192.0;
        angle += Math.toRadians(20) + offset;
        angle %= 2 * Math.PI;
        return angle;
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
        /*double y = (-4.0174 * (10 ^ -8)) * Math.pow(myDist, 6.0);
        y += 0.0000160441 * Math.pow(myDist, 5.0);
        y -= 0.00260761 * Math.pow(myDist, 4.0);
        y += 0.219813 * Math.pow(myDist, 3.0);
        y -= 10.0727 * Math.pow(myDist, 2.0);
        y += 235.376 * myDist;
        y -= 2116.72;*/

        double y = (-0.191303 * myDist) + 42.5342;

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
        rightSlide.setPower(Range.clip(power + 0.08, -1, 1));
        leftSlide.setPower(Range.clip(power + 0.08, -1, 1));
        telemetry.addData("Slide Power", power);
    }

    double desiredAngle = 25;
    public boolean reset = false;

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
            flicker.setPosition(flickPosUp);
            shooter.setPower(0.2);
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

    public boolean powerShotAngle = false;
    public boolean powerShots;

    public void operate(GamepadEx gamepad1, GamepadEx gamepad2, double distFromGoal){
        double currentAngle = getShooterAngle();
        double shooterTargetAngle = calculateShooterAngle(distFromGoal);

        telemetry.addData("Shooter Angle Required", shooterTargetAngle);
        telemetry.addData("State", mRobotState);
        S4T_Localizer.packet.put("Raw Tick Shooter" , data.getMotorCurrentPosition(encoder));
        telemetry.addData("Raw Tick Shooter" , data.getMotorCurrentPosition(encoder));

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

        if(gamepad2.isPress(GamepadEx.Control.a)){
            yToggle = !yToggle;
            if(yToggle) {
                shooter.setPower(1.0);
            }else{
                shooter.setPower(0.0);
            }
        }

        if(gamepad2.isPress(GamepadEx.Control.dpad_up)){
            PROTO_AlignSlides = true;
            powerShotAngle = false;
            reset = false;
            //desiredAngle += 1;
        }

        if(gamepad2.isPress(GamepadEx.Control.dpad_right)){
            PROTO_AlignSlides = true;
            powerShotAngle = true;
        }

        if(gamepad2.isPress(GamepadEx.Control.dpad_down)){
            PROTO_AlignSlides = false;
            powerShotAngle = false;
            reset = true;
        }

        if(gamepad2.isPress(GamepadEx.Control.start)){
            powerShot(powerShotToggle);
            powerShotToggle += 1;
            powerShotToggle %= 4;
        }

        telemetry.addData("Current Angle", Math.toDegrees(currentAngle));

        if(gamepad2.isPress(GamepadEx.Control.right_trigger)){
            flickerToggle = !flickerToggle;
            if (flickerToggle) {
                flicker.setPosition(flickPosDownInitial);
                shooter.setPower(1.0);
            }else{
                flicker.setPosition(flickPosUp);
                shooter.setPower(shooterff);
            }
        }

        if(gamepad2.isPress(GamepadEx.Control.b)){
            encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        telemetry.addData("Offset", offset);


        if(PROTO_AlignSlides){
            reset = false;
            if(Math.abs(currentAngle - (powerShotAngle ? Math.toRadians(20.9) : Math.toRadians(shooterTargetAngle))) < Math.toRadians(0.05) || Math.abs(gamepad2.gamepad.left_stick_y) >= 0.15){
                if(mStateTime.time() >= 0.5){
                    PROTO_AlignSlides = false;
                    powerShotAngle = false;
                }else{
                    setShooterAngle(powerShotAngle ? Math.toRadians(20.9) : Math.toRadians(SlidesTunerAngle), currentAngle, 1.0);
                }
            }else{
                mStateTime.reset();
                //setShooterAngle(powerShotAngle ? Math.toRadians(20.9) : Math.toRadians(shooterTargetAngle), currentAngle, 1.0);
                setShooterAngle(powerShotAngle ? Math.toRadians(20.9) : Math.toRadians(SlidesTunerAngle), currentAngle, 1.0);
            }
        }else{
            //if(Math.abs(getShooterAngle() - Math.toRadians(desiredAngle)) >= Math.toRadians(0.1) && gamepad2.gamepad.atRest()){
            //    setShooterAngle(Math.toRadians(desiredAngle), getShooterAngle());
            //}else{

            //}

            if(reset){
                setShooterAngle(downPos, currentAngle, 0.5);
                if(currentAngle <= downPos){
                    powerShotAngle = false;
                    reset = false;
                }
            }else{
                if(!powerShots){
                    slideSetPower(gamepad2.gamepad.left_stick_y * 0.25);
                }
            }
        }

        switch (mRobotState){
            case PREPARE:
                double velo = shooter.motor.getVelocity(AngleUnit.RADIANS);
                telemetry.addData("Motor Velocity: ", velo);
                if(Math.abs(velo) <= 5.4){
                    shooter.setPower(1);
                    flicker.setPosition(flickPosDown);
                    stopper.setPosition(stopPosUp);
                } else {
                    mStateTime.reset();
                    mRobotState = ShootState.SHOOT;
                }
                break;
            case SHOOT:
                if(mStateTime.time() <= 1.5){
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

                break;
        }
    }
}
