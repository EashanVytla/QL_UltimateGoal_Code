package org.firstinspires.ftc.teamcode.Components;

import android.transition.Slide;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
    public static double kp_shooter = 32.5;
    public static double ki_shooter = 0;
    public static double kd_shooter = 1.0;

    public static double SlidesTunerAngle = 26.4;

    public Caching_Servo pushSlide;
    public Caching_Servo stopper;
    public Caching_Servo flicker;

    public Caching_Motor shooter;
    private Telemetry telemetry;

    public final double flickPosDown = 0.56252; //0.1919;//0.22;//.07
    public final double flickPosUp = 0.4839;

    public  double stopPosUp = 0.626;
    public  double stopPosDown = 0.923;

    public final double pushIdle = 0.975;
    public final double pushForward = 0.4179;//0.37;//0.346;

    private boolean rbToggle = false;
    public boolean PROTO_AlignSlides = false;

    private Caching_Motor leftSlide;
    private Caching_Motor rightSlide;

    public ElapsedTime mStateTime;
    ///private DistanceSensor sensor;
    public ExpansionHubMotor encoder;
    private RevBulkData data;

    private PIDFController slidesController;
    private boolean flickerToggle = false;
    private boolean shooterToggle = false;

    public double downPos;

    public int powerShotToggle = 0;
    private double shooterff = 0.4;
    private double ff = 0;
    public static double ffConstant = 0.095;
    private double shooterPower = 1.0;

    private boolean yToggle = false;
    public boolean automation = false;

    public double kp_Flywheel = 15;
    public double ki_Flywheel = 0;
    public double kd_Flywheel = 0.1;

    PIDFController controller = new PIDFController(new PIDCoefficients(kp_Flywheel, ki_Flywheel, kd_Flywheel));

    public enum ShootState{
        PREPARE,
        SHOOT,
        IDLE
    }
    public ShootState mRobotState = ShootState.IDLE;

    public Shooter(HardwareMap map, Telemetry telemetry){
        downPos = Math.toRadians(20.85);

        rightSlide = new Caching_Motor(map, "right_slide");
        leftSlide = new Caching_Motor(map, "left_slide");

        pushSlide = new Caching_Servo(map, "push_slide");
        stopper = new Caching_Servo(map, "stopper");
        flicker = new Caching_Servo(map, "flicker");

        shooter = new Caching_Motor(map, "shooter");
        mStateTime = new ElapsedTime();
        mStateTime.startTime();

        //sensor = map.get(DistanceSensor.class, "dist");
        encoder = (ExpansionHubMotor) map.get(DcMotor.class, "left_slide");

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slidesController = new PIDFController(new PIDCoefficients(kp_shooter, ki_shooter, kd_shooter));

        leftSlide.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlide.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

    public void setShooterVelocity(double target_velo, double current_velo){
        controller.setTargetPosition(target_velo);
        shooter.setPower(controller.update(current_velo));
        shooter.write();
    }

    double offset = 0;

    private double angle = 0;

    public double getShooterAngle(){
        //return Math.atan2(sensor.getDistance(DistanceUnit.INCH), 1.6693915023766982548289189220147);
        //return sensor.getDistance(DistanceUnit.MM);
        if(data != null){
            angle = (-(data.getMotorCurrentPosition(encoder)) * (2 * Math.PI)) / 8192.0;
            angle += Math.toRadians(20);
            angle %= 2 * Math.PI;
            return angle;
        }

        return angle;
    }

    public void write(){
       // rightSlide.write();
       // leftSlide.write();
        pushSlide.write();
        stopper.write();
        flicker.write();
        shooter.write();
    }

    public double calculateShooterAngle(double dist) {
        double myDist = Range.clip(dist - 7.25, 65, 120);
        double y = (2.5486 * Math.pow(10, -9)) * (Math.pow(myDist, 6));
        y -= 0.00000148331 * (Math.pow(myDist, 5));
        y += 0.000355865 * (Math.pow(myDist, 4));
        y -= 0.0450403 * (Math.pow(myDist, 3));
        y += 3.17236 * (Math.pow(myDist, 2));
        y -= 118.014 * myDist;
        y += 1841.77;

        return y;
    }

    public void setShooterAngle(double targetAngle, double currentAngle, double maxPower) {
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
        if(getShooterAngle() > Math.toRadians(20.5)){
            rightSlide.setPower(Range.clip(-power - ff, -1, 1));
            leftSlide.setPower(Range.clip(-power - ff, -1, 1));
        }else{
            rightSlide.setPower(Range.clip(-power, -1, 1));
            leftSlide.setPower(Range.clip(-power, -1, 1));
        }

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

    double theta = 125.9145;
    public double getPusherSlidePredictedPos(double time){
        if(theta >= 49.95){
            theta = 125.9145 - (time / 0.001837);
        }

        return 2 * Math.cos(Math.toRadians(theta)) * 7.7;
    }

    public void reset(){
        //double currentAngle = getShooterAngle();
        //setShooterAngle(downPos, currentAngle, 0.5);
        shooter.setPower(shooterff);
        stopper.setPosition(stopPosDown);
        flicker.setPosition(flickPosUp);
        pushSlide.setPosition(pushIdle);

        theta = 125.9145;
        rbToggle = false;
        yToggle = false;
        flickerToggle = false;

        mStateTime.reset();
    }

    public void resetAuto(){
        double currentAngle = getShooterAngle();

        setShooterAngle(downPos, currentAngle, 0.5);
        stopper.setPosition(stopPosDown);
        flicker.setPosition(flickPosUp);
        pushSlide.setPosition(pushIdle);

        theta = 125.9145;
        rbToggle = false;
        yToggle = false;
        flickerToggle = false;

        mStateTime.reset();
    }

    public void reset(double downPos){
        double currentAngle = getShooterAngle();
        setShooterAngle(Math.toRadians(downPos), currentAngle, 0.5);
        shooter.setPower(shooterff);
        stopper.setPosition(stopPosDown);
        flicker.setPosition(flickPosUp);
        pushSlide.setPosition(pushIdle);

        theta = 125.9145;
        rbToggle = false;
        yToggle = false;
        flickerToggle = false;

        mStateTime.reset();
    }

    public boolean powerShotAngle = false;
    private double resetTime = 0;
    public boolean kickOutEnabled = true;
    public boolean positionAutoAlign = false;

    public double getShooterVelocity(){
        if(data != null){
            return data.getMotorVelocity(shooter.motor);
        }else{
            return 0;
        }
    }

    public void operate(GamepadEx gamepad1, GamepadEx gamepad2, double distFromGoal, TelemetryPacket packet){
        double currentAngle = getShooterAngle();
        double shooterTargetAngle = calculateShooterAngle(distFromGoal);

        double velo = getShooterVelocity();

        telemetry.addData("Shooter Angle Required", shooterTargetAngle);
        telemetry.addData("State", mRobotState);
        if(data != null){
            packet.put("Raw Tick Shooter" , data.getMotorCurrentPosition(encoder));
        }

        telemetry.addData("Shooter Velocity: ", velo);

        if(gamepad1.isPress(GamepadEx.Control.left_trigger)){
            kickOutEnabled = true;
            mStateTime.reset();
            flicker.setPosition(flickPosDown);
            stopper.setPosition(stopPosUp);
            rbToggle = !rbToggle;

            slideSetPower(0.0);
            PROTO_AlignSlides = false;
            powerShotAngle = false;

            mRobotState = rbToggle ? ShootState.PREPARE : ShootState.IDLE;
        }

        if(gamepad1.isPress(GamepadEx.Control.a)){
            stopper.setPosition(stopPosUp);
        }

        if(gamepad2.isPress(GamepadEx.Control.a)){
            yToggle = !yToggle;
            if(!yToggle){
                shooter.setPower(0.0);
            }else{
                shooter.setPower(shooterPower);
            }
        }

        if(gamepad2.isPress(GamepadEx.Control.dpad_up)){
            PROTO_AlignSlides = true;
            powerShotAngle = false;
            reset = false;
            mStateTime.reset();
            //desiredAngle += 1;
        }

        if(gamepad1.isPress(GamepadEx.Control.dpad_right)){
            flickerToggle = !flickerToggle;

            if(!flickerToggle){
                flicker.setPosition(flickPosUp);
                shooter.setPower(shooterff);
            }

            PROTO_AlignSlides = true;
            powerShotAngle = true;
            reset = false;
            mStateTime.reset();
        }

        if (flickerToggle) {
            flicker.setPosition(flickPosDown);
            shooter.setPower(shooterPower);
        }

        if(gamepad2.isPress(GamepadEx.Control.dpad_down)){
            PROTO_AlignSlides = false;
            powerShotAngle = false;
            reset = true;
        }

        if(gamepad1.isPress(GamepadEx.Control.dpad_down)){
            PROTO_AlignSlides = false;
            powerShotAngle = false;
            reset = true;
        }

        if(gamepad1.isPress(GamepadEx.Control.right_bumper)){
            //PROTO_AlignSlides = false;
            //powerShotAngle = false;
            //reset = true;
        }

        if(gamepad2.isPress(GamepadEx.Control.start) || gamepad1.isPress(GamepadEx.Control.b)){
            powerShot(powerShotToggle);
            powerShotToggle += 1;
            powerShotToggle %= 4;
        }

        telemetry.addData("Current Angle", Math.toDegrees(currentAngle));

        if(gamepad1.isPress(GamepadEx.Control.right_trigger)){
            flickerToggle = !flickerToggle;
            if (flickerToggle) {
                flicker.setPosition(flickPosDown);
                shooter.setPower(shooterPower);
            }else{
                flicker.setPosition(flickPosUp);
                shooter.setPower(shooterff);
            }

            PROTO_AlignSlides = true;
            powerShotAngle = false;
            reset = false;
            mStateTime.reset();
        }

        if(gamepad2.isPress(GamepadEx.Control.b)){
            encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        /*if(flickerToggle || yToggle){
            setShooterVelocity(5.2, shooter.motor.getVelocity(AngleUnit.RADIANS));
        }*/

        if(PROTO_AlignSlides){
            reset = false;
            if(positionAutoAlign){
                if(Math.abs(currentAngle - (powerShotAngle ? Math.toRadians(25.36) : Math.toRadians(shooterTargetAngle))) < Math.toRadians(0.15) || Math.abs(gamepad2.gamepad.left_stick_y) >= 0.15){
                    slideSetPower(0.0);
                    PROTO_AlignSlides = false;
                    powerShotAngle = false;
                }else {
                    resetTime = mStateTime.time();
                    if (mStateTime.time() >= 0.15) {
                        //setShooterAngle(powerShotAngle ? Math.toRadians(25.36) : Math.toRadians(SlidesTunerAngle), currentAngle, 1.0);
                    } else {
                        if (getShooterAngle() <= Math.toRadians(20.5)) {
                            encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        }
                    }
                }
            }else{
                if(Math.abs(currentAngle - (powerShotAngle ? Math.toRadians(25.36) : Math.toRadians(shooterTargetAngle))) < Math.toRadians(0.15) || Math.abs(gamepad2.gamepad.left_stick_y) >= 0.15){
                    if(mStateTime.time() - resetTime >= 0.25 && kickOutEnabled){
                        slideSetPower(0.0);
                        PROTO_AlignSlides = false;
                        powerShotAngle = false;
                    }else{
                        setShooterAngle(powerShotAngle ? Math.toRadians(25.36) : Math.toRadians(shooterTargetAngle), currentAngle, 1.0);
                    }
                }else {
                    resetTime = mStateTime.time();
                    if (mStateTime.time() >= 0.15) {
                        setShooterAngle(powerShotAngle ? Math.toRadians(25.36) : Math.toRadians(shooterTargetAngle), currentAngle, 1.0);
                    } else {
                        if (getShooterAngle() <= Math.toRadians(20.5)) {
                            encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        }
                    }
                }
            }

        }else{
            if(reset){

                mRobotState = ShootState.IDLE;
                if(currentAngle <= downPos){
                    slideSetPower(-0.01);
                    if((mStateTime.time() - resetTime) >= 0.25){
                        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                        powerShotAngle = false;
                        reset = false;
                    }

                }else{
                    resetTime = mStateTime.time();
                    slideSetPower(-0.5);

                    shooter.setPower(shooterff);
                    stopper.setPosition(stopPosDown);
                    flicker.setPosition(flickPosUp);
                    pushSlide.setPosition(pushIdle);

                    theta = 125.9145;
                    rbToggle = false;
                    yToggle = false;
                    flickerToggle = false;

                    mStateTime.reset();
                }
            }else{
                if(!powerShotAngle){
                    rightSlide.write();
                    leftSlide.write();
                    slideSetPower(gamepad2.gamepad.left_stick_y * 0.5);
                }
            }
        }

        packet.put("Slide Angle", currentAngle);
        packet.put("ff", ff);

        switch (mRobotState){
            case PREPARE:
                if(Math.abs(velo) >= 2200 && mStateTime.time() >= 0.25){
                    mStateTime.reset();
                    mRobotState = ShootState.SHOOT;
                }

                //setShooterVelocity(5.2, velo);
                shooter.setPower(shooterPower);
                flicker.setPosition(flickPosDown);
                stopper.setPosition(stopPosUp);
                break;
            case SHOOT:
                if(mStateTime.time() <= 1.5){
                    //setShooterVelocity(5.2, velo);
                    shooter.setPower(shooterPower);
                    pushSlide.setPosition(pushForward);
                    //ff = ffConstant * getPusherSlidePredictedPos(mStateTime.time());
                    ff = ffConstant;
                } else {
                    reset();
                    mRobotState = ShootState.IDLE;
                }
                break;
            case IDLE:

                if (mStateTime.time() <= 0.25){
                    ff = 0;
                    pushSlide.setPosition(pushIdle);
                }

                break;
        }

    }
}