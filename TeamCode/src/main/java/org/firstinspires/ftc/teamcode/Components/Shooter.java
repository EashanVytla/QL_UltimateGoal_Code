package org.firstinspires.ftc.teamcode.Components;

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
    public static double kp_shooter = 8.5;
    public static double ki_shooter = 0;
    public static double kd_shooter = 0.1;

    public static double SlidesTunerAngle = 26.2;

    public Caching_Servo pushSlide;
    public Caching_Servo stopper;
    public Caching_Servo flicker;

    public Caching_Motor shooter;
    private Telemetry telemetry;

    public final double flickPosDown = 0.1919;//0.22;//.07
    public final double flickPosUp = 0.0;

    public  double stopPosUp = 0.0;
    public  double stopPosDown = 0.315;

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
    private double ff = 0.08;
    public static double ffConstant = 0.0015;

    private boolean yToggle = false;
    public boolean automation = false;

    public enum ShootState{
        PREPARE,
        SHOOT,
        IDLE
    }
    public ShootState mRobotState = ShootState.IDLE;

    public Shooter(HardwareMap map, Telemetry telemetry){
        downPos = Math.toRadians(20.5);

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
        angle += Math.toRadians(20);
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
        if(getShooterAngle() > Math.toRadians(20.5)){
            rightSlide.setPower(Range.clip(power + ff, -1, 1));
            leftSlide.setPower(Range.clip(power + ff, -1, 1));
        }else{
            rightSlide.setPower(Range.clip(power, -1, 1));
            leftSlide.setPower(Range.clip(power, -1, 1));
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
        double currentAngle = getShooterAngle();
        setShooterAngle(downPos, currentAngle, 0.5);
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
    public boolean powerShots;
    private double resetTime = 0;

    public void operate(GamepadEx gamepad1, GamepadEx gamepad2, double distFromGoal, TelemetryPacket packet){
        double currentAngle = getShooterAngle();
        double shooterTargetAngle = calculateShooterAngle(distFromGoal);

        telemetry.addData("Shooter Angle Required", shooterTargetAngle);
        telemetry.addData("State", mRobotState);
        packet.put("Raw Tick Shooter" , data.getMotorCurrentPosition(encoder));
        telemetry.addData("Raw Tick Shooter" , data.getMotorCurrentPosition(encoder));

        if(gamepad1.isPress(GamepadEx.Control.left_trigger)){
            mStateTime.reset();
            flicker.setPosition(flickPosDown);
            stopper.setPosition(stopPosUp);
            rbToggle = !rbToggle;
            mRobotState = rbToggle ? ShootState.PREPARE : ShootState.IDLE;
        }

        telemetry.addData("RB Toggle", rbToggle);
        telemetry.addData("Flicker Toggle", flickerToggle);
        telemetry.addData("Y Toggle", yToggle);

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
            powerShots = false;
            reset = false;
            mStateTime.reset();
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

        if(gamepad1.isPress(GamepadEx.Control.dpad_down)){
            PROTO_AlignSlides = false;
            powerShotAngle = false;
            reset = true;
        }

        if(gamepad1.isPress(GamepadEx.Control.right_bumper)){
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

        if(gamepad1.isPress(GamepadEx.Control.right_trigger)){
            flickerToggle = !flickerToggle;
            if (flickerToggle) {
                flicker.setPosition(flickPosDown);
                shooter.setPower(1.0);
            }else{
                flicker.setPosition(flickPosUp);
                shooter.setPower(shooterff);
            }

            PROTO_AlignSlides = true;
            powerShots = false;
            reset = false;
            mStateTime.reset();
        }

        if(gamepad2.isPress(GamepadEx.Control.b)){
            encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        telemetry.addData("Power Shot", powerShots);

        if(PROTO_AlignSlides){
            reset = false;
            if(Math.abs(currentAngle - (powerShotAngle ? Math.toRadians(23.86) : Math.toRadians(SlidesTunerAngle))) < Math.toRadians(0.3) || Math.abs(gamepad2.gamepad.left_stick_y) >= 0.15){
                if(mStateTime.time() - resetTime >= 0.5){
                    PROTO_AlignSlides = false;
                    powerShotAngle = false;
                }else{
                    setShooterAngle(powerShotAngle ? Math.toRadians(23.86) : Math.toRadians(SlidesTunerAngle), currentAngle, 1.0);
                }
            }else{
                resetTime = mStateTime.time();
                //setShooterAngle(powerShotAngle ? Math.toRadians(23.86) : Math.toRadians(shooterTargetAngle), currentAngle, 1.0);
                if(mStateTime.time() >= 0.15){
                    setShooterAngle(powerShotAngle ? Math.toRadians(23.86) : Math.toRadians(SlidesTunerAngle), currentAngle, 1.0);
                }else{
                    if(getShooterAngle() <= Math.toRadians(22)){
                        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    }
                }
            }
        }else{
            //if(Math.abs(getShooterAngle() - Math.toRadians(desiredAngle)) >= Math.toRadians(0.1) && gamepad2.gamepad.atRest()){
            //    setShooterAngle(Math.toRadians(desiredAngle), getShooterAngle());
            //}else{

            //}

            if(reset){

                mRobotState = ShootState.IDLE;
                if(currentAngle <= downPos){
                    slideSetPower(0.0);
                    if((mStateTime.time() - resetTime) >= 0.25){
                        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        powerShotAngle = false;
                        reset = false;
                    }
                }else{
                    resetTime = mStateTime.time();
                    reset();
                }
            }else{
                if(!powerShots){
                    slideSetPower(gamepad2.gamepad.left_stick_y * 0.25);
                }
            }
        }


        packet.put("Slide Angle", currentAngle);
        packet.put("ff", ff);

        switch (mRobotState){
            case PREPARE:
                double velo = shooter.motor.getVelocity(AngleUnit.RADIANS);
                telemetry.addData("Motor Velocity: ", velo);
                if(Math.abs(velo) >= 4.5 && mStateTime.time() >= 0.25){
                    mStateTime.reset();
                    mRobotState = ShootState.SHOOT;
                }

                shooter.setPower(1);
                flicker.setPosition(flickPosDown);
                stopper.setPosition(stopPosUp);
                break;
            case SHOOT:
                if(mStateTime.time() <= 1.5){
                    shooter.setPower(1);
                    pushSlide.setPosition(pushForward);
                    ff = 0.08 + ffConstant * getPusherSlidePredictedPos(mStateTime.time());
                } else {
                    reset();
                    mRobotState = ShootState.IDLE;
                }
                break;
            case IDLE:
                if (mStateTime.time() <= 0.25){
                    ff = 0.08;
                    pushSlide.setPosition(pushIdle);
                }

                break;
        }
    }
}
