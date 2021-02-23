package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Motor;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;
import org.openftc.revextensions2.RevBulkData;

@Config
public class Shooter {
    public static double kp_shooter = 0.01;
    public static double ki_shooter = 0.0;
    public static double kd_shooter = 0.0;

    public static double SlidesTunerAngle = 26.4;

    public Caching_Servo pushSlide;
    public Caching_Servo stopper;
    public Caching_Servo flicker;

    public Caching_Motor shooter;
    private Telemetry telemetry;

    public final double flickPosDown = 0.19;//0.22;//.07
    public final double flickPosUp = 0.06;

    public  double stopPosUp = 0.626;
    public  double stopPosDown = 0.922;

    public final double pushIdle = 0.999;
    public final double pushForward = 0.31;//0.346;

    private boolean rbToggle = false;
    public boolean PROTO_AlignSlides;

    public Caching_Motor leftSlide;
    public Caching_Motor rightSlide;

    public ElapsedTime mStateTime;
    private RevBulkData data;
    private RevBulkData data2;

    private PIDFController slidesController;
    private boolean flickerToggle = false;

    public double downPos;

    public int powerShotToggle = 0;
    private double shooterff = 0.4;
    private double ff = 0;
    public static double ffConstant = 0.095;
    private double shooterPower = 1.0;

    private boolean yToggle = false;
    public boolean automation = false;

    public double kp_Flywheel = 300;
    public double ki_Flywheel = 25;
    public double kd_Flywheel = 50;
    private AnalogInput gyro;

    public enum ShootState{
        PREPARE,
        SHOOT,
        IDLE
    }
    public ShootState mRobotState = ShootState.IDLE;

    public Shooter(HardwareMap map, Telemetry telemetry){
        PROTO_AlignSlides = false;
        downPos = Math.toRadians(20.85);

        rightSlide = new Caching_Motor(map, "right_slide");
        leftSlide = new Caching_Motor(map, "left_slide");

        pushSlide = new Caching_Servo(map, "push_slide");
        stopper = new Caching_Servo(map, "stopper");
        flicker = new Caching_Servo(map, "flicker");

        shooter = new Caching_Motor(map, "shooter");
        mStateTime = new ElapsedTime();
        mStateTime.startTime();

        shooter.motor.setVelocityPIDFCoefficients(kp_Flywheel, ki_Flywheel, kd_Flywheel, 0.0);

        slidesController = new PIDFController(new PIDCoefficients(kp_shooter, ki_shooter, kd_shooter));

        leftSlide.motor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightSlide.motor.setMode(DcMotor.RunMode.RESET_ENCODERS);

        leftSlide.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlide.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftSlide.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gyro = map.get(AnalogInput.class, "sangyro");

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

    public void setShooterVelocity(double target_velo){
        shooter.motor.setVelocity(target_velo);
    }

    double offset = 0;

    private double angle = 0;

    public double getShooterAngle(){
        //return Math.atan2(sensor.getDistance(DistanceUnit.INCH), 1.6693915023766982548289189220147);
        //return sensor.getDistance(DistanceUnit.MM);
        /*if(data != null){
            angle = (-(data.getMotorCurrentPosition(encoder)) * (2 * Math.PI)) / 8192.0;
            angle += Math.toRadians(20);
            angle %= 2 * Math.PI;
            return angle;
        }*/

        if(data != null){
            return data.getMotorCurrentPosition(leftSlide.motor)/* + QL_Auto_Linear.SLIDE_POS_END*/;
        }else{
            return leftSlide.motor.getCurrentPosition();
        }
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
        double myDist = Range.clip(dist - 7.25, 65, 100);
        double y = 0.0871429 * Math.pow(myDist, 2);
        y -= 20.4548 * myDist;
        y += 1973.71;



        return y + 10;
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

    public void setData(RevBulkData data, RevBulkData data2){
        this.data = data;
        this.data2 = data2;
    }

    public void slideSetPower(double power){
        rightSlide.setPower(Range.clip(-power - ff, -1, 1));
        leftSlide.setPower(Range.clip(-power - ff, -1, 1));

        telemetry.addData("Slide Power", power);
    }

    double desiredAngle = 25;
    public boolean reset = false;

    public void powerShot(ElapsedTime time){
        int counter = 0;

        if(time.time() >= 3){
            counter = 4;
        }else if(time.time() >= 2){
            counter = 3;
        }else if(time.time() >= 1){
            counter = 2;
        }else{
            counter = 1;
        }

        powerShot(counter);
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
        setShooterAngle(10, currentAngle, 0.5);
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
            return shooter.motor.getVelocity();
        }
    }

    public void operate(GamepadEx gamepad1, GamepadEx gamepad2, double distFromGoal, TelemetryPacket packet){
        double currentAngle = getShooterAngle();
        double shooterTargetAngle = calculateShooterAngle(distFromGoal);

        double velo = getShooterVelocity();

        telemetry.addData("Shooter Angle Required", shooterTargetAngle);
        telemetry.addData("State", mRobotState);

        packet.put("Shooter Velocity: ", velo);

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
            }else{
                flicker.setPosition(flickPosDown);
                shooter.setPower(shooterPower);
            }

            PROTO_AlignSlides = true;
            powerShotAngle = true;
            reset = false;
            mStateTime.reset();
        }

        if(gamepad2.isPress(GamepadEx.Control.start) || gamepad1.isPress(GamepadEx.Control.b)){
            powerShot(powerShotToggle);
            powerShotToggle += 1;
            powerShotToggle %= 4;
        }

        telemetry.addData("Current Angle", currentAngle);

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

        if(gamepad1.isPress(GamepadEx.Control.right_bumper)){
            flickerToggle = false;
            flicker.setPosition(flickPosUp);
        }


        if(flickerToggle || yToggle){
            shooter.setPower(1.0);
        }

        if(PROTO_AlignSlides){
            reset = false;
            if(positionAutoAlign){
                setShooterAngle(powerShotAngle ? (25.36) : SlidesTunerAngle, currentAngle, 1.0);
            }else{
                setShooterAngle(powerShotAngle ? (25.36) : (shooterTargetAngle), currentAngle, 1.0);
            }
        }else{
            if(reset){

                mRobotState = ShootState.IDLE;
                if(currentAngle <= downPos){
                    slideSetPower(-0.01);
                    if((mStateTime.time() - resetTime) >= 0.25){
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
                PROTO_AlignSlides = true;

                if (mStateTime.time() <= 0.25){
                    ff = 0;
                    pushSlide.setPosition(pushIdle);
                }

                break;
        }
    }
}
