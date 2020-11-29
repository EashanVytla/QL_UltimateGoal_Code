package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Shooter {
    private Caching_Servo heightSlide;
    private Caching_Servo pushSlide;
    private Caching_Servo stopper;
    private Caching_Servo flicker;
    //private Caching_Motor shooter;
    private DcMotorEx shooter;
    private Telemetry telemetry;
    private final double AUTO_LIFT_HEIGHT = 0.2;
    private final double RESTING_POS = 0.01;
    private double slidePos = 0.12;
    private final double flickPosDown = 0.0;
    private final double flickPosUp = 0.2;
    private final double stopPosUp = 0.6;
    private final double stopPosDown = 1;
    private boolean first = true;
    private ElapsedTime mStateTime;
    private boolean previousLB = false;
    private boolean previousA = false;
    public boolean aToggle = false;
    private boolean rbToggle = false;
    public final double pushIdle = 0.90;
    public final double pushForward = 0.29;

    private enum ShootState{
        PREPARE,
        SHOOT,
        IDLE
    }
    ShootState mRobotState = ShootState.IDLE;

    public Shooter(HardwareMap map, Telemetry telemetry){
        //heightSlide = new Caching_Servo(map, "vertical_slide");
        pushSlide = new Caching_Servo(map, "push_slide");
        stopper = new Caching_Servo(map, "stopper");
        flicker = new Caching_Servo(map, "flicker");
        //shooter = new Caching_Motor(map, "shooter");
        shooter = map.get(DcMotorEx.class, "shooter");
        mStateTime = new ElapsedTime();
        mStateTime.startTime();
        flicker.setPosition(flickPosDown);
        stopper.setPosition(stopPosDown);
        pushSlide.setPosition(pushIdle);

        this.telemetry = telemetry;
    }

    public void lift_auto(){
        heightSlide.setPosition(AUTO_LIFT_HEIGHT);
    }

    public void drop(){
        heightSlide.setPosition(RESTING_POS);
    }

    public void setHeightSlidePos(double pos){
        heightSlide.setPosition(pos);
    }

    public void write(){
        //heightSlide.write();
        pushSlide.write();
        stopper.write();
        flicker.write();
        //shooter.write();
    }

    public void operate(Gamepad gamepad1, Gamepad gamepad2){
        /*setHeightSlidePos(slidePos);
        if(gamepad2.left_trigger > 0.3){
            slidePos += 0.001;
        }*/

        /*if(gamepad1.dpad_up){
            slidePos+=.001;
        }else if(gamepad1.dpad_down){
            slidePos-=.001;
        }*/

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

        if(gamepad1.right_bumper || gamepad2.a){
            shooter.setPower(-1.0);
        } else {
            shooter.setPower(0);
        }

        previousA = gamepad1.a;
        previousLB = gamepad1.left_bumper;


        switch (mRobotState){
            case PREPARE:
                if(mStateTime.time() <= 4){
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
                    mRobotState = ShootState.IDLE;
                    mStateTime.reset();
                }
                break;
            case IDLE:
                if (mStateTime.time() <= 1){
                    pushSlide.setPosition(pushIdle);
                    shooter.setPower(0);
                } else {
                    stopper.setPosition(stopPosDown);
                    flicker.setPosition(flickPosUp);
                }
                break;
        }

        telemetry.addData("Velocity: ", shooter.getVelocity(AngleUnit.RADIANS));
        telemetry.addData("slide pos: ", slidePos);
        telemetry.addData("servo pos: ", pushSlide.getPosition());
    }
}
