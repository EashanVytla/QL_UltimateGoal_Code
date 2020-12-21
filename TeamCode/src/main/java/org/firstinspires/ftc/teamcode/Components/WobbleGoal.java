package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

public class WobbleGoal {
    private double clamp_pos = 0.178;
    private double grabber_idle = 0.489;
    private double release_pos = 0.95;
    private double lift_pos = 0.97;
    private double drop_over_lift_pos = 0.36;
    private double middle_lift_pos = 0.66;
    private double down_pos = 0.07;
    private Caching_Servo servo_lift;
    private Caching_Servo servo_grab;
    private Telemetry telemetry;
    boolean grabberToggle = false;
    int grabberLiftToggle = 0;

    public WobbleGoal(HardwareMap map, Telemetry telemetry, double initPos){
        servo_lift = new Caching_Servo(map, "wobble_lift");
        servo_grab = new Caching_Servo(map, "wobble_grab");
        servo_grab.setPosition(release_pos);
        servo_lift.setPosition(lift_pos);
        servo_lift.write();
        servo_grab.write();
        this.telemetry = telemetry;
    }

    public void clamp(){
        servo_grab.setPosition(clamp_pos);
    }

    public void kick() {
        servo_grab.setPosition(release_pos);
    }

    public void release(){
        servo_grab.setPosition(grabber_idle);
    }

    public void write(){
        servo_grab.write();
        servo_lift.write();
    }

    public void lift(){
        servo_lift.setPosition(lift_pos);
    }

    public void midLift(){
        servo_lift.setPosition(middle_lift_pos);
    }

    public void down(){
        servo_lift.setPosition(down_pos);
    }

    public void DropOverWallLift(){
        servo_lift.setPosition(drop_over_lift_pos);
    }

    public void operate(GamepadEx gamepad){
        if(gamepad.isPress(GamepadEx.Control.a)){
            grabberToggle = !grabberToggle;

            if(grabberToggle){
                clamp();
            }else{
                kick();
            }
        }

        if(gamepad.isPress(GamepadEx.Control.x)){
            release();
        }

        if(gamepad.isPress(GamepadEx.Control.dpad_left)){
            if(grabberLiftToggle == 0){
                down();
            }else if(grabberLiftToggle == 1){
                midLift();
            }else if(grabberLiftToggle == 2){
                lift();
            }else if(grabberLiftToggle == 3){
                DropOverWallLift();
            }

            grabberLiftToggle = (grabberLiftToggle + 1) % 4;
        }
    }
}
