package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class WobbleGoal {
    private double clamp_pos = 0.5;
    private double release_pos = 0.5;
    private Caching_Servo servo;
    private Telemetry telemetry;
    boolean previousB = false;
    boolean grabberToggle = false;

    public WobbleGoal(HardwareMap map, Telemetry telemetry, double initPos){
        servo = new Caching_Servo(map, "wobble");
        servo.setPosition(initPos);
        servo.write();
        this.telemetry = telemetry;
    }

    public void clamp(){
        servo.setPosition(clamp_pos);
    }

    public void write(){
        servo.write();
    }

    public void release(){
        servo.setPosition(release_pos);
    }

    public void operate(Gamepad gamepad){
        if(gamepad.b && !previousB){
            grabberToggle = !grabberToggle;

            if(grabberToggle){
                clamp();
            }else{
                release();
            }
        }

        previousB = gamepad.b;
    }
}
