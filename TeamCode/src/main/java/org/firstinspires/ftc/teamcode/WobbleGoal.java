package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
class WobbleGoal {
    private double clamp_pos = 0.5;
    private double release_pos = 0.5;
    Caching_Servo servo;
    private Telemetry telemetry;

    public WobbleGoal(HardwareMap map, Telemetry telemetry){
        servo = new Caching_Servo(map, "wobble");
        servo.setPosition(0.5);
        servo.write();
        this.telemetry = telemetry;
    }

    public void clamp(){
        servo.setPosition(clamp_pos);
        servo.write();
    }

    public void release(){
        servo.setPosition(release_pos);
        servo.write();
    }
}
