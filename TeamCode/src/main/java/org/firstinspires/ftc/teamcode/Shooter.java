package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
class Shooter {
    private Caching_Servo slide;
    private Telemetry telemetry;
    private final double AUTO_LIFT_HEIGHT = 0.2;
    private final double RESTING_POS = 0.01;
    private double slidePos = 0;

    public Shooter(HardwareMap map, Telemetry telemetry){
        slide = new Caching_Servo(map, "vertical_slide");
        this.telemetry = telemetry;
    }

    public void lift_auto(){
        slide.setPosition(AUTO_LIFT_HEIGHT);
    }

    public void drop(){
        slide.setPosition(RESTING_POS);
    }

    public void setSlidePos(double pos){
        slide.setPosition(pos);
    }

    public void write(){
        slide.write();
    }

    public void operate(Gamepad gamepad1, Gamepad gamepad2){
        setSlidePos(slidePos);
        if(gamepad2.left_trigger > 0.3){
            slidePos += 0.001;
        }
    }
}
