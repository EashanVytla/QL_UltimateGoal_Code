package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
class Shooter {
    private Caching_Servo slide;
    private Telemetry telemetry;
    private final double AUTO_LIFT_HEIGHT = 0.2;
    private final double RESTING_POS = 0.01;

    public Shooter(HardwareMap map, Telemetry telemetry){
        slide = new Caching_Servo(map, "vertical_slide");
        this.telemetry = telemetry;
    }

    public void lift_auto(){
        slide.setPosition(AUTO_LIFT_HEIGHT);
        slide.write();
    }

    public void drop(){
        slide.setPosition(RESTING_POS);
        slide.write();
    }

    public void setSlidePos(double pos){
        slide.setPosition(pos);
        slide.write();
    }
}
