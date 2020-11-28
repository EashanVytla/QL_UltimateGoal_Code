package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Shooter {
    private Caching_Servo heightSlide;
    private Caching_Servo pushSlide;
    private Telemetry telemetry;
    private final double AUTO_LIFT_HEIGHT = 0.2;
    private final double RESTING_POS = 0.01;
    private double slidePos = 0.12;

    public Shooter(HardwareMap map, Telemetry telemetry){
        //heightSlide = new Caching_Servo(map, "vertical_slide");
        pushSlide = new Caching_Servo(map, "push_slide");
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
            pushSlide.setPosition(0.46);
        }else{
            pushSlide.setPosition(0.92);
        }

        telemetry.addData("slide pos: ", slidePos);
        telemetry.addData("servo pos: ", pushSlide.getPosition());
    }
}
