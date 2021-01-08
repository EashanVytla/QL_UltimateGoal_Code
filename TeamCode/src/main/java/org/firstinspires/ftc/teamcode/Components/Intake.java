package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Motor;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

public class Intake {
    private Caching_Motor intake;
    private boolean intakeToggle = false;
    private  boolean prevRB = false;


    public Intake(HardwareMap map){
        intake = new Caching_Motor(map, "intake");
    }

    public void setPower(double power){
        intake.setPower(power);
    }

    public void write(){
        intake.write();
    }

    public void operate(GamepadEx gamepad1, GamepadEx gamepad2, Telemetry telemetry){
        //a = start intake
        //y = outake
        //if y let go then whatever a command is doing


        if(gamepad1.gamepad.right_bumper && !prevRB){
            intakeToggle = !intakeToggle;
        }

        if(gamepad1.gamepad.left_bumper){
            intake.setPower(1.0);
        }else if(intakeToggle){
            intake.setPower(-1.0);
        }else{
            intake.setPower(0.0);
        }

        prevRB = gamepad1.gamepad.right_bumper;
    }
}
