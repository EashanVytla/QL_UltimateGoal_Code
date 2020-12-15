package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Wrapper.Caching_Motor;

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

    public void operate(Gamepad gamepad1, Gamepad gamepad2){
        //a = start intake
        //y = outake
        //if y let go then whatever a command is doing


        if(gamepad1.right_bumper && !prevRB){
            intakeToggle = !intakeToggle;
        }

        if(gamepad1.left_bumper){
            intake.setPower(1.0);
        }else if(intakeToggle){
            intake.setPower(-1.0);
        }else{
            intake.setPower(0.0);
        }

        prevRB = gamepad1.right_bumper;

        //intake.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
    }
}
