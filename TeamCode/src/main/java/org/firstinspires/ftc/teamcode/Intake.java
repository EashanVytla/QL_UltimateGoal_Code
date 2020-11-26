package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

class Intake {
    private Caching_Motor intake;
    private boolean intakeToggle = false;
    private  boolean previousA = false;

    public Intake(HardwareMap map){
        intake = new Caching_Motor(map, "intake");
    }

    public void setPower(double power){
        intake.setPower(power);
    }

    public void write(){
        intake.write();
    }

    public void operate(Gamepad gamepad1){
        //a = start intake
        //y = outake
        //if y let go then whatever a command is doing


        if(gamepad1.a && !previousA){
            intakeToggle = !intakeToggle;
        }

        if(gamepad1.y){
            intake.setPower(-1.0);
        }else if(intakeToggle){
            intake.setPower(1.0);
        }else{
            intake.setPower(0.0);
        }

        previousA = gamepad1.a;
    }
}
