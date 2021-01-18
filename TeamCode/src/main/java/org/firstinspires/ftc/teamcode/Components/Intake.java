package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Motor;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

public class Intake {
    private Caching_Motor intake;
    private boolean intakeToggle = false;
    private ElapsedTime timer = new ElapsedTime();

    public Intake(HardwareMap map){
        intake = new Caching_Motor(map, "intake");
        intake.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        timer.startTime();
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


        if(gamepad1.isPress(GamepadEx.Control.right_bumper)){
            intakeToggle = !intakeToggle;
        }

        if(gamepad1.isPress(GamepadEx.Control.right_trigger)){
            if(timer.time() >= 0.3){
                intakeToggle = false;
            }
        }else{
            timer.reset();
        }

        if(gamepad1.gamepad.left_bumper){
            intake.setPower(1.0);
        }else if(intakeToggle){
            intake.setPower(-1.0);
        }else{
            intake.setPower(0.0);
        }
    }
}
