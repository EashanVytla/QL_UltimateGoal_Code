package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Motor;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

public class Intake {
    private Caching_Motor intake;
    private boolean intakeToggle = false;
    private ElapsedTime timer = new ElapsedTime();
    private Caching_Servo intake_holder;
    private double dropPos = 0.6;
    private double upPos = 0.9;
    private boolean dropToggle = false;

    public Intake(HardwareMap map){
        intake = new Caching_Motor(map, "intake");
        intake_holder = new Caching_Servo(map, "intake_holder");
        intake.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        timer.startTime();
    }

    public void setPower(double power){
        intake.setPower(power);
    }

    public void dropIntake(){
        intake_holder.setPosition(dropPos);
    }

    public void closeIntake(){
        intake_holder.setPosition(upPos);
    }

    public void write(){
        intake.write();
        intake_holder.write();
    }

    public void operate(GamepadEx gamepad1, GamepadEx gamepad2, Telemetry telemetry){
        //a = start intake
        //y = outake
        //if y let go then whatever a command is doing


        if(gamepad1.isPress(GamepadEx.Control.right_bumper)){
            intakeToggle = !intakeToggle;
        }

        if(gamepad1.isPress(GamepadEx.Control.left_trigger)){
            intakeToggle = false;
        }

        if(gamepad2.isPress(GamepadEx.Control.x)){
            dropToggle = !dropToggle;

            intake_holder.setPosition(dropToggle ? dropPos : upPos);
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
