package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.openftc.revextensions2.ExpansionHubEx;

@TeleOp
public class ShooterPrototype extends OpMode {
    ExpansionHubEx hub1;
    Caching_Servo servo;
    Caching_Motor shooterMotor;
    private boolean shooterOn = false;
    private boolean previousa = false;
    private boolean previousy = false;
    private boolean linkage = false;
    private double speed = 1;
    private boolean previousDpadUp = false;
    private boolean previousDpadDown = false;

    public void init() {
        hub1 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        servo = new Caching_Servo(hardwareMap, "linkage");
        shooterMotor = new Caching_Motor(hardwareMap, "flywheel");
        shooterMotor.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void loop() {
        if(gamepad1.a && !previousa){
            shooterOn = !shooterOn;
        }

        if(shooterOn){
            //shooterMotor.setPower(-speed);
            shooterMotor.motor.setVelocity(-2000);
        }else{
            shooterMotor.motor.setVelocity(0.0);
        }

        if(gamepad1.y && !previousy){
            linkage = !linkage;
        }

        if(linkage){
            servo.setPosition(0.45);
        }else{
            servo.setPosition(0.03);
        }

        if(gamepad1.dpad_up && !previousDpadUp){
            speed += 0.1;
        }

        if(gamepad1.dpad_down && !previousDpadDown){
            speed -= 0.1;
        }

        telemetry.addData("Servo: ", servo.getPosition());
        telemetry.addData("Speed: ", speed);
        //telemetry.addData("Velocity: ", shooterMotor.motor.getVelocity(AngleUnit.RADIANS));

        previousa = gamepad1.a;
        previousy = gamepad1.y;
        previousDpadUp = gamepad1.dpad_up;
        previousDpadDown = gamepad1.dpad_down;

        shooterMotor.write();
        servo.write();
    }
}
