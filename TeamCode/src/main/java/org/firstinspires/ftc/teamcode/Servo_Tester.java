package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Servo_Tester extends OpMode {
    Caching_Servo Servo;
    String name = "flicker";
    double pos = 0;

    @Override
    public void init() {
        Servo = new Caching_Servo(hardwareMap, name);
    }

    public void loop(){
        if(gamepad1.dpad_up){
            Servo.setPosition(pos += .001);
        }

        if(gamepad1.dpad_down){
            Servo.setPosition(pos -= .001);
        }

        telemetry.addData("Position", pos);

        Servo.write();
    }
}
