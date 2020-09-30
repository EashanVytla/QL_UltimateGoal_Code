package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "S4T Localizer Tester")
public class S4T_Localizer_Tester extends OpMode {
    Robot robot;

    public void init() {
        robot = Robot.getInstance(hardwareMap, telemetry);
    }

    @Override
    public void stop() {
        robot.stop();
    }

    public void loop(){
        robot.updatePos();

        telemetry.addData("Pos: ", robot.localizer.getPose());
    }
}
