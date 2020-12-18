package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Robot;

@TeleOp(name = "S4T Localizer Tester")
public class S4T_Localizer_Tester extends OpMode {
    Robot robot;

    public void init() {
        robot = new Robot(hardwareMap, telemetry);
    }

    public void loop(){
        robot.updateBulkData();

        robot.updatePos();

        robot.drive.drive(gamepad1, 1.0, 1.0);

        telemetry.addData("Pos: ", robot.getPos());
        robot.drive.write();
    }
}
