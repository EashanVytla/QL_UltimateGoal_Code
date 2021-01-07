package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

@Autonomous
public class RevEncoderTester extends LinearOpMode {
    RevBulkData data;
    ExpansionHubEx hub;
    ExpansionHubMotor encoder;

    @Override
    public void runOpMode() throws InterruptedException {
        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        encoder = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "intake");
        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while(opModeIsActive()){
            data = hub.getBulkInputData();
            double angle = (data.getMotorCurrentPosition(encoder) * (2 * Math.PI)) / 8192.0;
            angle += Math.toRadians(20);
            angle %= 2 * Math.PI;
            telemetry.addData("value", Math.toDegrees(angle));
            telemetry.update();
        }
    }
}
