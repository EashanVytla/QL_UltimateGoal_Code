package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.network.PreferenceRemoter;

@Autonomous(name = "RefreshRateMeasure")
public class RefreshRateMeasure extends LinearOpMode {
    long prevTime = 0;

    long refreshRate = 0;

    public void runOpMode(){
        waitForStart();

        while (opModeIsActive()) {
            long dt = (System.currentTimeMillis() - prevTime);

            if(dt != 0){
                refreshRate = 1000/dt;
            }

            telemetry.addData("delta time: ", refreshRate);

            prevTime = System.currentTimeMillis();

            telemetry.update();
        }
    }
}
