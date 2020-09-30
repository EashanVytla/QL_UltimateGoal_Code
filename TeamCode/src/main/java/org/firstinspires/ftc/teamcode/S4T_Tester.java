package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

@TeleOp(name = "S4T_Tester")
public class S4T_Tester extends OpMode {
    S4T_Encoder encoderLY;
    S4T_Encoder encoderLX;
    S4T_Encoder encoderRY;
    S4T_Encoder encoderRX;
    RevBulkData data;
    ExpansionHubEx expansionHub1;
    S4T_Localizer localizer;
    boolean preva;

    public void init(){
        expansionHub1 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        encoderLY = new S4T_Encoder(hardwareMap, "back_left");
        encoderLX = new S4T_Encoder(hardwareMap, "front_left");
        encoderRY = new S4T_Encoder(hardwareMap, "front_right");
        encoderRX = new S4T_Encoder(hardwareMap, "back_right");

        encoderRY.reverse = true;
        encoderRX.reverse = true;
        localizer = new S4T_Localizer(telemetry);
    }

    double mydecx;

    public void loop(){
        data = expansionHub1.getBulkInputData();
        encoderLX.update(data);
        encoderLY.update(data);
        encoderRX.update(data);
        encoderRY.update(data);
        localizer.update(encoderLX.getDist() * 0.6, encoderLY.getDist(), encoderRX.getDist() * 0.6, encoderRY.getDist());


        telemetry.addData("TUNER DECX: ", mydecx);

        telemetry.addData("Left X: ", encoderLX.getDist() * 0.6);
        telemetry.addData("Left Y: ", encoderLY.getDist());
        telemetry.addData("Right X: ", encoderRX.getDist() * 0.6);
        telemetry.addData("Right Y: ", encoderRY.getDist());
        telemetry.addData("Pose: ", localizer.getPose());
    }
}
