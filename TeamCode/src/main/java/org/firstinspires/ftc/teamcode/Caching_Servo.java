package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Caching_Servo {
    Servo servo;
    private double prev_pos = 0.0;

    private double query = -2.0;

    private double EPSILON = 0.001;

    public Caching_Servo(HardwareMap map, String name){
        servo = map.servo.get(name);
    }

    public void setPosition(double pos){
        if (Math.abs(pos - prev_pos) > EPSILON){
            query = pos;
        }
        else{
            query = -1.0;
        }
    }

    public double getPosition(){
        return prev_pos;
    }

    public void write(){
        if (query != -1.0) {
            servo.setPosition(query);
            prev_pos = query;
        }
    }
}