package org.firstinspires.ftc.teamcode.PurePusuit;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.Math.Vector2;

public class CurvePoint {
    public double x;
    public double y;
    public double moveSpeed;
    public double turnSpeed;
    public double followDistance;
    public double heading;

    public CurvePoint(double x, double y, double moveSpeed, double turnSpeed, double followDistance, double heading){
        this.x = x;
        this.y = y;
        this.moveSpeed = moveSpeed;
        this.turnSpeed = turnSpeed;
        this.followDistance = followDistance;
        this.heading = heading;
    }

    public CurvePoint(CurvePoint thisPoint){
        this.x = thisPoint.x;
        this.y = thisPoint.y;
        this.moveSpeed = thisPoint.moveSpeed;
        this.turnSpeed = thisPoint.turnSpeed;
        this.followDistance = thisPoint.followDistance;
        this.heading = thisPoint.heading;
    }

    public Vector2d toVec(){
        return new Vector2d(x, y);
    }

    public void setPoint(Vector2d Point) {
        this.x = Point.getX();
        this.y = Point.getY();
    }
}