package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Point;

@Config
public class VisionConstants {
    public static Point REGION_TOPLEFT_ANCHOR_POINT = new Point(110, 75);

    public static int REGION_WIDTH = 90;
    public static int REGION_HEIGHT = 60;

    public static int FOUR_RING_THRESHOLD = 80;
    public static int ONE_RING_THRESHOLD = 94;
}
