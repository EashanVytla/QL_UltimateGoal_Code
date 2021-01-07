package org.firstinspires.ftc.teamcode.Vision;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.firstinspires.ftc.teamcode.Vision.VisionConstants.FOUR_RING_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Vision.VisionConstants.ONE_RING_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Vision.VisionConstants.REGION_HEIGHT;
import static org.firstinspires.ftc.teamcode.Vision.VisionConstants.REGION_TOPLEFT_ANCHOR_POINT;
import static org.firstinspires.ftc.teamcode.Vision.VisionConstants.REGION_WIDTH;

public class RingDetectionPipeline extends OpenCvPipeline{
    OpenCvCamera webcam;

    public boolean viewportPaused;



    Point upper_region_pointA = new Point(
            REGION_TOPLEFT_ANCHOR_POINT.x,
            REGION_TOPLEFT_ANCHOR_POINT.y);
    Point upper_region_pointB = new Point(
            REGION_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION_TOPLEFT_ANCHOR_POINT.y + (REGION_HEIGHT * (2.0/3.0)));

    Point lower_region_pointA = new Point(
            REGION_TOPLEFT_ANCHOR_POINT.x,
            REGION_TOPLEFT_ANCHOR_POINT.y + (REGION_HEIGHT * (2.0/3.0)));
    Point lower_region_pointB = new Point(
            REGION_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    Mat upper_region_Cb;
    Mat lower_region_Cb;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    int avgUpper;
    int avgLower;
    private Bitmap image;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    private volatile int ringCase = 0;

    /*
     * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
     * highly recommended to declare them here as instance variables and re-use them for
     * each invocation of processFrame(), rather than declaring them as new local variables
     * each time through processFrame(). This removes the danger of causing a memory leak
     * by forgetting to call mat.release(), and it also reduces memory pressure by not
     * constantly allocating and freeing large chunks of memory.
     */

    private void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 2);
    }

    @Override
    public void init(Mat firstFrame)
    {
        //Converts the first frame to YCrCb and extracts to Cb channel
        inputToCb(firstFrame);

        //Creating the region box using submat
        upper_region_Cb = YCrCb.submat(new Rect(upper_region_pointA, upper_region_pointB));
        lower_region_Cb = YCrCb.submat(new Rect(lower_region_pointA, lower_region_pointB));
    }

    @Override
    public Mat processFrame(Mat input)
    {
        /*
         * Overview of what we're doing:
         *
         * We first convert to YCrCb color space, from RGB color space.
         * Why do we do this? Well, in the RGB color space, chroma and
         * luma are intertwined. In YCrCb, chroma and luma are separated.
         * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
         * are Y, the luma channel (which essentially just a B&W image), the
         * Cr channel, which records the difference from red, and the Cb channel,
         * which records the difference from blue. Because chroma and luma are
         * not related in YCrCb, vision code written to look for certain values
         * in the Cr/Cb channels will not be severely affected by differing
         * light intensity, since that difference would most likely just be
         * reflected in the Y channel.
         *
         * After we've converted to YCrCb, we extract just the 2nd channel, the
         * Cb channel. We do this because stones are bright yellow and contrast
         * STRONGLY on the Cb channel against everything else, including SkyStones
         * (because SkyStones have a black label).
         *
         * We then take the average pixel value of 3 different regions on that Cb
         * channel, one positioned over each stone. The brightest of the 3 regions
         * is where we assume the SkyStone to be, since the normal stones show up
         * extremely darkly.
         *
         * We also draw rectangles on the screen showing where the sample regions
         * are, as well as drawing a solid rectangle over top the sample region
         * we believe is on top of the SkyStone.
         *
         * In order for this whole process to work correctly, each sample region
         * should be positioned in the center of each of the first 3 stones, and
         * be small enough such that only the stone is sampled, and not any of the
         * surroundings.
         */

        //Converting the input matrix to Cb channel of YCrCb color space
        inputToCb(input);

        /*
         * Compute the average pixel value of each submat region. We're
         * taking the average of a single channel buffer, so the value
         * we need is at index 0. We could have also taken the average
         * pixel value of the 3-channel image, and referenced the value
         * at index 2 here.
         */
        avgUpper = (int) Core.mean(upper_region_Cb).val[0];
        avgLower = (int) Core.mean(lower_region_Cb).val[0];

        /*
         * Draw a simple box around the region of interest
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                upper_region_pointA, // First point which defines the rectangle
                upper_region_pointB, // Second point which defines the rectangle
                new Scalar(0, 0, 255), // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        Imgproc.rectangle(
                input, // Buffer to draw on
                lower_region_pointA, // First point which defines the rectangle
                lower_region_pointB, // Second point which defines the rectangle
                new Scalar(0, 0, 255), // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        //Decision logic using mean thresholds:
        //if the average is greater than 150 -> ringcase = 4
        //if the average is greater than 135 and less than 150 -> ringcase = 1
        //if the average is lower than 135 -> ringcase = 0

        if(avgLower > ONE_RING_THRESHOLD){
            ringCase = 0;
        }else if(avgLower < ONE_RING_THRESHOLD){
            ringCase = 1;
        }

        if(avgUpper < FOUR_RING_THRESHOLD && ringCase == 1){
            ringCase = 4;
        }

        image = Bitmap.createBitmap(input.cols(), input.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(input, image);

        return input;
    }

    private Bitmap getImage(){
        if(image != null){
            return image;
        }
        return Bitmap.createBitmap(1, 1, Bitmap.Config.ARGB_8888);
    }

    @Override
    public void onViewportTapped()
    {
        viewportPaused = !viewportPaused;

        if(viewportPaused)
        {
            webcam.pauseViewport();
        }
        else
        {
            webcam.resumeViewport();
        }
    }

    public int getAnalysis()
    {
        return ringCase;
    }
}
