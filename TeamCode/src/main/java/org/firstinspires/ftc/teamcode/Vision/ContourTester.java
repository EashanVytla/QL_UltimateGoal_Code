package org.firstinspires.ftc.teamcode.Vision;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Rectangle;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.Vision.VisionConstants.FOUR_RING_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Vision.VisionConstants.ONE_RING_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Vision.VisionConstants.REGION_HEIGHT;
import static org.firstinspires.ftc.teamcode.Vision.VisionConstants.REGION_TOPLEFT_ANCHOR_POINT;
import static org.firstinspires.ftc.teamcode.Vision.VisionConstants.REGION_WIDTH;

@TeleOp
public class ContourTester extends LinearOpMode
{
    OpenCvCamera webcam;

    @Override
    public void runOpMode()
    {
        //Instantiating our Webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        SamplePipeline pipeline = new SamplePipeline();
        webcam.setPipeline(pipeline);

        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                //Tells EasyOpenCv to start streaming the webcam feed and process the information
                //The resolution is set here
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        while (opModeIsActive())
        {
            /*
             * Send some stats to the telemetry
             */
            FtcDashboard.getInstance().sendImage(pipeline.getImage());
            telemetry.addData("Analysis", pipeline.getAnalysis());


            //The "if" statement below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
            if(gamepad1.a)
            {
                /*
                 * IMPORTANT NOTE: calling stopStreaming() will indeed stop the stream of images
                 * from the camera (and, by extension, stop calling your vision pipeline). HOWEVER,
                 * if the reason you wish to stop the stream early is to switch use of the camera
                 * over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()
                 * (commented out below), because according to the Android Camera API documentation:
                 *         "Your application should only have one Camera object active at a time for
                 *          a particular hardware camera."
                 *
                 * NB: calling closeCameraDevice() will internally call stopStreaming() if applicable,
                 * but it doesn't hurt to call it anyway, if for no other reason than clarity.
                 *
                 * NB2: if you are stopping the camera stream to simply save some processing power
                 * (or battery power) for a short while when you do not need your vision pipeline,
                 * it is recommended to NOT call closeCameraDevice() as you will then need to re-open
                 * it the next time you wish to activate your vision pipeline, which can take a bit of
                 * time. Of course, this comment is irrelevant in light of the use case described in
                 * the above "important note".
                 */
                webcam.stopStreaming();
                //webcam.closeCameraDevice();
            }

            telemetry.update();

            //sleep(100);
        }
    }

    class SamplePipeline extends OpenCvPipeline
    {
        public boolean viewportPaused;

        Mat HSVMat = new Mat();
        Mat HMat = new Mat();
        Mat thresholdMat = new Mat();
        Mat contoursOnFrameMat = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();
        int numContoursFound = 0;

        private Bitmap image;

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        private volatile int ringCase = 0;

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();

            Imgproc.GaussianBlur(input, input, new Size(5, 5), 0);

            //FOR RINGS:
            Imgproc.cvtColor(input, HSVMat, Imgproc.COLOR_RGB2HSV);

            //FOR RINGS:
            Core.inRange(HSVMat, new Scalar(15, 155, 149), new Scalar(28, 255, 255), thresholdMat);

            //Imgproc.threshold(yCbCrChan2MatRing, thresholdMatRing, 50, 255, Imgproc.THRESH_BINARY_INV);

            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            numContoursFound = contoursList.size();
            input.copyTo(contoursOnFrameMat);
            //Imgproc.drawContours(contoursOnFrameMat, contoursList, -1, new Scalar(0, 0, 255), 2, 8);


            /////////////////////////////////////////////////////////////////////////////////////////////////////////

            //FOR RINGS:
            Core.inRange(HSVMat, new Scalar(2, 255, 170), new Scalar(32, 255, 255), thresholdMat);

            //Core.extractChannel(HSVMat, HMat, 0);

            //Imgproc.threshold(HMat, thresholdMat, 32, 255, Imgproc.THRESH_BINARY_INV);

            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            numContoursFound = contoursList.size();
            input.copyTo(contoursOnFrameMat);
            //Imgproc.drawContours(contoursOnFrameMat, contoursList, -1, new Scalar(0, 0, 255), 2, 8);

            for (MatOfPoint contour : contoursList) {
                Rect rect = Imgproc.boundingRect(contour);

                // Show chosen result
                if(rect.area() >= 1000) {
                    Imgproc.rectangle(contoursOnFrameMat, rect.tl(), rect.br(), new Scalar(255, 0, 0), 2);
                    Imgproc.putText(contoursOnFrameMat, String.valueOf(rect.area()), rect.tl(), 0, 0.5, new Scalar(255, 255, 255));
                }else if(rect.area() >= 500){
                    Imgproc.rectangle(contoursOnFrameMat, rect.tl(), rect.br(), new Scalar(255, 0, 0), 2);
                    Imgproc.putText(contoursOnFrameMat, String.valueOf(rect.area()), rect.tl(), 0, 0.5, new Scalar(255, 255, 255));
                }
            }

            image = Bitmap.createBitmap(contoursOnFrameMat.cols(), contoursOnFrameMat.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(contoursOnFrameMat, image);


            return input;
        }

        public Bitmap getImage(){
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
}
