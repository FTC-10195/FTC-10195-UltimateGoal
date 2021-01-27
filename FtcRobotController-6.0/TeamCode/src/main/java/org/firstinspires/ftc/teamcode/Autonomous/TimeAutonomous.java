package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name = "TimeAutonomous", group = "1")
public class TimeAutonomous extends LinearOpMode {

    // Motor variables
    DcMotorEx fl, fr, bl, br;

    // Setup of ElapsedTime to track how long the robot is running; integral to a time-based autonomous
    ElapsedTime elapsedTime = new ElapsedTime();

    // OpenCV variables
    UltimateGoalDeterminationPipeline pipeline;
    OpenCvInternalCamera phoneCam;

    // The zone to place the wobble goals in, depends on the number of rings
    String zone;

    /**
     * The main setup function; this occurs during init
     * Contains two other setup functions: motorSetup() and OpenCVSetup()
     */
    public void setup() {
        motorSetup();
        OpenCVSetup();
    }

    /**
     * The motor setup function
     * Motors are set to brake so that the robot doesn't continue gliding forward after being told to stop
     */
    public void motorSetup() {
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");

        fl.setDirection(DcMotorEx.Direction.REVERSE);
        bl.setDirection(DcMotorEx.Direction.REVERSE);
        fr.setDirection(DcMotorEx.Direction.REVERSE);
        br.setDirection(DcMotorEx.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    /**
     * The OpenCV setup function
     * This setups up the camera to start streaming using OpenCV and with the right specifications
     */
    private void OpenCVSetup() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(
                OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new UltimateGoalDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // Optimized so the preview isn't messed up
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });
    }

    /**
     * The function that runs when the robot is initialized and when it is played; waitForStart()
     separates the init functions from the functions that run after play is pressed
     * @throws InterruptedException If the function is interrupted, it throws an error and the robot will stop
     */
    public void runOpMode() throws InterruptedException {
        setup();
        initLoop();
        waitForStart();
        runRobot();
    }

    /**
     * This loop constantly updates the zone while in initialization and provides telemetry data used for
     camera positioning
     */
    public void initLoop() {
        do {
            setZone();
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Rings", pipeline.position);
            telemetry.addData("Zone", zone);
            telemetry.update();
        } while (!opModeIsActive());
    }

    /**
     * Sets the zone depending on the # of rings that the camera detects
     */
    public void setZone() {
        switch (pipeline.position) {
            case ZERO: default:
                zone = "A";
                break;
            case ONE:
                zone = "B";
                break;
            case FOUR:
                zone = "C";
                break;
        }
    }

    /**
     * This function contains the main steps for the robot to follow
     */
    public void runRobot() {
        // Set movePower negative to move left/down
        if (opModeIsActive() && !isStopRequested()) {
            moveStraight(5, 0.25);
            moveStrafe(5, 0.25);
        }
    }

    /**
     * This function controls the forward and backward movement of the robot
     * @param time The amount of time (in seconds) that the robot runs
     * @param movePower The power given to the wheels; negative if moving backwards
     */
    public void moveStraight(double time, double movePower) {
        fl.setPower(movePower);
        fr.setPower(movePower);
        bl.setPower(movePower);
        br.setPower(movePower);

        elapsedTime.reset();

        while (opModeIsActive() && elapsedTime.seconds() < time) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", elapsedTime.seconds());
            telemetry.update();
        }
    }

    /**
     * This function controls the strafing movement of the robot
     * @param time The amount of time (in seconds) that the robot runs
     * @param movePower The power given to the wheels; negative if moving left and positive if moving right
     */
    public void moveStrafe(double time, double movePower) {
        fl.setPower(movePower);
        fr.setPower(-movePower);
        bl.setPower(-movePower);
        br.setPower(movePower);

        elapsedTime.reset();

        while (opModeIsActive() && elapsedTime.seconds() < time) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", elapsedTime.seconds());
            telemetry.update();
        }
    }

    public static class UltimateGoalDeterminationPipeline extends OpenCvPipeline
    {
        // The enumeration containing the positions of the rings
        public enum RingPosition
        {
            FOUR,
            ONE,
            ZERO
        }

        // Some colour constants for readability
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar RED = new Scalar(255, 0, 0);

        // The values for the region that the program detects in
        static final Point RegionTopLeftPoint = new Point(181,115);
        Point RegionTopLeft = new Point(
                RegionTopLeftPoint.x,
                RegionTopLeftPoint.y);
        static final int REGION_WIDTH = 70;
        static final int REGION_HEIGHT = 50;
        Point RegionTopRight = new Point(
                RegionTopLeftPoint.x + REGION_WIDTH,
                RegionTopLeftPoint.y + REGION_HEIGHT);

        // The values for the ring detection
        final int FOUR_RING_THRESHOLD = 140;
        final int ONE_RING_THRESHOLD = 130;

        Mat RegionCb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int average;

        // Volatile because it's accessed by the OpMode thread without synchronization
        private volatile UltimateGoalDeterminationPipeline.RingPosition position =
                UltimateGoalDeterminationPipeline.RingPosition.ZERO;

        /**
         * Takes the RGB frame, converts it to YCrCb, and extracts the Cb channel to the "Cb" variable
         * @param input the RGB frame that the camera detects
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        /**
         * The initialization period of the vision program
         * @param firstFrame The captured frame
         */
        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            RegionCb = Cb.submat(new Rect(RegionTopLeft, RegionTopRight));
        }

        /**
         * Draws the rectangle, converts the region colour values to a single number, then sets the
         * position of the ring depending on what value is returned
         * @param input
         * @return
         */
        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            average = (int) Core.mean(RegionCb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    RegionTopLeft, // First point which defines the rectangle
                    RegionTopRight, // Second point which defines the rectangle
                    BLUE, // The colour the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            // Set initial ring position and zone
            position = UltimateGoalDeterminationPipeline.RingPosition.ZERO;

            // Set position
            if (average >= FOUR_RING_THRESHOLD) {
                position = UltimateGoalDeterminationPipeline.RingPosition.FOUR;
            } else if (average >= ONE_RING_THRESHOLD) {
                position = UltimateGoalDeterminationPipeline.RingPosition.ONE;
            } else {
                position = UltimateGoalDeterminationPipeline.RingPosition.ZERO;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    RegionTopLeft, // First point which defines the rectangle
                    RegionTopRight, // Second point which defines the rectangle
                    RED, // The colour the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        /**
         * Used for camera positioning
         * @return The number used to calculate the number of rings
         */
        public int getAnalysis() {
            return average;
        }
    }
}