package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@Autonomous(name = "RealAutonomous", group = "a")
public class RealAutonomous extends LinearOpMode {
    ElapsedTime timeElapsed;
    String zone = "A";

    // State variables
    private DcMotor fl, fr, bl, br;
    BNO055IMU imu;
    EasyOpenCV.UltimateGoalDeterminationPipeline pipeline;
    OpenCvInternalCamera phoneCam;


    RobotControlMethods robot = new RobotControlMethods(null, null, null, null,
            null);

    public void setup() {
        IMUSetup();
        motorSetup();
        gyroSetup();
        RobotControlMethodsSetup();
        OpenCVSetup();
    }

    public void IMUSetup() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void motorSetup() {
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");

        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void gyroSetup() {
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
    }

    private void RobotControlMethodsSetup() {
        robot.resetRobotControlMethods(fl, fr, bl, br, imu);
    }

    private void OpenCVSetup() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new EasyOpenCV.UltimateGoalDeterminationPipeline();
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

    public void runOpMode() throws InterruptedException {
        timeElapsed = new ElapsedTime();
        setup();
        runRobot();
    }

    public void runRobot() throws InterruptedException {

        do {
            setZone();
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Rings", pipeline.position);
            telemetry.addData("Zone", zone);
            telemetry.update();
        } while (!opModeIsActive());

        if (opModeIsActive() && !isStopRequested()) {
            // Robot instructions go here
            timeElapsed.startTime();

            setZone();

            robot.move("forward", 36, 1);

            switch (zone) {
                case "A":
                    robot.move("forward", 28, 1);
                    robot.move("right", 37, 0.5);
                    robot.move("backward", 72, 1);
                    robot.move("left", 15, 1);
                    robot.move("forward", 64, 1);
                    robot.move("right", 9, 0.5);
                    robot.move("backward", 9, 1);
                    robot.move("right", 12, 0.35);
                    robot.move("left", 55, 1);
                    break;

                case "B":
                    robot.move("forward", 42, 1);
                    robot.move("right", 10, 0.5);
                    robot.move("backward", 24, 1);
                    robot.move("right", 24, 1);
                    robot.move("backward", 54, 1);
                    robot.move("left", 12, 1);
                    robot.move("forward", 78, 1);
                    robot.move("left", 12, 1);
                    robot.move("backward", 24, 1);
                    robot.move("left", 30, 1);

                case "C":
                    robot.move("forward", 66, 1);
                    robot.move("right", 33, 0.5);
                    robot.move("backward", 101, 1);
                    robot.move("left", 12, 1);
                    robot.move("forward", 101, 1);
                    robot.move("right", 12, 0.5);
                    robot.move("backward", 48, 1);
                    robot.move("left", 55,1);

                default:
                    try {
                        throw new IllegalStateException("Zone cannot be determined");
                    } catch (IllegalStateException e) {
                        e.printStackTrace();
                    }
            }

            // shoot ring
            robot.move("right", 7.5, 1);
            // shoot ring
            robot.move("right", 7.5, 1);
            // shoot ring

            robot.move("forward", 9, 1);


            telemetry.addData("Time Elapsed", timeElapsed.time());
            telemetry.update();
            timeElapsed.reset();
            sleep(5000);
        }
    }

    private void setZone() {
        switch(pipeline.position) {
            case ZERO:
                zone = "A";
            case ONE:
                zone = "B";
            case FOUR:
                zone = "C";
        }
    }

    public static class UltimateGoalDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            ZERO
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar RED = new Scalar(255, 0, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point RegionTopLeftPoint = new Point(181,115);

        static final int REGION_WIDTH = 70;
        static final int REGION_HEIGHT = 50;

        final int FOUR_RING_THRESHOLD = 145;
        final int ONE_RING_THRESHOLD = 130;

        Point RegionTopLeft = new Point(
                RegionTopLeftPoint.x,
                RegionTopLeftPoint.y);
        Point RegionTopRight = new Point(
                RegionTopLeftPoint.x + REGION_WIDTH,
                RegionTopLeftPoint.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat RegionCb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int average;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile EasyOpenCV.UltimateGoalDeterminationPipeline.RingPosition position = EasyOpenCV.UltimateGoalDeterminationPipeline.RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            RegionCb = Cb.submat(new Rect(RegionTopLeft, RegionTopRight));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            average = (int) Core.mean(RegionCb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    RegionTopLeft, // First point which defines the rectangle
                    RegionTopRight, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = EasyOpenCV.UltimateGoalDeterminationPipeline.RingPosition.ZERO; // Record our analysis
            if (average > FOUR_RING_THRESHOLD) {
                position = EasyOpenCV.UltimateGoalDeterminationPipeline.RingPosition.FOUR;
            } else if (average > ONE_RING_THRESHOLD) {
                position = EasyOpenCV.UltimateGoalDeterminationPipeline.RingPosition.ONE;
            } else {
                position = EasyOpenCV.UltimateGoalDeterminationPipeline.RingPosition.ZERO;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    RegionTopLeft, // First point which defines the rectangle
                    RegionTopRight, // Second point which defines the rectangle
                    RED, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return average;
        }
    }
}