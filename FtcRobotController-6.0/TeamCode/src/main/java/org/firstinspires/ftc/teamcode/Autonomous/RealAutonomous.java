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
    UltimateGoalDeterminationPipeline pipeline;
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

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
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

    public void runOpMode() throws InterruptedException {
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

        while (opModeIsActive()) {
            switch (zone) {
                case "A": default:
                    ZoneA();
                    break;
                case "B":
                    ZoneB();
                    break;
                case "C":
                    ZoneC();
                    break;
            }

            // shoot ring
            robot.move("right", 7.5, 1);
            // shoot ring
            robot.move("right", 7.5, 1);
            // shoot ring

            robot.move("forward", 18, 1);

            break;
        }
    }

    private void setZone() {
        switch(pipeline.position) {
            case ZERO:
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

    private void ZoneA() throws InterruptedException {
        robot.move("forward", 64, 1);
        robot.move("right", 37, 0.3);
        robot.move("backward", 72, 1);
        robot.move("left", 15, 1);
        robot.move("forward", 64, 1);
        robot.move("right", 9, 0.5);
        robot.move("backward", 9, 1);
        robot.move("right", 12, 0.35);
        robot.move("left", 55, 1);
    }

    private void ZoneB() throws InterruptedException {
        robot.move("forward", 88, 1);
        robot.move("right", 12, 0.5);
        robot.move("backward", 24, 1);
        robot.move("right", 24, 1);
        robot.move("right", 4, 0.3);
        robot.move("left", 1, 0.3);
        robot.move("backward", 66, 1);
        robot.move("right", 5, 0.35);
        robot.move("left", 12, 1);
        robot.move("forward", 84, 1);
        robot.move("left", 16, 1);
        robot.move("backward", 30, 1);
        robot.move("left", 26, 1);
    }

    private void ZoneC() throws InterruptedException {
        robot.move("forward", 102, 1);
        robot.move("right", 33, 0.5);
        robot.move("backward", 101, 1);
        robot.move("left", 12, 1);
        robot.move("forward", 101, 1);
        robot.move("right", 12, 0.5);
        robot.move("backward", 48, 1);
        robot.move("left", 55,1);
    }

    public static class UltimateGoalDeterminationPipeline extends OpenCvPipeline
    {
        public enum RingPosition
        {
            FOUR,
            ONE,
            ZERO
        }

        // Some colour constants
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

        // The values for the ring detection; TODO: Tune
        final int FOUR_RING_THRESHOLD = 140;
        final int ONE_RING_THRESHOLD = 130;

        Mat RegionCb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int average;

        // Volatile since accessed by OpMode thread without synchronization
        private volatile UltimateGoalDeterminationPipeline.RingPosition position =
                UltimateGoalDeterminationPipeline.RingPosition.ZERO;

        // This function takes the RGB frame, converts to YCrCb, and extracts the Cb channel to the
        // 'Cb' variable
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

            // Set initial position
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