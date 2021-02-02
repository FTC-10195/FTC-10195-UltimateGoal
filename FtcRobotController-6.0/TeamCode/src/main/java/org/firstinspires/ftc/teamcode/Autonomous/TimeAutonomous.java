/*
26.75 inches per second
Make sure battery voltage is as close to 12.77V as possible
 */

package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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

import static java.lang.Math.abs;

@Config
@Autonomous(name = "Autonomous", group = "1", preselectTeleOp = "KevalMechTele")
public class TimeAutonomous extends LinearOpMode {

    // Motor variables
    DcMotorEx fl, fr, bl, br, shooter, topIntake, bottomIntake, wobbleLifter;
    Servo ringPusher, wobbleGrabber;

    // Shooter variables
    public static int ringPusherIteration = 1;
    Integer[] cooldowns = {600, 100, 250};
    int shooterCooldown = cooldowns[0];
    Double[] ringPusherPositions = {0.3, 0.1, 0.4};
    int currentArrayIndex = 0;
    ElapsedTime shooterTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    ElapsedTime ringPusherTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    // Setup of ElapsedTime to track how long the robot is running; integral to a time-based autonomous
    ElapsedTime elapsedTime = new ElapsedTime();

    // OpenCV variables
    UltimateGoalDeterminationPipeline pipeline;
    OpenCvInternalCamera phoneCam;

    // The zone to place the wobble goals in, depends on the number of rings
    String zone;

    // IMU variables
    BNO055IMU imu;
    private double currentAngle = 0;
    Orientation lastAngles = new Orientation();

    FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double inchesInOneSecond = 26.75;
    public static double defaultPower = 0.5;

    /**
     * The main setup function; this occurs during init
     * Contains two other setup functions: motorSetup() and OpenCVSetup()
     */
    public void setup() {
        motorSetup();
        OpenCVSetup();
        IMUSetup();
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
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        topIntake = hardwareMap.get(DcMotorEx.class, "topRoller");
        bottomIntake = hardwareMap.get(DcMotorEx.class, "bottomRoller");
        wobbleLifter = hardwareMap.get(DcMotorEx.class, "lift");

        ringPusher = hardwareMap.get(Servo.class, "push");
        wobbleGrabber = hardwareMap.get(Servo.class, "grab");

        fl.setDirection(DcMotorEx.Direction.REVERSE);
        bl.setDirection(DcMotorEx.Direction.FORWARD);
        fr.setDirection(DcMotorEx.Direction.REVERSE);
        br.setDirection(DcMotorEx.Direction.REVERSE);
        topIntake.setDirection(DcMotorEx.Direction.REVERSE);
        wobbleLifter.setDirection(DcMotorEx.Direction.REVERSE);

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

    public void IMUSetup() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
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
            shooter.setPower(1);
            lowerWobble(0.5);

            sleep(1000);

            //wobble("grab");
            moveStraight(57, 0.75);
            sleep(500);
            shootRings(3);
            switch (zone) {
                case "A": default:
                    strafe(60, 0.75);
                    sleep(500);
                    strafe(24, 0.25);
                    sleep(500);
                    strafe(4, -0.25);
                    //wobble("release");
                    moveStraight(44, -0.3);
                    sleep(500);
                    moveStraight(12, -0.2);
                    //wobble("grab");
                    moveStraight(50, 0.3);
                    sleep(500);
                    moveStraight(20, 0.2);
                    //wobble("release");

                    break;

                case "B":
                    strafe(20, 0.75);
                    moveStraight(32, 0.75);
                    //wobble("release");
                    strafe(22, 0.75);
                    moveStraight(72, -0.75);
                    moveStraight(8, -0.25);
                    //wobble("grab");
                    moveStraight(72, 0.75);
                    strafe(10, -0.25);
                    //wobble("release");
                    moveStraight(16, -1);

                    break;

                case "C":
                    shooter.setPower(0);
                    strafe(30, 0.5);
                    moveStraight(24, -1);
                    sleep(250);
                    moveStraight(6, 0.5);
                    intakeOn();
                    sleep(500);
                    moveStraight(38, -0.125);
                    strafe(24, -0.5);
                    shooter.setPower(1);
                    moveStraight(36, 0.5);
                    shootRings(3);
                    intakeOff();

                    strafe(60, 0.75);
                    sleep(250);
                    strafe(12, 0.45);
                    sleep(250);
                    strafe(4, -0.25);
                    moveStraight(60, 0.5);
                    //wobble("release")
                    sleep(250);
                    moveStraight(48, -0.5);

                    /*
                    strafe(24, 0.5);
                    intakeOn();
                    moveStraight(48, -0.5);
                    moveStraight(12, -0.1);
                    intakeOff();
                    sleep(500);
                    strafe(20, -0.75);
                    sleep(500);
                    moveStraight(58, 0.75);
                    shootRings(1);
                    moveStraight(6, 0.5);

                     */

                    break;
            }
        }
    }

    /**
     * This function controls the forward and backward movement of the robot
     * @param inches The number of inches that the robot moves
     * @param movePower The power given to the wheels; negative if moving backwards
     */
    public void moveStraight(double inches, double movePower) {
        inches *= (defaultPower / abs(movePower)); // normalizes value of "inches" back to value at default power
        double time = inches / inchesInOneSecond; // time = distance / speed

        telemetry.addData("Time to Run", time);

        fl.setPower(movePower);
        fr.setPower(movePower);
        bl.setPower(movePower);
        br.setPower(movePower);

        elapsedTime.reset();

        while (opModeIsActive() && elapsedTime.seconds() < time) {
            telemetry.addData("Path", "Moving Straight: %2.5f S Elapsed", elapsedTime.seconds());
            telemetry.update();
        }

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    /**
     * This function controls the strafing movement of the robot
     * @param inches The amount of time (in seconds) that the robot runs
     * @param movePower The power given to the wheels; negative if moving left and positive if moving right
     */
    public void strafe(double inches, double movePower) {
        inches *= (defaultPower / abs(movePower));
        double time = inches / inchesInOneSecond;

        fl.setPower(movePower);
        fr.setPower(-movePower);
        bl.setPower(-movePower);
        br.setPower(movePower);

        elapsedTime.reset();

        while (opModeIsActive() && elapsedTime.seconds() < time) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", elapsedTime.seconds());
            telemetry.update();
        }

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    public void turn(double angle, double motorPower)
    {
        double flPower = 0, frPower = 0, blPower = 0, brPower = 0;
        resetAngle();

        flPower = motorPower;
        frPower = -motorPower;
        blPower = motorPower;
        brPower = -motorPower;

        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);

        TurnUntilAngleReached(angle);
    }

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentAngle = 0;
    }

    private void TurnUntilAngleReached(double degrees)
    {
        if (degrees < 0)
        {
            while (true)
            {
                if ((angleConversion() <= degrees)) break;
            }
        }

        else // degrees >= 0
        {
            while (true)
            {
                if ((angleConversion() >= degrees)) break;
            }
        }

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    private double angleConversion()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double changeInAngle = angles.firstAngle - lastAngles.firstAngle;
        currentAngle += changeInAngle;

        if (currentAngle > 179 || currentAngle > 360){
            currentAngle -= 360;
        } else if(currentAngle < -180){
            currentAngle += 360;
        }

        lastAngles = angles;
        return currentAngle;
    }

    public void shootRings(int numberOfRings) {
        while (ringPusherIteration <= (3 * numberOfRings)) {
            switch (ringPusherIteration % 3) {
                case 0:
                    shooterCooldown = cooldowns[2];
                    break;

                case 1:
                    shooterCooldown = cooldowns[0];
                    break;

                case 2:
                    shooterCooldown = cooldowns[1];
                    break;
            }

            telemetry.addData("ringPusherIteration", ringPusherIteration);
            telemetry.update();
            currentArrayIndex++;
            if (currentArrayIndex >= ringPusherPositions.length) {
                currentArrayIndex = 0;
            }
            telemetry.addLine("Running!");
            telemetry.update();
            ringPusher.setPosition(ringPusherPositions[currentArrayIndex]);
            telemetry.addLine("Still Running!");
            telemetry.update();
            ringPusherTimer.reset();
            ringPusherIteration++;

            sleep(shooterCooldown);
        }

        ringPusher.setPosition(ringPusherPositions[0]);
        ringPusherIteration = 1;
    }

    public void grabWobble() {
        wobbleGrabber.setPosition(0);
        sleep(300);
    }

    public void releaseWobble() {
        wobbleGrabber.setPosition(0.5);
        sleep(300);
    }

    public void liftWobble() {
        wobbleLifter.setPower(0.4);
    }

    public void lowerWobble(double time) {
        wobbleLifter.setPower(-0.4);
        elapsedTime.reset();
        while (opModeIsActive() && elapsedTime.time() < time) {}
        wobbleLifter.setPower(0);
    }

    public void wobble(String action) {
        lowerWobble(0.5);
        switch (action.toLowerCase()) {
            case "grab": default:
                grabWobble();
                break;

            case "release":
                releaseWobble();
                break;
        }
        liftWobble();
    }

    public void intakeOn() {
        topIntake.setPower(1);
        bottomIntake.setPower(1);
    }

    public void intakeOff() {
        topIntake.setPower(0);
        bottomIntake.setPower(0);
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
        static final int REGION_HEIGHT = 60;
        Point RegionBottomRight = new Point(
                RegionTopLeftPoint.x + REGION_WIDTH,
                RegionTopLeftPoint.y + REGION_HEIGHT);

        // The values for the ring detection
        final int FOUR_RING_THRESHOLD = 135;
        final int ONE_RING_THRESHOLD = 130;

        Mat RegionCb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int average;

        // Volatile because it's accessed by the OpMode thread without synchronization
        private volatile RingPosition position = RingPosition.ZERO;

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

            RegionCb = Cb.submat(new Rect(RegionTopLeft, RegionBottomRight));
        }

        /**
         * Draws the rectangle, converts the region colour values to a single value, then sets the
         * position of the ring depending on what value is returned
         * @param input
         * @return
         */
        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            average = (int) Core.mean(RegionCb).val[0];

            // Set initial ring position and zone
            position = RingPosition.ZERO;

            // Set position
            if (average >= FOUR_RING_THRESHOLD) {
                position = RingPosition.FOUR;
            } else if (average >= ONE_RING_THRESHOLD) {
                position = RingPosition.ONE;
            } else {
                position = RingPosition.ZERO;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    RegionTopLeft, // First point which defines the rectangle
                    RegionBottomRight, // Second point which defines the rectangle
                    RED, // The colour the rectangle is drawn in
                    2); // Negative thickness means solid fill

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